"""Concretize one abstract transition at a time on a local VFM road model."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

from .common import ROOT, both, either, dump, execute, check, states
from .concretize import arguments
from .endpoint import collision_constraints
from .geometry import build, location_expression
from .local_model import GeometryIndex, OCCUPANCY, flatten, flat_body, global_state, handoff, schema


def handoff_stop_cars(left, right, following, next_milestone=None):
    """No artificial rest boundaries; parking observation still requires v=0.

    Length-dependent limits and the two-goal window constrain continuation.
    Kept as the policy hook for callers inspecting the handoff requirements.
    """
    return []


def step_constraints(left, right, geometry,following=None,collisions=True):
    lines=[]
    for car,(u,v) in enumerate(zip(left,right)):
        terms=[location_expression(geometry,car,u),location_expression(geometry,car,v)]
        if u!=v:
            section=next(s for s in geometry['sections'] if set(s['edge'])=={u,v})
            terms.append(f"env.veh___6{car}9___.on_straight_section = {section['id']}")
        lines.append('INVAR '+either(terms)+';')
        if following is not None and following[car]!=v:
            # A grid node abstracts several concrete connectors. Entering a
            # connector unrelated to the next road can strand the next local
            # query: changing connectors would require leaving the junction.
            onward=next(s['id'] for s in geometry['sections'] if set(s['edge'])=={v,following[car]})
            p=f'env.veh___6{car}9___'
            compatible=either([f'{p}.on_straight_section >= 0',
                               f'{p}.traversion_from = {onward}',f'{p}.traversion_to = {onward}'])
            lines.append(f'INVAR {location_expression(geometry,car,v)} -> {compatible};')
    if collisions:
        lines.append(collision_constraints(geometry,len(left)))
    return '\n'.join(lines)+'\n'


def write_step(text, variables, geometry, mapping, left, right, previous, directory,
               rejected=(),require_transition=True,following=None,stop_cars=(),
               scenario_phase=None, initial_scenario_phase=None, contract_graph=None,
               lookahead=None):
    from .stable import observation, support
    if lookahead is not None and contract_graph:
        raise ValueError('lookahead is only supported for legacy semantics')
    observe=(lambda g,c,n,stopped=False: observation(g,c,n,contract_graph)) if contract_graph else location_expression
    initial={}
    model=flat_body(text,resumed=previous is not None)
    if previous is None:
        model+='\n'.join('INIT '+observe(geometry,c,n,stopped=True)+';'
                          for c,n in enumerate(left))+'\n'
        counter=0
        if initial_scenario_phase is not None:
            model += f'INIT scenario_phase = {initial_scenario_phase};\n'
    else:
        initial=handoff(previous,variables,mapping)
        model+='\n'.join(f'INIT {k} = {v};' for k,v in initial.items())+'\n'
        counter=int(previous['env.cnt'])
    if contract_graph:
        model+='\n'.join('INVAR '+support(geometry,c,u,v,contract_graph)+';' for c,(u,v) in enumerate(zip(left,right)))+'\n'
        model+=collision_constraints(geometry,len(left))
    elif lookahead is not None:
        for phase, (source, destination, onward) in enumerate((
                (left,right,following), (right,lookahead['target'],lookahead['following']))):
            constraints=step_constraints(source,destination,geometry,onward,collisions=False)
            guard='lookahead_phase = 0' if phase==0 else 'lookahead_phase >= 1'
            for line in constraints.splitlines():
                if line.startswith('INVAR '):
                    model+=f'INVAR ({guard}) -> ({line[6:-1]});\n'
                elif line.strip():
                    raise ValueError('unexpected non-invariant stage support')
        model+=collision_constraints(geometry,len(left))
    else:
        model+=step_constraints(left,right,geometry,following)
    for terminal in rejected:
        assignment=handoff(terminal,variables,mapping)
        model+='INVAR !'+both([f'{name} = {value}' for name,value in assignment.items()])+';\n'
    dump(directory/'initial_state.json',initial)
    target=both([observe(geometry,c,n) for c,n in enumerate(right)])
    if contract_graph and scenario_phase is not None:
        model+=f'TRANS next({target}) -> next(scenario_phase) = {scenario_phase};\n'
    target=both([target]+[f'env.veh___6{c}9___.v = 0 & env.veh___6{c}9___.a = 0' for c in stop_cars])
    if scenario_phase is not None:
        target=both([target, f'scenario_phase = {scenario_phase}'])
    dump(directory/'target.json',{'source':left,'target':right,'predicate':target,'after_tick':counter,
                                'stopped_cars':list(stop_cars)})
    # The existing concrete clock enforces a real transition, even for repeats.
    progress=f' & env.cnt > {counter}' if require_transition else ''
    reach=f'INVARSPEC !({target}{progress});\n'
    if lookahead is not None:
        final=both([observe(geometry,c,n) for c,n in enumerate(lookahead['target'])]+
                   [f'env.veh___6{c}9___.v = 0 & env.veh___6{c}9___.a = 0'
                    for c in lookahead['stop_cars']])
        if lookahead.get('scenario_phase') is not None:
            final=both([final,f"scenario_phase = {lookahead['scenario_phase']}"])
        model+=('VAR lookahead_phase : 0..2;\nASSIGN init(lookahead_phase) := 0;\n'
                'next(lookahead_phase) := case\n'
                f'lookahead_phase = 0 & next({target}) : 1;\n'
                f'lookahead_phase = 1 & next({final}) : 2;\n'
                'TRUE : lookahead_phase; esac;\n')
        reach='INVARSPEC lookahead_phase != 2;\n'
        dump(directory/'lookahead.json',dict(lookahead, handoff_predicate=target,
                                           final_predicate=final))
    (directory/'segment.smv').write_text(model+reach)
    sanity=directory/'initial_check'
    sanity.mkdir()
    (sanity/'initial.smv').write_text(model+'INVARSPEC FALSE;\n')
    return initial


def annotate_monitor(frames,graph,geometry):
    """Recompute the deterministic global monitor from physical observations.

Braking can keep a local query running after its positions were first reached.
The monitor advances on that first observation, not on the query boundary.
"""
    milestones=graph['milestones']; kinds={n['id']:n['kind'] for n in geometry['nodes']}
    connectors={(c['from'],c['to']):c['node'] for c in geometry['connections']}
    def observed(frame,car,node):
        p=f'env.veh___6{car}9___.'; s=int(frame[p+'on_straight_section'])
        if graph.get('semantics')=='stable_concurrent':
            from .stable import anchors
            neighbour=anchors(graph)[node]
            section=next(t for t in geometry['sections'] if set(t['edge'])=={node,neighbour})
            pos=section['length'] if section['edge'][1]==node else 0
            return s==section['id'] and int(frame[p+'abs_pos'])==pos and int(frame[p+'v'])==0 and int(frame[p+'a'])==0
        if s<0:
            return kinds[node]=='N' and connectors.get((int(frame[p+'traversion_from']),int(frame[p+'traversion_to'])))==node
        section=geometry['sections'][s]; pos=int(frame[p+'abs_pos'])
        at=(section['edge'][0]==node and pos==0) or (section['edge'][1]==node and pos==section['length'])
        return at and (kinds[node]!='P' or int(frame[p+'v'])==0)
    phase=0
    for tick,frame in enumerate(frames):
        if tick and phase+1<len(milestones) and all(observed(frame,c,n) for c,n in enumerate(milestones[phase+1])):
            phase+=1
        frame['refinement_phase']=str(phase)


def cached_baseline(directory,key,geometry,cars,config,template_config):
    """Reuse an explicitly supplied, completed cache from the same VFM build."""
    if directory is None: return None
    base=Path(directory).resolve()/key[:16]
    if not (base/'flat/baseline.smv').exists() or not (base/'generation_job.json').exists(): return None
    job=json.loads((base/'generation_job.json').read_text())
    # JSON normalizes integer dictionary keys (geometry centers) to strings.
    matches=(job.get('baseline_only') and job['geometry']==json.loads(json.dumps(geometry))
             and len(job['graph']['milestones'][0])==cars
             and job.get('vfm',{})==config.get('vfm',{})
             and job.get('semantics','legacy')==config.get('semantics','legacy')
             and Path(job['template_config']).resolve()==Path(template_config).resolve())
    if not matches: return None
    text=(base/'flat/baseline.smv').read_text()
    return text,schema(text),base


def geometry_values(geometry):
    """Native-renderer metadata in global coordinates, no symbolic full model."""
    result={}
    outgoing={s['id']:[] for s in geometry['sections']}
    def put(name,value): result['env.'+name]=str(value)
    for s in geometry['sections']:
        i=s['id']; angle=s['angle']
        put(f'section_{i}_end',s['length'])
        for point in ('source','drain'):
            for axis,value in zip(('x','y'),s[point]): put(f'section_{i}.{point}.{axis}',value)
        put(f'section_{i}.angle',angle); put(f'section_{i}.angle_raw',angle//45)
        put(f'sin_of_section_{i}_angle',round(100*math.sin(math.radians(angle))))
        put(f'cos_of_section_{i}_angle',round(100*math.cos(math.radians(angle))))
        put(f'section_{i}_segment_0_pos_begin',0)
        put(f'section_{i}_segment_0_min_lane',0); put(f'section_{i}_segment_0_max_lane',0)
    for c in geometry['connections']:
        a,b=c['from'],c['to']; outgoing[a].append(b)
        put(f'endpoint_{a}_{b}_from_end',str(c['from_end']).upper())
        put(f'endpoint_{a}_{b}_to_end',str(c['to_end']).upper())
        put(f'arclength_from_sec_{a}_to_sec_{b}_on_lane_0',c['length'])
    slots=max(1,max(map(len,outgoing.values())))
    for section,targets in outgoing.items():
        for slot,target in enumerate(targets+[-1]*(slots-len(targets))):
            put(f'outgoing_connection_{slot}_of_section_{section}',target)
    return result


def export_trace(frames,geometry,directory,complete,model=None,description=None,validate_geometry=True):
    """Full XML states and delta-encoded text, in nuXmv's trace format.

Only a model-completed, validated export is marked complete. Partial files
remain useful for diagnostics before the global model has been generated.
"""
    prefix='' if complete else 'partial_'
    if complete and model is None:
        raise ValueError('a complete export requires the global model inventory')
    if model is not None:
        from .benchmark_timing import interval
        with interval('validation'):
            frames=model.complete(frames,geometry,preserve_topology=not validate_geometry)
            model.validate(frames)
        static=geometry_values(geometry)
        for name,value in (static.items() if validate_geometry else ()):
            if name in frames[0] and frames[0][name]!=value:
                raise ValueError(f'global model geometry mismatch: {name}')
    description = description or ('Python-stitched sequential concrete trace' if complete else 'Partial sequential concrete trace')
    root=ET.Element('counter-example',{'type':'0','id':'1','desc':description})
    text=['Trace Description: '+description, 'Trace Type: Counterexample']
    # Clear aliases that occurred in a different patch, otherwise sparse trace
    # readers can retain an old TRUE traversal flag across a boundary.
    aliases={k for s in frames for k in s if OCCUPANCY.match(k)}
    static=geometry_values(geometry)
    old={}
    for i,frame in enumerate(frames):
        current=frame if model is not None else {**static,**{k:'0' for k in aliases},**frame}
        node=ET.SubElement(root,'node')
        state=ET.SubElement(node,'state',{'id':str(i+1)})
        text.append(f'  -> State: 1.{i+1} <-')
        for key,value in sorted(current.items()):
            ET.SubElement(state,'value',{'variable':key}).text=str(value)
            if old.get(key)!=str(value):
                text.append(f'    {key} = {value}')
        old=current
    ET.SubElement(root,'loops').text=' '
    xml=directory/(prefix+'trace.xml')
    pending=directory/(prefix+'trace.pending.xml')
    ET.indent(root)
    ET.ElementTree(root).write(pending,encoding='unicode')
    if model is not None:
        with interval('validation'):
            report=model.validate(states(pending))
        if hasattr(model,'source'):
            report['xml_import']=model.check_xml_import(pending,directory/'xml_import',model.timeout)
        report.update(complete=complete,source='Python concatenation, checked against full concrete model')
        dump(directory/(prefix+'model_validation.json'),report)
    pending.replace(xml)
    (directory/(prefix+'debug_trace_array.txt')).write_text('\n'.join(text)+'\n')
    return xml


def global_inventory(graph,geometry,config,output,template_config,timeout,model=None):
    """Generate/flatten the global model once, without a global solver search."""
    from .trace_model import TraceModel
    output.mkdir(parents=True,exist_ok=True)
    if model is None:
        dump(output/'generation_job.json',{'graph':graph,'geometry':geometry,
             'template_config':str(Path(template_config).resolve()),'vfm':config.get('vfm',{}),
             'scenario':config if config.get('scenario_stages') else None,
             'braking_margin':config.get('braking_margin',0)})
        result=execute([sys.executable,'-m','vfm_grid_python.concretize','--worker',str(output)],
                       ROOT,output/'generation.log',timeout)
        dump(output/'generation_result.json',result)
        if result['status']!='finished': raise RuntimeError(f'global inventory generation {result["status"]}')
        model=Path(json.loads((output/'model.json').read_text())['model'])
    text,variables,result=flatten(model,output/'flat',timeout)
    inventory=TraceModel(text)
    inventory.source=(output/'flat/baseline.smv').resolve()
    inventory.timeout=timeout
    expected_cars={f'env.veh___6{i}9___.on_straight_section' for i in range(len(graph['milestones'][0]))}
    import re
    actual_cars={n for n in set(variables)|inventory.state_defines
                 if re.fullmatch(r'env\.veh___6\d+9___\.on_straight_section',n)}
    if actual_cars!=expected_cars: raise ValueError('global validation model has a different fleet')
    symbols=set(variables)|inventory.state_defines
    required=set(geometry_values(geometry))|{'refinement_phase'}
    if required-symbols:
        raise ValueError(f'global validation model lacks required geometry/monitor symbols: {sorted(required-symbols)[:8]}')
    expected_sections={f'env.section_{s["id"]}_end' for s in geometry['sections']}
    actual_sections={n for n in symbols if re.fullmatch(r'env\.section_\d+_end',n)}
    if actual_sections!=expected_sections: raise ValueError('global validation model has different sections')
    dump(output/'inventory.json',{'model':str(Path(model).resolve()),'variables':variables,
         'state_defines':sorted(inventory.state_defines),
         'transition_defines':sorted(n for n in inventory.defines
                                    if not n.startswith('__expr') and n not in inventory.state_defines)})
    return inventory


def validate_graph(graph):
    milestones=graph['milestones']
    if not milestones or not milestones[0]: raise ValueError('empty abstract witness')
    cars=len(milestones[0]); edges={frozenset(e) for e in graph['edges']}
    nodes={n['id'] for n in graph['nodes']}
    for m in milestones:
        if len(m)!=cars or len(set(m))!=cars or not set(m)<=nodes: raise ValueError('invalid abstract state')
    for a,b in zip(milestones,milestones[1:]):
        if any(u!=v and frozenset((u,v)) not in edges for u,v in zip(a,b)):
            raise ValueError('nonadjacent abstract move')
        if any(a[i]==b[j] and a[j]==b[i] for i in range(cars) for j in range(i)):
            raise ValueError('adjacent car exchange')


def replay(full_model,frames,geometry,output,timeout):
    """Check the entire fixed state sequence against an existing full model."""
    output=Path(output).resolve()
    text,variables,flat_result=flatten(Path(full_model).resolve(),output/'flat',timeout)
    mapping={'local_to_global':list(range(len(geometry['sections'])))}
    lines=[flat_body(text)]
    for frame in frames:
        assignment=handoff(frame,variables,mapping)
        lines.append('INVAR env.cnt = '+frame['env.cnt']+' -> '+both([f'{k} = {v}' for k,v in assignment.items()])+';')
    lines.append(f"INVARSPEC env.cnt != {frames[-1]['env.cnt']};")
    model=output/'replay.smv'; model.write_text('\n'.join(lines))
    result=check(model,len(frames)-1,timeout)
    result['flatten_seconds']=flat_result['seconds']
    dump(output/'replay_result.json',result)
    return result


def run(abstract_run,output,bound=200,timeout=1800,road_length=20,parking_length=7,
        radius=6,template_config=ROOT/'main_template.json',gif=True,generate_only=False,
        replay_model=None,validation_model=None,backtracks=0,model_cache=None,lookahead=True,
        max_speed=None,braking_margin=0,braking_check_only=False):
    if generate_only: raise ValueError('sequential-local requires checking to determine subsequent initial states')
    if bound<1 or timeout<=0: raise ValueError('bound must be >=1 and timeout >0')
    if backtracks<0: raise ValueError('backtracks must be nonnegative')
    abstract_run,output=Path(abstract_run).resolve(),Path(output).resolve()
    if output.exists() and any(output.iterdir()): raise FileExistsError(f'use a new output directory: {output}')
    output.mkdir(parents=True,exist_ok=True)
    started=time.perf_counter()
    graph=json.loads((abstract_run/'selected_graph.json').read_text())
    config=json.loads((abstract_run/'config.json').read_text())
    from .braking import with_speed, preflight
    config=with_speed(config,max_speed)
    config['braking_margin']=braking_margin
    validate_graph(graph)
    stable=config.get('semantics')=='stable_concurrent'
    if stable:
        from .stable import profile,validate_witness
        if graph.get('semantics')!='stable_concurrent': raise ValueError('abstract witness has no stable contract')
        dump(output/'contract.json',profile(config,road_length,parking_length,radius))
        dump(output/'contract_validation.json',validate_witness(graph,config))
    if config.get('scenario_stages'):
        phases=graph.get('scenario_phases')
        if phases is None or len(phases)!=len(graph['milestones']):
            raise ValueError('scenario witness lacks observer phases')
        if phases[-1]!=len(config['scenario_stages']):
            raise ValueError('abstract witness did not accept the original scenario')
    from .benchmark_timing import interval
    with interval('generation'):
        geometry=build(graph,road_length,parking_length,radius)
    geometry_seconds=time.perf_counter()-started
    dump(output/'geometry.json',geometry)
    dump(output/'selected_graph.json',graph)
    dump(output/'config.json',{**config,'abstract_run':str(abstract_run),'bound':bound,'timeout':timeout,
                             'strategy':'sequential-local','vfm':config.get('vfm',{}),
                             'lookahead':bool(lookahead and not stable),
                             'backtracks':backtracks,'model_cache':str(model_cache) if model_cache else None})
    report=preflight(geometry,config,template_config,output,braking_margin)
    if not report['passed'] or braking_check_only:
        summary={'status':'braking_checked' if report['passed'] else 'braking_budget_failed',
                 'braking_check':report,'seconds':time.perf_counter()-started,
                 'generation_seconds':0.0,'checking_seconds':0.0,'solver_seconds':0.0}
        dump(output/'result.json',summary)
        return summary
    index=GeometryIndex(geometry)
    summary={'status':'running','strategy':'sequential-local','segments':[],
             'lookahead':bool(lookahead and not stable),
             'geometry_seconds':geometry_seconds,
             'generation_seconds':0.0,'flatten_seconds':0.0,'initial_check_seconds':0.0,
             'checking_seconds':0.0,'validation_seconds':0.0,'rendering_seconds':0.0,
             'backtracks_used':0,'accepted_segments':[]}
    dump(output/'result.json',summary)
    frames=[]; previous=None; cache={}
    milestones=graph['milestones']
    pairs=list(zip(milestones,milestones[1:]))
    # A zero-move abstract witness is a valid initial-state reachability query.
    initial_only=not pairs
    if not pairs: pairs=[(milestones[0],milestones[0])]
    attempts={}; rejected={}; checkpoints=[]; i=0
    try:
        while i<len(pairs):
            left,right=pairs[i]
            # Look beyond waits at the same node, too. No exact coordinates or
            # velocity strategy are prescribed by this compatibility filter.
            following=[next((m[c] for m in milestones[i+2:] if m[c]!=v),v)
                       for c,v in enumerate(right)]
            stop_cars=handoff_stop_cars(
                left,right,following,
                milestones[i+2] if i+2<len(milestones) else None)
            preview=None
            incoming=[next((m[c] for m in reversed(milestones[:i]) if m[c]!=u),u)
                      for c,u in enumerate(left)]
            if lookahead and not stable and i+2<len(milestones):
                future=milestones[i+2]
                onward=[next((m[c] for m in milestones[i+3:] if m[c]!=v),v)
                        for c,v in enumerate(future)]
                preview=dict(target=future,following=onward,
                    stop_cars=handoff_stop_cars(right,future,onward,
                        milestones[i+3] if i+3<len(milestones) else None),
                    scenario_phase=graph['scenario_phases'][i+2] if config.get('scenario_stages') else None)
            attempt=attempts.get(i,0); attempts[i]=attempt+1
            suffix=f'_attempt_{attempt:02d}' if attempt else ''
            step=output/f'segment_{i:04d}{suffix}'; step.mkdir()
            record={'index':i,'attempt':attempt,'directory':str(step),'status':'running'}
            summary['segments'].append(record)
            dump(output/'result.json',summary)
            local,mapping=index.select(left,right,previous,following=None if stable else following,
                                       incoming=incoming if lookahead and not stable else None,
                                       continuation=(preview['target'],preview['following']) if preview else None)
            dump(step/'geometry.json',local); dump(step/'mapping.json',mapping)
            record.update(sections=len(local['sections']),connectors=len(local['connections']))
            cache_description=dict(geometry=local,contract='local-speed.v1',braking_margin=braking_margin,
                                   semantics='stable-concurrent.v1' if stable else 'legacy')
            key=hashlib.sha256(json.dumps(cache_description,sort_keys=True).encode()).hexdigest()
            if key not in cache:
                reused=cached_baseline(model_cache,key,local,len(left),config,template_config)
                if reused:
                    cache[key]=reused; record['reused_model']=True
                    print(f'Step {i+1}/{len(pairs)}: reusing cached local model',flush=True)
            if key not in cache:
                generation=output/'models'/key[:16]; generation.mkdir(parents=True)
                dump(generation/'generation_job.json',{'graph':{'milestones':[left]},'geometry':local,
                     'template_config':str(Path(template_config).resolve()),'vfm':config.get('vfm',{}),'baseline_only':True,
                     'semantics':config.get('semantics','legacy'),'braking_margin':braking_margin})
                print(f"Step {i+1}/{len(pairs)}: generating {len(local['sections'])} sections",flush=True)
                result=execute([sys.executable,'-m','vfm_grid_python.concretize','--worker',str(generation)],
                               ROOT,generation/'generation.log',timeout)
                dump(generation/'generation_result.json',result)
                summary['generation_seconds']+=result['seconds']
                if result['status']!='finished':
                    record['status']='generation_'+result['status']; summary['status']=record['status']; break
                model=Path(json.loads((generation/'model.json').read_text())['model'])
                text,variables,flat_result=flatten(model,generation/'flat',timeout)
                summary['flatten_seconds']+=flat_result['seconds']
                cache[key]=(text,variables,generation)
                record['generation_seconds']=result['seconds']
            else: record['generation_seconds']=0.0
            text,variables,generation=cache[key]
            if config.get('scenario_stages'):
                from .benchmark_scenarios import observer
                from .stable import observation
                boundary='next('+both([observation(local,c,n,graph) for c,n in enumerate(right)])+')' if stable else 'TRUE'
                text += '\n' + observer(config,local,graph,boundary)
                variables = schema(text)
            record['baseline']=str(generation)
            initial=write_step(text,variables,local,mapping,left,right,previous,step,
                               rejected.get(i,()),require_transition=not initial_only,
                               following=following,stop_cars=stop_cars,
                               scenario_phase=(graph['scenario_phases'][min(i+1,len(milestones)-1)]
                                               if config.get('scenario_stages') else None),
                               initial_scenario_phase=(graph['scenario_phases'][0]
                                                       if config.get('scenario_stages') else None),
                               contract_graph=graph if stable else None,lookahead=preview)
            initial_result=check(step/'initial_check/initial.smv',0,timeout)
            summary['initial_check_seconds']+=initial_result['seconds']
            record['initial_check']=initial_result
            if initial_result['status']!='witness':
                record['status']=('inconsistent_initial_state' if initial_result['status']=='no_witness_within_bound'
                                  else 'initial_check_'+initial_result['status'])
                summary['status']=record['status']; break
            result=check(step/'segment.smv',bound,timeout)
            summary['checking_seconds']+=result['seconds']
            record.update(status=result['status'],checking_seconds=result['seconds'])
            if result['status']!='witness':
                if (result['status'] in ('no_witness_within_bound','timeout') and i>0
                        and summary['backtracks_used']<backtracks):
                    # Reject exactly the unsuccessful handoff, not just its
                    # positions. Other velocities/history may admit continuation.
                    rejected.setdefault(i-1,[]).append(previous)
                    for later in list(rejected):
                        if later>=i: del rejected[later]
                    frames=frames[:checkpoints.pop()]
                    previous=frames[-1] if frames else None
                    summary['accepted_segments'].pop()
                    summary['backtracks_used']+=1
                    record['backtracked']=True
                    i-=1
                    if frames: export_trace(frames,geometry,output,False)
                    else:
                        for name in ('partial_trace.xml','partial_debug_trace_array.txt'):
                            (output/name).unlink(missing_ok=True)
                    dump(output/'result.json',summary)
                    print(f'Retrying step {i+1} with a different terminal state',flush=True)
                    continue
                summary['status']=result['status']; break
            local_frames=states(step/'trace.xml')
            if preview is not None:
                if local_frames[-1].get('lookahead_phase')!='2':
                    raise ValueError('lookahead witness did not reach its second goal')
                cut=next(j for j,s in enumerate(local_frames) if s.get('lookahead_phase')=='1')
                record.update(lookahead_states=len(local_frames),handoff_index=cut)
                # Keep the exact witnessed boundary; the proven continuation is
                # retained in segment/trace.xml but is not committed twice.
                local_frames=local_frames[:cut+1]
                for state in local_frames:
                    state.pop('lookahead_phase',None)
            if len(local_frames)<(1 if initial_only else 2):
                raise ValueError('segment did not perform a concrete transition')
            for name,value in initial.items():
                if local_frames[0].get(name)!=value:
                    raise ValueError(f'seam does not preserve {name}')
            for state in local_frames:
                missing=set(variables)-state.keys()
                if missing: raise ValueError(f'trace lacks declared state: {sorted(missing)}')
            projected=[global_state(s,mapping) for s in local_frames]
            for state in projected: state['refinement_phase']=str(i if len(milestones)>1 else 0)
            projected[-1]['refinement_phase']=str(min(i+1,len(milestones)-1))
            checkpoints.append(len(frames))
            if frames:
                if frames[-1]['env.cnt']!=projected[0]['env.cnt']: raise ValueError('clock reset at seam')
                frames.extend(projected[1:])
            else: frames.extend(projected)
            previous=projected[-1]
            dump(step/'terminal_global_state.json',previous)
            record['states']=len(local_frames)
            record['status']='witness'
            summary['accepted_segments'].append(str(step))
            export_trace(frames,geometry,output,False)
            dump(output/'result.json',summary)
            i+=1
        else:
            summary['status']='witness'
    except KeyboardInterrupt:
        summary.update(status='interrupted')
        if summary['segments']: summary['segments'][-1].update(status='interrupted')
    except Exception as exc:
        summary.update(status='error',error=str(exc))
        if summary['segments']: summary['segments'][-1].update(status='error',error=str(exc))
    if frames:
        complete=summary['status']=='witness'
        partial_graph={**graph,'milestones':milestones[:int(frames[-1]['refinement_phase'])+1]}
        annotate_monitor(frames,graph if complete else partial_graph,geometry)
        xml=export_trace(frames,geometry,output,False)
        summary['time_to_prefix_seconds']=time.perf_counter()-started
        validation_started=time.perf_counter()
        try:
            from .validate import validate_trace
            report=validate_trace(graph if complete else partial_graph,geometry,xml)
            report.update(complete=complete,source='stitched local solver witnesses; not one monolithic check')
            if complete and replay_model:
                summary['replay']=replay(replay_model,frames,geometry,output/'replay',timeout)
                if summary['replay']['status']!='witness': summary['status']='replay_failed'
            if complete and summary['status']=='witness':
                print('Completing and validating all global model variables...',flush=True)
                inventory=global_inventory(graph,geometry,config,output/'global_model',
                                           template_config,timeout,validation_model or replay_model)
                xml=export_trace(frames,geometry,output,True,inventory)
                if config.get('scenario_stages') and int(frames[-1]['scenario_phase']) != len(config['scenario_stages']):
                    raise ValueError('concrete witness did not accept the original scenario')
                summary['time_to_trace_seconds']=time.perf_counter()-started
                report['full_model']=json.loads((output/'model_validation.json').read_text())
            else:
                report['complete']=False
            dump(output/('validation.json' if summary['status']=='witness' else 'partial_validation.json'),report)
        except Exception as exc:
            summary.update(status='validation_error',error=str(exc))
        summary['validation_seconds']=time.perf_counter()-validation_started
        summary['trace']=str(xml)
        summary['states']=len(frames)
        if gif:
            render_started=time.perf_counter()
            try:
                from .visualize import concrete_plot
                concrete_plot(geometry,output,xml,True)
            except Exception as exc: summary['render_error']=str(exc)
            summary['rendering_seconds']=time.perf_counter()-render_started
    summary['seconds']=time.perf_counter()-started
    summary['solver_seconds']=summary['initial_check_seconds']+summary['checking_seconds']
    dump(output/'result.json',summary)
    print(f"Sequential result: {summary['status']}; solver {summary['solver_seconds']:.2f}s; {output}",flush=True)
    if summary.get('error'): print(summary['error'],flush=True)
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('abstract_run',type=Path)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--timeout',type=float,default=1800)
    p.add_argument('--no-gif',action='store_true')
    p.add_argument('--replay-model',type=Path,help='optional full concrete model for fixed-trace replay validation')
    p.add_argument('--validation-model',type=Path,help='reuse a matching global monitor model for Python trace validation')
    p.add_argument('--backtracks',type=int,default=0,
                   help='maximum retries of a preceding segment after a failed continuation (default: 0)')
    p.add_argument('--model-cache',type=Path,
                   help='reuse completed local models from a previous run/models directory using the same VFM build/templates')
    p.add_argument('--no-lookahead',action='store_true',help='disable the default two-goal window for legacy refinement')
    arguments(p)
    a=p.parse_args()
    result=run(a.abstract_run,a.output_dir,a.concrete_bound,a.timeout,a.road_length,a.parking_length,
               a.junction_radius,a.template_config,not a.no_gif,replay_model=a.replay_model,
               validation_model=a.validation_model,backtracks=a.backtracks,model_cache=a.model_cache,
               lookahead=not a.no_lookahead,max_speed=a.max_speed,
               braking_margin=a.braking_margin,braking_check_only=a.braking_check_only)
    return 0 if result['status'] in ('witness','braking_checked') else 2


if __name__=='__main__': raise SystemExit(main())
