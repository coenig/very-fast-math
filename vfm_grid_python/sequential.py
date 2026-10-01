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
from .local_model import GeometryIndex, OCCUPANCY, flatten, flat_body, global_state, handoff


def step_constraints(left, right, geometry):
    lines=[]
    for car,(u,v) in enumerate(zip(left,right)):
        terms=[location_expression(geometry,car,u),location_expression(geometry,car,v)]
        if u!=v:
            section=next(s for s in geometry['sections'] if set(s['edge'])=={u,v})
            terms.append(f"env.veh___6{car}9___.on_straight_section = {section['id']}")
        lines.append('INVAR '+either(terms)+';')
    lines.append(collision_constraints(geometry,len(left)))
    return '\n'.join(lines)+'\n'


def write_step(text, variables, geometry, mapping, left, right, previous, directory):
    initial={}
    model=flat_body(text,resumed=previous is not None)
    if previous is None:
        model+='\n'.join('INIT '+location_expression(geometry,c,n,stopped=True)+';'
                          for c,n in enumerate(left))+'\n'
        counter=0
    else:
        initial=handoff(previous,variables,mapping)
        model+='\n'.join(f'INIT {k} = {v};' for k,v in initial.items())+'\n'
        counter=int(previous['env.cnt'])
    model+=step_constraints(left,right,geometry)
    dump(directory/'initial_state.json',initial)
    target=both([location_expression(geometry,c,n) for c,n in enumerate(right)])
    dump(directory/'target.json',{'source':left,'target':right,'predicate':target,'after_tick':counter})
    # The existing concrete clock enforces a real transition, even for repeats.
    reach=f'INVARSPEC !({target} & env.cnt > {counter});\n'
    (directory/'segment.smv').write_text(model+reach)
    sanity=directory/'initial_check'
    sanity.mkdir()
    (sanity/'initial.smv').write_text(model+'INVARSPEC FALSE;\n')
    return initial


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


def export_trace(frames,geometry,directory,complete):
    """Delta-encoded stitched trace, explicitly not a single solver witness."""
    prefix='' if complete else 'partial_'
    root=ET.Element('counter-example',{'type':'stitched-sequential','complete':str(complete).lower(),'id':'1'})
    text=['Trace Description: Stitched sequential-local concrete trace', 'Trace Type: Counterexample']
    # Clear aliases that occurred in a different patch, otherwise sparse trace
    # readers can retain an old TRUE traversal flag across a boundary.
    aliases={k for s in frames for k in s if OCCUPANCY.match(k)}
    static=geometry_values(geometry)
    old={}
    for i,frame in enumerate(frames):
        current={**static,**{k:'0' for k in aliases},**frame}
        node=ET.SubElement(root,'node')
        state=ET.SubElement(node,'state',{'id':str(i+1)})
        text.append(f'  -> State: 1.{i+1} <-')
        for key,value in sorted(current.items()):
            if old.get(key)!=str(value):
                ET.SubElement(state,'value',{'variable':key}).text=str(value)
                text.append(f'    {key} = {value}')
        old=current
    xml=directory/(prefix+'trace.xml')
    ET.ElementTree(root).write(xml,encoding='unicode')
    (directory/(prefix+'debug_trace_array.txt')).write_text('\n'.join(text)+'\n')
    return xml


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
        replay_model=None):
    if generate_only: raise ValueError('sequential-local requires checking to determine subsequent initial states')
    if bound<1 or timeout<=0: raise ValueError('bound must be >=1 and timeout >0')
    abstract_run,output=Path(abstract_run).resolve(),Path(output).resolve()
    if output.exists() and any(output.iterdir()): raise FileExistsError(f'use a new output directory: {output}')
    output.mkdir(parents=True,exist_ok=True)
    started=time.perf_counter()
    graph=json.loads((abstract_run/'selected_graph.json').read_text())
    config=json.loads((abstract_run/'config.json').read_text())
    validate_graph(graph)
    geometry=build(graph,road_length,parking_length,radius)
    geometry_seconds=time.perf_counter()-started
    dump(output/'geometry.json',geometry)
    dump(output/'selected_graph.json',graph)
    dump(output/'config.json',{'abstract_run':str(abstract_run),'bound':bound,'timeout':timeout,
                             'strategy':'sequential-local','vfm':config.get('vfm',{})})
    index=GeometryIndex(geometry)
    summary={'status':'running','strategy':'sequential-local','segments':[],
             'geometry_seconds':geometry_seconds,
             'generation_seconds':0.0,'flatten_seconds':0.0,'initial_check_seconds':0.0,
             'checking_seconds':0.0,'validation_seconds':0.0,'rendering_seconds':0.0}
    dump(output/'result.json',summary)
    frames=[]; previous=None; cache={}
    milestones=graph['milestones']
    pairs=list(zip(milestones,milestones[1:]))
    # A zero-move abstract witness is a valid initial-state reachability query.
    if not pairs: pairs=[(milestones[0],milestones[0])]
    try:
        for i,(left,right) in enumerate(pairs):
            step=output/f'segment_{i:04d}'; step.mkdir()
            record={'index':i,'status':'running'}; summary['segments'].append(record)
            dump(output/'result.json',summary)
            local,mapping=index.select(left,right,previous)
            dump(step/'geometry.json',local); dump(step/'mapping.json',mapping)
            record.update(sections=len(local['sections']),connectors=len(local['connections']))
            key=hashlib.sha256(json.dumps(local,sort_keys=True).encode()).hexdigest()
            if key not in cache:
                generation=output/'models'/key[:16]; generation.mkdir(parents=True)
                dump(generation/'generation_job.json',{'graph':{'milestones':[left]},'geometry':local,
                     'template_config':str(Path(template_config).resolve()),'vfm':config.get('vfm',{}),'baseline_only':True})
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
            record['baseline']=str(generation)
            initial=write_step(text,variables,local,mapping,left,right,previous,step)
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
            if result['status']!='witness': summary['status']=result['status']; break
            local_frames=states(step/'trace.xml')
            if len(local_frames)<2: raise ValueError('segment did not perform a concrete transition')
            for name,value in initial.items():
                if local_frames[0].get(name)!=value:
                    raise ValueError(f'seam does not preserve {name}')
            for state in local_frames:
                missing=set(variables)-state.keys()
                if missing: raise ValueError(f'trace lacks declared state: {sorted(missing)}')
            projected=[global_state(s,mapping) for s in local_frames]
            for state in projected: state['refinement_phase']=str(i if len(milestones)>1 else 0)
            projected[-1]['refinement_phase']=str(min(i+1,len(milestones)-1))
            if frames:
                if frames[-1]['env.cnt']!=projected[0]['env.cnt']: raise ValueError('clock reset at seam')
                frames.extend(projected[1:])
            else: frames.extend(projected)
            previous=projected[-1]
            dump(step/'terminal_global_state.json',previous)
            record['states']=len(local_frames)
            record['status']='witness'
            export_trace(frames,geometry,output,False)
            dump(output/'result.json',summary)
        else:
            summary['status']='witness'
    except Exception as exc:
        summary.update(status='error',error=str(exc))
        if summary['segments']: summary['segments'][-1].update(status='error',error=str(exc))
    if frames:
        complete=summary['status']=='witness'
        xml=export_trace(frames,geometry,output,False)
        summary['time_to_trace_seconds']=time.perf_counter()-started
        validation_started=time.perf_counter()
        try:
            from .validate import validate_trace
            partial_graph={**graph,'milestones':milestones[:int(frames[-1]['refinement_phase'])+1]}
            report=validate_trace(graph if complete else partial_graph,geometry,xml)
            report.update(complete=complete,source='stitched local solver witnesses; not one monolithic check')
            if complete and replay_model:
                summary['replay']=replay(replay_model,frames,geometry,output/'replay',timeout)
                if summary['replay']['status']!='witness': summary['status']='replay_failed'
            if complete and summary['status']=='witness':
                xml=export_trace(frames,geometry,output,True)
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
    arguments(p)
    a=p.parse_args()
    result=run(a.abstract_run,a.output_dir,a.concrete_bound,a.timeout,a.road_length,a.parking_length,
               a.junction_radius,a.template_config,not a.no_gif,replay_model=a.replay_model)
    return 0 if result['status']=='witness' else 2


if __name__=='__main__': raise SystemExit(main())
