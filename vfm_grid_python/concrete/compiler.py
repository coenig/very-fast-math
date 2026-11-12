"""Shared concrete queue-property compiler: fixed or symbolic topology."""
import json
import re
import sys
from pathlib import Path
from copy import deepcopy
from vfm_grid_python.common import ROOT, grid_edges, either, dump, execute, check, states
from vfm_grid_python.geometry import build
from vfm_grid_python.scenarios.language import queue_observer
from .observations import observations, fleet_safety
from .symbolic import skeleton, edge_name, validate_specialization

def constraints(config, graph, geometry, symbolic=False):
    n,cars=config['rows']*config['columns'],config['cars']
    lines=[x for x in observations(config,graph,geometry,symbolic)
           if not re.match(r'\s*(?:start_\d+|goal_\d+|role_\w+)\s*:=',x)]
    edges={tuple(sorted(e)) for e in graph['edges']}
    for u,v in grid_edges(config['rows'],config['columns']):
        lines += [f'DEFINE edge_{u}_{v} := {f"env.edge_{u}_{v}" if symbolic else "TRUE" if (u,v) in edges else "FALSE"};']
    for node in graph['nodes']:
        u=node['id']
        degree = (' + '.join(f'(case edge_{a}_{b} : 1; TRUE : 0; esac)'
                             for a,b in grid_edges(config['rows'],config['columns']) if u in (a,b)) or '0'
                  if symbolic else str(sum(u in edge for edge in edges)))
        lines += [f'DEFINE parking_{u} := {f"env.parking_{u}" if symbolic else "TRUE" if node["kind"]=="P" else "FALSE"};',
                  f'DEFINE degree_{u} := {degree};']
    # Fixed topology is already known in Python. Keeping a sum of 25 constant
    # case expressions can still trigger exponential case lifting in SMT
    # preprocessing, before the constants are simplified.
    parking_count = (' + '.join(f'(case parking_{u} : 1; TRUE : 0; esac)' for u in range(n))
                     if symbolic else str(sum(node['kind']=='P' for node in graph['nodes'])))
    lines += [f'DEFINE parking_count := {parking_count};']
    lines += [f'INVAR parking_{u} -> degree_{u} = 1;' for u in range(n)]
    lines += [f'INVAR edge_{u}_{v} -> !(parking_{u} & parking_{v});' for u,v in grid_edges(config['rows'],config['columns'])]
    if 'parkings' in config: lines += [f'INIT parking_count = {config["parkings"]};']
    if 'edges' in config:
        chosen={tuple(sorted(e)) for e in config['edges']}
        lines += [f'INIT edge_{u}_{v} = {"TRUE" if (u,v) in chosen else "FALSE"};' for u,v in grid_edges(config['rows'],config['columns'])]
    lines += [f'INIT parking_{u} = {"TRUE" if kind=="P" else "FALSE"};' for u,kind in config.get('labels',{}).items()]
    if config.get('profile') == 'paper':
        from vfm_grid_python.scenarios import paper
        lines += paper.static(config) + paper.safety_observations(config,geometry,symbolic)
        for i in range(cars):
            lines += ['INIT '+paper.atom({'at':[i,f'start_{i}']},config)+';',
                      f'INIT env.veh___6{i}9___.v = 0 & env.veh___6{i}9___.a = 0;',
                      f'INVAR road_{i} >= 0;']
        for expression in config.get('initial_constraints',[]):
            lines.append('INIT '+expression+';')
        lines += fleet_safety(config)
        lines += [paper.observer(config),'DEFINE refinement_phase := 0;', 'INVARSPEC !scenario_accepted;']
        return '\n'.join(lines)+'\n'
    lines += ['FROZENVAR']
    lines += [f' {p}_{i} : 0..{n-1};' for p in ('start','goal') for i in range(cars)]
    lines += [f' role_{name} : 0..{n-1};' for name in config.get('roles',{})]
    def label(var,kind):
        return either([f'{var} = {u} & '+('TRUE' if kind=='any' else ('' if kind=='P' else '!')+f'parking_{u}') for u in range(n)])
    for name,kind in config.get('roles',{}).items():
        lines += [f'INIT {label("role_"+name,kind)};',
                  f'DEFINE role_{name}_degree := case '+ ' '.join(f'role_{name} = {u} : degree_{u};' for u in range(n))+' TRUE : 0; esac;']
    for i in range(cars):
        columns = config['columns']
        lines += [f'DEFINE distance_to_goal_{i} := '
                  f'abs(start_{i} / {columns} - goal_{i} / {columns}) + '
                  f'abs(start_{i} mod {columns} - goal_{i} mod {columns});']
        lines += [f'INIT loc_{i} = start_{i};',f'INIT {label(f"start_{i}",config.get("start_types",["N"]*cars)[i])};',
                  f'INIT {label(f"goal_{i}","P")};',
                  f'INIT env.veh___6{i}9___.v = 0 & env.veh___6{i}9___.a = 0;',
                  f'INVAR road_{i} >= 0;']
        for key,p in (('starts','start'),('goals','goal')):
            if key in config: lines += [f'INIT {p}_{i} = {config[key][i]};']
        for j in range(i):
            if config.get('distinct_starts',True): lines += [f'INIT start_{i} != start_{j};']
            if config.get('distinct_goals',True): lines += [f'INIT goal_{i} != goal_{j};']
    # These pilots use topology/role constraints, not abstract tip-state syntax.
    for field,prefix in (('initial_constraints','INIT'),('constraints','INVAR')):
        for expression in config.get(field,[]):
            if re.search(r'\btip_\d+',expression):
                raise ValueError('enumeration does not accept raw abstract tip syntax; use attachment predicates')
            lines += [f'{prefix} {expression};']
    lines += fleet_safety(config)
    lines += ['DEFINE all_parked := '+ ' & '.join(f'loc_{i} = goal_{i}' for i in range(cars))+';']
    lines += [queue_observer(config),'DEFINE refinement_phase := 0;', 'INVARSPEC !scenario_accepted;']
    return '\n'.join(lines)+'\n'

def run(config,layout,output,bound,timeout,template=ROOT/'main_template.json',
        *, refinement=None, generate_only=False):
    symbolic = layout is None
    if symbolic:
        layout = skeleton(config)
    output=Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    graph=dict(layout, starts=[0]*config['cars'],goals=[0]*config['cars'],
               roles={name:0 for name in config.get('roles',{})},milestones=[[0]*config['cars']])
    geometry=build(graph,config.get('section_length',20),config.get('parking_length',7),config.get('junction_radius',6))
    text=constraints(config,graph,geometry,symbolic)
    if refinement is not None:
        text = text.replace('DEFINE refinement_phase := 0;', '')
        text = text.replace('INVARSPEC !scenario_accepted;', '')
        text += '\n' + refinement(config, graph, geometry)
    from .braking import with_speed, effective_parameters
    config=with_speed(config,config.get('vfm', {}).get('MAXSPEEDNONEGO', 5))
    if config.get('queue_clearance',6)<=effective_parameters(template,config['vfm'])['VEHICLE_LENGTH']:
        raise ValueError('queue clearance must exceed concrete car length')
    dump(output/'geometry.json',geometry)
    dump(output/'config.json',config)
    dump(output/'generation_job.json',dict(graph=graph,geometry=geometry,template_config=str(Path(template).resolve()),
                                         vfm=config['vfm'],baseline_only=True,symbolic_endpoint=symbolic))
    generation=execute([sys.executable,'-m','vfm_grid_python.concrete.generation','--worker',str(output)],ROOT,output/'generation.log',timeout)
    dump(output/'generation_result.json',generation)
    if generation['status']!='finished':return dict(status='generation_'+generation['status'])
    base=Path(json.loads((output/'model.json').read_text())['model'])
    main,count=re.subn(r'(?m)^INVARSPEC FALSE;?\s*$',lambda _:text,base.read_text())
    if count!=1:raise ValueError('cannot replace baseline property')
    model=base.with_name('main_queue_direct.smv')
    model.write_text(main)
    dump(output/'model.json',dict(model=str(model)))
    if generate_only:
        result = dict(status='generated', model=str(model))
        dump(output/'result.json', result)
        return result
    result=check(model,bound,timeout)
    if result['status']=='witness':
        from .traces import global_inventory,export_trace
        frames=states(model.parent/'trace.xml')
        if int(frames[-1]['scenario_phase'])!=len(config['scenario_stages']):
            raise ValueError('scenario not accepted')
        first=frames[0]
        graph.update(starts=[int(first[f'start_{i}']) for i in range(config['cars'])],
                     goals=[int(first[f'goal_{i}']) for i in range(config['cars'])],
                     roles={name:int(first[f'role_{name}']) for name in config.get('roles',{})})
        if config.get('profile') == 'paper':
            from vfm_grid_python.scenarios.roads import inventory
            from vfm_grid_python.scenarios.paper import names
            _, joints = inventory(config)
            graph['joint_assignments'] = {z:int(first[z]) for z in names(config)}
            graph.update(starts=[joints[j]['node'] for j in graph['starts']],
                         goals=[joints[j]['node'] for j in graph['goals']],
                         roles={n:joints[j]['node'] for n,j in graph['roles'].items()})
        if symbolic:
            graph['nodes'] = [dict(n,kind='P' if first[f'env.parking_{n["id"]}']=='TRUE' else 'N') for n in graph['nodes']]
            graph['edges'] = [s['edge'] for s in geometry['sections'] if first[edge_name(s,'env.')]=='TRUE']
        dump(output/'selected_graph.json',graph)
        inv=global_inventory(graph,geometry,config,output/'inventory',template,timeout,model=model)
        xml=export_trace(frames,geometry,output,True,inv,description='Direct VFM queue-scenario witness',validate_geometry=not symbolic)
        if symbolic:
            active = selected_geometry(geometry, graph, first)
            dump(output/'geometry.json',active)
            dump(output/'geometry_validation.json',validate_specialization(active,graph))
        result.update(trace=str(xml),validated=True,states=len(frames))
    dump(output/'result.json',result)
    return result


def selected_geometry(geometry, graph, first):
    active=deepcopy(geometry); active['nodes']=graph['nodes']
    active['sections']=[s for s in active['sections'] if first[edge_name(s,'env.')]=='TRUE']
    for s in active['sections']:
        i=s['id']; prefix=f'env.section_{i}'
        s['length']=int(first[f'env.section_{i}_end'])
        s['source']=[int(first[f'{prefix}.source.{a}']) for a in ('x','y')]
        s['drain']=[int(first[f'{prefix}.drain.{a}']) for a in ('x','y')]
        s['source_x'],s['source_y']=s['source']; s['angle']=int(first[f'{prefix}.angle'])
        if first[f"env.parking_{s['edge'][0]}"]=='TRUE': s['edge'].reverse()
        s['kind']='parking' if first[f"env.parking_{s['edge'][1]}"]=='TRUE' else 'road'
        if s['kind']=='parking': active['centers'][s['edge'][1]]=s['drain']
    ids={s['id'] for s in active['sections']}
    active['connections']=[c for c in active['connections'] if c['from'] in ids and c['to'] in ids and first[f"env.parking_{c['node']}"]=='FALSE']
    for c in active['connections']:
        for side in ('from','to'):
            c[side+'_end']=first[f"env.endpoint_{c['from']}_{c['to']}_{side}_end"]=='TRUE'
    return active
