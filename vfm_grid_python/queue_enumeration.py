"""Property-blind topology enumeration followed by direct VFM queue properties.

No abstract search or abstract trace is used. Placements and roles remain
symbolic in every concrete query. The native geometry/dynamics/observations
and collision additions are shared with queue_monitor, without its route or
milestone restrictions. Bounded failures are not proofs of unsatisfiability.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import random
import re
import sys
import time

from .common import ROOT, both, either, grid_edges, dump, execute, check, states
from .queue_monitor import observations, fleet_safety
from .queue_abstract import queue_observer
from .geometry import build
from .benchmark_timing import JOURNAL, PHASE, DEADLINE, totals, interval


def timed_layouts(layouts):
    """Measure candidate generation/rejection, never time spent checking a yield."""
    iterator = iter(layouts)
    while True:
        with interval('enumeration'):
            try:
                layout = next(iterator)
            except StopIteration:
                return
        yield layout


def bounded_layouts(rows,columns,parkings,seed,deadline):
    """Same property-blind affine order as benchmark_oracle, with deadline checks.

    Check rejected raw assignments too: waiting for the next valid layout may
    otherwise take longer than the whole campaign budget on a larger grid.
    """
    from .benchmark_oracle import combination_at
    n=rows*columns
    edges=grid_edges(rows,columns)
    masks=1 << len(edges)
    total=masks*math.comb(n,parkings)
    rng=random.Random(seed)
    stride=rng.randrange(1,total) if total>1 else 1
    while math.gcd(stride,total)!=1:stride=rng.randrange(1,total)
    offset=rng.randrange(total)
    for attempt in range(total):
        if attempt%128==0 and time.monotonic()>=deadline:
            raise TimeoutError('deadline during raw topology enumeration')
        index=(offset+attempt*stride)%total
        labels=set(combination_at(n,parkings,index//masks))
        chosen=[e for bit,e in enumerate(edges) if index%masks & (1<<bit)]
        degree=[0]*n
        if any(u in labels and v in labels for u,v in chosen):continue
        for u,v in chosen:degree[u]+=1;degree[v]+=1
        if any(degree[p]!=1 for p in labels):continue
        yield dict(rows=rows,columns=columns,
                   nodes=[dict(id=u,row=u//columns,column=u%columns,kind='P' if u in labels else 'N') for u in range(n)],
                   edges=[list(e) for e in chosen],oracle_assignment=str(index),oracle_assignments_examined=attempt+1)


def constraints(config, graph, geometry):
    n,cars=config['rows']*config['columns'],config['cars']
    lines=[x for x in observations(config,graph,geometry)
           if not re.match(r'\s*(?:start_\d+|goal_\d+|role_\w+)\s*:=',x)]
    edges={tuple(sorted(e)) for e in graph['edges']}
    for u,v in grid_edges(config['rows'],config['columns']):
        lines += [f'DEFINE edge_{u}_{v} := {"TRUE" if (u,v) in edges else "FALSE"};']
    for node in graph['nodes']:
        u=node['id']
        lines += [f'DEFINE parking_{u} := {"TRUE" if node["kind"]=="P" else "FALSE"};',
                  f'DEFINE degree_{u} := {sum(u in e for e in edges)};']
    lines += ['FROZENVAR']
    lines += [f' {p}_{i} : 0..{n-1};' for p in ('start','goal') for i in range(cars)]
    lines += [f' role_{name} : 0..{n-1};' for name in config.get('roles',{})]
    def label(var,kind):
        return either([f'{var} = {x["id"]}' for x in graph['nodes'] if kind=='any' or x['kind']==kind])
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
    lines += [queue_observer(config),'DEFINE refinement_phase := 0;', 'INVARSPEC !scenario_accepted;']
    return '\n'.join(lines)+'\n'


def fixed(config,layout,output,bound,timeout,template=ROOT/'main_template.json'):
    output=Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    graph=dict(layout, starts=[0]*config['cars'],goals=[0]*config['cars'],
               roles={name:0 for name in config.get('roles',{})},milestones=[[0]*config['cars']])
    geometry=build(graph,config.get('section_length',20),config.get('parking_length',7),config.get('junction_radius',6))
    text=constraints(config,graph,geometry)
    from .braking import with_speed, effective_parameters
    config=with_speed(config,5)
    if config.get('queue_clearance',6)<=effective_parameters(template,config['vfm'])['VEHICLE_LENGTH']:
        raise ValueError('queue clearance must exceed concrete car length')
    dump(output/'geometry.json',geometry)
    dump(output/'config.json',config)
    dump(output/'generation_job.json',dict(graph=graph,geometry=geometry,template_config=str(Path(template).resolve()),
                                         vfm=config['vfm'],baseline_only=True))
    generation=execute([sys.executable,'-m','vfm_grid_python.concretize','--worker',str(output)],ROOT,output/'generation.log',timeout)
    dump(output/'generation_result.json',generation)
    if generation['status']!='finished':return dict(status='generation_'+generation['status'])
    base=Path(json.loads((output/'model.json').read_text())['model'])
    main,count=re.subn(r'(?m)^INVARSPEC FALSE;?\s*$',lambda _:text,base.read_text())
    if count!=1:raise ValueError('cannot replace baseline property')
    model=base.with_name('main_queue_direct.smv')
    model.write_text(main)
    dump(output/'model.json',dict(model=str(model)))
    result=check(model,bound,timeout)
    if result['status']=='witness':
        from .sequential import global_inventory,export_trace
        frames=states(model.parent/'trace.xml')
        if int(frames[-1]['scenario_phase'])!=len(config['scenario_stages']):
            raise ValueError('scenario not accepted')
        first=frames[0]
        graph.update(starts=[int(first[f'start_{i}']) for i in range(config['cars'])],
                     goals=[int(first[f'goal_{i}']) for i in range(config['cars'])],
                     roles={name:int(first[f'role_{name}']) for name in config.get('roles',{})})
        dump(output/'selected_graph.json',graph)
        inv=global_inventory(graph,geometry,config,output/'inventory',template,timeout,model=model)
        xml=export_trace(frames,geometry,output,True,inv,description='Direct VFM queue-scenario witness on enumerated topology')
        result.update(trace=str(xml),validated=True,states=len(frames))
    dump(output/'result.json',result)
    return result


def connected_parkings(graph):
    parking={n['id'] for n in graph['nodes'] if n['kind']=='P'}
    if not parking:return True
    adj={n['id']:set() for n in graph['nodes']}
    for u,v in graph['edges']:adj[u].add(v);adj[v].add(u)
    seen={next(iter(parking))};todo=list(seen)
    while todo:
        for v in adj[todo.pop()]-seen:seen.add(v);todo.append(v)
    return parking<=seen


def run(config,output,bound=250,timeout=120,seed=0,max_topologies=100,component_prefilter=False):
    if component_prefilter and not config.get('requires_common_parking_component'):
        raise ValueError('component prefilter requires a scenario whose stages imply all parking nodes are connected')
    root=Path(output).resolve()
    root.mkdir(parents=True,exist_ok=False)
    dump(root/'config.json',config)
    saved={k:os.environ.get(k) for k in (JOURNAL,PHASE,DEADLINE)}
    started=time.monotonic()
    os.environ[JOURNAL],os.environ[PHASE],os.environ[DEADLINE]=str(root/'timing.jsonl'),'concrete',str(started+timeout)
    result=dict(status='topology_limit',topologies_checked=0,topologies_skipped=0,
                component_prefilter=component_prefilter,seed=seed,timeout=timeout)
    try:
        for index,graph in enumerate(timed_layouts(bounded_layouts(config['rows'],config['columns'],config['parkings'],seed,started+timeout))):
            if index>=max_topologies:
                result['status']='topology_limit';break
            if time.monotonic()-started>=timeout:
                result['status']='timeout';break
            result['topologies_considered']=index+1
            result['oracle_assignments_examined']=graph.get('oracle_assignments_examined')
            connected = True
            if component_prefilter:
                with interval('enumeration'):
                    connected = connected_parkings(graph)
            if not connected:
                result['topologies_skipped']+=1
                with (root/'topologies.jsonl').open('a') as stream:
                    stream.write(json.dumps(dict(index=index,graph=graph,status='skipped_disconnected_parking_component'))+'\n')
                continue
            attempt=fixed(config,graph,root/f'topology_{index:04d}',bound,timeout)
            with (root/'topologies.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(index=index,graph=graph,result=attempt))+'\n')
            result.update(status=attempt['status'],topologies_checked=result['topologies_checked']+1,
                          oracle_assignments_examined=graph.get('oracle_assignments_examined'))
            if attempt['status']=='witness':
                result.update(attempt);break
            if attempt['status']!='no_witness_within_bound':break
        else:
            result['status']='all_layouts_exhausted_within_bound'
    except TimeoutError as exc:
        result.update(status='timeout',error=str(exc))
    except Exception as exc:
        result.update(status='error',error=str(exc))
        raise
    finally:
        result.update(wall_seconds=time.monotonic()-started,
                      timings_seconds=totals(root/'timing.jsonl',time.monotonic()))
        result['enumeration_seconds']=result['timings_seconds'].get('enumeration',0)
        dump(root/'result.json',result)
        for k,v in saved.items():
            if v is None:os.environ.pop(k,None)
            else:os.environ[k]=v
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('config',type=Path)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--bound',type=int,default=250)
    p.add_argument('--timeout',type=float,default=120)
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--max-topologies',type=int,default=100)
    p.add_argument('--component-prefilter',action='store_true',help='separate property-aware connectivity control, not blind enumeration')
    args=p.parse_args()
    print(json.dumps(run(json.loads(args.config.read_text()),args.output_dir,args.bound,args.timeout,args.seed,args.max_topologies,args.component_prefilter),indent=2))


if __name__=='__main__':main()
