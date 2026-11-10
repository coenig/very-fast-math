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

from vfm_grid_python.common import ROOT, both, either, grid_edges, dump, execute, check, states
from vfm_grid_python.concrete.observations import observations, fleet_safety
from vfm_grid_python.scenarios.language import queue_observer
from vfm_grid_python.geometry import build
from vfm_grid_python.benchmark_timing import JOURNAL, PHASE, DEADLINE, totals, interval


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
    from vfm_grid_python.scenarios.topologies import combination_at
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


from vfm_grid_python.concrete.compiler import run as fixed, constraints

def connected_parkings(graph):
    parking={n['id'] for n in graph['nodes'] if n['kind']=='P'}
    if not parking:return True
    adj={n['id']:set() for n in graph['nodes']}
    for u,v in graph['edges']:adj[u].add(v);adj[v].add(u)
    seen={next(iter(parking))};todo=list(seen)
    while todo:
        for v in adj[todo.pop()]-seen:seen.add(v);todo.append(v)
    return parking<=seen


def run(config,output,bound=250,timeout=120,seed=0,max_topologies=100,
        enumerator='blind',projection='topology',solver='z3'):
    """Check candidate topologies until the first witness.

    ``enumerator='blind'`` is the property-blind affine order. ``'sat'``
    enumerates solutions of the static constraints (gamma) with PySMT; in that
    case ``projection='placement'`` also fixes starts/goals/roles per query.
    SAT solver calls are timed as ``enumeration``.
    """
    if enumerator not in ('blind','sat'):raise ValueError('enumerator must be blind or sat')
    if enumerator=='blind' and projection!='topology':raise ValueError('placement projection needs the sat enumerator')
    root=Path(output).resolve()
    root.mkdir(parents=True,exist_ok=False)
    dump(root/'config.json',config)
    saved={k:os.environ.get(k) for k in (JOURNAL,PHASE,DEADLINE)}
    started=time.monotonic()
    os.environ[JOURNAL],os.environ[PHASE],os.environ[DEADLINE]=str(root/'timing.jsonl'),'concrete',str(started+timeout)
    result=dict(status='topology_limit',topologies_checked=0,topologies_skipped=0,
                component_prefilter=False,seed=seed,timeout=timeout,enumerator=enumerator)
    if enumerator=='sat':
        from vfm_grid_python.scenarios.topologies import sat_layouts
        result.update(projection=projection,solver=solver)
        candidates=sat_layouts(config,seed,projection,solver,timer=lambda:interval('enumeration'),
                               report=root/'enumeration/static_constraints.json')
    else:
        candidates=timed_layouts(bounded_layouts(config['rows'],config['columns'],config['parkings'],seed,started+timeout))
    try:
        for index,graph in enumerate(candidates):
            if index>=max_topologies:
                result['status']='topology_limit';break
            if time.monotonic()-started>=timeout:
                result['status']='timeout';break
            result['topologies_considered']=index+1
            result['oracle_assignments_examined']=graph.get('oracle_assignments_examined')
            case=config
            if 'eta' in graph:
                from vfm_grid_python.scenarios.topologies import placement_constraints
                case=dict(config,initial_constraints=config.get('initial_constraints',[])+placement_constraints(graph['eta']))
            attempt=fixed(case,graph,root/f'topology_{index:04d}',bound,timeout)
            with (root/'topologies.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(index=index,graph=graph,result=attempt))+'\n')
            result.update(status=attempt['status'],topologies_checked=result['topologies_checked']+1,
                          oracle_assignments_examined=graph.get('oracle_assignments_examined'))
            if attempt['status']=='witness':
                result.update(attempt);break
            if attempt['status']!='no_witness_within_bound':break
        else:
            result['status']=('gamma_unsatisfiable' if enumerator=='sat' and not result['topologies_checked']
                              else 'all_layouts_exhausted_within_bound')
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
    p.add_argument('--enumerator',choices=('blind','sat'),default='blind')
    p.add_argument('--projection',choices=('topology','placement'),default='topology')
    p.add_argument('--solver',choices=('z3','msat'),default='z3')
    args=p.parse_args()
    print(json.dumps(run(json.loads(args.config.read_text()),args.output_dir,args.bound,args.timeout,args.seed,
                         args.max_topologies,args.enumerator,args.projection,args.solver),indent=2))


if __name__=='__main__':main()
