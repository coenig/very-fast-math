"""Declarative queue manoeuvres on sparse symbolic grids; optional VFM monitor.

This is NOT a concrete enumeration benchmark. It records queue-search and
monitor checking times separately, with intermediate checkpoints.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time
import tomllib

from .common import dump, grid_edges
from .benchmark_scenarios import at, conjunction
from .benchmark_timing import JOURNAL, PHASE, totals


def configuration(family, scale, settings):
    search_options = {key: settings[key] for key in ('queue_solver', 'queue_transfer_encoding', 'queue_capacity_encoding')
                      if key in settings}
    if settings.get('property_source') == 'fleet_congestion':
        from .queue_congestion_scenarios import configuration as fleet_configuration
        c = fleet_configuration(family, scale)
        c.update(search_options)
        return c
    if settings.get('property_source') == 'paper':
        from .benchmark_scenarios import scenario
        from .queue_abstract import queue_predicate
        c=scenario(family,*scale['grid'],scale['cars'],scale.get('rounds',1))
        c.update(section_length=20,parking_length=7,junction_radius=6,
                 connection_length=9,queue_clearance=6)
        c.update(search_options)
        for stage in c['scenario_stages']:
            queue_predicate(stage['event'],c)
            queue_predicate(stage.get('hold',True),c)
        return c
    # Some families require more distinct ordinary nodes than others.
    scale = {**scale, **scale.get('family_overrides', {}).get(family, {})}
    rows,columns = scale['grid']
    cars = scale['cars']
    if cars < 2 or cars % 2 or cars >= rows*columns:
        raise ValueError('use even fleets >=2 and leave at least one ordinary node')
    gap = settings.get('queue_clearance',6)
    # One whole fleet must fit in reversal/rotation, half in an arm exchange.
    occupants = cars if family != 'exchange_arms' else cars//2
    length = max(20,(occupants-1)*gap+2)
    roles = ({'gate':'N','arm_a':'N','arm_b':'N'}
             if family in ('exchange_arms','merge_park') else {'a':'N','b':'N'})
    if rows*columns-cars < len(roles):
        raise ValueError(f'{family} needs {len(roles)} distinct ordinary nodes in addition to {cars} parking nodes')
    config = dict(rows=rows,columns=columns,cars=cars,parkings=cars,
                  requires_common_parking_component=True,
                  roles=roles, distinct_starts=False, section_length=length, parking_length=7,
                  connection_length=settings.get('connection_length',12), queue_clearance=gap,
                  junction_radius=settings.get('junction_radius',8),
                  vfm=dict(MAXSPEEDNONEGO=5,LANES_MIN_SPEEDS='@(-5)@',LANES_MAX_SPEEDS='@(5)@'),
                  initial_constraints=[],scenario_stages=[])
    names = list(roles)
    config['initial_constraints'] += [f'role_{a} != role_{b}' for i,a in enumerate(names) for b in names[:i]]
    count = ' + '.join(f'(case edge_{a}_{b} : 1; TRUE : 0; esac)' for a,b in grid_edges(rows,columns))
    config['initial_constraints'] += [f"({count}) <= {scale['edge_budget']}"]
    def queue(group, ends, ordered=True):
        return {'queue':dict(cars=group,between=['role_'+x for x in ends],ordered=ordered)}
    group = list(range(cars))
    if family == 'gather_park':
        config['scenario_stages'].append(dict(event=queue(group,('a','b'),ordered=False)))
    elif family == 'merge_park':
        left,right=group[:cars//2],group[cars//2:]
        config['scenario_stages'] += [
            dict(event=conjunction(queue(left,('gate','arm_a'),ordered=False),
                                   queue(right,('gate','arm_b'),ordered=False))),
            dict(event=queue(group,('gate','arm_b'),ordered=False))]
    elif family in ('reverse_queue','rotate_queue'):
        config['scenario_stages'].append(dict(event=queue(group,('a','b'))))
        permutation = group[::-1] if family == 'reverse_queue' else [group[-1]]+group[:-1]
        config['scenario_stages'].append(dict(event=queue(permutation,('a','b'))))
    elif family == 'exchange_arms':
        left,right=group[:cars//2],group[cars//2:]
        config['scenario_stages'] += [
            dict(event=conjunction(queue(left,('gate','arm_a')),queue(right,('gate','arm_b')))),
            dict(event=conjunction(queue(left,('gate','arm_b')),queue(right,('gate','arm_a'))))]
    else:
        raise ValueError(f'unknown family: {family}')
    config['scenario_stages'].append(dict(event=conjunction(*(at(i,f'goal_{i}') for i in group))))
    config.update(search_options)
    return config


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--catalog',type=Path,default=Path('parking_benchmark_scenarios/queues/manoeuvres.toml'))
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--only',nargs='+')
    parser.add_argument('--abstract-timeout',type=float)
    parser.add_argument('--concrete-timeout',type=float)
    parser.add_argument('--monitor',action='store_true')
    parser.add_argument('--generate-only',action='store_true')
    args=parser.parse_args()
    settings=tomllib.loads(args.catalog.read_text())
    cases=[(f"{f['id']}__{s['id']}",f,s) for f in settings['family'] for s in settings['scale']]
    if args.only and set(args.only)-{x[0] for x in cases}:
        parser.error('unknown --only case')
    root=args.output_dir.resolve()
    root.mkdir(parents=True,exist_ok=False)
    results=[]
    saved={k:os.environ.get(k) for k in (JOURNAL,PHASE)}
    try:
        for name,f,s in cases:
            if args.only and name not in args.only:
                continue
            directory=root/name
            directory.mkdir()
            config=configuration(f['id'],s,settings)
            dump(directory/'config.json',config)
            effective_scale={**s,**s.get('family_overrides',{}).get(f['id'],{})}
            row=dict(name=name,description=f['description'],grid=[config['rows'],config['columns']],cars=config['cars'],
                     edge_budget=effective_scale['edge_budget'],section_length=config['section_length'])
            results.append(row)
            if args.generate_only:
                row['status']='configured'
                dump(root/'results.json',results)
                continue
            from .queue_abstract import run
            os.environ[JOURNAL],os.environ[PHASE]=str(directory/'abstract_timing.jsonl'),'abstract'
            row['abstract']=run(config,directory/'abstract',settings['abstract_bound'],
                                args.abstract_timeout or settings['abstract_timeout'])
            row['abstract_timings']=totals(directory/'abstract_timing.jsonl',time.monotonic())
            dump(root/'results.json',results)
            if args.monitor and row['abstract']['status']=='witness':
                from .queue_monitor import run as refine
                try:
                    row['concrete']=refine(directory/'abstract',directory/'concrete',
                        settings['concrete_bound'],args.concrete_timeout or settings['concrete_timeout'],gif=True)
                except Exception as exc:
                    p=directory/'concrete/result.json'
                    row['concrete']=json.loads(p.read_text()) if p.exists() else dict(status='error',error=str(exc))
                dump(root/'results.json',results)
    finally:
        for k,v in saved.items():
            if v is None: os.environ.pop(k,None)
            else: os.environ[k]=v
    lines=['# Declarative manoeuvre pilot','',
           '| Case | Abstract | Solver s | Monitor | Solver s |',
           '|---|---|---:|---|---:|']
    for r in results:
        a,c=r.get('abstract',{}),r.get('concrete',{})
        at=f"{r['abstract_timings']['model_checking']:.3f}" if 'abstract_timings' in r else '—'
        ct=f"{c['model_checking_seconds']:.3f}" if 'model_checking_seconds' in c else '—'
        lines.append(f"| {r['name']} | {a.get('status','not run')} | {at} | {c.get('status','not run')} | {ct} |")
    (root/'REPORT.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':
    main()
