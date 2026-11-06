"""Finite Cartesian catalogs and non-transitive, per-method axial pruning."""
from concurrent.futures import ThreadPoolExecutor, as_completed
from itertools import groupby, product
import json
import re
from pathlib import Path

from .benchmark_scenarios import scenario
from .braking import with_speed
from .common import dump


def expand(raw):
    sweep = raw['adaptive']
    grids, fleets, rounds = sweep['grids'], sweep['cars'], sweep['rounds']
    for values in (grids, fleets, rounds):
        if not values or len({json.dumps(v) for v in values}) != len(values):
            raise ValueError('adaptive levels must be nonempty and unique')
    if any(len(g) != 2 or any(type(v) is not int or v < 1 for v in g) for g in grids):
        raise ValueError('grid levels must be positive integer pairs')
    for values in (fleets, rounds):
        if any(type(v) is not int or v < 1 for v in values) or values != sorted(values):
            raise ValueError('cars and rounds must be increasing positive integers')
    if any(a[0] > b[0] or a[1] > b[1] for a, b in zip(grids, grids[1:])):
        raise ValueError('grid dimensions must be nondecreasing')
    cases, excluded = [], []
    for entry in raw['scenario']:
        if not re.fullmatch(r'[a-zA-Z0-9_-]+',entry['id']):
            raise ValueError('unsafe scenario ID')
        family = entry['family']
        if family == 'nearby_parking':
            candidates = [((-1,-1,-1), (2,3), 4, 1)]
        else:
            candidates = [(idx, grids[idx[0]], fleets[idx[1]], rounds[idx[2]])
                          for idx in product(range(len(grids)), range(len(fleets)), range(len(rounds)))]
        for indices, (rows, columns), cars, repeat in candidates:
            identifier = f"{entry['id']}__g{rows}x{columns}_c{cars}_r{repeat}"
            try:
                if family not in ('parking','ordered_arrivals','nearby_parking') and cars < 2:
                    raise ValueError('interaction requires at least two cars')
                parkings = entry.get('parkings', cars + (family in ('shared_bay','fleet_yield')))
                if family != 'nearby_parking' and rows*columns < cars + parkings:
                    raise ValueError('not enough distinct N starts and P goals')
                extra = 2 if family == 'two_shared_gates' else int(family == 'shared_gate')
                if extra and rows*columns < cars + parkings + extra:
                    raise ValueError('gate nodes must be distinct from all starts')
                if family == 'fleet_yield' and rows <= 2 and columns <= 2:
                    raise ValueError('grid cannot contain the required degree-three gate')
                config = scenario(family, rows, columns, cars, repeat, entry.get('parkings'))
            except ValueError as exc:
                # Unknown family/configuration errors must not disappear as exclusions.
                from .benchmark_scenarios import FAMILIES
                if family not in FAMILIES:
                    raise
                excluded.append(dict(id=identifier, reason=str(exc)))
                continue
            config['semantics'] = raw.get('semantics','legacy')
            config = with_speed(config, raw.get('max_speed',5))
            stages = len(config['scenario_stages'])
            cases.append(dict(id=identifier, family=family, config=config, repetitions=repeat,
                              stage_count=stages,
                              property_size_chars=len(json.dumps(config['scenario_stages'],sort_keys=True)),
                              sweep_family=entry['id'], scaling_indices=list(indices),
                              abstract_bound=entry.get('abstract_bound',raw.get('abstract_bound',max(200,10*stages+1))),
                              concrete_bound=entry.get('concrete_bound',raw.get('concrete_bound',max(1000,50*stages)))))
    if len({c['id'] for c in cases}) != len(cases):
        raise ValueError('duplicate adaptive scenario IDs')
    cases.sort(key=lambda c:(sum(c['scaling_indices']), c['id']))
    return cases, excluded


def closes_line(status):
    if status.startswith('pruned_') or status == 'witness':
        return None
    if 'timeout' in status:
        return 'timeout'
    if 'no_witness_within_bound' in status or status == 'all_layouts_exhausted_within_bound':
        return None
    return 'error'


def blocker(job, completed):
    case = job['case']
    indices = case['scaling_indices']
    if indices[0] < 0:
        return None
    for old in completed:
        reason = closes_line(old['status'])
        if not reason or old['method'] != job['method'] or old.get('sweep_family') != case['sweep_family']:
            continue
        prior = old.get('scaling_indices', [])
        if len(prior) != 3 or min(prior) < 0:
            continue
        changed = [i for i in range(3) if prior[i] != indices[i]]
        if len(changed) == 1 and prior[changed[0]] < indices[changed[0]]:
            return dict(instance=old['instance'], method=old['method'], reason=reason,
                        axis=('grid','cars','rounds')[changed[0]])
    return None


def schedule(jobs, completed, parallel, run_job, save):
    """Rank barriers prevent speculative descendants from defeating pruning.

    Jobs of equal rank can run concurrently; axial successors have greater rank.
    Resume reconstructs decisions from immutable per-attempt result records.
    """
    jobs = sorted(jobs, key=lambda j:(sum(j['case']['scaling_indices']), j['case']['id'], j['method']))
    with ThreadPoolExecutor(max_workers=parallel) as pool:
        for _, group in groupby(jobs, key=lambda j:sum(j['case']['scaling_indices'])):
            futures = []
            for job in group:
                blocked = blocker(job, completed)
                if blocked:
                    c = job['case']
                    result = dict(schema='endpoint-benchmark.v1', instance=c['id'], method=job['method'],
                                  semantics=c['config'].get('semantics','legacy'),
                                  contract_version='stable-concurrent.v1' if c['config'].get('semantics')=='stable_concurrent' else 'legacy',
                                  seed=job.get('seed',0),parallel=job.get('parallel',parallel),
                                  timeout_seconds=job.get('timeout'),
                                  abstract_bound=c['abstract_bound'],concrete_bound=c['concrete_bound'],
                                  directory=job['directory'], family=c['family'],
                                  rows=c['config']['rows'], columns=c['config']['columns'], cars=c['config']['cars'],
                                  repetitions=c['repetitions'], stage_count=c['stage_count'],
                                  property_size_chars=c['property_size_chars'],
                                  scaling_indices=c['scaling_indices'], sweep_family=c['sweep_family'],
                                  status='pruned_'+blocked['reason'], pruning=blocked, validated=False,
                                  wall_seconds=0, model_checking_seconds=0, time_to_validated_xml_seconds=None)
                    directory = Path(job['directory']); directory.mkdir(parents=True,exist_ok=False)
                    dump(directory/'job.json',job); dump(directory/'result.json',result)
                    save(result)
                else:
                    futures.append(pool.submit(run_job,job))
            batch=[]
            for future in as_completed(futures):
                result=future.result()
                batch.append(result)
                save(result)
            completed.extend(sorted(batch,key=lambda r:(r['instance'],r['method'])))
