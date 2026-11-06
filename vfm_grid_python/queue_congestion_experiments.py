"""Compare endpoint-described congestion with explicit queue events.

These are different properties, not equivalent encodings of one property.
Only the `shared_gate` row repeats the historical event/hold specification.
Optional whole-trace concrete monitor runs are never sequential refinements.
"""
from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
import time

from .benchmark_scenarios import scenario, conjunction, at
from .benchmark_timing import JOURNAL, PHASE, totals
from .common import dump
from .queue_abstract import run


def configurations(rows, columns, cars):
    base = scenario('shared_gate', rows, columns, cars)
    yield 'shared_gate', base
    for ordered in (False, True):
        config = copy.deepcopy(base)
        # Drop the auxiliary queue-node role, which this property does not use.
        config['roles'] = {'gate': 'N'}
        config['initial_constraints'] = [f'role_gate_degree = 2'] + [
            f'start_{i} != role_gate' for i in range(cars)] + [
            f'distance_to_goal_{i} >= 1' for i in range(cars)]
        config['scenario_stages'] = [
            dict(event={'queue': dict(cars=list(range(cars)), near='role_gate',
                                     kind='section', ordered=ordered)}),
            dict(event=conjunction(*(at(i, f'goal_{i}') for i in range(cars))))]
        # Grow capacity explicitly for fleets >3; no silent concrete mismatch.
        config['section_length'] = max(20, (cars-1)*6+2)
        yield 'ordered_queue_then_park' if ordered else 'queue_then_park', config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--rows', type=int, default=3)
    parser.add_argument('--columns', type=int, default=3)
    parser.add_argument('--cars', type=int, default=3)
    parser.add_argument('--bound', type=int, default=100)
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--monitor', action='store_true')
    parser.add_argument('--concrete-bound', type=int, default=200)
    parser.add_argument('--concrete-timeout', type=float, default=120)
    args = parser.parse_args()
    root = args.output_dir.resolve()
    root.mkdir(parents=True, exist_ok=False)
    results = []
    saved = {k: os.environ.get(k) for k in (JOURNAL,PHASE)}
    try:
        for name, config in configurations(args.rows,args.columns,args.cars):
            directory = root/name
            directory.mkdir()
            os.environ[JOURNAL], os.environ[PHASE] = str(directory/'abstract_timing.jsonl'), 'abstract'
            result = run(config, directory/'abstract', args.bound, args.timeout)
            row = dict(name=name, stages=len(config['scenario_stages']),
                       abstract=result, abstract_timings=totals(directory/'abstract_timing.jsonl', time.monotonic()))
            results.append(row)
            dump(root/'results.json', results)
            if args.monitor and result['status'] == 'witness':
                from .queue_monitor import run as refine
                try:
                    row['concrete'] = refine(directory/'abstract',directory/'concrete',
                                             args.concrete_bound,args.concrete_timeout,gif=True)
                except Exception as exc:
                    saved_result = directory/'concrete/result.json'
                    row['concrete'] = (json.loads(saved_result.read_text()) if saved_result.exists()
                                       else dict(status='error',error=str(exc)))
                dump(root/'results.json', results)
    finally:
        for k,v in saved.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
    lines = ['# Congestion-property pilot', '',
             'Different properties: do not read this as an equivalent-encoding speedup.', '',
             '| Property | Stages | Abstract status | Solver s | Monitor status | Monitor solver s |',
             '|---|---:|---|---:|---|---:|']
    for r in results:
        concrete = r.get('concrete', {})
        checking = f"{concrete['model_checking_seconds']:.3f}" if 'model_checking_seconds' in concrete else '—'
        lines.append(f"| {r['name']} | {r['stages']} | {r['abstract']['status']} | "
                     f"{r['abstract_timings'].get('model_checking',0):.3f} | {concrete.get('status','not run')} | "
                     f"{checking} |")
    (root/'REPORT.md').write_text('\n'.join(lines)+'\n')


if __name__ == '__main__':
    main()
