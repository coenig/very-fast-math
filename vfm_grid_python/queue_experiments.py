"""Run bounded queue-abstraction pilots and an endpoint-abstraction baseline.

No concrete refinement is attempted: the endpoint sequentializer cannot consume
queue interiors. Historical concrete results are context, not fresh comparisons.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time

from . import abstract, queue_abstract
from .benchmark_scenarios import scenario
from .benchmark_timing import JOURNAL, PHASE, totals
from .common import dump, states


CASES = [
    ('parking_sanity', 'parking', 2, 3, 2, 1),
    ('row_to_column__g4x4_c2_r1', 'row_to_column', 4, 4, 2, 1),
    ('regular_parking__g3x3_c3_r4', 'parking', 3, 3, 3, 4),
    ('shared_gate__g3x3_c3_r1', 'shared_gate', 3, 3, 3, 1),
    ('two_shared_gates__g4x4_c4_r1', 'two_shared_gates', 4, 4, 4, 1),
]


def campaign(output, timeout=120, bound=100, only=None, historical=None):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    history = {}
    if historical:
        for line in Path(historical).read_text().splitlines():
            record = json.loads(line)
            history.setdefault(record['instance'], {})[record['method']] = {
                k: record.get(k) for k in ('status', 'wall_seconds',
                    'model_checking_seconds', 'abstract_model_checking_seconds',
                    'timeout_seconds', 'execution_environment')}
    results = []
    saved = {key: os.environ.get(key) for key in (JOURNAL, PHASE)}
    try:
        for name, family, rows, columns, cars, rounds in CASES:
            if only and name not in only:
                continue
            config = scenario(family, rows, columns, cars, rounds)
            case_dir = output/name
            case_dir.mkdir()
            dump(case_dir/'config.json', config)
            dump(case_dir/'historical.json', history.get(name, {}))
            for method in ('endpoint', 'queues'):
                run_dir = case_dir/method
                journal = case_dir/f'{method}_timing.jsonl'
                os.environ[JOURNAL] = str(journal)
                os.environ[PHASE] = 'abstract'
                started = time.monotonic()
                if method == 'endpoint':
                    result = abstract.run(config, run_dir, bound, timeout,
                                          gif=False, visualize=False)
                else:
                    result = queue_abstract.run(config, run_dir, bound, timeout)
                elapsed = time.monotonic()-started
                if result['status'] == 'witness' and 'states' not in result:
                    result['states'] = len(states(run_dir/'trace.xml'))
                timing = totals(journal, time.monotonic())
                row = dict(instance=name, method=method, result=result,
                           wall_seconds=elapsed, timings_seconds=timing,
                           directory=str(run_dir), bound=bound, timeout=timeout,
                           rows=rows, columns=columns, cars=cars, rounds=rounds)
                results.append(row)
                dump(output/'results.json', results)
    finally:
        for key, value in saved.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
    lines = ['# Ordered-queue abstraction pilot', '',
             'Fresh runs below measure abstract search only. They are NOT concrete witnesses.',
             'The endpoint and queue abstractions have different transition granularity;',
             'equal BMC bounds do not cover equal physical journey lengths.', '',
             '| Case | Abstraction | Status | Checking (s) | Wall (s) | States |',
             '|---|---|---|---:|---:|---:|']
    for row in results:
        r = row['result']
        lines.append(f"| {row['instance']} | {row['method']} | {r['status']} | "
                     f"{row['timings_seconds'].get('model_checking', 0):.3f} | "
                     f"{row['wall_seconds']:.3f} | {r.get('states', '—')} |")
    lines += ['', '## Historical selection context', '',
              'These are earlier cluster runs of the named scenario families, not reruns.',
              'The current scenario builder may differ from that historical code snapshot.', '',
              '| Case | Sequential wall (s) | Direct | Enumeration |',
              '|---|---:|---|---|']
    for name in dict.fromkeys(r['instance'] for r in results):
        h = history.get(name, {})
        seq = h.get('abstract_sequential', {})
        def entry(method):
            r = h.get(method, {})
            return f"{r['status']} ({r['wall_seconds']:.1f}s)" if r else '—'
        seq_time = f"{seq['wall_seconds']:.3f}" if seq else '—'
        lines.append(f"| {name} | {seq_time} | {entry('direct_symbolic')} | {entry('enumerated_concrete')} |")
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--bound', type=int, default=100)
    parser.add_argument('--only', nargs='+', choices=[c[0] for c in CASES])
    parser.add_argument('--historical', type=Path, default=Path('cluster_results/results.jsonl'))
    args = parser.parse_args()
    campaign(args.output_dir, args.timeout, args.bound, args.only, args.historical)


if __name__ == '__main__':
    main()
