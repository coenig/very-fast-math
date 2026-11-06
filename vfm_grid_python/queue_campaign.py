"""Run isolated queue comparison attempts and checkpoint a combined report.

Each subprocess runs one scenario/method with its own overall deadline.
No environment-variable timing state is shared between concurrent attempts.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import tomllib
import uuid

from .common import ROOT
from .queue_manoeuvres import configuration

METHODS = ('abstract_monitor', 'enumerated_concrete')


def write_json(path, data):
    temporary = path.with_suffix(path.suffix + '.' + uuid.uuid4().hex + '.tmp')
    temporary.write_text(json.dumps(data, indent=2) + '\n')
    temporary.replace(path)


def jobs(settings, methods):
    # Small fleets first, with both approaches for each property.
    return [dict(instance=f"{f['id']}__{s['id']}", method=m,
                 description=f['description'], config=configuration(f['id'], s, settings))
            for s in settings['scale'] for f in settings['family'] for m in methods]


def comparison_metric(row):
    """Requested metric; never invent enumeration time for older results."""
    if row['status'] in ('pending', 'running', 'driver_error'):
        return None
    checking = row.get('model_checking_seconds')
    if checking is None:
        return None
    if row['method'] in ('abstract_monitor','endpoint_monitor','direct_symbolic'):
        return checking
    enumeration = row.get('enumeration_seconds', row.get('timings_seconds',{}).get('enumeration'))
    return None if enumeration is None else checking + enumeration


def comparison_report(root, rows):
    paper=any(r['method']=='endpoint_monitor' for r in rows)
    lines = ['# Search-time comparison', '',
        'Seconds. Abstraction → monitor: abstract checking + concrete checking.',
        'Enumeration → concrete: topology enumeration + all concrete checking calls.',
        'Both exclude SMV generation, solver setup, export, validation and rendering.',
        'The campaign timeout remains a wall-time budget, not a limit on this metric.',
        'An inequality marks older runs without separately recorded enumeration time;',
        'it is a lower bound (checking only), not a measured total.', '',
        '| Scenario | Queue → monitor | Previous endpoint → monitor |' if paper else '| Scenario | Abstraction → monitor | Enumeration → concrete |',
        '|---|---:|---:|']
    def cell(row):
        if row is None:
            return '—'
        status=row['status']
        if status in ('pending','running','interrupted','not_run'):
            return status
        metric=comparison_metric(row)
        if metric is None:
            checking=row.get('model_checking_seconds')
            text=f'≥ {checking:.3f} (enumeration unmeasured)' if checking is not None else '—'
        else:
            text=f'{metric:.3f}'
        return text if status=='witness' else f'{status}; spent {text}'
    for name in dict.fromkeys(r['instance'] for r in rows):
        group={r['method']:r for r in rows if r['instance']==name}
        lines.append(f"| {name} | {cell(group.get('abstract_monitor'))} | {cell(group.get('endpoint_monitor' if paper else 'enumerated_concrete'))} |")
    if paper:
        lines[2:4]=['Both columns sum abstract and concrete model-checking time.',
                    'Matched paper properties; previous and queue pipelines have different state/route restrictions.']
    if any(r['method']=='direct_symbolic' for r in rows):
        methods=('endpoint_monitor','abstract_monitor','endpoint_enumerated','direct_symbolic')
        lines=['# Four-pipeline checking-time comparison','',
               'Summed model-checking seconds only; generation, enumeration, validation and rendering excluded.',
               'Timeout is still an overall wall-time limit. Pipelines have different admissible behaviours.', '',
               '| Scenario | Previous monitor | Queue monitor | Concrete enumeration | Direct symbolic |',
               '|---|---:|---:|---:|---:|']
        for name in dict.fromkeys(r['instance'] for r in rows):
            group={r['method']:dict(r,method='direct_symbolic') for r in rows if r['instance']==name}
            lines.append('| '+name+' | '+' | '.join(cell(group.get(m)) for m in methods)+' |')
    (root/'COMPARISON.md').write_text('\n'.join(lines)+'\n')
    write_json(root/'comparison.json', [dict(instance=r['instance'],method=r['method'],
        status=r['status'],search_seconds=comparison_metric(r),
        checking_seconds=r.get('model_checking_seconds'),
        enumeration_seconds=r.get('enumeration_seconds',r.get('timings_seconds',{}).get('enumeration')))
        for r in rows])


def collect(root):
    manifest = json.loads((root/'campaign.json').read_text())
    rows = []
    for job in manifest['jobs']:
        folder = root/'attempts'/job['instance']/job['method']
        saved = folder/'results.json'
        terminal = folder/'driver_result.json'
        result = (
            json.loads(terminal.read_text()) if terminal.exists() else
            dict(status=('interrupted' if folder.exists() else 'not_run')
                 if (root/'STOPPED.json').exists() else
                 ('running' if folder.exists() else 'pending')))
        if saved.exists():
            try:
                result = json.loads(saved.read_text())[0]
            except (json.JSONDecodeError, IndexError):
                pass  # A worker may still be writing its first checkpoint.
        config = job['config']
        abstract = result.get('abstract_timings_seconds', {})
        concrete = result.get('concrete', {}).get('timings_seconds', {})
        direct = result.get('timings_seconds', {})
        row = dict(result, instance=job['instance'], method=job['method'],
                   grid=[config['rows'], config['columns']], cars=config['cars'],
                   stages=len(config['scenario_stages']),
                   section_length=config['section_length'],
                   abstract_checking_seconds=abstract.get('model_checking', 0),
                   concrete_checking_seconds=concrete.get('model_checking', direct.get('model_checking', 0)),
                   generation_seconds=abstract.get('generation', 0)+concrete.get('generation', 0)+direct.get('generation', 0))
        rows.append(row)
    write_json(root/'results.json', rows)
    lines = ['# Queue congestion scaling', '',
        f"Budget: {manifest['timeout']} seconds per instance/approach, including generation and validation.",
        f"Bounds: abstract {manifest['abstract_bound']}; concrete {manifest['concrete_bound']}.",
        f"Concurrent attempts: {manifest['parallel']}. Enumeration seed: {manifest['seed']}.",
        '', 'Timeouts are not unsatisfiability proofs. Pending/running rows are not completed measurements.',
        'Checking time is summed solver time, not rendering time. No GIF rendering occurs during these runs.', '',
        '| Scenario | Grid | Cars | Approach | Status | Abstract check s | Concrete check s | Generation s | Wall s |',
        '|---|---|---:|---|---|---:|---:|---:|---:|']
    for r in rows:
        def value(key):
            return f"{r.get(key, 0):.2f}" if r['status'] not in ('pending','running','interrupted','not_run') else '—'
        lines.append(f"| {r['instance']} | {r['grid'][0]}×{r['grid'][1]} | {r['cars']} | {r['method']} | {r['status']} | "
                     + ' | '.join(value(k) for k in ('abstract_checking_seconds','concrete_checking_seconds','generation_seconds','wall_seconds')) + ' |')
    (root/'REPORT.md').write_text('\n'.join(lines)+'\n')
    comparison_report(root, rows)
    return rows


def attempt(root, job, manifest):
    folder = root/'attempts'/job['instance']/job['method']
    folder.parent.mkdir(parents=True, exist_ok=True)
    log = root/'logs'/f"{job['instance']}__{job['method']}.log"
    cmd = [sys.executable, '-m', 'vfm_grid_python.queue_compare',
           '--catalog', str(root/'catalog.toml'), '--output-dir', str(folder),
           '--only', job['instance'], '--method', job['method'],
           '--timeout', str(manifest['timeout']), '--seed', str(manifest['seed']),
           '--max-topologies', str(manifest['max_topologies'])]
    env = dict(os.environ)
    # Do not inherit a previous benchmark's deadline or timing journal.
    for key in ('VFM_BENCHMARK_DEADLINE','VFM_BENCHMARK_JOURNAL','VFM_BENCHMARK_PHASE'):
        env.pop(key, None)
    env.setdefault('MPLCONFIGDIR', '/tmp/vfm-queue-mpl')
    started = time.monotonic()
    print(f"START {job['instance']}/{job['method']}", flush=True)
    with log.open('w') as stream:
        process = subprocess.run(cmd, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT)
    if not (folder/'results.json').exists():
        folder.mkdir(parents=True, exist_ok=True)
        write_json(folder/'driver_result.json', dict(status='driver_error', exit_code=process.returncode,
                   wall_seconds=time.monotonic()-started, log=str(log)))
    saved = folder/'results.json'
    status = json.loads(saved.read_text())[0]['status'] if saved.exists() else 'driver_error'
    print(f"DONE {job['instance']}/{job['method']}: {status} ({time.monotonic()-started:.2f}s)", flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--catalog', type=Path, default=Path('parking_benchmark_scenarios/queues/congestion_scaling.toml'))
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--timeout', type=float)
    p.add_argument('--parallel', type=int, default=2)
    p.add_argument('--method', nargs='+', choices=METHODS+('endpoint_monitor','endpoint_enumerated','direct_symbolic'), default=list(METHODS))
    p.add_argument('--only', nargs='+')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--max-topologies', type=int, default=1000000)
    p.add_argument('--preview', action='store_true')
    p.add_argument('--report-only', action='store_true')
    args = p.parse_args()
    root = args.output_dir.resolve()
    if args.report_only:
        rows = collect(root)
        print(f"{sum(r['status'] not in ('running','pending','interrupted','not_run') for r in rows)}/{len(rows)} completed; {root/'REPORT.md'}")
        return
    settings = tomllib.loads(args.catalog.read_text())
    selected = jobs(settings, args.method)
    if args.only:
        if set(args.only)-{j['instance'] for j in selected}:
            p.error('unknown --only instance')
        selected = [j for j in selected if j['instance'] in args.only]
    timeout = args.timeout if args.timeout is not None else settings.get('timeout', 1200)
    if timeout <= 0 or args.parallel < 1 or args.max_topologies < 1:
        p.error('positive timeout, parallelism and topology limit required')
    if args.preview:
        print(json.dumps(dict(attempts=len(selected), timeout=timeout, parallel=args.parallel,
                              instances=list(dict.fromkeys(j['instance'] for j in selected))), indent=2))
        return
    root.mkdir(parents=True, exist_ok=False)
    (root/'logs').mkdir()
    (root/'catalog.toml').write_text(args.catalog.read_text())
    manifest = dict(created_at=datetime.now(timezone.utc).isoformat(), timeout=timeout,
                    parallel=args.parallel, seed=args.seed, max_topologies=args.max_topologies,
                    abstract_bound=settings['abstract_bound'], concrete_bound=settings['concrete_bound'],
                    python=sys.executable, cpu_affinity=sorted(os.sched_getaffinity(0)), jobs=selected)
    manifest['catalog_sha256'] = hashlib.sha256((root/'catalog.toml').read_bytes()).hexdigest()
    write_json(root/'campaign.json', manifest)
    collect(root)
    with ThreadPoolExecutor(max_workers=args.parallel) as pool:
        futures = [pool.submit(attempt, root, job, manifest) for job in selected]
        for future in as_completed(futures):
            future.result()
            collect(root)
    print(f"Results: {root/'REPORT.md'}", flush=True)


if __name__ == '__main__':
    main()
