"""Matched queue-encoding experiments: topology remains symbolic."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import time

from .benchmark_scenarios import scenario
from .benchmark_timing import JOURNAL, PHASE, DEADLINE, totals
from .common import dump
from .queue_abstract import run

CASES={
    'gate2': ('shared_gate',2,3,2,1),
    'gate3': ('shared_gate',3,3,3,1),
    'rotation3': ('parking_rotation',3,3,3,1),
    'two_gates4': ('two_shared_gates',4,4,4,2),
    'gather4': ('gather_park',3,3,4,1),
    'merge4': ('merge_park',3,3,4,1),
}


def case_config(name):
    family, rows, columns, cars, rounds = CASES[name]
    if family in ('gather_park', 'merge_park'):
        from .queue_manoeuvres import configuration
        return configuration(family, dict(grid=[rows, columns], cars=cars,
            edge_budget=rows*columns-1), {})
    base=scenario(family,rows,columns,cars,rounds)
    base.update(section_length=20,parking_length=7,junction_radius=6,connection_length=9,queue_clearance=6)
    return base


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-dir',required=True,type=Path)
    p.add_argument('--only',nargs='+',choices=list(CASES),default=['gate2','gate3','two_gates4'])
    p.add_argument('--encoding',nargs='+',choices=['explicit','joint','grouped'],default=['explicit','joint','grouped'])
    p.add_argument('--timeout',type=float,default=60)
    p.add_argument('--bound',type=int,default=100)
    p.add_argument('--backend', choices=['smt','sat'], default='smt')
    p.add_argument('--capacity-encoding', choices=['arithmetic','threshold'], default='arithmetic')
    args=p.parse_args()
    if args.timeout <= 0 or args.bound < 0:
        p.error('timeout must be positive and bound must be non-negative')
    root=args.output_dir.resolve();root.mkdir(parents=True,exist_ok=False)
    dump(root/'experiment.json',dict(vars(args),output_dir=str(root)))
    results=[]
    saved={k:os.environ.get(k) for k in (JOURNAL,PHASE,DEADLINE)}
    try:
        for name in args.only:
            base=case_config(name)
            digest=hashlib.sha256(json.dumps(base,sort_keys=True).encode()).hexdigest()
            for encoding in args.encoding:
                print(f'{name}/{encoding}/{args.backend}: starting', flush=True)
                folder=root/name/encoding;folder.parent.mkdir(parents=True,exist_ok=True)
                journal=folder.parent/(encoding+'_timing.jsonl')
                start=time.monotonic()
                os.environ[JOURNAL],os.environ[PHASE],os.environ[DEADLINE]=str(journal),'abstract',str(start+args.timeout)
                r=run(dict(base,queue_transfer_encoding=encoding,queue_solver=args.backend,
                    queue_capacity_encoding=args.capacity_encoding),folder,args.bound,args.timeout)
                log=(folder/'checking.log').read_text()
                bounds=re.findall(r'no (?:proof or )?counterexample found with bound (\d+)',log)
                results.append(dict(r,case=name,encoding=encoding,base_config_sha256=digest,
                    capacity_encoding=args.capacity_encoding,
                    timeout=args.timeout,last_unsat_bound=int(bounds[-1]) if bounds else None,
                    timings=totals(journal,time.monotonic())))
                dump(root/'results.json',results)
                lines=['# Symbolic queue encoding comparison','',
                    'Identical properties, topology domains and queue rules; equivalent encodings/backends differ.',
                    f'Backend: {args.backend}; capacity encoding: {args.capacity_encoding}; bound: {args.bound}; budget: {args.timeout:g}s.',
                    'Times are abstract solver time. Timeout bounds indicate progress, not a proof of impossibility.', '',
                    '| Case | Encoding | Backend | Status | Checking s | Model bytes | States | Last UNSAT bound |',
                    '|---|---|---|---|---:|---:|---:|---:|']
                for x in results:
                    lines.append(f"| {x['case']} | {x['encoding']} | {x['solver_backend']} | {x['status']} | {x['timings'].get('model_checking',0):.3f} | {x['model_bytes']} | {x.get('states','—')} | {x['last_unsat_bound']} |")
                (root/'REPORT.md').write_text('\n'.join(lines)+'\n')
    finally:
        for k,v in saved.items():
            if v is None:os.environ.pop(k,None)
            else:os.environ[k]=v


if __name__=='__main__':main()
