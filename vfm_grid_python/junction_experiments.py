"""Reproducible shared-junction smoke experiments (native logs stay in run dirs)."""
import argparse
import json
from pathlib import Path
import time

from .common import ROOT, dump
from . import abstract, sequential


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,
                        default=ROOT/'jucntion-extension-exp'/f'run_{time.time_ns()}')
    parser.add_argument('--timeout',type=float,default=300,help='timeout per generation/check call')
    parser.add_argument('--bound',type=int,default=10)
    parser.add_argument('--concrete-bound',type=int,default=120)
    selection=parser.add_mutually_exclusive_group()
    selection.add_argument('--configs',nargs='+',type=Path,
                           help='run arbitrary shared-junction JSON scenarios, one directory per filename')
    selection.add_argument('--only',nargs='+',choices=('independent_turns','arrive_then_turn','crossing_turns',
                        'symbolic_shared_junction','three_cars','four_approaches'))
    parser.add_argument('--abstract-only',action='store_true')
    args=parser.parse_args()
    output=args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f'use a new experiment directory: {output}')
    output.mkdir(parents=True,exist_ok=True)
    results=[]
    cases=args.only or ['independent_turns','arrive_then_turn','crossing_turns','symbolic_shared_junction',
                       'three_cars','four_approaches']
    paths=args.configs or [ROOT/'jucntion-extension-exp/configs'/f'{name}.json' for name in cases]
    if len({p.stem for p in paths})!=len(paths):
        parser.error('scenario filenames must have distinct stems')
    for path in paths:
        name=path.stem
        config=json.loads(path.read_text())
        from .junctions import profile
        dimensions=profile(config)
        if dimensions is None:
            raise ValueError(f'{path}: enable junction_extension for this runner')
        directory=output/name
        record=dict(case=name,directory=str(directory))
        results.append(record)
        try:
            record['abstract']=abstract.run(config,directory/'abstract',args.bound,args.timeout)
            if record['abstract']['status']=='witness' and not args.abstract_only:
                record['concrete']=sequential.run(directory/'abstract',directory/'concrete',
                    bound=args.concrete_bound,timeout=args.timeout,
                    road_length=dimensions['road_length'],parking_length=dimensions['parking_length'],
                    radius=dimensions['junction_radius'])
        except Exception as exc:
            record['error']=f'{type(exc).__name__}: {exc}'
            print(f'{name}: {record["error"]}',flush=True)
        dump(output/'results.json',results)
    print(f'Results: {output}')
    return 0 if all('error' not in r and r.get('concrete',r['abstract'])['status']=='witness'
                    for r in results) else 2


if __name__=='__main__':
    raise SystemExit(main())
