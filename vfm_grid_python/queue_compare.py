"""Equal overall time budgets for queue+monitor versus concrete enumeration.

Checkpoint after each method. No sequential queue refinement is implemented.
No historical results are mixed with these measurements.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time
import tomllib

from .common import dump
from .benchmark_timing import JOURNAL,PHASE,DEADLINE,totals
from .queue_manoeuvres import configuration


def symbolic(config,directory,settings,timeout,endpoint=False):
    from .queue_abstract import run
    from .queue_monitor import run as refine
    if endpoint:
        from .abstract import run
        from .concretize import run as refine
    saved={k:os.environ.get(k) for k in (JOURNAL,PHASE,DEADLINE)}
    start=time.monotonic()
    journal=directory/'abstract_timing.jsonl'
    os.environ[JOURNAL],os.environ[PHASE],os.environ[DEADLINE]=str(journal),'abstract',str(start+timeout)
    result={}
    try:
        options=dict(gif=False,visualize=False) if endpoint else {}
        a=run(config,directory/'abstract',settings['abstract_bound'],timeout,**options)
        result['abstract']=a
        result['abstract_timings_seconds']=totals(journal,time.monotonic())
        if a['status']!='witness':
            result['status']='abstract_'+a['status']
        elif time.monotonic()-start>=timeout:
            result['status']='timeout'
        else:
            os.environ[JOURNAL],os.environ[PHASE]=str(directory/'concrete_timing.jsonl'),'concrete'
            c=refine(directory/'abstract',directory/'concrete',settings['concrete_bound'],timeout,gif=False)
            if endpoint:
                c['timings_seconds']=totals(directory/'concrete_timing.jsonl',time.monotonic())
            result['concrete']=c
            result['status']='witness' if c['status']=='witness' else 'refinement_'+c['status']
            result['trace']=c.get('trace')
    except Exception as exc:
        result.update(status='error',error=str(exc))
        # Preserve refinement timings/status even if validation/export raised.
        concrete_result=directory/'concrete'/'result.json'
        if concrete_result.exists():
            result['concrete']=json.loads(concrete_result.read_text())
            result['status']='refinement_'+result['concrete']['status']
            if endpoint:
                result['concrete']['timings_seconds']=totals(directory/'concrete_timing.jsonl',time.monotonic())
    finally:
        result['wall_seconds']=time.monotonic()-start
        result['model_checking_seconds']=result.get('abstract_timings_seconds',{}).get('model_checking',0)+result.get('concrete',{}).get('timings_seconds',{}).get('model_checking',0)
        dump(directory/'result.json',result)
        for k,v in saved.items():
            if v is None:os.environ.pop(k,None)
            else:os.environ[k]=v
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--catalog',type=Path,default=Path('parking_benchmark_scenarios/queues/manoeuvres.toml'))
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--only',nargs='+')
    p.add_argument('--method',nargs='+',choices=('abstract_monitor','endpoint_monitor','enumerated_concrete','endpoint_enumerated','direct_symbolic'),default=['abstract_monitor','enumerated_concrete'])
    p.add_argument('--timeout',type=float,default=1800)
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--max-topologies',type=int,default=100)
    p.add_argument('--component-prefilter',action='store_true')
    args=p.parse_args()
    settings=tomllib.loads(args.catalog.read_text())
    cases=[(f"{f['id']}__{s['id']}",f,s) for f in settings['family'] for s in settings['scale']]
    if args.only and set(args.only)-{x[0] for x in cases}:p.error('unknown case')
    if args.timeout<=0 or args.max_topologies<1:p.error('positive timeout/topology limit required')
    root=args.output_dir.resolve();root.mkdir(parents=True,exist_ok=False)
    results=[]
    for name,f,s in cases:
        if args.only and name not in args.only:continue
        config=configuration(f['id'],s,settings)
        case_dir=root/name;case_dir.mkdir()
        dump(case_dir/'config.json',config)
        digest=hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest()
        for method in args.method:
            directory=case_dir/method
            if method in ('abstract_monitor','endpoint_monitor'):
                directory.mkdir()
                r=symbolic(config,directory,settings,args.timeout,endpoint=method=='endpoint_monitor')
            elif method in ('endpoint_enumerated','direct_symbolic'):
                from .benchmark import run_job
                case=dict(id=name,family=f['id'],config=config,repetitions=s.get('rounds',1),
                          abstract_bound=settings['abstract_bound'],concrete_bound=settings['concrete_bound'])
                r=run_job(dict(case=case,method='enumerated_concrete' if method=='endpoint_enumerated' else method,
                              directory=str(directory),timeout=args.timeout,seed=args.seed,parallel=1))
            else:
                from .queue_enumeration import run
                try:
                    r=run(config,directory,settings['concrete_bound'],args.timeout,args.seed,args.max_topologies,args.component_prefilter)
                except Exception as exc:
                    saved=directory/'result.json'
                    r=json.loads(saved.read_text()) if saved.exists() else dict(status='error',error=str(exc))
                r['model_checking_seconds']=r.get('timings_seconds',{}).get('model_checking',0)
            row={**r, 'instance':name,'method':method,'config_sha256':digest,'seed':args.seed,
                 'timeout':args.timeout,'abstract_bound':settings['abstract_bound'],
                 'concrete_bound':settings['concrete_bound'],'directory':str(directory)}
            results.append(row)
            dump(root/'results.json',results)
            print(f"{name}/{method}: {r['status']}",flush=True)
    lines=['# Queue monitor / concrete enumeration','',
           f'Enumeration connectivity prefilter: {args.component_prefilter}.', '',
           'Checking time sums all solver calls; wall time also includes generation, export and validation.',
           'Timeouts and bounded failures are not unsatisfiability proofs. Each approach has the same overall budget.', '',
           '| Case | Method | Status | Checking s | Wall s | Topologies checked |',
           '|---|---|---|---:|---:|---:|']
    for r in results:
        lines.append(f"| {r['instance']} | {r['method']} | {r['status']} | {r.get('model_checking_seconds',0):.3f} | {r.get('wall_seconds',0):.3f} | {r.get('topologies_checked','—')} |")
    (root/'REPORT.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':main()
