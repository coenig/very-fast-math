"""Local, subprocess-isolated benchmarks for the three queue-compatible approaches."""
from __future__ import annotations
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import tomllib

from .common import ROOT, dump
from .benchmark_timing import JOURNAL, PHASE, DEADLINE, totals

METHODS = {'external_enumeration':'enumerated_concrete',
           'direct_model_checking':'direct_symbolic',
           'abstraction_based':'abstract_monitor'}
SEMANTICS = 'ordered-queues.v1'
COMPILER = 'shared-queue-concrete.v1'


def worker(job):
    from .approaches import external_enumeration, direct_model_checking, abstraction_based
    output=Path(job['directory']); started=time.monotonic()
    journal=Path(job['journal'])
    os.environ[JOURNAL]=str(journal)
    os.environ[PHASE]='concrete'
    os.environ[DEADLINE]=str(started+job['timeout'])
    result={}
    try:
        c=job['config']; approach=job['approach']
        if approach=='external_enumeration':
            result=external_enumeration.run(c,output,job['concrete_bound'],job['timeout'],
                                           job['seed'],job['max_topologies'])
        elif approach=='direct_model_checking':
            result=direct_model_checking.run(c,output,job['concrete_bound'],job['timeout'])
            result['timings_seconds']=totals(journal,time.monotonic())
        else:
            result=abstraction_based.run(c,output,job['abstract_bound'],job['concrete_bound'],job['timeout'])
    except Exception as exc:
        result=dict(status='error',error=str(exc))
    result.setdefault('timings_seconds',totals(journal,time.monotonic()))
    result.setdefault('model_checking_seconds',result['timings_seconds'].get('model_checking',0))
    result.update(instance=job['instance'],approach=job['approach'],method=METHODS[job['approach']],
                  config_sha256=job['config_sha256'],semantics=SEMANTICS,compiler=COMPILER,
                  directory=str(output),timeout=job['timeout'],abstract_bound=job['abstract_bound'],
                  concrete_bound=job['concrete_bound'],wall_seconds=time.monotonic()-started)
    output.mkdir(parents=True,exist_ok=True)
    dump(output/'result.json',result)
    return result


def attempt(job, root):
    key=job['instance']+'__'+job['approach']
    jobfile=root/'jobs'/(key+'.json'); dump(jobfile,job)
    env=dict(os.environ,MPLCONFIGDIR='/tmp/vfm-queue-mpl',MPLBACKEND='Agg')
    for key_env in (JOURNAL,PHASE,DEADLINE): env.pop(key_env,None)
    print('START '+key,flush=True)
    with (root/'logs'/(key+'.log')).open('w') as stream:
        process=subprocess.Popen([sys.executable,'-m','vfm_grid_python.benchmark','--worker',str(jobfile)],
                                 cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            process.wait(timeout=job['timeout']+15)
        except subprocess.TimeoutExpired:
            import psutil
            try:
                parent=psutil.Process(process.pid)
                children=parent.children(recursive=True)
                for child in reversed(children):
                    try: child.kill()
                    except psutil.NoSuchProcess: pass
                parent.kill()
            except psutil.NoSuchProcess: pass
            process.wait()
    resultfile=Path(job['directory'])/'result.json'
    if resultfile.exists():
        result=json.loads(resultfile.read_text())
    else:
        result=dict(instance=job['instance'],approach=job['approach'],method=METHODS[job['approach']],
                    directory=job['directory'],semantics=SEMANTICS,compiler=COMPILER,
                    status='timeout' if process.returncode<0 else 'driver_error')
        resultfile.parent.mkdir(parents=True,exist_ok=True);dump(resultfile,result)
    # A killed worker may leave a backend checkpoint, not a completed attempt.
    # Preserve it for diagnostics but never count it as a validated pipeline run.
    if process.returncode != 0 or 'approach' not in result:
        result.update(instance=job['instance'],approach=job['approach'],method=METHODS[job['approach']],
                      directory=job['directory'],semantics=SEMANTICS,compiler=COMPILER,
                      status='timeout' if process.returncode<0 else 'driver_error')
        dump(resultfile,result)
    print(f"{key}: {result['status']}",flush=True)
    return result


def collect(root):
    rows=[]
    for job in json.loads((root/'campaign.json').read_text())['jobs']:
        path=Path(job['directory'])/'result.json'
        rows.append(json.loads(path.read_text()) if path.exists() else dict(
            instance=job['instance'],approach=job['approach'],method=METHODS[job['approach']],
            directory=job['directory'],status='pending',semantics=SEMANTICS,compiler=COMPILER))
    dump(root/'results.json',rows)
    from .benchmark_report import write_report
    write_report(root,rows)
    return rows


def main():
    os.environ.setdefault('MPLCONFIGDIR','/tmp/vfm-queue-mpl')
    if len(sys.argv)==3 and sys.argv[1]=='--worker':
        worker(json.loads(Path(sys.argv[2]).read_text()));return
    p=argparse.ArgumentParser(description=__doc__)
    source=p.add_mutually_exclusive_group()
    source.add_argument('--config',type=Path)
    source.add_argument('--catalog',type=Path)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--approach',nargs='+',choices=METHODS,default=list(METHODS))
    p.add_argument('--refinement',choices=['monitor','sequential'],default='monitor')
    p.add_argument('--only',nargs='+')
    p.add_argument('--abstract-bound',type=int)
    p.add_argument('--concrete-bound',type=int)
    p.add_argument('--timeout',type=float)
    p.add_argument('--parallel',type=int,default=1)
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--max-topologies',type=int,default=1000000)
    p.add_argument('--no-gif',action='store_true')
    p.add_argument('--preview',action='store_true')
    p.add_argument('--report-only',action='store_true')
    args=p.parse_args()
    if args.refinement!='monitor':p.error('Queue sequentialization is not implemented; use --refinement monitor.')
    root=args.output_dir.resolve()
    if args.report_only:
        collect(root);print(root/'REPORT.md');return
    if not(args.config or args.catalog):p.error('provide --config or --catalog')
    from .scenarios.validation import normalize
    settings={}
    if args.config:
        cases=[(args.config.stem,json.loads(args.config.read_text()))]
    else:
        from .scenarios.catalog import configuration
        settings=tomllib.loads(args.catalog.read_text())
        cases=[(f"{f['id']}__{s['id']}",configuration(f['id'],s,settings))
               for s in settings['scale'] for f in settings['family']]
    if args.only:
        if set(args.only)-{n for n,c in cases}:p.error('unknown --only instance')
        cases=[(n,c) for n,c in cases if n in args.only]
    options=dict(timeout=args.timeout if args.timeout is not None else settings.get('timeout',300),
                 abstract_bound=args.abstract_bound if args.abstract_bound is not None else settings.get('abstract_bound',60),
                 concrete_bound=args.concrete_bound if args.concrete_bound is not None else settings.get('concrete_bound',200),
                 seed=args.seed,max_topologies=args.max_topologies)
    if options['timeout']<=0 or min(options['abstract_bound'],options['concrete_bound'])<0 or args.parallel<1 or args.max_topologies<1:
        p.error('positive timeout/parallelism/topology limit and nonnegative bounds required')
    jobs=[]
    for name,config in cases:
        if Path(name).name!=name or name in ('.','..'):p.error('unsafe instance name')
        c=normalize(config)
        digest=hashlib.sha256(json.dumps(c,sort_keys=True).encode()).hexdigest()
        for approach in dict.fromkeys(args.approach):
            jobs.append(dict(instance=name,config=c,config_sha256=digest,approach=approach,
                             directory=str(root/name/approach),
                             journal=str(root/'logs'/(name+'__'+approach+'.timing.jsonl')),**options))
    if args.preview:
        print(json.dumps(dict(attempts=len(jobs),instances=[n for n,c in cases],**options),indent=2));return
    root.mkdir(parents=True,exist_ok=False)
    (root/'logs').mkdir();(root/'jobs').mkdir()
    dump(root/'campaign.json',dict(jobs=jobs,semantics=SEMANTICS,compiler=COMPILER,parallel=args.parallel))
    for name,c in cases:
        (root/name).mkdir();dump(root/name/'config.json',normalize(c))
    collect(root)
    with ThreadPoolExecutor(max_workers=args.parallel) as pool:
        tasks=[pool.submit(attempt,job,root) for job in jobs]
        for task in as_completed(tasks):
            task.result();collect(root)
    if not args.no_gif:
        from .queue_render import render_completed
        render_completed(root)
    print('Results: '+str(root/'REPORT.md'),flush=True)


if __name__=='__main__':
    main()
