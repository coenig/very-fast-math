"""Checkpointed four-way endpoint-grid benchmarks; rendering is excluded."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import tomllib

from .benchmark_scenarios import scenario
from .benchmark_timing import DEADLINE, JOURNAL, PHASE, totals
from .common import ROOT, dump

METHODS = ("enumerated_concrete", "direct_symbolic", "abstract_sequential", "abstract_monitor")
AVAILABLE = METHODS
SCHEMA = "endpoint-benchmark.v1"


def catalog(path):
    raw = tomllib.loads(Path(path).read_text())
    if 'adaptive' in raw:
        from .adaptive_benchmark import expand
        return expand(raw)[0]
    result, identifiers = [], set()
    for entry in raw.get("scenario", []):
        variants = [dict(entry, variant_id="base")]
        variants += [dict(entry, **{k:v for k,v in variant.items() if k != "id"},
                          variant_id=variant["id"]) for variant in entry.get("variant", [])]
        for variant in variants:
            identifier = entry["id"] + "__" + variant["variant_id"]
            if not re.fullmatch(r"[a-zA-Z0-9_-]+", identifier):
                raise ValueError(f"unsafe scenario id: {identifier}")
            if identifier in identifiers:
                raise ValueError(f"duplicate scenario: {identifier}")
            identifiers.add(identifier)
            rows, columns = variant["grid"]
            config = scenario(variant["family"], rows, columns, variant["cars"],
                              variant.get("repetitions",1), variant.get("parkings"),
                              parking_span=variant.get('parking_span'))
            config['semantics']=variant.get('semantics',raw.get('semantics','legacy'))
            from .braking import with_speed
            config=with_speed(config,variant.get('max_speed',raw.get('max_speed')))
            from .stable import enabled,profile
            if enabled(config): profile(config)
            result.append(dict(id=identifier, family=variant["family"], config=config,
                               repetitions=variant.get("repetitions",1),
                               abstract_bound=variant.get("abstract_bound",raw.get("abstract_bound",100)),
                               concrete_bound=variant.get("concrete_bound",raw.get("concrete_bound",200))))
    if not result:
        raise ValueError("catalog has no scenarios")
    return result


def worker(job):
    output = Path(job["directory"])
    config = job["case"]["config"]
    timeout = job["timeout"]
    if job["method"] in ("abstract_sequential", "abstract_monitor"):
        from . import abstract, sequential, concretize
        start = time.monotonic()
        os.environ[PHASE]='abstract'
        result = abstract.run(config, output/"abstract", job["case"]["abstract_bound"],
                              timeout, gif=False, visualize=False)
        abstract_seconds = time.monotonic()-start
        dump(output/"abstract_result.json", dict(result, witness_seconds=abstract_seconds
                                                 if result["status"]=="witness" else None))
        if result["status"] != "witness":
            return dict(status="abstract_"+result["status"])
        os.environ[PHASE]='concrete'
        if job['method'] == 'abstract_monitor':
            result = concretize.run(output/'abstract', output/'concrete',
                                    bound=job['case']['concrete_bound'], timeout=timeout,
                                    gif=False)
            if result['status'] != 'witness':
                result = dict(result, status='refinement_'+result['status'])
            return dict(result, abstract_witness_seconds=abstract_seconds)
        concrete_started=time.monotonic()
        result = sequential.run(output/"abstract", output/"concrete",
                                bound=job["case"]["concrete_bound"], timeout=timeout,
                                gif=False, backtracks=0)
        if result["status"] == "witness":
            if not (output/"concrete/model_validation.json").exists():
                raise ValueError("sequential backend did not produce full-model validation")
            return dict(status="witness", trace=result["trace"], validated=True,
                        states=result["states"], abstract_witness_seconds=abstract_seconds,
                        validated_at_monotonic=concrete_started+result['time_to_trace_seconds'])
        return dict(status="refinement_"+result["status"], error=result.get("error"),
                    abstract_witness_seconds=abstract_seconds)
    if job["method"] == "enumerated_concrete":
        from .benchmark_symbolic import run
        from .benchmark_oracle import layouts
        for index, graph in enumerate(layouts(config["rows"],config["columns"],config["parkings"],job["seed"])):
            directory = output/f"topology_{index:06d}"
            result = run(config, directory, job["case"]["concrete_bound"], timeout, fixed=graph)
            record = dict(index=index, graph=graph, **result)
            with (output/"topologies.jsonl").open("a") as stream:
                stream.write(json.dumps(record)+"\n")
            if result["status"] == "witness":
                return dict(result, topologies_checked=index+1)
            if result["status"] != "no_witness_within_bound":
                return dict(result, topologies_checked=index+1)
        return dict(status="all_layouts_exhausted_within_bound")
    if job['method']=='direct_symbolic':
        from .benchmark_symbolic import run
        return run(config,output/'concrete',job['case']['concrete_bound'],timeout)
    raise ValueError(f"unknown method: {job['method']}")


def kill_tree(pid):
    """Generators/checkers start new sessions; kill descendants as well."""
    stopped=set()
    pending,ordered=[pid],[]
    while pending:
        for parent in pending:
            try: os.kill(parent,signal.SIGSTOP)
            except ProcessLookupError: pass
            stopped.add(parent); ordered.append(parent)
        pending=[]
        for path in Path('/proc').glob('[0-9]*/stat'):
            try:
                fields=path.read_text().rsplit(')',1)[1].split()
                child,ppid=int(path.parent.name),int(fields[1])
                if ppid in stopped and child not in stopped: pending.append(child)
            except (OSError,ValueError,IndexError): pass
    for child in reversed(ordered):
        try:
            os.kill(child, signal.SIGKILL)
        except ProcessLookupError:
            pass


def run_job(job):
    output = Path(job["directory"])
    output.mkdir(parents=True, exist_ok=False)
    dump(output/"job.json", job)
    start = time.monotonic()
    env = dict(os.environ, **{JOURNAL:str(output/"timing.jsonl"), DEADLINE:str(start+job["timeout"])})
    with (output/"worker.log").open("w") as log:
        process = subprocess.Popen([sys.executable,"-m","vfm_grid_python.benchmark","--worker",str(output/"job.json")],
                                   cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            process.wait(timeout=job["timeout"])
            result_path = output/"worker_result.json"
            result = json.loads(result_path.read_text()) if result_path.exists() else dict(status="worker_error")
        except subprocess.TimeoutExpired:
            kill_tree(process.pid)
            process.wait()
            result = dict(status="timeout")
        except BaseException:
            kill_tree(process.pid)
            process.wait()
            raise
    stop = time.monotonic()
    timing = totals(output/"timing.jsonl", stop)
    abstract_result = output/"abstract_result.json"
    if abstract_result.exists():
        result.setdefault("abstract_witness_seconds", json.loads(abstract_result.read_text()).get("witness_seconds"))
    result.update(schema=SCHEMA, instance=job["case"]["id"], method=job["method"],
                  semantics=job['case']['config'].get('semantics','legacy'),
                  contract_version=('stable-concurrent.v1' if job['case']['config'].get('semantics')=='stable_concurrent' else 'legacy'),
                  directory=str(output), seed=job["seed"], parallel=job["parallel"],
                  family=job["case"]["family"], rows=job["case"]["config"]["rows"],
                  columns=job["case"]["config"]["columns"], cars=job["case"]["config"]["cars"],
                  repetitions=job["case"]["repetitions"], timeout_seconds=job["timeout"],
                  wall_seconds=stop-start, timings_seconds=timing,
                  model_checking_seconds=timing.get("model_checking",0.),
                  abstract_model_checking_seconds=totals(output/'timing.jsonl',stop,'abstract').get('model_checking',0.),
                  concrete_model_checking_seconds=totals(output/'timing.jsonl',stop,'concrete').get('model_checking',0.),
                  generation_seconds=timing.get("generation",0.),
                  setup_seconds=timing.get('setup',0.),trace_export_seconds=timing.get('trace_export',0.),
                  validation_seconds=timing.get('validation',0.),preprocessing_seconds=timing.get('preprocessing',0.),
                  time_to_validated_xml_seconds=(result.get('validated_at_monotonic',stop)-start)
                                               if result.get("validated") else None)
    for name in ('stage_count','property_size_chars','scaling_indices','sweep_family','abstract_bound','concrete_bound'):
        if name in job['case']: result[name]=job['case'][name]
    # A returned partial prefix is never a successful benchmark witness.
    if result["status"] == "witness" and not result.get("validated"):
        result["status"] = "unvalidated_witness"
    dump(output/"result.json", result)
    return result


def main():
    if len(sys.argv)==3 and sys.argv[1]=="--worker":
        job=json.loads(Path(sys.argv[2]).read_text())
        try:
            result=worker(job)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            result=dict(status="error",error=str(exc))
        dump(Path(job["directory"])/"worker_result.json",result)
        return 0
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog",type=Path,required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--method",choices=METHODS,nargs="+",default=list(AVAILABLE))
    parser.add_argument("--only",nargs="+")
    parser.add_argument("--timeout",type=float,default=None)
    parser.add_argument("--parallel",type=int,default=1)
    parser.add_argument("--seed",type=int,default=0)
    parser.add_argument("--resume",action="store_true")
    parser.add_argument('--semantics',choices=('legacy','stable_concurrent'),help='override catalog semantics for a new campaign')
    parser.add_argument("--list",action="store_true",help="validate and list cases without running")
    args=parser.parse_args()
    raw_catalog=tomllib.loads(args.catalog.read_text())
    adaptive='adaptive' in raw_catalog
    if args.timeout is None:
        args.timeout=raw_catalog.get('timeout',3600 if adaptive else 1800)
    if args.timeout<=0 or args.parallel<1:
        parser.error("timeout and parallel must be positive")
    cases=catalog(args.catalog)
    if args.semantics:
        for case in cases: case['config']['semantics']=args.semantics
    if args.only:
        unknown=set(args.only)-{case["id"] for case in cases}
        if unknown: parser.error(f"unknown cases: {sorted(unknown)}")
        cases=[case for case in cases if case["id"] in args.only]
    if args.list:
        for case in cases:
            c=case["config"]
            print(f"{case['id']}: {c['rows']}x{c['columns']}, {c['cars']} cars, {case['repetitions']} rounds, {len(c['scenario_stages'])} stages")
        print(f'{len(cases)} cases; up to {len(cases)*len(args.method)} jobs before pruning; {args.timeout:g}s per job')
        return 0
    output=args.output_dir.resolve()
    if output.exists() and not args.resume:
        parser.error("output exists; choose a new directory or use --resume")
    output.mkdir(parents=True,exist_ok=True)
    jobs=[dict(case=case,method=method,timeout=args.timeout,parallel=args.parallel,seed=args.seed,
               directory=str(output/case["id"]/method)) for case in cases for method in args.method]
    manifest=dict(schema=SCHEMA,jobs=jobs,catalog=str(args.catalog.resolve()),
                  revision=subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip(),
                  native_library_sha256=hashlib.sha256((ROOT/"lib/libvfm.so").read_bytes()).hexdigest(),
                  template_sha256=hashlib.sha256((ROOT/"main_template.json").read_bytes()).hexdigest())
    inputs=(list((ROOT/'vfm_grid_python').glob('*.py'))+list((ROOT/'src/templates').rglob('*.tpl'))+
            [ROOT/name for name in ('scenario_generation.py','model_checking.py','fixed_graph_model.py','vfm_grid_parking.py')])
    manifest['source_sha256']={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
                               for path in sorted(inputs)}
    manifest_path=output/"manifest.json"
    if args.resume:
        if not manifest_path.exists() or json.loads(manifest_path.read_text())!=manifest:
            parser.error("resume manifest differs (cases, settings, revision, or build)")
    else: dump(manifest_path,manifest)
    if adaptive:
        from .adaptive_benchmark import expand
        dump(output/'excluded.json',expand(raw_catalog)[1])
    todo=[]; completed=[]
    for job in jobs:
        directory=Path(job["directory"])
        if (directory/"result.json").exists():
            completed.append(json.loads((directory/'result.json').read_text()))
            continue
        if directory.exists():
            parser.error(f"unfinished attempt exists: {directory}; preserve it and use a new output directory")
        directory.parent.mkdir(parents=True,exist_ok=True)
        todo.append(job)
    def save(result):
        with (output/'results.jsonl').open('a') as stream:
            stream.write(json.dumps(result)+'\n')
        print(f"{result['instance']}/{result['method']}: {result['status']} ({result['wall_seconds']:.2f}s)",flush=True)
    if adaptive:
        from .adaptive_benchmark import schedule
        schedule(todo,completed,args.parallel,run_job,save)
        return 0
    with ThreadPoolExecutor(max_workers=args.parallel) as pool:
        for future in as_completed([pool.submit(run_job,job) for job in todo]):
            result=future.result()
            with (output/"results.jsonl").open("a") as stream:
                stream.write(json.dumps(result)+"\n")
            print(f"{result['instance']}/{result['method']}: {result['status']} ({result['wall_seconds']:.2f}s)",flush=True)
    return 0


if __name__=="__main__":
    raise SystemExit(main())
