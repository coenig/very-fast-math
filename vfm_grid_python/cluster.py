"""DICLUB/benchtools adapter. Preparation and collection never submit jobs."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import getpass
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
PARTITIONS = 'identical-9,identical-10'


def read(path):
    return json.loads(Path(path).read_text())


def atomic(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_name(path.name+'.pending')
    pending.write_text(json.dumps(value, indent=2)+'\n')
    pending.replace(path)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


@contextmanager
def locked(root):
    with (Path(root)/'.controller.lock').open('a') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield


def snapshot_sources(root, output):
    """Actual working contents, including uncommitted Python/template changes."""
    tracked = subprocess.check_output(['git','ls-files','-z'],cwd=root).decode().split('\0')
    paths = {Path(p) for p in tracked if p and not p.startswith(('.git/','build/'))}
    for directory in ('vfm_grid_python','benchtools','src/templates','parking_benchmark_scenarios'):
        paths.update(p.relative_to(root) for p in (root/directory).rglob('*') if p.is_file())
    paths.update(p.relative_to(root) for p in root.glob('*.py'))
    paths.update(Path(p) for p in ('main_template.json','requirements.txt') if (root/p).is_file())
    hashes = {}
    with tarfile.open(output,'w:gz',dereference=True) as archive:
        for rel in sorted(paths):
            source = root/rel
            if not source.is_file() or '__pycache__' in rel.parts or '.git' in rel.parts:
                continue
            archive.add(source,arcname=str(rel),recursive=False)
            hashes[str(rel)] = digest(source)
    return hashes


def migrate(manifest, source):
    pending, imported, uncertain = [], [], []
    for original in manifest['jobs']:
        case, method = original['case']['id'], original['method']
        if not re.fullmatch(r'[A-Za-z0-9_-]+',case) or not re.fullmatch(r'[a-z_]+',method):
            raise ValueError('unsafe case or method in source manifest')
        result_path = source/case/method/'result.json'
        result = None
        if result_path.exists():
            # Fail closed on a concurrently written/corrupt result; retry prepare later.
            result = read(result_path)
        if result and not result['status'].startswith('pruned_'):
            imported.append(dict(result, execution_environment='local:'+str(source),
                                 imported_from=str(result_path)))
            continue
        if (source/case/method).exists() and result is None:
            uncertain.append(f'{case}/{method}')
        job = dict(original, timeout=3600, parallel=1)
        job.pop('directory',None)
        pending.append(job)
    return pending, imported, uncertain


def prepare(args):
    source, output = args.source.resolve(), args.output.resolve()
    if output.exists(): raise FileExistsError('choose a new migration output directory')
    shared = Path(args.shared_root)
    if not shared.is_absolute() or not Path(args.python).is_absolute():
        raise ValueError('shared-root and python must be absolute cluster paths')
    if any(ch.isspace() for ch in str(shared)+args.python):
        raise ValueError('benchtools deployment paths must not contain whitespace')
    manifest = read(source/'manifest.json')
    jobs, imported, uncertain = migrate(manifest,source)
    output.mkdir(parents=True)
    campaign_id = 'vfm-'+uuid.uuid4().hex[:12]
    config = dict(schema='vfm-cluster.v1', campaign_id=campaign_id, shared_root=str(shared),
                  python=args.python, partitions=PARTITIONS, cpus=4, memory_mib=32768,
                  workload_seconds=3600, wrapper_seconds=4500, slurm_minutes=80,
                  source_campaign=str(source), prepared_at=datetime.now(timezone.utc).isoformat(),
                  source_manifest=manifest, imported_count=len(imported), jobs=[])
    atomic(output/'imported_results.json',imported)
    # Also retain a reporting-compatible JSONL without modifying the originals.
    (output/'imported_results.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in imported))
    for index, job in enumerate(jobs):
        identifier=f'{index:05d}'
        config['jobs'].append(dict(id=identifier, instance=job['case']['id'],method=job['method']))
        atomic(output/'instances'/f'{identifier}.json',dict(job=job,id=identifier,attempt=1))
    hashes=snapshot_sources(ROOT,output/'source.tar.gz')
    config.update(source_hashes=hashes,source_archive_sha256=digest(output/'source.tar.gz'))
    atomic(output/'cluster.json',config)
    # Standalone launcher can run before a Python package has been installed.
    shutil.copy2(__file__,output/'cluster_launcher.py')
    shutil.copytree(ROOT/'benchtools',output/'benchtools',ignore=shutil.ignore_patterns('.git','__pycache__'))
    shutil.copy2(ROOT/'parking_benchmark_scenarios/cluster/README.md',output/'README.md')
    atomic(output/'preview.json',dict(eligible_jobs=len(jobs),imported=len(imported),
                                    possibly_still_running_locally=uncertain,
                                    pruning='disabled; imported prune records reopened',
                                    cpus_per_job=4,memory_gib=32,partitions=PARTITIONS))
    (output/'wrapper.sh').write_text('#!/bin/sh\nset -eu\nexec '+shlex.quote(args.python)+' '+
                                   shlex.quote(str(shared/'cluster_launcher.py'))+
                                   ' execute --root '+shlex.quote(str(shared))+\
                                   ' --instance "$BENCHTOOLS_INSTANCE"\n')
    (output/'wrapper.sh').chmod(0o755)
    shutil.copy2(ROOT/'parking_benchmark_scenarios/cluster/setup.sbatch',output/'setup.sbatch')
    print(json.dumps(read(output/'preview.json'),indent=2))


def extract(archive, destination):
    # Archives generated here contain only ordinary files/directories.
    with tarfile.open(archive) as stream:
        for member in stream.getmembers():
            target = (destination/member.name).resolve()
            if not target.is_relative_to(destination.resolve()) or member.issym() or member.islnk():
                raise ValueError('unsafe archive member')
        stream.extractall(destination, filter='data')


def seal(args):
    """Freeze a compute-node-installed source tree, virtualenv and solver."""
    root=args.root.resolve()
    if not os.environ.get('SLURM_JOB_ID'):
        raise RuntimeError('seal/preflight must run on an allocated compute node')
    config=read(root/'cluster.json')
    source=args.source.resolve(); environment=args.environment.resolve(); solver=args.nuxmv.resolve()
    if not (source/'lib/libvfm.so').is_file() or not (source/'main_template.json').is_file():
        raise ValueError('build lib/libvfm.so and generate main_template.json before sealing')
    if not (environment/'bin/python').is_file() or not solver.is_file():
        raise ValueError('missing virtualenv Python or nuXmv executable')
    if (root/'runtime.tar.gz').exists(): raise FileExistsError('runtime already sealed; use a new migration for rebuilds')
    # Verify copied source against the migration (native binaries may be rebuilt).
    for rel, expected in config['source_hashes'].items():
        if rel.endswith(('.so','.a','.o')) or rel.startswith(('bin/','lib/')): continue
        if not (source/rel).is_file() or digest(source/rel)!=expected:
            raise ValueError(f'frozen source changed during setup: {rel}')
    with tarfile.open(root/'runtime.pending.tar.gz','w:gz',dereference=True) as tar:
        for prefix, directory in [('source',source),('environment',environment)]:
            for path in sorted(directory.rglob('*')):
                rel=path.relative_to(directory)
                if any(part in ('build','.git','__pycache__') for part in rel.parts): continue
                if path.is_file(): tar.add(path,arcname=str(Path(prefix)/rel),recursive=False)
        tar.add(solver,arcname='tools/nuXmv',recursive=False)
    (root/'runtime.pending.tar.gz').replace(root/'runtime.tar.gz')
    versions=subprocess.check_output([str(environment/'bin/python'),'-m','pip','freeze'],text=True)
    atomic(root/'runtime.json',dict(sha256=digest(root/'runtime.tar.gz'),python=platform.python_version(),
                                  dependencies=versions,libvfm_sha256=digest(source/'lib/libvfm.so'),
                                  nuxmv_sha256=digest(solver),source_sha256=config['source_archive_sha256']))
    print('Runtime sealed. Run preflight before submission.')


def relocate(value, old, new):
    if isinstance(value,dict): return {k:relocate(v,old,new) for k,v in value.items()}
    if isinstance(value,list): return [relocate(v,old,new) for v in value]
    if isinstance(value,str) and (value==old or value.startswith(old+'/')):
        return new+value[len(old):]
    return value


def publish(local, destination, metadata):
    started=time.monotonic()
    destination.parent.mkdir(parents=True,exist_ok=True)
    pending=destination.with_name(destination.name+'.publishing')
    if destination.exists() or pending.exists(): raise FileExistsError('attempt output already exists')
    shutil.copytree(local,pending)
    for path in pending.rglob('*.json'):
        try: value=read(path)
        except ValueError: continue
        atomic(path,relocate(value,str(local),str(destination)))
    result=read(pending/'result.json')
    result.update(metadata,directory=str(destination))
    if 'cluster' in result:
        result['cluster']['publication_copy_seconds']=time.monotonic()-started
    atomic(pending/'result.json',result)
    atomic(pending/'publication.json',dict(complete=True,original_output=str(local),published_output=str(destination)))
    pending.rename(destination)


def execute(args):
    started=time.monotonic()
    root=args.root.resolve(); config=read(root/'cluster.json'); spec=read(args.instance)
    runtime=read(root/'runtime.json')
    if digest(root/'runtime.tar.gz')!=runtime['sha256']: raise ValueError('runtime checksum mismatch')
    attempt=spec.get('attempt',1)
    destination=attempt_path(root,spec)
    destination.parent.mkdir(parents=True,exist_ok=True)
    # Atomic claim prevents accidental duplicate executions/requeues.
    claim=destination.with_name(destination.name+'.claim')
    with claim.open('x') as stream: stream.write(os.environ.get('SLURM_JOB_ID','local'))
    scratch=Path(tempfile.mkdtemp(prefix='vfm-cluster-',dir='/tmp'))
    published=False
    try:
        extract(root/'runtime.tar.gz',scratch)
        staging_seconds=time.monotonic()-started
        source=scratch/'source'; python=scratch/'environment/bin/python'
        env=dict(os.environ,PATH=str(scratch/'tools')+os.pathsep+os.environ.get('PATH',''),
                 PYTHONPATH=str(source),MPLCONFIGDIR=str(scratch/'mpl'),OMP_NUM_THREADS='1',
                 OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
        job=dict(spec['job'],directory=str(scratch/'result'),timeout=3600,parallel=1)
        atomic(scratch/'job.json',job)
        command=[str(python),'-m','vfm_grid_python.cluster','internal-run',str(scratch/'job.json')]
        with (scratch/'wrapper.log').open('w') as log:
            subprocess.run(command,cwd=source,env=env,stdout=log,stderr=subprocess.STDOUT,check=False)
        local=scratch/'result'; local.mkdir(exist_ok=True)
        if not (local/'result.json').exists():
            atomic(local/'result.json',failure_record(job,'worker_error',None,destination,config))
        shutil.copy2(scratch/'wrapper.log',local/'cluster_wrapper.log')
        cpu=next((line.split(':',1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines()
                  if line.startswith('model name')),'unknown')
        metadata=dict(execution_environment=config['campaign_id'],cluster=dict(
            hostname=platform.node(),cpu_model=cpu,slurm_job_id=os.environ.get('SLURM_JOB_ID'),
            partition=os.environ.get('SLURM_JOB_PARTITION'),cpus=4,memory_mib=32768,
            runtime=runtime,attempt=attempt,staging_seconds=staging_seconds))
        publish(local,destination,metadata)
        published=True
    finally:
        # Only this explicitly created scratch directory is removed.
        if published: shutil.rmtree(scratch)
        else: print(f'Publication incomplete; scratch retained at {scratch}',file=sys.stderr)


def attempt_path(root,spec):
    return (root/('pilot_attempts' if spec.get('pilot') else 'attempts')/
            spec['job']['case']['id']/spec['job']['method']/f'attempt_{spec.get("attempt",1):03d}')


def failure_record(job,status,elapsed,directory,config):
    case=job['case']
    result=dict(schema='endpoint-benchmark.v1',instance=case['id'],method=job['method'],
                directory=str(directory),family=case['family'],rows=case['config']['rows'],
                columns=case['config']['columns'],cars=case['config']['cars'],repetitions=case['repetitions'],
                status=status,validated=False,wall_seconds=elapsed,model_checking_seconds=None,
                time_to_validated_xml_seconds=None,execution_environment=config['campaign_id'])
    result.update(seed=job.get('seed'),parallel=1,timeout_seconds=3600,
                  semantics=case['config'].get('semantics','legacy'),
                  contract_version=('stable-concurrent.v1' if case['config'].get('semantics')=='stable_concurrent' else 'legacy'))
    for name in ('stage_count','scaling_indices','abstract_bound','concrete_bound','property_size_chars'):
        if name in case: result[name]=case[name]
    return result


def bench_config(root, identifier, spec, backend='slurm'):
    config=read(root/'cluster.json'); work=root/'submission'/f'{identifier}_{spec.get("attempt",1):03d}'
    work.mkdir(parents=True,exist_ok=True)
    jobname=f'{config["campaign_id"]}-{hashlib.sha256(identifier.encode()).hexdigest()[:16]}-{spec.get("attempt",1)}'
    instance=root/'instances'/f'{identifier}.json'
    text=(f'[environment]\nscheduler = {backend}\ndb_uri = {work}/benchtools.sqlite\ncreate_db = true\n'
          f'delay_notifications = true\nslurm_queue = {config["partitions"]}\n'
          f'slurm_resources = --time=01:20:00 --job-name={jobname}\n'
          f'dump_scheduler_script = {work}/submitted.sh\n\n[group_{identifier}]\n'
          f'executable = {root}/wrapper.sh\nparameters =\nbasedir = {root}/instances\n'
          'num_cores = 4\ncputime_limit = 0\nwctime_limit = 4500\n'
          'rssmem_limit = 33554432\nvirtmem_limit = 0\n'
          f'instances_file = {work}/instances.txt\n')
    (work/'instances.txt').write_text(instance.name+'\n')
    (work/'benchtools.conf').write_text(text)
    return work,jobname


def submit(args):
    root=args.root.resolve(); config=read(root/'cluster.json')
    if str(root)!=config['shared_root']: raise ValueError('deploy at configured shared-root before submitting')
    preflight=read(root/'preflight.json')
    if preflight.get('status')!='passed' or preflight.get('runtime_sha256')!=read(root/'runtime.json')['sha256']:
        raise ValueError('successful preflight for this runtime required')
    skip_pilot=getattr(args,'skip_pilot',False)
    if args.pilot and skip_pilot:
        raise ValueError('--pilot and --skip-pilot cannot be combined')
    if not args.pilot and not skip_pilot:
        approval=read(root/'pilot_approved.json') if (root/'pilot_approved.json').exists() else {}
        if approval.get('runtime_sha256')!=read(root/'runtime.json')['sha256']:
            raise ValueError('run the pilot and approve-pilot before bulk submission')
    with locked(root):
        if skip_pilot:
            atomic(root/'pilot_approved.json',dict(
                approval_type='user_override',pilot_passed=False,
                reason='User explicitly chose bulk submission without requiring all pilot witnesses',
                approved_at=datetime.now(timezone.utc).isoformat(),
                runtime_sha256=read(root/'runtime.json')['sha256']))
            print('Pilot requirement waived by user; preflight remains required.',flush=True)
        state=read(root/'submissions.json') if (root/'submissions.json').exists() else {}
        candidates=read(root/'pilot_jobs.json') if args.pilot else config['jobs']
        if not args.pilot and (root/'retry_jobs.json').exists(): candidates=candidates+read(root/'retry_jobs.json')
        for entry in candidates:
            identifier=entry['id']
            if identifier in state: continue  # Uncertain submissions fail closed; collector reconciles them.
            spec=read(root/'instances'/f'{identifier}.json')
            work,name=bench_config(root,identifier,spec)
            state[identifier]=dict(state='submitting',job_name=name,attempt=spec.get('attempt',1))
            atomic(root/'submissions.json',state)
            command=[config['python'],str(root/'benchtools/exec/run_group.py'),str(work/'benchtools.conf')]
            with (work/'submit.log').open('w') as stream:
                try:
                    result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,timeout=120)
                except subprocess.TimeoutExpired:
                    state[identifier]['state']='submission_uncertain'; atomic(root/'submissions.json',state)
                    raise RuntimeError(f'submission uncertain for {identifier}; collect before resubmitting')
            found=re.findall(r'scheduled job (\d+) in group', (work/'submit.log').read_text())
            if result.returncode or len(found)!=1:
                state[identifier]['state']='submission_uncertain'; atomic(root/'submissions.json',state)
                raise RuntimeError(f'cannot confirm submission; inspect {work}/submit.log')
            state[identifier].update(state='submitted',slurm_job_id=found[0])
            atomic(root/'submissions.json',state)
            print(identifier,found[0],flush=True)
            time.sleep(.1)  # Be considerate to the scheduler controller.


def collect(args):
    root=args.root.resolve(); config=read(root/'cluster.json')
    with locked(root):
        state=read(root/'submissions.json') if (root/'submissions.json').exists() else {}
        accounting={}
        if not args.offline:
            text=subprocess.check_output(['sacct','-X','-n','-P','-u',getpass.getuser(),'--starttime',config['prepared_at'][:10],
                                          '--format=JobIDRaw,JobName%100,State%40,ExitCode,MaxRSS,NodeList,Partition,ElapsedRaw'],text=True)
            for line in text.splitlines():
                parts=line.split('|')
                if len(parts)>=8: accounting[parts[0]]=parts
        records=[]; pilots=[]
        for identifier, sub in state.items():
            spec=read(root/'instances'/f'{identifier}.json'); job=spec['job']; case=job['case']
            directory=attempt_path(root,spec)
            # Notification helpers can share the benchmark's name. Only IDs
            # identify executions; recover an uncertain submission from its log.
            job_id=sub.get('slurm_job_id')
            submit_log=root/'submission'/f'{identifier}_{sub["attempt"]:03d}'/'submit.log'
            if submit_log.exists():
                matches=re.findall(r'scheduled job (\d+) in group',submit_log.read_text())
                if len(matches)==1: job_id=matches[0]
            account=accounting.get(str(job_id)) if job_id is not None else None
            if account:
                sub.update(slurm_job_id=account[0],slurm_state=account[2],accounting=account)
            if (directory/'publication.json').exists():
                record=read(directory/'result.json'); sub['state']='collected'
            elif account and account[2].split()[0] in ('TIMEOUT','OUT_OF_MEMORY','CANCELLED','NODE_FAIL','FAILED','PREEMPTED','COMPLETED','BOOT_FAIL'):
                # Wait a minute before diagnosing missing publication to avoid NFS visibility races.
                sub.setdefault('terminal_seen_at',time.time())
                if time.time()-sub['terminal_seen_at']<60: continue
                record=failure_record(job,'cluster_'+account[2].split()[0].lower(),
                                      float(account[7]) if account[7] else None,directory,config)
                record['cluster']=dict(slurm_job_id=account[0],accounting=account,attempt=spec.get('attempt',1))
                # Keep scheduler failures separate from any incomplete artifact directory.
                atomic(root/'failures'/f'{identifier}_{sub["attempt"]:03d}.json',record)
                sub['state']='collected_failure'
            elif sub['state']=='collected_failure':
                record=read(root/'failures'/f'{identifier}_{sub["attempt"]:03d}.json')
            else: continue
            (pilots if spec.get('pilot') else records).append(record)
        atomic(root/'submissions.json',state)
        pending=root/'results.jsonl.pending'
        pending.write_text(''.join(json.dumps(r)+'\n' for r in records))
        pending.replace(root/'results.jsonl')
        atomic(root/'pilot_results.json',pilots)
        print(f'{len(records)} collected; {sum(s["state"] not in ("collected","collected_failure") for s in state.values())} outstanding')


def preflight(args):
    root=args.root.resolve()
    if not os.environ.get('SLURM_JOB_ID'): raise RuntimeError('preflight must run on a compute node')
    runtime=read(root/'runtime.json')
    if digest(root/'runtime.tar.gz')!=runtime['sha256']: raise ValueError('runtime checksum mismatch')
    with tempfile.TemporaryDirectory(prefix='vfm-preflight-',dir='/tmp') as temp:
        work=Path(temp); extract(root/'runtime.tar.gz',work)
        env=dict(os.environ,PATH=str(work/'tools')+os.pathsep+os.environ.get('PATH',''),PYTHONPATH=str(work/'source'),
                 MPLCONFIGDIR=str(work/'mpl'))
        py=work/'environment/bin/python'
        # run_job does not require .git, unlike the interactive campaign CLI.
        code='''import json
from pathlib import Path
from vfm_grid_python.benchmark import catalog, METHODS, run_job
cases=catalog('parking_benchmark_scenarios/adaptive_campaign/smoke.toml')
jobs=[dict(case=c,method=m,timeout=3600,parallel=1,seed=0) for c in cases for m in METHODS]
Path('../pilot_jobs.json').write_text(json.dumps(jobs))
job=next(j for j in jobs if j['method']=='abstract_monitor' and j['case']['config']['cars']==1)
job=dict(job,directory=str(Path('../smoke').resolve()),timeout=120)
result=run_job(job)
assert result.get('validated'), result
'''
        commands=[['-c','import ctypes, networkx, matplotlib, tomllib; ctypes.CDLL("lib/libvfm.so")'],['-c',code]]
        with (root/'preflight.log').open('w') as log:
            for command in commands:
                subprocess.run([str(py)]+command,cwd=work/'source',env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=300)
        entries=[]
        for index,job in enumerate(read(work/'pilot_jobs.json')):
            identifier=f'pilot_{index:03d}'
            atomic(root/'instances'/f'{identifier}.json',dict(id=identifier,job=job,attempt=1,pilot=True))
            entries.append(dict(id=identifier,instance=job['case']['id'],method=job['method']))
        shutil.copytree(work/'smoke',root/'preflight_artifacts',dirs_exist_ok=True)
    atomic(root/'preflight.json',dict(status='passed',runtime_sha256=runtime['sha256'],host=platform.node()))
    atomic(root/'pilot_jobs.json',entries)
    print('Preflight passed; submit --pilot next.')


def retry(args):
    root=args.root.resolve()
    with locked(root):
        state=read(root/'submissions.json'); previous=state[args.id]
        if previous['state'] not in ('collected','collected_failure'):
            raise ValueError('collect a terminal attempt before retrying; uncertain/live submissions cannot be retried')
        spec=read(root/'instances'/f'{args.id}.json')
        if spec.get('pilot'): raise ValueError('use a new migration if the pilot setup needs changes')
        attempt=spec.get('attempt',1)+1
        identifier=args.id.split('_retry_')[0]+f'_retry_{attempt}'
        path=root/'instances'/f'{identifier}.json'
        if path.exists(): raise FileExistsError('retry already queued')
        spec.update(id=identifier,attempt=attempt)
        atomic(path,spec)
        jobs=read(root/'retry_jobs.json') if (root/'retry_jobs.json').exists() else []
        jobs.append(dict(id=identifier,instance=spec['job']['case']['id'],method=spec['job']['method']))
        atomic(root/'retry_jobs.json',jobs)
        print(f'Queued {identifier}; submit resumes safely and includes this new attempt.')


def unpack_source(args):
    root=args.root.resolve()
    if digest(root/'source.tar.gz')!=read(root/'cluster.json')['source_archive_sha256']:
        raise ValueError('source checksum mismatch')
    args.destination.mkdir(parents=True,exist_ok=False)
    extract(root/'source.tar.gz',args.destination)


def preview(args):
    source=args.source.resolve()
    jobs,imported,uncertain=migrate(read(source/'manifest.json'),source)
    print(json.dumps(dict(eligible_jobs=len(jobs),completed_imports=len(imported),
                          possibly_still_running_locally=uncertain),indent=2))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    subs=parser.add_subparsers(dest='command',required=True)
    p=subs.add_parser('preview'); p.add_argument('source',type=Path)
    p=subs.add_parser('unpack-source'); p.add_argument('root',type=Path); p.add_argument('destination',type=Path)
    p=subs.add_parser('prepare'); p.add_argument('source',type=Path); p.add_argument('--output',type=Path,required=True)
    p.add_argument('--shared-root',required=True); p.add_argument('--python',required=True)
    p=subs.add_parser('seal'); p.add_argument('root',type=Path); p.add_argument('--source',type=Path,required=True)
    p.add_argument('--environment',type=Path,required=True); p.add_argument('--nuxmv',type=Path,required=True)
    for name in ('preflight','approve-pilot'):
        p=subs.add_parser(name); p.add_argument('root',type=Path)
    p=subs.add_parser('submit'); p.add_argument('root',type=Path); p.add_argument('--pilot',action='store_true')
    p.add_argument('--skip-pilot',action='store_true',help='explicitly waive pilot approval; successful preflight is still required')
    p=subs.add_parser('collect'); p.add_argument('root',type=Path); p.add_argument('--offline',action='store_true')
    p=subs.add_parser('retry'); p.add_argument('root',type=Path); p.add_argument('--id',required=True)
    p=subs.add_parser('execute'); p.add_argument('--root',type=Path,required=True); p.add_argument('--instance',type=Path,required=True)
    p=subs.add_parser('internal-run'); p.add_argument('job',type=Path)
    args=parser.parse_args()
    if args.command=='internal-run':
        from .benchmark import run_job
        run_job(read(args.job))
    elif args.command=='approve-pilot':
        root=args.root.resolve(); entries=read(root/'pilot_jobs.json'); submissions=read(root/'submissions.json')
        if not all(e['id'] in submissions and submissions[e['id']]['state']=='collected' for e in entries):
            raise RuntimeError('all pilot jobs must finish and publish before approval')
        for entry in entries:
            spec=read(root/'instances'/f'{entry["id"]}.json')
            result=read(attempt_path(root,spec)/'result.json')
            if not result.get('validated') or result.get('status')!='witness':
                raise RuntimeError('pilot must produce validated witnesses on all approaches; inspect pilot_results.json')
        atomic(root/'pilot_approved.json',dict(approved_at=datetime.now(timezone.utc).isoformat(),
                                             runtime_sha256=read(root/'runtime.json')['sha256']))
    else: globals()[args.command.replace('-','_')](args)


if __name__=='__main__': main()
