"""Prepare a smaller submission list without changing deployed code or results.

Standalone: copy this file to the cluster campaign and run it with its Python.
Default is preview; --apply backs up and replaces only the submission inventory.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile


def read(path):
    return json.loads(path.read_text())


def atomic(path, value):
    temporary=path.with_name(path.name+'.subset-pending')
    temporary.write_text(json.dumps(value,indent=2)+'\n')
    temporary.replace(path)


def generate(root, config, grids, cars, rounds):
    archive=root/'source.tar.gz'
    with archive.open('rb') as stream:
        actual=hashlib.file_digest(stream,'sha256').hexdigest()
    if actual!=config['source_archive_sha256']:
        raise ValueError('frozen source archive checksum mismatch')
    original=config['source_manifest']['catalog']
    marker='parking_benchmark_scenarios/'
    if marker not in original: raise ValueError('cannot locate the frozen catalog')
    relative=marker+original.split(marker,1)[1]
    with tempfile.TemporaryDirectory(prefix='vfm-subset-',dir='/tmp') as temp:
        work=Path(temp)
        with tarfile.open(archive) as source:
            for member in source:
                name=Path(member.name)
                if not (str(name).startswith('vfm_grid_python/') or str(name).startswith(marker)):
                    continue
                if not member.isfile(): continue
                if not (work/name).resolve().is_relative_to(work.resolve()):
                    raise ValueError('unsafe frozen source archive member')
                target=work/name; target.parent.mkdir(parents=True,exist_ok=True)
                with source.extractfile(member) as src, target.open('wb') as dst:
                    shutil.copyfileobj(src,dst)
        atomic(work/'selection.json',dict(grids=grids,cars=cars,rounds=rounds,catalog=relative))
        code='''import json,tomllib
from pathlib import Path
from vfm_grid_python.adaptive_benchmark import expand
request=json.loads(Path('selection.json').read_text())
raw=tomllib.loads(Path(request['catalog']).read_text())
raw['adaptive']={k:request[k] for k in ('grids','cars','rounds')}
# This selection varies fleet size; omit the separate fixed four-car sanity case.
raw['scenario']=[s for s in raw['scenario'] if s['family']!='nearby_parking']
cases,excluded=expand(raw)
Path('expanded.json').write_text(json.dumps(dict(cases=cases,excluded=excluded)))
'''
        subprocess.run([sys.executable,'-c',code],cwd=work,
                       env=dict(os.environ,PYTHONPATH=str(work)),check=True)
        return read(work/'expanded.json')


def select(root, config, cases, excluded, apply=False):
    if not cases: raise ValueError('selection contains no valid configurations')
    existing={}
    for path in sorted((root/'instances').glob('*.json')):
        spec=read(path)
        if spec.get('pilot') or spec.get('attempt',1)!=1: continue
        job=spec['job']; key=(job['case']['id'],job['method'])
        if key in existing: raise ValueError(f'duplicate primary instance: {key}')
        existing[key]=(path.stem,spec)
    state=read(root/'submissions.json') if (root/'submissions.json').exists() else {}
    for sub in state.values():
        if sub['state'] in ('submitting','submission_uncertain'):
            raise ValueError('reconcile uncertain submissions before changing the selection')
    methods=sorted({j['method'] for j in config['source_manifest']['jobs']})
    seeds={j.get('seed',0) for j in config['source_manifest']['jobs']}
    if len(seeds)!=1: raise ValueError('mixed seed campaign requires explicit per-case selection')
    entries=[]; additions=[]
    for case in cases:
        for method in methods:
            key=(case['id'],method)
            if key in existing:
                identifier,spec=existing[key]
                old=spec['job']['case']
                for field in ('config','abstract_bound','concrete_bound','repetitions','family'):
                    if old.get(field)!=case.get(field):
                        raise ValueError(f'frozen generated case differs for {key}: {field}')
            else:
                identifier='subset_'+hashlib.sha256('/'.join(key).encode()).hexdigest()[:20]
                if (root/'instances'/f'{identifier}.json').exists():
                    raise ValueError('new instance ID collision')
                spec=dict(id=identifier,attempt=1,job=dict(case=case,method=method,
                          timeout=config.get('workload_seconds',3600),parallel=1,seed=next(iter(seeds))))
                additions.append((identifier,spec))
            entries.append(dict(id=identifier,instance=case['id'],method=method))
    submitted=sum(e['id'] in state for e in entries)
    report=dict(configurations=len(cases),selected_attempts=len(entries),already_submitted=submitted,
                remaining_to_submit=len(entries)-submitted,new_instance_files=len(additions),
                by_family=dict(Counter(c['family'] for c in cases)),excluded=excluded,
                selection=entries,applied=apply)
    if apply:
        stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        backup=root/'selection_backups'/stamp; backup.mkdir(parents=True,exist_ok=False)
        shutil.copy2(root/'cluster.json',backup/'cluster.json')
        # submit() appends this list; exclude off-selection retries as well.
        if (root/'retry_jobs.json').exists():
            shutil.copy2(root/'retry_jobs.json',backup/'retry_jobs.json')
            selected_pairs={(e['instance'],e['method']) for e in entries}
            retries=read(root/'retry_jobs.json')
            atomic(root/'retry_jobs.json',[e for e in retries if (e['instance'],e['method']) in selected_pairs])
        for identifier,spec in additions: atomic(root/'instances'/f'{identifier}.json',spec)
        config=dict(config,jobs=entries)
        atomic(root/'cluster.json',config)
        report['backup']=str(backup)
        atomic(root/'subset_selection.json',report)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaign',type=Path)
    parser.add_argument('--grids',nargs='+',default=['5x5','8x8'])
    parser.add_argument('--cars',nargs='+',type=int,default=[5,6,7,8,9])
    parser.add_argument('--rounds',nargs='+',type=int,default=[2,3])
    parser.add_argument('--apply',action='store_true')
    args=parser.parse_args(); root=args.campaign.resolve()
    grids=[list(map(int,g.lower().split('x'))) for g in args.grids]
    with (root/'.controller.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        config=read(root/'cluster.json')
        expanded=generate(root,config,grids,args.cars,args.rounds)
        report=select(root,config,expanded['cases'],expanded['excluded'],args.apply)
    print(json.dumps({k:v for k,v in report.items() if k not in ('selection','excluded')},indent=2))
    print(f'{len(report["excluded"])} structurally invalid combinations excluded.')
    print('Selection saved. Existing launcher can now submit.' if args.apply else 'Preview only. Add --apply to update the submission list.')


if __name__=='__main__': main()
