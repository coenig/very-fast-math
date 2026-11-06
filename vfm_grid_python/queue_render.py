"""Render completed queue-campaign witnesses outside benchmark timing.

Use --watch to render later completions too. Enumeration has no abstract trace.
Abstract GIFs are queue diagrams with representative positions, not dynamics.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from .common import ROOT
from .queue_campaign import write_json


def artifacts(result):
    targets=[]
    directory=Path(result['directory'])
    if result.get('abstract',{}).get('status')=='witness':
        source=directory/'abstract'
        queue=(source/'queue_trace.json').exists()
        targets.append(('abstract',source/('queues.gif' if queue else 'abstract.gif'),
                        [sys.executable,'-m','vfm_grid_python.queue_visualize' if queue else 'vfm_grid_python.visualize',str(source)]))
    if result.get('status')=='witness' and result.get('trace'):
        trace=Path(result['trace'])
        targets.append(('concrete',trace.parent/'concrete.gif',
                        [sys.executable,'-m','vfm_grid_python.visualize',str(trace.parent),'--trace',str(trace)]))
    return targets


def render_completed(root, retry=False):
    manifest=json.loads((root/'campaign.json').read_text())
    record=root/'rendering.json'
    rendered=json.loads(record.read_text()) if record.exists() else {}
    logs=root/'render_logs';logs.mkdir(exist_ok=True)
    lines=['# Witness animations','',
           'Abstract: queue diagrams (representative positions, not vehicle dynamics).',
           'Concrete: selected topology and positions from the concrete XML trace.',
           'Enumeration runs have no abstract trace. Rendering is outside benchmark timings.', '',
           '| Scenario | Approach | Result | Abstract GIF | Concrete GIF |',
           '|---|---|---|---|---|']
    completed=0
    for job in manifest['jobs']:
        saved=root/'attempts'/job['instance']/job['method']/'results.json'
        if not saved.exists():
            continue
        try:
            result=json.loads(saved.read_text())[0]
        except (json.JSONDecodeError,IndexError):
            continue
        completed+=1
        links={}
        for kind,target,command in artifacts(result):
            key=f"{job['instance']}/{job['method']}/{kind}"
            if not target.exists() and (retry or rendered.get(key,{}).get('status')!='error'):
                log=logs/(key.replace('/','__')+'.log')
                env=dict(os.environ,MPLBACKEND='Agg')
                env.setdefault('MPLCONFIGDIR','/tmp/vfm-queue-mpl')
                started=time.monotonic()
                try:
                    with log.open('w') as stream:
                        proc=subprocess.run(command,cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=180)
                    status='rendered' if proc.returncode==0 and target.exists() else 'error'
                except subprocess.TimeoutExpired:
                    status='error'
                rendered[key]=dict(status=status,gif=str(target),log=str(log),seconds=time.monotonic()-started)
                write_json(record,rendered)
                print(f'{key}: {status}',flush=True)
            if target.exists() and rendered.get(key,{}).get('status')!='error':
                links[kind]=f'[{kind}]({os.path.relpath(target,root)})'
            else:
                links[kind]='render error (see render_logs)'
        lines.append(f"| {job['instance']} | {job['method']} | {result['status']} | {links.get('abstract','—')} | {links.get('concrete','—')} |")
    (root/'GIFS.md').write_text('\n'.join(lines)+'\n')
    return completed,len(manifest['jobs'])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('campaign',type=Path)
    p.add_argument('--watch',action='store_true')
    p.add_argument('--interval',type=float,default=60)
    p.add_argument('--retry',action='store_true')
    args=p.parse_args()
    if args.interval<=0:p.error('positive interval required')
    root=args.campaign.resolve()
    while True:
        completed,total=render_completed(root,args.retry)
        print(f'{completed}/{total} completed attempts; GIF index: {root/"GIFS.md"}',flush=True)
        if not args.watch or completed==total:break
        time.sleep(args.interval)


if __name__=='__main__':main()
