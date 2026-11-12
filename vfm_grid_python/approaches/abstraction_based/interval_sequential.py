"""Actual greedy two-goal queries, carrying the full concrete state.

Uses one-milestone lookahead, not suffix witness reuse. By default it retains
the cluster query unchanged. Optional stopped lookahead strengthens only the
second goal; the committed milestone can still have moving cars. Canonical
mode additionally prescribes integer stopping positions inside the cells.
A failed handoff is reported, not asserted impossible or silently backtracked.
Spatial slicing is not performed: each query retains the same full dynamics.
"""
import argparse
import json
import os
import re
from pathlib import Path
import time

from vfm_grid_python.common import check, dump, states
from vfm_grid_python.concrete.schema import flatten, flat_body, validate_value
from vfm_grid_python.concrete.trace_model import TraceModel, Evaluation, parse
from vfm_grid_python.concrete.traces import export_trace
from vfm_grid_python.benchmark_timing import DEADLINE, JOURNAL, PHASE, totals


def resume_init(frame, variables):
    missing = set(variables) - frame.keys()
    if missing:
        raise ValueError(f'missing persistent state: {sorted(missing)[:5]}')
    return '\n'.join(f'INIT {name} = {validate_value(name, frame[name], decl["domain"])};'
                     for name, decl in variables.items())


def cut_prefix(frames, target):
    """Commit only the first goal; the lookahead tail is discarded."""
    for tick, frame in enumerate(frames):
        if int(frame['refinement_phase']) == target:
            return frames[:tick+1]
    raise ValueError('solver witness omitted the commit milestone')


def run(source, output, bound=300, timeout=2000, method='sequential', *,
        lookahead='ordinary', abstract_trace=None):
    if method not in ('sequential','monitor'):
        raise ValueError(method)
    if lookahead not in ('ordinary', 'stopped', 'canonical-stopped'):
        raise ValueError(lookahead)
    if method != 'sequential' and lookahead != 'ordinary':
        raise ValueError('stopped lookahead is only supported by sequential refinement')
    source, output = Path(source).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    saved = {k: os.environ.get(k) for k in (DEADLINE,JOURNAL,PHASE)}
    os.environ[DEADLINE] = str(min(float(saved[DEADLINE] or 'inf'),started+timeout))
    os.environ[JOURNAL],os.environ[PHASE] = str(output/'timing.jsonl'),'concrete'
    result = dict(status='error',method=method,lookahead=lookahead,queries=[],
                  spatial_slicing=False,abstract_search_included=False)
    try:
        model = Path(json.loads((source/'model.json').read_text())['model'])
        geometry = json.loads((source/'geometry.json').read_text())
        config = json.loads((source/'config.json').read_text())
        text, variables, _ = flatten(model,output/'flat',timeout)
        last = max(map(int,re.findall(r'\d+',variables['refinement_phase']['domain'])))
        targets = None
        if lookahead != 'ordinary':
            from .stopped_lookahead import build_targets, UnsupportedStoppedTarget
            trace = Path(abstract_trace) if abstract_trace else source.parent/'abstract/trace.xml'
            if not trace.is_file():
                raise ValueError('stopped lookahead needs the paper-cell abstract trace; '
                                 'supply --abstract-trace')
            abstract_frames = states(trace)
            if len(abstract_frames) < last + 1:
                raise ValueError('abstract trace has fewer states than the concrete monitor')
            # Fixed-bound searches may contain padding after acceptance.
            try:
                targets, certificate = build_targets(abstract_frames[:last+1], config, geometry,
                                                     mode=lookahead)
            except UnsupportedStoppedTarget as exc:
                result.update(status='unsupported_stopped_target', error=str(exc))
                dump(output/'stopped_targets.json', dict(status=result['status'], error=str(exc)))
                return result
            certificate['abstract_trace'] = str(trace.resolve())
            dump(output/'stopped_targets.json', certificate)
            result['stopped_targets'] = str(output/'stopped_targets.json')
        inventory = TraceModel(text)
        inventory.source = output/'flat/baseline.smv'; inventory.timeout=timeout
        joined=[]
        for k in range(max(1,last) if method=='sequential' else 1):
            target = min(k+2,last) if method=='sequential' else last
            commit = min(k+1,last) if method=='sequential' else last
            directory=output/f'query_{k:04d}'; directory.mkdir()
            body=flat_body(text,resumed=bool(joined))
            if joined:
                body+='\n'+resume_init(joined[-1],variables)+'\n'
            elif targets is not None:
                body += '\nINIT ' + targets[0] + ';\n'
            goal=f'refinement_phase = {target}'
            if targets is not None:
                goal += ' & ' + targets[target]
            if target==last:
                goal+=' & scenario_accepted'
            query=directory/'main.smv'
            query.write_text(body+f'\nINVARSPEC !({goal});\n')
            checked=check(query,bound,timeout)
            result['queries'].append(dict(index=k,target=target,commit=commit,**checked))
            dump(output/'progress.json',result)
            if checked['status']!='witness':
                result.update(status=checked['status'],failed_segment=k,
                              committed_states=len(joined))
                return result
            frames=states(directory/'trace.xml')
            # Check each entire lookahead witness against its actual query.
            local=TraceModel(query.read_text())
            frames=local.complete(frames,geometry)
            dump(directory/'validation.json',local.validate(frames))
            if not Evaluation(local, frames).eval(parse(goal), len(frames)-1):
                raise ValueError('solver witness does not satisfy its terminal query goal')
            prefix=frames if commit==last else cut_prefix(frames,commit)
            if joined:
                if any(joined[-1][name]!=prefix[0][name] for name in variables):
                    raise ValueError('full-state boundary mismatch')
                joined.extend(prefix[1:])
            else:
                joined=prefix
            dump(output/'committed_state.json',{name:joined[-1][name] for name in variables})
            print(f'Committed {commit}/{last}; {len(joined)} ticks',flush=True)
        if int(joined[-1]['scenario_phase'])!=len(config['scenario_stages']):
            raise ValueError('concatenation did not accept original scenario')
        xml=export_trace(joined,geometry,output,True,inventory,
                         description=f'{method}: full-state concrete milestone refinement')
        dump(output/'geometry.json',geometry)
        result.update(status='witness',validated=True,trace=str(xml),states=len(joined))
    except MemoryError:
        result.update(status='memory_limit',error='allocation failed during sequential refinement')
    except Exception as exc:
        result.update(status='timeout' if time.monotonic() >= float(os.environ[DEADLINE]) else 'error',
                      error=str(exc))
    finally:
        result.update(wall_seconds=time.monotonic()-started,
                      timings_seconds=totals(output/'timing.jsonl',time.monotonic()))
        dump(output/'result.json',result)
        for key,value in saved.items():
            if value is None: os.environ.pop(key,None)
            else: os.environ[key]=value
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--bound',type=int,default=300)
    p.add_argument('--timeout',type=float,default=2000)
    p.add_argument('--method',choices=['sequential','monitor'],default='sequential')
    p.add_argument('--lookahead', choices=['ordinary','stopped','canonical-stopped'], default='ordinary')
    p.add_argument('--abstract-trace', type=Path,
                   help='paper-cell trace for stopped targets; defaults to source/../abstract/trace.xml')
    p.add_argument('--memory-gib', type=float, default=None)
    a=p.parse_args()
    from vfm_grid_python.resource_limits import install
    install(a.memory_gib)
    print(json.dumps(run(a.source,a.output_dir,a.bound,a.timeout,a.method,
                         lookahead=a.lookahead, abstract_trace=a.abstract_trace),indent=2))
