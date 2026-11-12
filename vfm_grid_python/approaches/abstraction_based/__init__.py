"""Paper-cell refinement and the separately supported legacy queue monitor."""
import os
import time
from pathlib import Path
from vfm_grid_python.benchmark_timing import JOURNAL, PHASE, DEADLINE, totals
from vfm_grid_python.common import dump


def run(config, output, abstract_bound=60, concrete_bound=200, timeout=300, refinement='monitor'):
    if config.get('profile') == 'paper':
        from .pipeline import run as paper_run
        return paper_run(config, output, refinement, abstract_bound, concrete_bound, timeout)
    if refinement != 'monitor':
        from .sequential import run as unavailable
        return unavailable()
    from .abstract import run as search
    from .monitor import run as refine
    output=Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    saved={k:os.environ.get(k) for k in (JOURNAL,PHASE,DEADLINE)}
    started=time.monotonic(); result={}
    try:
        os.environ[DEADLINE]=str(min(float(saved[DEADLINE] or 'inf'),started+timeout))
        os.environ[JOURNAL],os.environ[PHASE]=str(output/'abstract_timing.jsonl'),'abstract'
        a=search(config,output/'abstract',abstract_bound,timeout)
        result.update(abstract=a,abstract_timings_seconds=totals(output/'abstract_timing.jsonl',time.monotonic()))
        if a['status']!='witness':
            result['status']='abstract_'+a['status']
        elif time.monotonic()>=float(os.environ[DEADLINE]):
            result['status']='timeout'
        else:
            os.environ[PHASE]='concrete'
            c=refine(output/'abstract',output/'concrete',concrete_bound,timeout,gif=False)
            result.update(concrete=c,status='witness' if c['status']=='witness' else 'refinement_'+c['status'],trace=c.get('trace'))
    except Exception as exc:
        result.update(status='error',error=str(exc))
    finally:
        result['wall_seconds']=time.monotonic()-started
        result['model_checking_seconds']=result.get('abstract_timings_seconds',{}).get('model_checking',0)+result.get('concrete',{}).get('timings_seconds',{}).get('model_checking',0)
        dump(output/'result.json',result)
        for k,v in saved.items():
            if v is None: os.environ.pop(k,None)
            else: os.environ[k]=v
    return result
