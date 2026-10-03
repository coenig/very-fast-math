"""Finalize an existing sequential-local concretization.

This command resumes after all local segments have been solved. It does not
rerun the abstract search or any local nuXmv segment; it only builds the full
model inventory, completes the stitched states, validates the XML, and renders
the final artifacts.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from .common import ROOT, dump, states
from .geometry import build
from .sequential import annotate_monitor, export_trace, global_inventory


def run(concrete_run, timeout=3600, template_config=ROOT / "main_template.json",
        gif=True, validation_model=None):
    concrete_run=Path(concrete_run).resolve()
    result_path=concrete_run/'result.json'
    config=json.loads((concrete_run/'config.json').read_text())
    abstract_run=Path(config['abstract_run']).resolve()
    graph=json.loads((concrete_run/'selected_graph.json').read_text())
    geometry=json.loads((concrete_run/'geometry.json').read_text())
    trace=next((concrete_run/name for name in ('partial_trace.xml','trace.xml')
                if (concrete_run/name).exists()),None)
    if trace is None: raise FileNotFoundError('no partial_trace.xml or trace.xml in concrete run')
    frames=states(trace)
    if not frames: raise ValueError('the saved trace contains no states')
    started=time.perf_counter()
    annotate_monitor(frames,graph,geometry)
    inventory=global_inventory(graph,geometry,config,concrete_run/'global_model',
                               template_config,timeout,validation_model)
    final=export_trace(frames,geometry,concrete_run,True,inventory)
    from .validate import validate_trace
    report=validate_trace(graph,geometry,final)
    report.update(complete=True,source='Python concatenation, checked against full concrete model',
                  full_model=json.loads((concrete_run/'model_validation.json').read_text()))
    dump(concrete_run/'validation.json',report)
    if gif:
        from .visualize import concrete_plot
        concrete_plot(geometry,concrete_run,final,True)
    summary=json.loads(result_path.read_text()) if result_path.exists() else {}
    summary.update(status='witness',trace=str(final),states=len(frames),
                   finalization_seconds=time.perf_counter()-started,
                   model_validation=report['full_model'])
    dump(result_path,summary)
    print(f'Final trace: {final}\nFinalization seconds: {summary["finalization_seconds"]:.2f}',flush=True)
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('concrete_run',type=Path)
    p.add_argument('--timeout',type=float,default=3600)
    p.add_argument('--template-config',type=Path,default=ROOT/'main_template.json')
    p.add_argument('--validation-model',type=Path)
    p.add_argument('--no-gif',action='store_true')
    args=p.parse_args()
    try:
        run(args.concrete_run,args.timeout,args.template_config,not args.no_gif,args.validation_model)
    except Exception as exc:
        print(f'Finalization failed: {exc}',flush=True)
        return 2
    return 0


if __name__=='__main__': raise SystemExit(main())
