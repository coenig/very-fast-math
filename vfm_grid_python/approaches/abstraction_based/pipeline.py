"""Paper-cell search followed by monitoring or stopped-lookahead refinement."""
import argparse
import json
import os
from pathlib import Path
import re
import time

from vfm_grid_python.common import either, check, dump, states
from vfm_grid_python.scenarios.roads import inventory
from vfm_grid_python.concrete.observations import road_mapping
from vfm_grid_python.benchmark_timing import DEADLINE, JOURNAL, PHASE, totals
from . import paper_cells
from .monitor import selected


def refinement(frames):
    """Compile ordered paper-cell milestones, retaining property progress."""
    def compile(config, graph, geometry):
        mapping = road_mapping(config, geometry)
        predicates = [paper_cells.milestone(f, config, mapping) for f in frames]
        lines = [f'VAR refinement_phase : 0..{len(frames)-1};',
                 'INIT refinement_phase = 0;', 'INIT '+predicates[0]+';']
        # Freeze the selected roles/starts/goals, not only the topology.
        for key, value in frames[0].items():
            if re.fullmatch(r'(start_\d+|goal_\d+|role_\w+)', key) and re.fullmatch(r'\d+', value):
                lines.append(f'INIT {key} = {value};')
        moves = ['next(refinement_phase) = refinement_phase']
        for k in range(len(frames)-1):
            moves.append(f'refinement_phase = {k} & next(refinement_phase) = {k+1} & next({predicates[k+1]})')
        lines += ['TRANS '+either(moves)+';',
                  f'INVARSPEC !(refinement_phase = {len(frames)-1} & scenario_accepted);']
        return '\n'.join(lines)
    return compile


def run(config, output, mode='monitor', abstract_bound=100, concrete_bound=300, timeout=2000):
    """Search the symbolic topology once, then refine its paper-cell witness.

    Monitoring solves the whole concrete trace. Sequentialization commits
    the first goal of each two-goal query and requires a stopped second goal.
    """
    if mode not in ('monitor', 'sequential'):
        raise ValueError(mode)
    if config.get('profile') != 'paper':
        raise ValueError('the paper pipeline requires profile="paper"')
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    saved = {k: os.environ.get(k) for k in (DEADLINE, JOURNAL, PHASE)}
    os.environ[DEADLINE] = str(min(float(saved[DEADLINE] or 'inf'),started+timeout))
    os.environ[JOURNAL] = str(output/'timing.jsonl')
    result = dict(status='error', semantics='paper-cells.v1', mode=mode,
                  abstraction='paper_cells', lookahead='stopped' if mode=='sequential' else 'ordinary')
    try:
        a = output/'abstract'; a.mkdir(); dump(a/'config.json', config)
        result['assumptions'] = paper_cells.assumptions(config)
        dump(output/'assumptions.json', result['assumptions'])
        model = a/'paper_cells.smv'; model.write_text(paper_cells.render(config))
        result['abstract_model_lines'] = len(model.read_text().splitlines())
        os.environ[PHASE] = 'abstract'
        strategy = config.get('abstract_bound_strategy', 'incremental')
        if strategy not in ('incremental', 'fixed'):
            raise ValueError('abstract_bound_strategy must be incremental or fixed')
        search = check(model, abstract_bound, timeout, backend='sat',
                       sat_fixed_invariant='!scenario_accepted' if strategy == 'fixed' else None)
        result['abstract_bound_strategy'] = strategy
        result['abstract'] = search
        result['abstract_witness_wall_seconds'] = time.monotonic()-started if search['status']=='witness' else None
        if search['status'] != 'witness':
            result['status'] = 'abstract_'+search['status']; return result
        frames = states(a/'trace.xml')
        result['abstract_trace_states'] = len(frames)
        if strategy == 'fixed':
            # Refine only the accepting prefix, not fixed-bound padding.
            accepted = str(len(config['scenario_stages']))
            first = next(k for k,f in enumerate(frames) if f['scenario_phase'] == accepted)
            frames = frames[:first+1]
        result['abstract_states'] = len(frames)
        errors = paper_cells.validate_trace(frames, config)
        dump(a/'validation.json', dict(valid=not errors, errors=errors))
        if errors:
            raise ValueError(f'invalid paper-cell witness: {errors[:3]}')
        layout = selected(config, frames[0])
        from vfm_grid_python.scenarios import paper
        _, joints = inventory(config)
        layout['joint_assignments'] = {z:int(frames[0][z]) for z in paper.names(config)}
        layout.update(starts=[joints[j]['node'] for j in layout['starts']],
                      goals=[joints[j]['node'] for j in layout['goals']],
                      roles={n:joints[j]['node'] for n,j in layout['roles'].items()})
        dump(a/'selected_graph.json', layout)
        os.environ[PHASE] = 'concrete'
        from vfm_grid_python.concrete.compiler import run as concrete
        c = concrete(config, layout, output/'concrete', concrete_bound, timeout,
                     refinement=refinement(frames), generate_only=mode=='sequential')
        if mode=='sequential' and c['status']=='generated':
            from .sequential_refinement import run as sequential
            c=sequential(output/'concrete',output/'segments',concrete_bound,timeout,
                         abstract_trace=a/'trace.xml')
        result.update(concrete=c, status=c['status'], trace=c.get('trace'))
    except MemoryError:
        result.update(status='memory_limit',error='allocation failed during abstraction/refinement')
    except Exception as exc:
        result.update(status='error', error=str(exc))
    finally:
        result.update(wall_seconds=time.monotonic()-started,
                      timings_seconds=totals(output/'timing.jsonl', time.monotonic()))
        if mode=='sequential':
            for category,seconds in result.get('concrete',{}).get('timings_seconds',{}).items():
                result['timings_seconds'][category]=result['timings_seconds'].get(category,0)+seconds
        dump(output/'result.json', result)
        for k, value in saved.items():
            if value is None: os.environ.pop(k, None)
            else: os.environ[k] = value
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('config', type=Path); p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--mode', choices=['monitor', 'sequential'], default='monitor')
    p.add_argument('--timeout', type=float, default=2000)
    p.add_argument('--abstract-bound', type=int, default=100)
    p.add_argument('--concrete-bound', type=int, default=300)
    p.add_argument('--memory-gib', type=float, default=None)
    args = p.parse_args()
    from vfm_grid_python.resource_limits import install
    install(args.memory_gib)
    from vfm_grid_python.scenarios.validation import normalize
    if args.config.suffix.lower() == '.toml':
        import tomllib
        config = tomllib.loads(args.config.read_text())
    else:
        config = json.loads(args.config.read_text())
    print(json.dumps(run(normalize(config), args.output_dir, args.mode,
                         args.abstract_bound, args.concrete_bound, args.timeout), indent=2))
