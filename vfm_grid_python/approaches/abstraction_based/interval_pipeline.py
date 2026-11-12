"""Experimental interval search and concrete milestone refinement.

Certified mode uses a full concrete suffix witness as its certificate, then
commits its segments greedily. This baseline does NOT reduce solver queries
to local models or claim a speedup over the monitor.
The separate sequential mode runs actual two-goal queries and commits prefixes.
"""
import argparse
import json
import os
from pathlib import Path
import re
import time

from vfm_grid_python.common import both, either, check, dump, states
from vfm_grid_python.geometry import build
from vfm_grid_python.scenarios.roads import inventory
from vfm_grid_python.concrete.observations import road_mapping
from vfm_grid_python.concrete.symbolic import skeleton
from vfm_grid_python.benchmark_timing import DEADLINE, JOURNAL, PHASE, totals
from . import abstract
from .monitor import selected
from .interval_domain import IntervalDomain


def render(config):
    """May abstraction: speed/position bounds, but hidden acceleration.

Reuses declarations/topology/observer from the queue compiler, replacing its
endpoint-to-endpoint transfer requirement and adding interval observations.
The inherited same-road order and road-exchange restrictions also occur in
the shared concrete model. Actual connector lengths are used.
"""
    roads, joints = inventory(config)
    geometry = build(skeleton(config), config['section_length'],
                     config['parking_length'], config['junction_radius'])
    mapping = road_mapping(config, geometry)
    text = abstract.render(config)
    for road in roads:
        r = road['id']
        if road['kind'] == 'connection':
            text = re.sub(rf' length_{r} := \d+;',
                          f' length_{r} := {mapping[r]["length"]};', text)
    maximum = max(config['section_length'], config['parking_length'],
                  *(m['length'] for m in mapping.values()))
    gap = config['queue_clearance']
    speed = int(config['vfm']['MAXSPEEDNONEGO'])
    extra = []
    for i in range(config['cars']):
        old = abstract.transfer_constraint(roads, i, config['queue_transfer_encoding'])
        if old not in text:
            raise ValueError('queue compiler transfer interface changed')
        text = text.replace(old, '')
        extra += [f'VAR interval_{i} : 0..{1+(maximum-1+gap-1)//gap};',
                  f'DEFINE ilen_{i} := case ' + ' '.join(
                      f'road_{i} = {r["id"]} : length_{r["id"]};' for r in roads) + ' TRUE : 1; esac;',
                  f'DEFINE lo_{i} := case interval_{i} = 0 : 0; interval_{i} = 1 : ilen_{i}; TRUE : 1 + (interval_{i} - 2)*{gap}; esac;',
                  f'DEFINE hi_{i} := case interval_{i} < 2 : lo_{i}; TRUE : min(lo_{i} + {gap-1}, ilen_{i} - 1); esac;',
                  f'INVAR lo_{i} <= hi_{i} & lo_{i} >= 0;',
                  f'INVAR (tip_{i} = left <-> interval_{i} = 0) & (tip_{i} = right <-> interval_{i} = 1);']
        moves = [f'road_{i} = next(road_{i}) & next(lo_{i}) - hi_{i} <= {speed} & lo_{i} - next(hi_{i}) <= {speed}']
        for a in roads:
            for b in roads:
                if a['kind'] == b['kind']:
                    continue
                for j in set(a['joints']) & set(b['joints']):
                    da = f'lo_{i}' if a['joints'][0] == j else f'(ilen_{i} - hi_{i})'
                    db = f'next(lo_{i})' if b['joints'][0] == j else f'(next(ilen_{i}) - next(hi_{i}))'
                    moves.append(f'road_{i} = {a["id"]} & next(road_{i}) = {b["id"]} & {da}+{db} <= {speed}')
        extra.append('TRANS '+either(moves)+';')
        for j in range(i):
            extra += [f'INVAR road_{i} = road_{j} -> interval_{i} != interval_{j};',
                      f'INVAR road_{i} = road_{j} -> ((rank_{i} < rank_{j}) <-> (lo_{i} < lo_{j}));',
                      f'INVAR road_{i} = road_{j} & rank_{i} < rank_{j} -> hi_{j} - lo_{i} >= {gap};',
                      f'INVAR road_{i} = road_{j} & rank_{j} < rank_{i} -> hi_{i} - lo_{j} >= {gap};']
    return text+'\n'+'\n'.join(extra)+'\n'


def milestone(frame, config, mapping):
    terms = []
    for i in range(config['cars']):
        road = int(frame[f'road_{i}'])
        cell = int(frame[f'interval_{i}'])
        domain = IntervalDomain(mapping[road]['length'], config['queue_clearance'])
        # SMV uses 0/1 for left/right, then increasing interior bins.
        index = 0 if cell == 0 else len(domain.intervals)-1 if cell == 1 else cell-1
        lo, hi = domain.bounds(index)
        terms += [f'road_{i} = {road}', f'qpos_{i} >= {lo}', f'qpos_{i} <= {hi}',
                  f'rank_{i} = {int(frame[f"rank_{i}"])}']
    terms += [f'scenario_phase = {int(frame["scenario_phase"])}']
    return both(terms)


def refinement(frames, milestone_builder=milestone):
    def compile(config, graph, geometry):
        mapping = road_mapping(config, geometry)
        predicates = [milestone_builder(f, config, mapping) for f in frames]
        lines = [f'VAR refinement_phase : 0..{len(frames)-1};',
                 'INIT refinement_phase = 0;', 'INIT '+predicates[0]+';']
        # Freeze the chosen symbolic roles/starts/goals, not only topology.
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


def certify(output, trace, last):
    """Commit segments of a validated full suffix, with exact full-frame joins."""
    frames = states(trace)
    cuts = [0]
    for k in range(1, last+1):
        cuts.append(next(t for t in range(cuts[-1]+1, len(frames))
                         if int(frames[t]['refinement_phase']) == k))
    if int(frames[-1]['scenario_phase']) < 1:
        raise ValueError('nonaccepting certificate')
    segments = output/'committed_segments'
    segments.mkdir()
    joined = [frames[0]]
    for k, (a, b) in enumerate(zip(cuts, cuts[1:])):
        fragment = frames[a:b+1]
        if joined[-1] != fragment[0]:
            raise ValueError('full-state boundary mismatch')
        dump(segments/f'{k:04d}.json', fragment)
        joined.extend(fragment[1:])
    # Preserve any final accepting suffix after the last milestone.
    joined.extend(frames[cuts[-1]+1:])
    if joined != frames:
        raise ValueError('concatenation changed the certified execution')
    dump(output/'certificate.json', dict(kind='full_suffix_witness_reuse',
         segments=len(cuts)-1, full_state_joins=True, validated_source=str(trace),
         note='Full monitor solve counted; no local-query speedup claimed.'))


def run(config, output, mode='monitor', abstract_bound=100, concrete_bound=300, timeout=2000,
        *, abstraction='interval', lookahead='ordinary'):
    if mode not in ('monitor', 'certified', 'sequential'):
        raise ValueError(mode)
    if abstraction not in ('interval', 'paper_cells', 'endpoint_proof'):
        raise ValueError(abstraction)
    if lookahead not in ('ordinary', 'stopped', 'canonical-stopped'):
        raise ValueError(lookahead)
    if lookahead != 'ordinary' and (mode != 'sequential' or abstraction != 'paper_cells'):
        raise ValueError('stopped lookahead requires paper_cells sequential mode')
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    saved = {k: os.environ.get(k) for k in (DEADLINE, JOURNAL, PHASE)}
    os.environ[DEADLINE] = str(min(float(saved[DEADLINE] or 'inf'),started+timeout))
    os.environ[JOURNAL] = str(output/'timing.jsonl')
    result = dict(status='error', semantics={'interval': 'interval-may.v1',
                  'paper_cells': 'paper-cells.v1', 'endpoint_proof': 'endpoint-contract.v1'}[abstraction], mode=mode,
                  abstraction=abstraction, lookahead=lookahead)
    try:
        a = output/'abstract'; a.mkdir(); dump(a/'config.json', config)
        renderer, milestone_builder = render, milestone
        if abstraction == 'paper_cells':
            from . import paper_cells
            renderer, milestone_builder = paper_cells.render, paper_cells.milestone
            result['assumptions'] = paper_cells.assumptions(config)
            dump(output/'assumptions.json', result['assumptions'])
        elif abstraction == 'endpoint_proof':
            from . import endpoint_proof
            renderer = endpoint_proof.render
            result['assumptions'] = endpoint_proof.assumptions(config)
            dump(output/'assumptions.json', result['assumptions'])
        model = a/f'{abstraction}.smv'; model.write_text(renderer(config))
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
            # Refine only the accepting prefix, not padding in the fixed bound.
            accepted = str(len(config['scenario_stages']))
            first = next(k for k,f in enumerate(frames) if f['scenario_phase'] == accepted)
            frames = frames[:first+1]
        result['abstract_states'] = len(frames)
        if abstraction == 'paper_cells':
            errors = paper_cells.validate_trace(frames, config)
            dump(a/'validation.json', dict(valid=not errors, errors=errors))
            if errors:
                raise ValueError(f'invalid paper-cell witness: {errors[:3]}')
        elif abstraction == 'endpoint_proof':
            errors = endpoint_proof.validate_trace(frames, config)
            dump(a/'validation.json', dict(valid=not errors, errors=errors))
            if errors:
                raise ValueError(f'invalid endpoint witness: {errors[:3]}')
        layout = selected(config, frames[0])
        if config.get('profile') == 'paper':
            from vfm_grid_python.scenarios import paper
            _, joints = inventory(config)
            layout['joint_assignments'] = {z:int(frames[0][z]) for z in paper.names(config)}
            layout.update(starts=[joints[j]['node'] for j in layout['starts']],
                          goals=[joints[j]['node'] for j in layout['goals']],
                          roles={n:joints[j]['node'] for n,j in layout['roles'].items()})
        dump(a/'selected_graph.json', layout)
        os.environ[PHASE] = 'concrete'
        from vfm_grid_python.concrete.compiler import run as concrete
        contract = (endpoint_proof.refinement(frames) if abstraction == 'endpoint_proof'
                    else refinement(frames, milestone_builder))
        c = concrete(config, layout, output/'concrete', concrete_bound, timeout,
                     refinement=contract, generate_only=mode=='sequential')
        if mode=='sequential' and c['status']=='generated':
            from .interval_sequential import run as sequential
            c=sequential(output/'concrete',output/'segments',concrete_bound,timeout,
                         lookahead=lookahead, abstract_trace=a/'trace.xml')
        result.update(concrete=c, status=c['status'], trace=c.get('trace'))
        if c['status'] == 'witness' and mode == 'certified':
            certify(output, c['trace'], len(frames)-1)
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
    p.add_argument('--mode', choices=['monitor', 'certified', 'sequential'], default='monitor')
    p.add_argument('--abstraction', choices=['interval', 'paper_cells', 'endpoint_proof'], default='interval')
    p.add_argument('--timeout', type=float, default=2000)
    p.add_argument('--abstract-bound', type=int, default=100)
    p.add_argument('--concrete-bound', type=int, default=300)
    p.add_argument('--lookahead', choices=['ordinary','stopped','canonical-stopped'], default='ordinary')
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
    print(json.dumps(run(normalize(config), args.output_dir,
                         args.mode, args.abstract_bound, args.concrete_bound, args.timeout,
                         abstraction=args.abstraction, lookahead=args.lookahead), indent=2))
