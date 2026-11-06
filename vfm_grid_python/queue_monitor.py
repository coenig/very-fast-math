"""Whole-trace VFM monitor refinement of ordered-queue witnesses.

Retains road identity, endpoint/interior flags and order for EVERY car. Uses
native road-wise VFM dynamics and collisions, not whole-section reservations.
No greedy sequentialization or completeness claim is made.
"""
from __future__ import annotations

import argparse
import json
from itertools import combinations
from pathlib import Path
import re
import sys
import time

from .common import ROOT, both, either, dump, execute, check, states
from .geometry import build
from .queue_abstract import inventory, queue_observer, validate_trace as validate_queue


def selected(config, first):
    from .common import grid_edges
    return dict(rows=config['rows'], columns=config['columns'],
                nodes=[dict(id=u, row=u//config['columns'], column=u%config['columns'],
                            kind='P' if first[f'parking_{u}'] == 'TRUE' else 'N')
                       for u in range(config['rows']*config['columns'])],
                edges=[list(e) for e in grid_edges(config['rows'], config['columns'])
                       if first[f'edge_{e[0]}_{e[1]}'] == 'TRUE'],
                starts=[int(first[f'start_{i}']) for i in range(config['cars'])],
                goals=[int(first[f'goal_{i}']) for i in range(config['cars'])],
                roles={name: int(first[f'role_{name}']) for name in config.get('roles', {})},
                # Used only for fleet size by the common generation worker.
                milestones=[[int(first[f'start_{i}']) for i in range(config['cars'])]])


def road_mapping(config, geometry):
    """Map candidate IDs to actual selected roads, accounting for reversals."""
    roads, joints = inventory(config)
    sections = {frozenset(s['edge']): s for s in geometry['sections']}
    mapping = {}
    for road in roads:
        if road['kind'] == 'section':
            s = sections.get(frozenset(road['edge']))
            if s:
                mapping[road['id']] = dict(kind='section', id=s['id'], length=s['length'],
                                          flipped=s['edge'] != road['edge'], joints=road['joints'])
        else:
            a,b = [sections.get(frozenset((joints[j]['node'], joints[j]['neighbour'])))
                   for j in road['joints']]
            if a and b:
                c = next(c for c in geometry['connections'] if {c['from'],c['to']} == {a['id'],b['id']})
                mapping[road['id']] = dict(kind='connection', id=c['id'],
                                          source=c['from'], target=c['to'], length=c['length'],
                                          flipped=c['from'] != a['id'], joints=road['joints'])
    return mapping


def occupied(car, item):
    p = f'env.veh___6{car}9___'
    if item['kind'] == 'section':
        return f'{p}.on_straight_section = {item["id"]}'
    return f'{p}.traversion_from = {item["source"]} & {p}.traversion_to = {item["target"]}'


def milestone(tick, mapping):
    terms = []
    for i, car in enumerate(tick):
        r = car['road']
        if r not in mapping:
            raise ValueError(f'queue witness uses disabled road {r}')
        terms += [f'road_{i} = {r}', f'rank_{i} = {car["rank"]}']
        terms += [f'qpos_{i} = 0' if car['tip'] == 'left' else
                  f'qpos_{i} = {mapping[r]["length"]}' if car['tip'] == 'right' else
                  f'qpos_{i} > 0 & qpos_{i} < {mapping[r]["length"]}']
    return both(terms)


def packing_check(config, ticks, mapping):
    gap = config.get('queue_clearance', 6)
    for k,tick in enumerate(ticks):
        for r in {c['road'] for c in tick}:
            cars = [c for c in tick if c['road'] == r]
            required = (len(cars)-1)*gap + (not any(c['tip']=='left' for c in cars)) + (not any(c['tip']=='right' for c in cars))
            if required > mapping[r]['length']:
                raise ValueError(f'milestone {k}, road {r}: queue requires {required}m, actual geometry has {mapping[r]["length"]}m')


def observations(config, graph, geometry):
    """Concrete road/rank/joint observations shared with fixed-topology search."""
    mapping = road_mapping(config, geometry)
    _, joints = inventory(config)
    cars = config['cars']
    lines = ['-- Queue monitor: native concrete dynamics, no whole-road reservations.', 'DEFINE']
    for prefix in ('start','goal'):
        lines += [f' {prefix}_{i} := {v};' for i,v in enumerate(graph[prefix+'s'])]
    lines += [f' role_{name} := {v};' for name,v in graph['roles'].items()]
    for i in range(cars):
        p = f'env.veh___6{i}9___'
        lines += [f' road_{i} := case']
        lines += [f' {occupied(i,m)} : {r};' for r,m in mapping.items()]
        lines += [' TRUE : -1; esac;', f' qpos_{i} := case']
        lines += [f' road_{i} = {r} : ' + (f'{m["length"]} - {p}.abs_pos' if m['flipped'] else f'{p}.abs_pos') + ';' for r,m in mapping.items()]
        lines += [' TRUE : -1; esac;', f' loc_{i} := case']
        for r,m in mapping.items():
            if m['kind'] != 'section':
                continue
            for j,pos in zip(m['joints'], (0,m['length'])):
                node = joints[j]['node']
                stop = f' & {p}.v = 0' if graph['nodes'][node]['kind'] == 'P' else ''
                lines += [f' road_{i} = {r} & qpos_{i} = {pos}{stop} : {node};']
        lines += [' TRUE : -1; esac;', f' joint_{i} := case']
        for r,m in mapping.items():
            for j,pos in zip(m['joints'], (0,m['length'])):
                lines += [f' road_{i} = {r} & qpos_{i} = {pos} : {j};']
        lines += [' TRUE : -1; esac;']
        terms = [f'(case road_{j} = road_{i} & qpos_{j} < qpos_{i} : 1; TRUE : 0; esac)' for j in range(cars) if j != i]
        lines += [f' rank_{i} := ' + (' + '.join(terms) or '0') + ';']
    return lines


def fleet_safety(config):
    lines = []
    for i,j in combinations(range(config['cars']), 2):
        lines += [f'INVAR road_{i} = road_{j} -> abs(qpos_{i} - qpos_{j}) >= {config.get("queue_clearance",6)};',
                  f'INVAR joint_{i} >= 0 -> joint_{i} != joint_{j};',
                  f'TRANS !(road_{i} != road_{j} & next(road_{i}) = road_{j} & next(road_{j}) = road_{i});']
    return lines


def render(config, graph, geometry, ticks, abstract_states):
    mapping = road_mapping(config, geometry)
    packing_check(config, ticks, mapping)
    cars, last = config['cars'], len(ticks)-1
    lines = observations(config, graph, geometry)
    lines += [f'VAR refinement_phase : 0..{last};', 'INIT refinement_phase = 0;',
              f'INIT {milestone(ticks[0], mapping)};']
    for i in range(cars):
        lines += [f'INIT env.veh___6{i}9___.v = 0 & env.veh___6{i}9___.a = 0;']
        lines += [f'INVAR road_{i} >= 0;']
    options = ['next(refinement_phase) = refinement_phase']
    for k in range(last):
        target = milestone(ticks[k+1], mapping)
        if config.get('scenario_stages'):
            target = both([target, f'scenario_phase = {abstract_states[k+1]["scenario_phase"]}'])
        options += [f'refinement_phase = {k} & next(refinement_phase) = {k+1} & next({target})']
    lines += ['TRANS ' + either(options) + ';']
    for k,tick in enumerate(ticks):
        target = ticks[min(k+1,last)]
        for i,(a,b) in enumerate(zip(tick,target)):
            lines += [f'INVAR refinement_phase = {k} -> (road_{i} = {a["road"]} | road_{i} = {b["road"]});']
            if a['road'] == b['road'] and a['tip'] == b['tip'] and a['tip'] != 'interior':
                pos = 0 if a['tip'] == 'left' else mapping[a['road']]['length']
                lines += [f'INVAR refinement_phase = {k} -> (qpos_{i} = {pos} & env.veh___6{i}9___.v = 0);']
    lines += fleet_safety(config)
    if config.get('scenario_stages'):
        lines += [queue_observer(config), f'INIT scenario_phase = {abstract_states[0]["scenario_phase"]};']
        goal = f'refinement_phase = {last} & scenario_accepted'
    else:
        goal = both([f'refinement_phase = {last}'] + [f'loc_{i} = goal_{i}' for i in range(cars)])
    lines += [f'INVARSPEC !({goal});']
    return '\n'.join(lines)+'\n'


def validate_observations(config, geometry, ticks, abstract_states, frames):
    """Numerically verify milestones/order from raw VFM fields, not DEFINEs."""
    mapping = road_mapping(config, geometry)
    previous_phase = 0
    milestone_ticks = []
    for t,frame in enumerate(frames):
        phase = int(frame['refinement_phase'])
        if (t == 0 and phase != 0) or (t and phase not in (previous_phase, previous_phase+1)):
            raise ValueError(f'tick {t}: illegal monitor phase')
        decoded = []
        for i in range(config['cars']):
            p = f'env.veh___6{i}9___'
            on = int(frame[p+'.on_straight_section'])
            from_, to = int(frame[p+'.traversion_from']), int(frame[p+'.traversion_to'])
            r,m = next((r,m) for r,m in mapping.items() if
                       (m['kind'] == 'section' and on == m['id']) or
                       (m['kind'] == 'connection' and on == -1 and (from_,to) == (m['source'],m['target'])))
            raw = int(frame[p+'.abs_pos'])
            pos = m['length']-raw if m['flipped'] else raw
            if not 0 <= pos <= m['length']:
                raise ValueError(f'tick {t}: invalid position')
            tip = 'left' if pos == 0 else 'right' if pos == m['length'] else 'interior'
            if r not in (ticks[phase][i]['road'], ticks[min(phase+1,len(ticks)-1)][i]['road']):
                raise ValueError(f'tick {t}: car {i} outside selected route')
            decoded.append(dict(road=r, pos=pos, tip=tip))
        for c in decoded:
            c['rank'] = sum(d['road'] == c['road'] and d['pos'] < c['pos'] for d in decoded)
        for a,b in combinations(decoded,2):
            if a['road'] == b['road'] and abs(a['pos']-b['pos']) < config.get('queue_clearance',6):
                raise ValueError(f'tick {t}: queue separation violated')
        if t == 0 or phase != previous_phase:
            milestone_ticks.append(t)
            for i,(actual, expected) in enumerate(zip(decoded,ticks[phase])):
                if any(actual[k] != expected[k] for k in ('road','rank','tip')):
                    raise ValueError(f'tick {t}: incorrect queue milestone for car {i}')
            if config.get('scenario_stages') and frame['scenario_phase'] != abstract_states[phase]['scenario_phase']:
                raise ValueError(f'tick {t}: scenario phase differs from abstract milestone')
        previous_phase = phase
    if not frames or previous_phase != len(ticks)-1:
        raise ValueError('queue monitor did not finish')
    if config.get('scenario_stages') and int(frames[-1]['scenario_phase']) != len(config['scenario_stages']):
        raise ValueError('scenario was not accepted')
    return dict(valid=True, milestone_ticks=milestone_ticks,
                checked='raw VFM road IDs, canonical coordinates, rank, tip, separation and phases')


def run(source, output, bound=200, timeout=120, template=ROOT/'main_template.json',
        radius=None, max_speed=5, gif=False):
    source, output = Path(source).resolve(), Path(output).resolve()
    config = json.loads((source/'config.json').read_text())
    radius = config.get('junction_radius', 6) if radius is None else radius
    abstract_states = states(source/'trace.xml')
    errors = validate_queue(config, abstract_states)
    if errors:
        raise ValueError(f'invalid queue witness: {errors[:3]}')
    # Decode from XML, never trust a stale/edited sidecar.
    ticks = [[dict(road=int(s[f'road_{i}']), rank=int(s[f'rank_{i}']), tip=s[f'tip_{i}'])
              for i in range(config['cars'])] for s in abstract_states]
    graph = selected(config, abstract_states[0])
    geometry = build(graph, config.get('section_length',20), config.get('parking_length',7), radius)
    text = render(config, graph, geometry, ticks, abstract_states)
    output.mkdir(parents=True, exist_ok=False)
    dump(output/'geometry.json', geometry)
    dump(output/'selected_graph.json', graph)
    dump(output/'config.json', config)
    dump(output/'road_mapping.json', road_mapping(config, geometry))
    (output/'queue_monitor.smv').write_text(text)
    from .braking import with_speed, effective_parameters
    concrete_config = with_speed(config, max_speed)
    params = effective_parameters(template, concrete_config.get('vfm', {}))
    if config.get('queue_clearance',6) <= params['VEHICLE_LENGTH']:
        raise ValueError('queue clearance must exceed actual VFM vehicle length')
    dump(output/'generation_job.json', dict(graph=graph, geometry=geometry,
         template_config=str(Path(template).resolve()), vfm=concrete_config.get('vfm',{}),
         baseline_only=True))
    from .benchmark_timing import JOURNAL, PHASE, totals
    import os
    saved = {key: os.environ.get(key) for key in (JOURNAL, PHASE)}
    os.environ[JOURNAL], os.environ[PHASE] = str(output/'timing.jsonl'), 'concrete'
    started = time.monotonic()
    result = {}
    try:
        generation = execute([sys.executable,'-m','vfm_grid_python.concretize','--worker',str(output)],
                             ROOT, output/'generation.log', timeout)
        dump(output/'generation_result.json', generation)
        if generation['status'] != 'finished':
            result = dict(status='generation_'+generation['status'])
            return result
        base = Path(json.loads((output/'model.json').read_text())['model'])
        main, count = re.subn(r'(?m)^INVARSPEC FALSE;?\s*$', lambda _: text, base.read_text())
        if count != 1:
            raise ValueError('cannot replace baseline property')
        model = base.with_name('main_queue_monitor.smv')
        model.write_text(main)
        dump(output/'model.json', dict(model=str(model)))
        result = check(model, bound, timeout)
        if result['status'] == 'witness':
            from .sequential import global_inventory, export_trace
            frames = states(model.parent/'trace.xml')
            inv = global_inventory(graph, geometry, concrete_config, output/'inventory', template, timeout, model=model)
            xml = export_trace(frames, geometry, output, True, inv,
                               description='VFM queue-monitor concrete witness')
            observed = validate_observations(config, geometry, ticks, abstract_states, states(xml))
            # Full TraceModel validation above checks the generated transition
            # relation, including the monitor, queue observations and collisions.
            result.update(trace=str(xml), states=len(frames), validated=True,
                          collision_scope='native same-road separation/order and endpoint exclusion; not swept geometric collision')
            dump(output/'validation.json', dict(**observed, full_model=True,
                 monitor='queue road/order/endpoint-interior milestones',
                 cross_element_swept_geometry_checked=False))
            if gif:
                from .visualize import concrete_plot
                try:
                    concrete_plot(geometry, output, xml, True)
                except Exception as exc:
                    # A renderer failure must not invalidate a checked witness.
                    result['render_error'] = str(exc)
    except Exception as exc:
        result = dict(result, status='validation_error' if result.get('status') == 'witness' else 'error', error=str(exc))
        raise
    finally:
        result.update(wall_seconds=time.monotonic()-started,
                      timings_seconds=totals(output/'timing.jsonl', time.monotonic()))
        dump(output/'result.json', result)
        for key, value in saved.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('abstract_run', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--bound', type=int, default=200)
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--template-config', type=Path, default=ROOT/'main_template.json')
    parser.add_argument('--junction-radius', type=int, help='default: config junction_radius, or 6')
    parser.add_argument('--max-speed', type=int, default=5)
    parser.add_argument('--gif', action='store_true')
    args = parser.parse_args()
    result = run(args.abstract_run,args.output_dir,args.bound,args.timeout,
                 args.template_config,args.junction_radius,args.max_speed,args.gif)
    print(json.dumps(result, indent=2))
    return 0 if result['status'] == 'witness' else 2


if __name__ == '__main__':
    raise SystemExit(main())
