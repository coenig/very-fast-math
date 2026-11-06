"""Opt-in shared junctions with explicit occupied section endpoints.

The archived ``independent_corridors`` policy reserves whole road elements.
The ``endpoint`` policy reserves only milestone endpoints and excludes obvious
head-on/blocked-entry moves. It delegates timing to concrete model checking,
using native same-seclet distance/order rules, not cross-element geometry.
The ``endpoint_turns`` policy additionally permits changing endpoints within a
junction. Its staged predicates use endpoint-only observations and conservative
pending-condition guards. None certifies arbitrary dynamic handoffs or coverage.
"""
from functools import lru_cache
from itertools import combinations_with_replacement
import math

from .common import both, either, grid_edges

DIRECTIONS = ('north', 'east', 'south', 'west')
DELTA = {'north': (-1, 0), 'east': (0, 1), 'south': (1, 0), 'west': (0, -1)}
OPPOSITE = dict(zip(DIRECTIONS, ('south', 'west', 'north', 'east')))
DEFAULT = dict(road_length=20, parking_length=7, junction_radius=16,
               footprint_radius=6)


def policy(config):
    """Archived runs keep their original conservative collision semantics."""
    value = config.get('junction_policy', 'independent_corridors')
    if value not in ('independent_corridors', 'endpoint', 'endpoint_turns'):
        raise ValueError('junction_policy must be independent_corridors, endpoint or endpoint_turns')
    return value


def native_collisions(config):
    return policy(config) in ('endpoint', 'endpoint_turns')


def enabled(config):
    return config.get('junction_extension', False) is not False


def profile(config):
    policy(config)
    value = config.get('junction_extension', False)
    if value is False:
        return None
    if value is not True and not isinstance(value, dict):
        raise ValueError('junction_extension must be true, false, or a geometry profile')
    if config.get('semantics', 'legacy') != 'legacy':
        raise ValueError('junction_extension cannot be combined with stable_concurrent')
    values = {} if value is True else value
    if values.keys() - DEFAULT.keys():
        raise ValueError('unknown junction_extension profile field')
    result = {**DEFAULT, **values}
    if any(type(v) is not int or v < 1 for v in result.values()):
        raise ValueError('junction profile values must be positive integer metres')
    if result['parking_length'] > result['road_length']:
        raise ValueError('parking_length must not exceed road_length')
    if result['footprint_radius'] < 6:
        raise ValueError('footprint_radius must be at least 6m for the supported vehicle profile')
    return result


def direction(nodes, source, target):
    a, b = nodes[source], nodes[target]
    delta = (b['row'] - a['row'], b['column'] - a['column'])
    for name, change in DELTA.items():
        if delta == change:
            return name
    raise ValueError(f'not neighbouring nodes: {source}, {target}')


def _box(points):
    return (min(p[0] for p in points), min(p[1] for p in points),
            max(p[0] for p in points), max(p[1] for p in points))


def _distance(a, b):
    return math.hypot(max(a[0]-b[2], b[0]-a[2], 0),
                      max(a[1]-b[3], b[1]-a[3], 0))


def _boxes(curve, depth=4):
    """Enclose the entire curve, not sampled points, using de Casteljau hulls."""
    if len(curve) == 2 or depth == 0:
        return (_box(curve),)
    levels = [curve]
    while len(levels[-1]) > 1:
        levels.append(tuple(tuple((a[k]+b[k])/2 for k in (0, 1))
                            for a, b in zip(levels[-1], levels[-1][1:])))
    left = tuple(level[0] for level in levels)
    right = tuple(level[-1] for level in reversed(levels))
    return _boxes(left, depth-1) + _boxes(right, depth-1)


def separated(a, b, radius):
    """A sufficient clearance test; uncertain/touching cases are conflicts."""
    clearance = 2*radius + 1e-6
    if _distance(_box(a), _box(b)) > clearance:
        return True
    return all(_distance(x, y) > clearance for x in _boxes(a) for y in _boxes(b))


def elements(geometry):
    return {**{('s', s['id']): (s['source'], s['drain']) for s in geometry['sections']},
            **{('c', c['from'], c['to']): c['curve'] for c in geometry['connections']}}


@lru_cache(maxsize=32)
def _conflicts(curves, radius):
    return tuple((a, b) for a, b in combinations_with_replacement(range(len(curves)), 2)
                 if not separated(curves[a], curves[b], radius))


def conflicts(geometry):
    items = elements(geometry)
    names = list(items)
    curves = tuple(tuple(tuple(p) for p in curve) for curve in items.values())
    radius = profile(geometry)['footprint_radius']
    return [(names[a], names[b]) for a, b in _conflicts(curves, radius)]


@lru_cache(maxsize=16)
def candidate_geometry(rows, columns, road, parking, radius, footprint):
    from .geometry import build
    graph = dict(nodes=[dict(id=u, row=u//columns, column=u % columns, kind='N')
                        for u in range(rows*columns)], edges=grid_edges(rows, columns))
    geometry = build(graph, road, parking, radius)
    geometry['junction_extension'] = dict(road_length=road, parking_length=parking,
                                         junction_radius=radius, footprint_radius=footprint)
    return geometry


def abstract_geometry(config):
    p = profile(config)
    return candidate_geometry(config['rows'], config['columns'], *(p[k] for k in DEFAULT))


def resource_name(resource):
    return '_'.join(map(str, resource))


def pair_guards(geometry, cars, definitions, *, swept=False):
    lines = []
    for i, expressions in enumerate(definitions):
        for resource, expr in expressions.items():
            lines.append(f'DEFINE occupied_{resource_name(resource)}_{i} := {expr};')
    for i in range(cars):
        for j in range(i):
            for a, b in conflicts(geometry):
                for x, y in ((a, b), (b, a)) if a != b else ((a, b),):
                    if x not in definitions[i] or y not in definitions[j]:
                        continue
                    xi, yj = f'occupied_{resource_name(x)}_{i}', f'occupied_{resource_name(y)}_{j}'
                    lines.append(f'INVAR !({xi} & {yj});')
                    if swept:
                        lines.append(f'TRANS !(({xi} | next({xi})) & ({yj} | next({yj})));')
    return '\n'.join(lines) + '\n'


def abstract_guards(config):
    g = abstract_geometry(config)
    nodes = {n['id']: n for n in g['nodes']}
    cars = config['cars']
    lines, definitions = [], []
    for i in range(cars):
        p, m, q = f'approach_{i}', f'move_{i}', f'loc_{i}'
        lines += [f'VAR {p} : {{north, east, south, west}};',
                  f'VAR {m} : {{hold, north, east, south, west}};']
        if 'start_approaches' in config:
            lines.append(f'INIT {p} = {config["start_approaches"][i]};')
        valid, moves, uses = [], [f'{m} = hold & next({q}) = {q} & next({p}) = {p}'], {}
        for s in g['sections']:
            u, v = s['edge']
            on, travelling = [], []
            for a, b in ((u, v), (v, u)):
                d = direction(nodes, a, b)
                on.append(f'{q} = {a} & {p} = {d}')
                travelling.append(f'{q} = {a} & {m} = {d}')
                moves.append(f'edge_{min(u,v)}_{max(u,v)} & {q} = {a} & {m} = {d} & '
                             f'next({q}) = {b} & next({p}) = {OPPOSITE[d]}')
                if policy(config) == 'endpoint_turns':
                    # Approach validity guarantees the incoming edge; this
                    # guard guarantees the outgoing edge. A turn traverses
                    # the connector, not the entire outgoing road.
                    moves.append(f'edge_{min(u,v)}_{max(u,v)} & {q} = {a} & {m} = {d} & '
                                 f'{p} != {d} & next({q}) = {q} & next({p}) = {d}')
            valid.append(f'edge_{min(u,v)}_{max(u,v)} & {either(on)}')
            uses['s', s['id']] = either(on + travelling)
        for c in g['connections']:
            u = c['node']
            a, b = (next(v for v in g['sections'][s]['edge'] if v != u)
                    for s in (c['from'], c['to']))
            da, db = direction(nodes, u, a), direction(nodes, u, b)
            uses['c', c['from'], c['to']] = (
                f'{q} = {u} & (({p} = {da} & {m} = {db}) | ({p} = {db} & {m} = {da}))')
        lines += [f'INVAR {either(valid)};', f'TRANS {either(moves)};']
        definitions.append(uses)
    if native_collisions(config):
        for i in range(cars):
            for j in range(i):
                same = f'loc_{i} = loc_{j}'
                lines.append(f'INVAR !({same} & approach_{i} = approach_{j});')
                # Node no-swap alone does not catch reversed turns by two
                # cars starting at different approaches of the SAME node.
                lines.append(f'INVAR !({same} & move_{i} != hold & move_{j} != hold & '
                             f'approach_{i} = move_{j} & approach_{j} = move_{i} & '
                             f'approach_{i} != approach_{j});')
                for a,b in ((i,j),(j,i)):
                    # A new entrant cannot pass a car retaining the entry
                    # endpoint of that section. Opposite-end waits are legal.
                    lines.append(f'INVAR !({same} & move_{a} != hold & move_{b} = hold & '
                                 f'move_{a} = approach_{b});')
    else:
        lines.append(pair_guards(g, cars, definitions))
    return '\n'.join(lines) + '\n'


def attach(geometry, graph):
    p = profile(graph)
    for key in ('road_length', 'parking_length', 'junction_radius'):
        if geometry[key] != p[key]:
            raise ValueError(f'junction profile requires --{key.replace("_", "-")} {p[key]}')
    geometry['junction_extension'] = p
    geometry['junction_policy'] = policy(graph)


def check_vehicle_profile(geometry, parameters, width):
    if not 0 < parameters['VEHICLE_LENGTH'] <= 5 or not 0 < int(width) <= 200:
        raise ValueError('junction extension supports vehicles up to 5m long and 2m wide')


def section_for(geometry, node, approach):
    nodes = {n['id']: n for n in geometry['nodes']}
    for s in geometry['sections']:
        if node in s['edge']:
            other = next(n for n in s['edge'] if n != node)
            if direction(nodes, node, other) == approach:
                return s
    raise ValueError(f'no enabled {approach} approach at node {node}')


def observe(geometry, car, node, approach, stopped=False):
    s = section_for(geometry, node, approach)
    pos = s['length'] if s['edge'][1] == node else 0
    p = f'env.veh___6{car}9___'
    terms = [f'{p}.on_straight_section = {s["id"]}', f'{p}.abs_pos = {pos}']
    if stopped or next(n['kind'] for n in geometry['nodes'] if n['id'] == node) == 'P':
        terms.append(f'{p}.v = 0')
    return both(terms)


def route(geometry, node, approach, target, target_approach=None):
    incoming = section_for(geometry, node, approach)['id']
    result = {('s', incoming)}
    if node != target or (target_approach is not None and target_approach != approach):
        outgoing = (section_for(geometry, target, target_approach)['id'] if node == target else
                    next(s['id'] for s in geometry['sections'] if set(s['edge']) == {node, target}))
        result.add(('s', outgoing))
        if incoming != outgoing:
            c = next(c for c in geometry['connections'] if c['node'] == node and
                     {c['from'], c['to']} == {incoming, outgoing})
            result.add(('c', c['from'], c['to']))
    return result


def occupied(car, resource):
    p = f'env.veh___6{car}9___'
    if resource[0] == 's':
        return f'{p}.on_straight_section = {resource[1]}'
    return f'{p}.traversion_from = {resource[1]} & {p}.traversion_to = {resource[2]}'


def holding_cars(graph, milestone):
    """Holds preserve BOTH node and attachment; terminal moves are holds.

New runs persist this schedule so a truncated diagnostic prefix does not turn
its final (possibly moving) handoff into an artificial terminal hold.
"""
    if 'stationary_hold_cars' in graph:
        return graph['stationary_hold_cars'][milestone]
    following = min(milestone + 1, len(graph['milestones']) - 1)
    return [c for c, node in enumerate(graph['milestones'][milestone])
            if node == graph['milestones'][following][c]
            and graph['approaches'][milestone][c] == graph['approaches'][following][c]]


def enable_stationary_holds(graph):
    """Version the sketch-aligned policy on newly generated concrete runs."""
    source = {k: v for k, v in graph.items() if k != 'stationary_hold_cars'}
    schedule = [holding_cars(source, k) for k in range(len(graph['milestones']))]
    graph.update(stationary_holds=True, stationary_hold_cars=schedule)


def resting_at(geometry, car, node, approach):
    # v=0 alone does not ensure standstill: the next displacement uses v+a.
    return both([observe(geometry, car, node, approach, stopped=True),
                 f'env.veh___6{car}9___.a = 0'])


def support(geometry, car, node, approach, target, target_approach=None,
            stationary_holds=False):
    if stationary_holds and node == target and target_approach in (None, approach):
        return resting_at(geometry, car, node, approach)
    return either([occupied(car, r) for r in sorted(route(geometry, node, approach, target, target_approach))])


def collisions(geometry, cars):
    if native_collisions(geometry):
        return '-- Endpoint policy: retain native same-seclet distance/order checks; no whole-element reservations.\n'
    definitions = []
    for car in range(cars):
        allowed = set(geometry.get('car_sections', [range(len(geometry['sections']))]*cars)[car])
        definitions.append({r: occupied(car, r) for r in elements(geometry)
                            if set(r[1:]) <= allowed})
    return pair_guards(geometry, cars, definitions, swept=True)


def validate_witness(graph, geometry=None):
    from .geometry import build
    p = profile(graph)
    geometry = geometry or build(graph, p['road_length'], p['parking_length'], p['junction_radius'])
    milestones, approaches = graph['milestones'], graph.get('approaches')
    cars = len(milestones[0]) if milestones else 0
    if not cars or approaches is None or len(approaches) != len(milestones):
        raise ValueError('shared-junction witness lacks approach history')
    nodes = {n['id']: n for n in geometry['nodes']}
    native = native_collisions(graph)
    if policy(geometry) != policy(graph):
        raise ValueError('graph and geometry have different junction policies')
    bad = set() if native else {frozenset(pair) for pair in conflicts(geometry)}
    for k, (state, ports) in enumerate(zip(milestones, approaches)):
        if len(state) != cars or len(ports) != cars:
            raise ValueError('inconsistent fleet/approach history')
        target = milestones[min(k+1, len(milestones)-1)]
        next_ports = approaches[min(k+1, len(milestones)-1)]
        if len(next_ports) != cars:
            raise ValueError('inconsistent fleet/approach history')
        for u, v, port, arrival in zip(state, target, ports, next_ports):
            if u != v and arrival != direction(nodes, v, u):
                raise ValueError(f'inconsistent arrival approach at abstract step {k+1}')
            if u == v and port != arrival and policy(graph) != 'endpoint_turns':
                raise ValueError(f'inconsistent arrival approach at abstract step {k+1}')
        routes = [route(geometry, u, port, v, arrival)
                  for u, port, v, arrival in zip(state, ports, target, next_ports)]
        for i in range(cars):
            for j in range(i):
                if state[i] != state[j] and target[i] == state[j] and target[j] == state[i]:
                    raise ValueError(f'node swap at abstract step {k}: cars {j}, {i}')
                if native:
                    if state[i] == state[j]:
                        if ports[i] == ports[j]:
                            raise ValueError(f'duplicate endpoint at abstract step {k}: cars {j}, {i}')
                        moves = {c: (next_ports[c] if ports[c] != next_ports[c] else 'hold')
                                 if state[c] == target[c] else direction(nodes,state[c],target[c])
                                 for c in (i,j)}
                        if moves[i] == ports[j] and moves[j] == ports[i]:
                            raise ValueError(f'reversed connector at abstract step {k}: cars {j}, {i}')
                        if any(moves[a] == ports[b] and moves[b] == 'hold' for a,b in ((i,j),(j,i))):
                            raise ValueError(f'blocked entry endpoint at abstract step {k}: cars {j}, {i}')
                elif any(frozenset((a, b)) in bad for a in routes[i] for b in routes[j]):
                    raise ValueError(f'conflicting junction corridors at abstract step {k}: cars {j}, {i}')
    return dict(valid=True, policy=('endpoint-turns.v1' if policy(graph) == 'endpoint_turns' else
                                   'endpoint-occupancy.v1' if native else 'independent-junction-corridors.v1'),
                milestones=len(milestones), cars=cars,
                collision_scope='native same-seclet checks' if native else 'disjoint geometric corridors',
                greedy_dynamic_completeness_claimed=False)


def monitor(graph, geometry):
    validate_witness(graph, geometry)
    milestones, ports = graph['milestones'], graph['approaches']
    strict = graph.get('stationary_holds', False)
    def target(k):
        holds = holding_cars(graph, k) if strict else []
        terms = [(resting_at if c in holds else observe)(geometry, c, n, ports[k][c])
                 for c, n in enumerate(milestones[k])]
        if graph.get('scenario_phases') is not None:
            terms.append(f'scenario_phase = {graph["scenario_phases"][k]}')
        return both(terms)
    lines = [f'VAR refinement_phase : 0..{len(milestones)-1};',
             'ASSIGN init(refinement_phase) := 0;', 'next(refinement_phase) := case']
    lines += [f'refinement_phase = {k} & next({target(k+1)}) : {k+1};'
              for k in range(len(milestones)-1)]
    lines += ['TRUE : refinement_phase; esac;',
              f'INVARSPEC refinement_phase != {len(milestones)-1};']
    lines += ['INIT '+(resting_at(geometry, c, n, ports[0][c]) if strict else
                      observe(geometry, c, n, ports[0][c], stopped=True))+';'
              for c, n in enumerate(milestones[0])]
    if graph.get('scenario_phases') is not None:
        lines.append(f'INIT scenario_phase = {graph["scenario_phases"][0]};')
    for k, state in enumerate(milestones):
        next_k = min(k+1, len(milestones)-1)
        destination = milestones[next_k]
        lines += [f'INVAR refinement_phase = {k} -> {support(geometry,c,n,ports[k][c],destination[c],ports[next_k][c],stationary_holds=strict)};'
                  for c, n in enumerate(state)]
    lines.append(collisions(geometry, len(milestones[0])))
    return '\n'.join(lines) + '\n'
