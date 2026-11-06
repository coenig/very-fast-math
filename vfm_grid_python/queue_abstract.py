"""Experimental ordered-road queues, deliberately separate from the endpoint model.

Each car stores a road, its dense rank on that road, and left/interior/right.
Roads include BOTH sections and junction connections. No metric velocity state
is stored. This is a search experiment, not a proven refinement contract: its
witnesses must NOT be passed to the existing endpoint sequentializer.
"""
from __future__ import annotations

import argparse
from itertools import combinations
import json
from pathlib import Path
import time

from .common import both, either, grid_edges, dump, check, states
from .benchmark_scenarios import predicate as endpoint_predicate


def queue_predicate(term, config):
    """Positive event language plus an explicitly shared, optionally ordered road.

    queue: {cars: [0,1,2], near: role_gate, kind: section, ordered: true}
    means those cars occupy the SAME road incident to the role's node. It does
    not assert a metric distance to the junction or a velocity. Rank is measured
    in the inventory's canonical direction, not in the direction of travel.
    """
    if type(term) is bool:
        return 'TRUE' if term else 'FALSE'
    if not isinstance(term, dict) or len(term) != 1:
        raise ValueError('invalid predicate')
    op, value = next(iter(term.items()))
    if op in ('all', 'any'):
        return (both if op == 'all' else either)([queue_predicate(x, config) for x in value])
    if op == 'at':
        return endpoint_predicate(term, config)
    if op == 'attachment':
        if not isinstance(value, list) or len(value) != 3:
            raise ValueError('attachment requires [car, node, neighbour]')
        car, node, neighbour = value
        endpoint_predicate({'at': [car,node]}, config)
        endpoint_predicate({'at': [car,neighbour]}, config)
        roads, joints = inventory(config)
        return either([both([f'road_{car} = {r["id"]}', f'joint_{car} = {j}',
                             f'{node} = {joints[j]["node"]}', f'{neighbour} = {joints[j]["neighbour"]}'])
                       for r in roads if r['kind'] == 'section' for j in r['joints']])
    if op != 'queue':
        raise ValueError('queue experiment supports positive At/queue events/holds only')
    group = value.get('cars', [])
    if (not isinstance(group, list) or len(group) < 1 or
            any(type(c) is not int or not 0 <= c < config['cars'] for c in group) or
            len(set(group)) != len(group)):
        raise ValueError('a queue requires a nonempty list of distinct valid cars')
    kind = value.get('kind', 'section')
    if kind not in ('section', 'connection', 'any'):
        raise ValueError('queue kind must be section, connection or any')
    if type(value.get('ordered', False)) is not bool:
        raise ValueError('queue ordered must be boolean')
    near = value.get('near')
    if near is not None:
        endpoint_predicate({'at': [group[0], near]}, config)  # validate rigid role
    between = value.get('between')
    if between is not None:
        if not isinstance(between,list) or len(between) != 2 or kind != 'section':
            raise ValueError('queue between requires two node roles and kind section')
        for role in between:
            endpoint_predicate({'at': [group[0],role]},config)
    roads, joints = inventory(config)
    places = []
    for road in roads:
        if kind != 'any' and road['kind'] != kind:
            continue
        nodes = sorted({joints[j]['node'] for j in road['joints']})
        binding = 'TRUE'
        if between:
            u,v = road['edge']
            a,b = between
            binding = f'(({a} = {u} & {b} = {v}) | ({a} = {v} & {b} = {u}))'
        places.append(both([f"road_{group[0]} = {road['id']}", binding,
                            either([f'{near} = {u}' for u in nodes]) if near else 'TRUE']))
    terms = [either(places)] + [f'road_{c} = road_{group[0]}' for c in group[1:]]
    if value.get('ordered', False):
        terms += [f'rank_{a} < rank_{b}' for a,b in zip(group, group[1:])]
    return both(terms)


def queue_observer(config):
    """Same nondeterministic event/hold protocol as the benchmark observer."""
    stages = config['scenario_stages']
    expr = lambda t: queue_predicate(t, config)
    lines = [f'VAR scenario_phase : 0..{len(stages)};',
             f'INIT (scenario_phase = 0 & ({expr(stages[0].get("hold", True))})) | '
             f'(scenario_phase = 1 & ({expr(stages[0]["event"])}));']
    options = [f'scenario_phase = {len(stages)} & next(scenario_phase) = scenario_phase']
    for index, stage in enumerate(stages):
        hold = expr(stage.get('hold', True))
        options += [f'scenario_phase = {index} & ({hold}) & next(scenario_phase) = {index}',
                    f'scenario_phase = {index} & ({hold}) & next(scenario_phase) = {index+1} & next({expr(stage["event"])})']
    return '\n'.join(lines + ['TRANS ' + either(options) + ';',
                              f'DEFINE scenario_accepted := scenario_phase = {len(stages)};'])


def capacity(length, clearance):
    """Integer reference positions in [0,L], separated by >= clearance.

    For the formal strict distance > LEN rule use clearance=LEN+1. This is
    NOT a claim about bumper-based physical capacity or vehicle end margins.
    """
    if type(length) is not int or length < 1:
        raise ValueError('road lengths must be positive integers')
    if type(clearance) is not int or clearance < 1:
        raise ValueError('clearance must be a positive integer')
    return length // clearance + 1


def threshold_capacity(count, left, right, length, gap, cars):
    """Exact integer packing test, simplified using the fleet-size bound.

    For q>0 and m empty ends, (q-1)*gap+m <= length iff
    q <= 1+floor((length-m)/gap). Empty queues are always legal.
    The three nested bounds cover m=0,1,2 without dynamic multiplication.
    """
    capacity(length, gap)  # Validate integer input before constant folding.
    limits = [max(0, min(cars, 1+(length-m)//gap)) for m in range(3)]
    terms = []
    if limits[0] < cars:
        terms.append(f'{count} <= {limits[0]}')
    if limits[1] < limits[0]:
        terms.append(f'!({left} & {right}) -> {count} <= {limits[1]}')
    if limits[2] < limits[1]:
        terms.append(f'!({left} | {right}) -> {count} <= {limits[2]}')
    return both(terms) if terms else 'TRUE'


def inventory(config):
    edges = grid_edges(config['rows'], config['columns'])
    roads, joints = [], []
    for u, v in edges:
        pair = [len(joints), len(joints)+1]
        joints.extend([dict(node=u, neighbour=v), dict(node=v, neighbour=u)])
        roads.append(dict(id=len(roads), kind='section', joints=pair,
                          active=f'edge_{u}_{v}', edge=[u, v]))
    for node in range(config['rows'] * config['columns']):
        attached = [(r, j) for r in roads if r['kind'] == 'section'
                    for j in r['joints'] if joints[j]['node'] == node]
        for (a, ja), (b, jb) in combinations(attached, 2):
            roads.append(dict(id=len(roads), kind='connection', joints=[ja, jb],
                              active=f"({a['active']} & {b['active']})"))
    return roads, joints


def transfer_constraint(roads, car, encoding='explicit'):
    """Equivalent transfer encodings; all retain the same road/rank/tip state."""
    i=car
    stay=f'next(road_{i}) = road_{i}'
    if encoding=='joint':
        n=sum(r['kind']=='section' for r in roads)
        # Sections precede connectors in inventory(). A transfer is possible
        # exactly when opposite kinds meet at the same occupied attachment.
        return f'TRANS ({stay}) | (joint_{i} >= 0 & next(joint_{i}) = joint_{i} & ((road_{i} < {n}) != (next(road_{i}) < {n})));'
    if encoding not in ('explicit','grouped'):
        raise ValueError('queue_transfer_encoding must be explicit, joint or grouped')
    transfers=[stay]
    for a in roads:
        destinations=[]
        for b in roads:
            if a['kind']==b['kind']: continue
            for joint in set(a['joints']) & set(b['joints']):
                side_a=('left','right')[a['joints'].index(joint)]
                side_b=('left','right')[b['joints'].index(joint)]
                move=f"tip_{i} = {side_a} & next(road_{i}) = {b['id']} & next(tip_{i}) = {side_b}"
                if encoding=='explicit':
                    transfers.append(f"road_{i} = {a['id']} & {move}")
                else: destinations.append(move)
        if destinations: transfers.append(f"road_{i} = {a['id']} & {either(destinations)}")
    return 'TRANS '+either(transfers)+';'


def render(config):
    """Compile the existing event/hold AST, with At true only at section ends.

    Topology, roles, starts and goals remain symbolic unless explicitly fixed.
    Raw constraints referring to loc_i see -1 while the car is in an interior
    or on a connection. This experiment accepts the positive event/hold subset.
    """
    for key in ('rows', 'columns', 'cars'):
        if type(config.get(key)) is not int or config[key] < 1:
            raise ValueError(f'{key} must be a positive integer')
    if config['rows'] * config['columns'] < 2:
        raise ValueError('at least two cells required')
    if config.get('desired_ltl') or config.get('junction_extension') or config.get('semantics'):
        raise ValueError('queue experiment accepts event/hold scenarios, not other motion profiles or raw LTL')
    for stage in config.get('scenario_stages', []):
        queue_predicate(stage['event'], config)
        queue_predicate(stage.get('hold', True), config)
    roads, joints = inventory(config)
    n, cars = config['rows'] * config['columns'], config['cars']
    edges = grid_edges(config['rows'], config['columns'])
    gap = config.get('queue_clearance', 6)
    section_length = config.get('section_length', 20)
    parking_length = config.get('parking_length', 7)
    connection_length = config.get('connection_length', 12)
    for length in (section_length, parking_length, connection_length):
        capacity(length, gap)
    capacity_encoding = config.get('queue_capacity_encoding', 'arithmetic')
    if capacity_encoding not in ('arithmetic', 'threshold'):
        raise ValueError('queue_capacity_encoding must be arithmetic or threshold')
    lines = ['MODULE main', 'FROZENVAR']
    lines += [f' edge_{u}_{v} : boolean;' for u, v in edges]
    lines += [f' parking_{u} : boolean;' for u in range(n)]
    for prefix in ('start', 'goal'):
        lines += [f' {prefix}_{i} : 0..{n-1};' for i in range(cars)]
    lines += [f' role_{name} : 0..{n-1};' for name in config.get('roles', {})]
    lines += ['VAR']
    for i in range(cars):
        lines += [f' road_{i} : 0..{len(roads)-1};',
                  f' rank_{i} : 0..{cars-1};', f' tip_{i} : {{left, interior, right}};']
    lines += ['DEFINE']
    count = lambda terms: ' + '.join(f'(case {x} : 1; TRUE : 0; esac)' for x in terms) or '0'
    lines += [' parking_count := ' + count([f'parking_{u}' for u in range(n)]) + ';']
    for u in range(n):
        lines += [f' degree_{u} := ' + count([f'edge_{a}_{b}' for a, b in edges if u in (a,b)]) + ';']
    for road in roads:
        r = road['id']
        length = str(connection_length)
        if road['kind'] == 'section':
            u, v = road['edge']
            length = f'case parking_{u} | parking_{v} : {parking_length}; TRUE : {section_length}; esac'
        lines += [f" active_{r} := {road['active']};", f' length_{r} := {length};',
                  f' count_{r} := ' + count([f'road_{i} = {r}' for i in range(cars)]) + ';']
        for tip in ('left', 'right'):
            lines += [f' occupied_{r}_{tip} := ' + either([f'road_{i} = {r} & tip_{i} = {tip}' for i in range(cars)]) + ';']
    for i in range(cars):
        lines += [f' loc_{i} := case']
        for road in roads:
            if road['kind'] == 'section':
                for tip, j in zip(('left', 'right'), road['joints']):
                    lines += [f" road_{i} = {road['id']} & tip_{i} = {tip} : {joints[j]['node']};"]
        lines += [' TRUE : -1; esac;']
        lines += [f' joint_{i} := case']
        for road in roads:
            for tip, j in zip(('left', 'right'), road['joints']):
                lines += [f" road_{i} = {road['id']} & tip_{i} = {tip} : {j};"]
        lines += [' TRUE : -1; esac;']
        c = config['columns']
        lines += [f' distance_to_goal_{i} := abs(start_{i} / {c} - goal_{i} / {c}) + abs(start_{i} mod {c} - goal_{i} mod {c});']
    lines += [' all_parked := ' + both([f'loc_{i} = goal_{i}' for i in range(cars)]) + ';']
    for name in config.get('roles', {}):
        lines += [f' role_{name}_degree := case ' + ' '.join(f'role_{name} = {u} : degree_{u};' for u in range(n)) + ' TRUE : 0; esac;']
    for u in range(n):
        lines += [f'INVAR parking_{u} -> degree_{u} = 1;']
    for u,v in edges:
        lines += [f'INVAR edge_{u}_{v} -> !(parking_{u} & parking_{v});']
    if 'parkings' in config:
        lines += [f"INIT parking_count = {config['parkings']};"]
    for u, kind in config.get('labels', {}).items():
        lines += [f"INIT parking_{u} = {'TRUE' if kind == 'P' else 'FALSE'};"]
    if 'edges' in config:
        fixed = {tuple(sorted(e)) for e in config['edges']}
        if not fixed <= set(edges):
            raise ValueError('fixed edges must be grid neighbours')
        lines += [f"INIT edge_{u}_{v} = {'TRUE' if (u,v) in fixed else 'FALSE'};" for u,v in edges]
    def label(target, kind):
        if kind == 'any':
            return 'TRUE'
        return either([f"{target} = {u} & {'!' if kind == 'N' else ''}parking_{u}" for u in range(n)])
    for name, kind in config.get('roles', {}).items():
        lines += [f'INIT {label("role_"+name, kind)};']
    for i in range(cars):
        lines += [f'INIT loc_{i} = start_{i};', f'INIT {label(f"goal_{i}", "P")};',
                  f'INIT {label(f"start_{i}", config.get("start_types", ["N"]*cars)[i])};']
        for key, prefix in (('starts','start'), ('goals','goal')):
            if key in config:
                lines += [f'INIT {prefix}_{i} = {config[key][i]};']
        for j in range(i):
            if config.get('distinct_starts', True):
                lines += [f'INIT start_{i} != start_{j};']
            if config.get('distinct_goals', True):
                lines += [f'INIT goal_{i} != goal_{j};']
    for road in roads:
        r = road['id']
        # Existence of integer ordered positions with these endpoint flags.
        # If an extreme is interior, reserve one integer unit at that end.
        if capacity_encoding == 'arithmetic':
            lines += [f'INVAR count_{r} > 0 -> ((count_{r} - 1)*{gap} + '
                      f'(case occupied_{r}_left : 0; TRUE : 1; esac) + '
                      f'(case occupied_{r}_right : 0; TRUE : 1; esac) <= length_{r});']
        else:
            bound = lambda length: threshold_capacity(f'count_{r}', f'occupied_{r}_left',
                                                       f'occupied_{r}_right', length, gap, cars)
            if road['kind'] == 'section':
                u, v = road['edge']
                p, normal = bound(parking_length), bound(section_length)
                if p == normal:
                    if p != 'TRUE': lines.append(f'INVAR {p};')
                else:
                    if p != 'TRUE': lines.append(f'INVAR (parking_{u} | parking_{v}) -> {p};')
                    if normal != 'TRUE': lines.append(f'INVAR !(parking_{u} | parking_{v}) -> {normal};')
            else:
                constraint = bound(connection_length)
                if constraint != 'TRUE': lines.append(f'INVAR {constraint};')
        for i in range(cars):
            lines += [f'INVAR road_{i} = {r} -> active_{r} & rank_{i} < count_{r};',
                      f'INVAR road_{i} = {r} & tip_{i} = left -> rank_{i} = 0;',
                      f'INVAR road_{i} = {r} & tip_{i} = right -> rank_{i} = count_{r} - 1;']
    for i in range(cars):
        lines += [transfer_constraint(roads,i,config.get('queue_transfer_encoding','explicit'))]
        for j in range(i):
            lines += [f'INVAR road_{i} = road_{j} -> rank_{i} != rank_{j};',
                      f'INVAR joint_{i} >= 0 -> joint_{i} != joint_{j};',
                      f'TRANS road_{i} = road_{j} & next(road_{i}) = road_{i} & next(road_{j}) = road_{j} -> '
                      f'((rank_{i} < rank_{j}) <-> (next(rank_{i}) < next(rank_{j})));',
                      f'TRANS !(road_{i} != road_{j} & next(road_{i}) = road_{j} & next(road_{j}) = road_{i});']
    lines += ['INIT ' + x + ';' for x in config.get('initial_constraints', [])]
    lines += ['INVAR ' + x + ';' for x in config.get('constraints', [])]
    if config.get('scenario_stages'):
        lines += [queue_observer(config), 'INVARSPEC !scenario_accepted;']
    else:
        lines += ['INVARSPEC !all_parked;']
    return '\n'.join(lines) + '\n'


def validate_trace(config, trace):
    """Independent checks of queue invariants, transfers and event observations.

    This validates the experimental discrete semantics, NOT concrete dynamics.
    Arbitrary textual INIT/INVAR clauses are left to the model checker.
    """
    roads, joints = inventory(config)
    errors = []
    gap = config.get('queue_clearance', 6)
    def fail(t, reason):
        errors.append(f'tick {t}: {reason}')
    def endpoint(car):
        return -1 if car['tip'] == 'interior' else roads[car['road']]['joints'][('left','right').index(car['tip'])]
    def loc(car):
        j = endpoint(car)
        return joints[j]['node'] if j >= 0 and roads[car['road']]['kind'] == 'section' else -1
    def event(term, cars, state):
        if type(term) is bool:
            return term
        op, value = next(iter(term.items()))
        if op == 'at':
            return loc(cars[value[0]]) == int(state[value[1]])
        if op == 'attachment':
            c,node,neighbour = value
            j = endpoint(cars[c])
            return (j >= 0 and roads[cars[c]['road']]['kind'] == 'section' and
                    joints[j]['node'] == int(state[node]) and
                    joints[j]['neighbour'] == int(state[neighbour]))
        if op == 'queue':
            group = [cars[c] for c in value['cars']]
            road = roads[group[0]['road']]
            return (all(c['road'] == road['id'] for c in group) and
                    value.get('kind', 'section') in ('any', road['kind']) and
                    ('near' not in value or int(state[value['near']]) in
                     {joints[j]['node'] for j in road['joints']}) and
                    ('between' not in value or (road['kind'] == 'section' and
                     {int(state[x]) for x in value['between']} == set(road['edge']))) and
                    (not value.get('ordered', False) or
                     all(a['rank'] < b['rank'] for a,b in zip(group, group[1:]))))
        return (all if op == 'all' else any)(event(x, cars, state) for x in value)
    previous = None
    first = trace[0] if trace else {}
    if not trace:
        return ['empty trace']
    for t, state in enumerate(trace):
        cars = [dict(road=int(state[f'road_{i}']), rank=int(state[f'rank_{i}']),
                     tip=state[f'tip_{i}']) for i in range(config['cars'])]
        for name, value in first.items():
            if name.startswith(('edge_', 'parking_', 'start_', 'goal_', 'role_')) and state[name] != value:
                fail(t, f'frozen value changed: {name}')
        for road in roads:
            occupants = [c for c in cars if c['road'] == road['id']]
            if not occupants:
                continue
            active_edges = [road['edge']] if road['kind'] == 'section' else [
                sorted((joints[j]['node'], joints[j]['neighbour'])) for j in road['joints']]
            if any(state[f'edge_{u}_{v}'] != 'TRUE' for u,v in active_edges):
                fail(t, 'occupied disabled road')
            if sorted(c['rank'] for c in occupants) != list(range(len(occupants))):
                fail(t, 'ranks are not dense and unique')
            length = config.get('connection_length', 12)
            if road['kind'] == 'section':
                length = config.get('parking_length', 7) if any(state[f'parking_{u}'] == 'TRUE' for u in road['edge']) else config.get('section_length', 20)
            left, right = any(c['tip'] == 'left' for c in occupants), any(c['tip'] == 'right' for c in occupants)
            if (len(occupants)-1)*gap + (not left) + (not right) > length:
                fail(t, 'capacity/endpoint margin exceeded')
            for c in occupants:
                if c['tip'] == 'left' and c['rank'] != 0 or c['tip'] == 'right' and c['rank'] != len(occupants)-1:
                    fail(t, 'endpoint occupant is not an extreme of queue')
        occupied_joints = [endpoint(c) for c in cars if endpoint(c) >= 0]
        if len(set(occupied_joints)) != len(occupied_joints):
            fail(t, 'joint multiply occupied')
        if previous is None:
            if any(loc(c) != int(state[f'start_{i}']) for i,c in enumerate(cars)):
                fail(t, 'incorrect initial location')
        else:
            for a,b in zip(previous, cars):
                if a['road'] != b['road'] and (endpoint(a) < 0 or endpoint(a) != endpoint(b) or roads[a['road']]['kind'] == roads[b['road']]['kind']):
                    fail(t, 'illegal transfer')
            for i,j in combinations(range(len(cars)), 2):
                a,b,an,bn = previous[i],previous[j],cars[i],cars[j]
                if a['road'] == b['road'] == an['road'] == bn['road'] and (a['rank'] < b['rank']) != (an['rank'] < bn['rank']):
                    fail(t, 'overtaking')
                if a['road'] != b['road'] and an['road'] == b['road'] and bn['road'] == a['road']:
                    fail(t, 'opposite transfer')
        stages = config.get('scenario_stages', [])
        if stages:
            phase = int(state['scenario_phase'])
            if t == 0:
                if not ((phase == 0 and event(stages[0].get('hold', True), cars, state)) or
                        (phase == 1 and event(stages[0]['event'], cars, state))):
                    fail(t, 'invalid observer initialization')
            else:
                old_phase = int(trace[t-1]['scenario_phase'])
                if old_phase == len(stages):
                    if phase != old_phase:
                        fail(t, 'accepted observer changed')
                elif not (phase in (old_phase, old_phase+1) and
                          event(stages[old_phase].get('hold', True), previous, trace[t-1]) and
                          (phase == old_phase or event(stages[old_phase]['event'], cars, state))):
                    fail(t, 'event/hold observer violation')
        previous = cars
    if config.get('scenario_stages'):
        if int(trace[-1]['scenario_phase']) != len(config['scenario_stages']):
            errors.append('final scenario not accepted')
    elif any(loc(c) != int(trace[-1][f'goal_{i}']) for i,c in enumerate(previous)):
        errors.append('not all cars at goals')
    return errors


def run(config, output, bound=100, timeout=120):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    model = output/'queue.smv'
    from .benchmark_timing import interval
    with interval('generation', model=str(model)):
        model.write_text(render(config))
        roads, joints = inventory(config)
        dump(output/'config.json', config)
        dump(output/'inventory.json', dict(roads=roads, joints=joints))
    generation = time.monotonic()-started
    backend = config.get('queue_solver', 'smt')
    result = check(model, bound, timeout, backend=backend)
    result.update(generation_seconds=generation, model_bytes=model.stat().st_size,
                  candidate_roads=len(roads), concretized=False,
                  solver_backend=backend,
                  semantics='experimental_ordered_queues')
    if result['status'] == 'witness':
        trace = states(output/'trace.xml')
        errors = validate_trace(config, trace)
        dump(output/'validation.json', dict(semantics='experimental queues only', errors=errors))
        result['queue_validated'] = not errors
        if errors:
            result['status'] = 'validation_error'
        result['states'] = len(trace)
        decoded = [[dict(road=int(s[f'road_{i}']), rank=int(s[f'rank_{i}']),
                         tip=s[f'tip_{i}']) for i in range(config['cars'])] for s in trace]
        dump(output/'queue_trace.json', decoded)
        result['max_road_occupancy'] = max(sum(c['road'] == r for c in tick)
                                         for tick in decoded for r in {c['road'] for c in tick})
        result['max_connection_occupancy'] = max([sum(c['road'] == r['id'] for c in tick)
                                                 for tick in decoded for r in roads
                                                 if r['kind'] == 'connection'] or [0])
    dump(output/'result.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--bound', type=int, default=100)
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--solver', choices=('smt', 'sat'),
                        help='override the queue-search backend (default: config, then smt)')
    parser.add_argument('--transfer-encoding', choices=('explicit', 'joint', 'grouped'),
                        help='equivalent transfer syntax (default: config, then explicit)')
    parser.add_argument('--capacity-encoding', choices=('arithmetic', 'threshold'),
                        help='equivalent packing syntax (default: config, then arithmetic)')
    parser.add_argument('--fixed-from', type=Path,
                        help='diagnostic: freeze topology and roles from an endpoint selected_graph.json')
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    if args.solver:
        config['queue_solver'] = args.solver
    if args.transfer_encoding:
        config['queue_transfer_encoding'] = args.transfer_encoding
    if args.capacity_encoding:
        config['queue_capacity_encoding'] = args.capacity_encoding
    if args.fixed_from:
        graph = json.loads(args.fixed_from.read_text())
        if (graph['rows'], graph['columns'], len(graph['starts'])) != (config['rows'], config['columns'], config['cars']):
            raise ValueError('selected graph dimensions/cars do not match config')
        config.update(edges=graph['edges'], starts=graph['starts'], goals=graph['goals'],
                      labels={str(n['id']): n['kind'] for n in graph['nodes']})
        config['initial_constraints'] = config.get('initial_constraints', []) + [
            f'role_{name} = {value}' for name, value in graph.get('roles', {}).items()]
        config['fixed_from'] = str(args.fixed_from.resolve())
    return 0 if run(config, args.output_dir,
                   args.bound, args.timeout)['status'] == 'witness' else 2


if __name__ == '__main__':
    raise SystemExit(main())
