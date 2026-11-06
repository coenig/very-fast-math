"""Scenario atoms; concrete and abstract compilers supply loc observations."""
from vfm_grid_python.common import both, either

def at(car, target):
    return {"at": [car, target]}


def conjunction(*terms):
    return {"all": list(terms)}


def negate(term):
    return {"not": term}


def predicate(term, config):
    if type(term) is bool:
        return "TRUE" if term else "FALSE"
    if not isinstance(term, dict) or len(term) != 1:
        raise ValueError(f"invalid scenario predicate: {term!r}")
    op, value = next(iter(term.items()))
    if op in ("all", "any"):
        return (both if op == "all" else either)(
            [predicate(t, config, geometry, graph) for t in value])
    if op == "not":
        return "!(" + predicate(value, config, geometry, graph) + ")"
    if op != "at" or len(value) != 2:
        raise ValueError(f"unknown scenario predicate: {op}")
    car, target = value
    if type(car) is not int or not 0 <= car < config["cars"]:
        raise ValueError("scenario refers to an unknown car")
    targets = ({f"start_{i}" for i in range(config["cars"])} |
               {f"goal_{i}" for i in range(config["cars"])} |
               {f"role_{name}" for name in config.get("roles", {})})
    if target not in targets:
        raise ValueError(f"unknown symbolic location: {target}")
    return f"loc_{car} = {target}"
    if target.startswith("role_"):
        node = graph["roles"][target[5:]]
    else:
        kind, index = target.split("_")
        node = graph[kind + "s"][int(index)]
    if node not in {n["id"] for n in geometry["nodes"]}:
        return "FALSE"

from .roads import inventory
endpoint_predicate = predicate

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
