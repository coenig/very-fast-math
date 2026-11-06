"""Positive staged fleet scenarios, with symbolic topology and unordered queues."""
from .benchmark_scenarios import at, conjunction
from .common import grid_edges, either


def configuration(family, scale):
    rows, columns = scale['grid']
    cars = scale['cars']
    if family not in ('arrival_rush', 'priority_passage', 'bay_turnover'):
        raise ValueError(f'unknown fleet congestion family: {family}')
    if not 4 <= cars <= 8 or min(rows, columns) < 3:
        raise ValueError('use 4..8 cars and grid dimensions at least 3')
    roles = dict(gate='N', arm_a='N', arm_b='N')
    if family == 'priority_passage':
        roles.update(exit='N', bay='P')
    if rows * columns - cars < sum(v == 'N' for v in roles.values()):
        raise ValueError('insufficient ordinary nodes for feeder roads')
    a, b = list(range(cars // 2)), list(range(cars // 2, cars))
    c = dict(rows=rows, columns=columns, cars=cars, parkings=cars,
             roles=roles, distinct_starts=False, distinct_goals=True,
             requires_common_parking_component=True,
             section_length=20, parking_length=7, junction_radius=6,
             connection_length=9, queue_clearance=6,
             vfm=dict(MAXSPEEDNONEGO=5, LANES_MIN_SPEEDS='@(-5)@', LANES_MAX_SPEEDS='@(5)@'),
             initial_constraints=[], scenario_stages=[])
    constraints = c['initial_constraints']
    names = list(roles)
    constraints.extend(f'role_{x} != role_{y}' for i,x in enumerate(names) for y in names[:i])
    edges = grid_edges(rows, columns)

    def adjacent(x, y):
        return either([f'edge_{u}_{v} & {x} = {s} & {y} = {t}'
                       for u,v in edges for s,t in ((u,v),(v,u))])

    def queue(group, arm):
        return {'queue': dict(cars=group, between=['role_gate', 'role_'+arm], ordered=False)}

    constraints.extend(adjacent('role_gate', 'role_'+arm) for arm in ('arm_a','arm_b'))
    stages = c['scenario_stages']
    stages.append(dict(event=conjunction(queue(a,'arm_a'), queue(b,'arm_b'))))
    if family == 'priority_passage':
        # A unique symbolic edge crosses a column cut. The rest of the grid is free.
        constraints.extend([
            f'role_gate / {columns} = role_exit / {columns}',
            f'role_exit mod {columns} = (role_gate mod {columns}) + 1',
            adjacent('role_gate','role_exit'),
            adjacent('role_gate','role_bay'),
            f'role_bay = goal_{b[0]}',
        ])
        for name in ('arm_a', 'arm_b', 'bay'):
            constraints.append(f'(role_{name} mod {columns}) <= (role_gate mod {columns})')
        for u,v in edges:
            if v == u+1:
                constraints.append(f'({u % columns} = (role_gate mod {columns})) -> '
                                   f'(!edge_{u}_{v} | (role_gate = {u} & role_exit = {v}))')
        waiting = at(a[0], 'role_bay')
        stages.append(dict(event=conjunction(waiting, at(b[0], 'role_gate'))))
        stages.append(dict(event=conjunction(waiting, at(b[0], 'role_exit')), hold=waiting))
    elif family == 'bay_turnover':
        stages.append(dict(event=conjunction(
            *(at(i, f'goal_{j}') for i,j in zip(a,b)), queue(b,'arm_b'))))
    stages.append(dict(event=conjunction(*(at(i,f'goal_{i}') for i in range(cars)))))
    return c
