"""Shared event/hold scenario language (independent of concrete tick duration).

An observer may choose when to consume an event. While waiting, its hold
predicate must remain true. Consecutive events occupy distinct states. This is
existential ordered-until semantics, not a greedy first-match automaton.
"""
from __future__ import annotations

from .common import both, either, grid_edges


def at(car, target):
    return {"at": [car, target]}


def conjunction(*terms):
    return {"all": list(terms)}


def negate(term):
    return {"not": term}


def predicate(term, config, geometry=None, graph=None):
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
    if geometry is None:
        return f"loc_{car} = {target}"
    if target.startswith("role_"):
        node = graph["roles"][target[5:]]
    else:
        kind, index = target.split("_")
        node = graph[kind + "s"][int(index)]
    if node not in {n["id"] for n in geometry["nodes"]}:
        return "FALSE"
    from .geometry import location_expression
    if config.get('semantics') == 'stable_concurrent':
        from .stable import observation
        return observation(geometry,car,node,graph)
    return location_expression(geometry, car, node)


def observer(config, geometry=None, graph=None, boundary='TRUE'):
    stages = config["scenario_stages"]
    if not stages:
        raise ValueError("scenario requires at least one event")
    expr = lambda term: predicate(term, config, geometry, graph)
    lines = [f"VAR scenario_phase : 0..{len(stages)};"]
    first = expr(stages[0]["event"])
    hold = expr(stages[0].get("hold", True))
    lines += [f"INIT (scenario_phase = 0 & ({hold})) | (scenario_phase = 1 & ({first}));"]
    options = [f"scenario_phase = {len(stages)} & next(scenario_phase) = scenario_phase"]
    for index, stage in enumerate(stages):
        event = expr(stage["event"])
        hold = expr(stage.get("hold", True))
        options += [f"scenario_phase = {index} & ({hold}) & next(scenario_phase) = {index}",
                    f"scenario_phase = {index} & ({hold}) & ({boundary}) & next(scenario_phase) = {index+1} & next({event})"]
    lines += ["TRANS " + either(options) + ";",
              f"DEFINE scenario_accepted := scenario_phase = {len(stages)};"]
    return "\n".join(lines) + "\n"


FAMILIES = (
    "parking", "ordered_arrivals", "shared_gate", "two_shared_gates", "shared_bay",
    "foreign_yield", "fleet_yield", "row_reversal", "row_to_column", "row_to_column_reversed",
    "column_to_row", "parking_rotation",
    "bottleneck_exchange", "bottleneck_yield", "bottleneck_rotation",
)


def scenario(family, rows, columns, cars, repetitions=1, parkings=None, parking_span=None):
    """Construct symbolic scenarios; no cells or edges are fixed by a family."""
    if family not in FAMILIES:
        raise ValueError(f"unknown family: {family}")
    if type(repetitions) is not int or repetitions < 1:
        raise ValueError("repetitions must be a positive integer")
    if family not in ("parking", "ordered_arrivals") and cars < 2:
        raise ValueError(f"{family} requires at least two cars")
    if family == "two_shared_gates" and cars < 4:
        raise ValueError("two_shared_gates requires at least four cars")
    if family.startswith('bottleneck_') and columns < 2:
        raise ValueError('bottleneck scenarios need at least two columns')
    if parking_span is not None and (type(parking_span) is not int or parking_span < 1):
        raise ValueError('parking_span must be a positive integer')
    if family in ('row_to_column','row_to_column_reversed','column_to_row') and cars==rows==columns:
        raise ValueError('full start and goal lines intersect: a node cannot be both a normal start and a parking goal')
    config = dict(rows=rows, columns=columns, cars=cars,
                  parkings=parkings if parkings is not None else cars + (family in ("foreign_yield", "fleet_yield", "shared_bay", "bottleneck_yield")),
                  roles={}, initial_constraints=[], scenario_stages=[],
                  vfm={"LANES_MIN_SPEEDS": "@(-5)@", "LANES_MAX_SPEEDS": "@(5)@", "MAXSPEEDNONEGO": 5})
    constraints, stages, roles = config["initial_constraints"], config["scenario_stages"], config["roles"]
    def event(term, hold=True):
        stages.append(dict(event=term, hold=hold))
    def goals(offset=0):
        return conjunction(*(at(i, f"goal_{(i+offset)%cars}") for i in range(cars)))
    def starts():
        return conjunction(*(at(i, f"start_{i}") for i in range(cars)))
    # Distinct starts/goals and parking leaf rules are supplied by the common model.
    constraints += [f"distance_to_goal_{i} >= 1" for i in range(cars)]
    if parking_span is not None:
        config['parking_span']=parking_span
        for i in range(cars):
            for j in range(i):
                for operator in ('/', 'mod'):
                    constraints.append(f'abs((goal_{i} {operator} {columns}) - (goal_{j} {operator} {columns})) <= {parking_span}')
    if family.startswith('bottleneck_'):
        roles.update(bridge_left='N', bridge_right='N')
        constraints += [f'role_bridge_left / {columns} = role_bridge_right / {columns}',
                        f'role_bridge_right mod {columns} = (role_bridge_left mod {columns}) + 1']
        candidates=grid_edges(rows,columns)
        constraints.append(either([
            f'edge_{u}_{v} & role_bridge_left = {u} & role_bridge_right = {v}'
            for u,v in candidates if v==u+1]))
        # Every edge crossing the symbolic vertical cut is absent except the
        # chosen bridge. Opposite-bank journeys must therefore share that edge.
        for u,v in candidates:
            if v==u+1:
                constraints.append(f'({u % columns} = (role_bridge_left mod {columns})) -> '
                                   f'(!edge_{u}_{v} | (role_bridge_left = {u} & role_bridge_right = {v}))')
        for i in range(cars):
            start_relation,goal_relation=('<=' ,'>') if i%2==0 else ('>', '<=')
            constraints += [f'(start_{i} mod {columns}) {start_relation} (role_bridge_left mod {columns})',
                            f'(goal_{i} mod {columns}) {goal_relation} (role_bridge_left mod {columns})']
        if family == 'bottleneck_yield':
            roles['bay']='P'
            constraints += ['start_0 = role_bridge_left', 'start_1 = role_bridge_right',
                            'role_bridge_left_degree = 3',
                            f'(role_bay mod {columns}) <= (role_bridge_left mod {columns})']
            constraints += [f'role_bay != goal_{i}' for i in range(cars)]
            constraints.append(either([
                f'edge_{u}_{v} & role_bridge_left = {a} & role_bay = {b}'
                for u,v in candidates for a,b in ((u,v),(v,u))]))
    if family in ("shared_gate", "two_shared_gates"):
        names = ["gate"] if family == "shared_gate" else ["gate_a", "gate_b"]
        for name in names:
            roles[name] = "N"
            roles['queue_'+name] = 'N'
            constraints += [f"role_{name}_degree = 2"]
            constraints += [either([f'edge_{u}_{v} & role_{name} = {a} & role_queue_{name} = {b}'
                                    for u,v in grid_edges(rows,columns) for a,b in ((u,v),(v,u))])]
            constraints += [f"start_{i} != role_{name}" for i in range(cars)]
        if len(names) == 2:
            constraints.append("role_gate_a != role_gate_b")
    if family in ("shared_bay", "foreign_yield", "fleet_yield"):
        roles["bay"] = "P"
        constraints += [f"role_bay != goal_{i}" for i in range(cars)]
    if family == "fleet_yield":
        roles.update(gate="N", approach="N", exit="N")
        constraints += ["start_0 = role_gate", "role_gate_degree = 3",
                        "role_approach != role_exit"]
        # The bay and two through-road neighbours are symbolic grid cells.
        # The rest of the road graph remains free, including alternate routes.
        for name in ("bay", "approach", "exit"):
            constraints.append(either([
                f"edge_{u}_{v} & role_gate = {a} & role_{name} = {b}"
                for u,v in grid_edges(rows,columns) for a,b in ((u,v),(v,u))]))
    if family in ("row_reversal", "row_to_column", "row_to_column_reversed", "column_to_row"):
        start_axis = "column" if family == "column_to_row" else "row"
        goal_axis = "row" if family in ("row_reversal", "column_to_row") else "column"
        for prefix, axis in (("start", start_axis), ("goal", goal_axis)):
            capacity = columns if axis == "row" else rows
            if cars > capacity:
                raise ValueError(f"{cars} cars cannot form a {axis} in a {rows}x{columns} grid")
            fixed = "/" if axis == "row" else "mod"
            ordered = "mod" if axis == "row" else "/"
            reverse = prefix == "goal" and family in ("row_reversal", "row_to_column_reversed")
            for i in range(1, cars):
                constraints += [f"{prefix}_{i} {fixed} {columns} = {prefix}_0 {fixed} {columns}",
                                f"{prefix}_{i-1} {ordered} {columns} {'>' if reverse else '<'} {prefix}_{i} {ordered} {columns}"]
    for repeat in range(repetitions):
        if repeat:
            event(starts())  # A real return journey, not repeated satisfaction at rest.
        if family == "ordered_arrivals":
            for i in range(cars):
                event(at(i, f"goal_{i}"), conjunction(*(negate(at(j, f"goal_{j}")) for j in range(i, cars))))
        elif family in ("shared_gate", "two_shared_gates"):
            groups=[('gate',list(range(cars)))] if family=='shared_gate' else [
                ('gate_a',list(range((cars+1)//2))),('gate_b',list(range((cars+1)//2,cars)))]
            for name,group in groups:
                leader,waiter=group[:2]
                event(conjunction(at(leader,f'role_{name}'),at(waiter,f'role_queue_{name}')))
                event(at(leader,f'goal_{leader}'),at(waiter,f'role_queue_{name}'))
                for i in group[1:]: event(at(i,f'role_{name}'))
        elif family == "shared_bay":
            for i in range(cars):
                event(at(i, "role_bay"))
                event(at(i, f"goal_{i}"))
        elif family == "foreign_yield":
            event(at(0, "role_bay"))
            event(at(1, "start_0"), at(0, "role_bay"))
            event(at(1, "goal_1"), at(0, "role_bay"))
        elif family == "fleet_yield":
            event(at(0, "role_bay"))
            for i in range(1, cars):
                for target in ("role_approach", "role_gate", "role_exit", f"goal_{i}"):
                    event(at(i, target), at(0, "role_bay"))
        elif family == "parking_rotation":
            event(goals())
            event(goals(1))
        elif family == 'bottleneck_yield':
            event(at(0,'role_bay'))
            event(at(1,'role_bridge_left'),at(0,'role_bay'))
            event(at(1,'goal_1'),at(0,'role_bay'))
        elif family == 'bottleneck_rotation':
            event(goals())
            event(goals(1))
        event(goals())
    from .abstract import validate
    validate(config)
    observer(config)  # Validate the scenario AST before starting any subprocess.
    return config
