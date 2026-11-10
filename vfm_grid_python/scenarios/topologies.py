"""Property-blind, seeded enumeration of labelled partial grids, without repeats."""
import math
import random
from pathlib import Path

from vfm_grid_python.common import grid_edges


def combination_at(n, k, rank):
    """Unrank lexicographic k-subsets without materializing all combinations."""
    selected, minimum = [], 0
    for left in range(k, 0, -1):
        for value in range(minimum, n):
            count = math.comb(n-value-1, left-1)
            if rank < count:
                selected.append(value)
                minimum = value+1
                break
            rank -= count
    return selected


def layouts(rows, columns, parkings, seed=0):
    """Affine permutation of edge-mask × parking-label assignments.

    Order is reproducible, but is not claimed to be a uniform shuffle of all
    permutations. Rejection checks only P degree one and no selected P-P edge.
    Connectivity, goals, roles, starts and the scenario are deliberately absent.
    Memory usage is independent of the number of visited assignments.
    """
    n = rows*columns
    if rows < 1 or columns < 1 or not 0 <= parkings <= n:
        raise ValueError("invalid oracle dimensions/parking count")
    edges = grid_edges(rows, columns)
    masks = 1 << len(edges)
    total = masks * math.comb(n, parkings)
    rng = random.Random(seed)
    stride = rng.randrange(1, total) if total > 1 else 1
    while math.gcd(stride, total) != 1:
        stride = rng.randrange(1, total)
    offset = rng.randrange(total)
    for attempt in range(total):
        index = (offset + attempt*stride) % total
        labels = set(combination_at(n, parkings, index//masks))
        mask = index % masks
        selected = [edge for bit, edge in enumerate(edges) if mask & (1 << bit)]
        degree = [0]*n
        if any(u in labels and v in labels for u,v in selected):
            continue
        for u,v in selected:
            degree[u] += 1
            degree[v] += 1
        if any(degree[p] != 1 for p in labels):
            continue
        yield dict(rows=rows, columns=columns,
                   nodes=[dict(id=u, row=u//columns, column=u%columns,
                               kind="P" if u in labels else "N") for u in range(n)],
                   edges=[list(e) for e in selected], oracle_assignment=str(index),
                   oracle_assignments_examined=attempt+1)


def static_problem(config):
    """Static part (validity + gamma) of the shared concrete queue compiler.

    The source is ``concrete.compiler.constraints`` on the full symbolic grid,
    i.e. the same text that the fixed-topology enumeration queries check.
    Edges and parking labels become free Boolean variables (they are frozen
    VFM variables there). Statements over vehicle state (road/rank/joint, the
    observer, safety) are dropped and reported, so the result over-approximates
    the topologies with a consistent initial state. As cars stand at a section
    endpoint of their start node, every start node needs an incident edge.
    """
    from vfm_grid_python.common import either
    from vfm_grid_python.concrete.compiler import constraints
    from vfm_grid_python.concrete.schema import blocks, statements, schema
    from vfm_grid_python.concrete.symbolic import skeleton
    from vfm_grid_python.concrete.trace_model import parse
    from vfm_grid_python.geometry import build
    from .topology_sat import StaticProblem, declaration
    rows, columns, cars = config['rows'], config['columns'], config['cars']
    n = rows*columns
    # A trivial stage keeps the observer well-formed; it is dropped as non-static.
    static = dict(config, scenario_stages=[dict(event=True)])
    graph = dict(skeleton(static), starts=[0]*cars, goals=[0]*cars,
                 roles={name: 0 for name in config.get('roles', {})})
    geometry = build(graph, config.get('section_length', 20), config.get('parking_length', 7),
                     config.get('junction_radius', 6))
    text = 'MODULE main\n' + constraints(static, graph, geometry, symbolic=True)
    topology = {f'edge_{u}_{v}': 'boolean' for u, v in grid_edges(rows, columns)}
    topology.update({f'parking_{u}': 'boolean' for u in range(n)})
    declarations = dict(topology)
    declarations.update({name: declaration(d['domain']) for name, d in schema(text).items()
                         if d['kind'] == 'FROZENVAR'})
    defines, rules = {}, []
    for kind, body in blocks(text):
        if kind == 'DEFINE':
            for statement in statements(body):
                name, expression = statement.split(':=', 1)
                if name.strip() not in topology:  # DEFINE edge_u_v := env.edge_u_v
                    defines[name.strip()] = expression.strip().rstrip(';')
        elif kind in ('INIT', 'INVAR'):
            rules += [(kind, s.rstrip(';')) for s in statements(body)]
    rules += [('INIT', either([f'start_{i} = {u} & degree_{u} > 0' for u in range(n)])) for i in range(cars)]
    aliases = {'env.' + name: name for name in topology}
    return StaticProblem(declarations, defines, rules, parse, aliases)


def sat_layouts(config, seed=0, projection='topology', solver='z3', timer=None, report=None):
    """Labelled partial grids satisfying the static constraints, without repeats.

    Same graph shape as ``layouts``; in ``placement`` mode each graph also
    carries ``eta`` (starts, goals, roles).
    """
    from vfm_grid_python.common import dump
    from .topology_sat import solutions, backend_version
    rows, columns = config['rows'], config['columns']
    problem = static_problem(config)
    if report is not None:
        Path(report).parent.mkdir(parents=True, exist_ok=True)
        dump(report, dict(problem.report(), projection=projection, solver=solver,
                          solver_version=backend_version(solver), seed=seed))
    for index, values in enumerate(solutions(problem, projection, solver, seed, timer)):
        labels = {u for u in range(rows*columns) if values[f'parking_{u}']}
        graph = dict(rows=rows, columns=columns,
                     nodes=[dict(id=u, row=u//columns, column=u%columns,
                                 kind='P' if u in labels else 'N') for u in range(rows*columns)],
                     edges=[list(e) for e in grid_edges(rows, columns) if values[f'edge_{e[0]}_{e[1]}']],
                     sat_index=index, sat_solver=solver)
        if projection == 'placement':
            graph['eta'] = dict(starts=[values[f'start_{i}'] for i in range(config['cars'])],
                                goals=[values[f'goal_{i}'] for i in range(config['cars'])],
                                roles={name: values[f'role_{name}'] for name in config.get('roles', {})})
        yield graph


def placement_constraints(eta):
    """INIT expressions fixing a placement in the concrete query."""
    return ([f'start_{i} = {v}' for i, v in enumerate(eta['starts'])] +
            [f'goal_{i} = {v}' for i, v in enumerate(eta['goals'])] +
            [f'role_{name} = {v}' for name, v in eta['roles'].items()])
