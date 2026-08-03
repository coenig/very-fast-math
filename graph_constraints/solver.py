from __future__ import annotations

import networkx as nx

from .model import problem_from_multidigraph


def graph_with_solution(
    graph: nx.MultiDiGraph,
    coordinates: dict[str, tuple[float, float]],
    report: dict,
    *,
    position_attr: str = "pos",
    method: str | None = None,
) -> nx.MultiDiGraph:
    solved_graph = graph.copy()
    nodes_by_problem_id = {str(node): node for node in solved_graph.nodes}
    for node, point in coordinates.items():
        original_node = nodes_by_problem_id[node]
        solved_graph.nodes[original_node][position_attr] = (
            float(point[0]),
            float(point[1]),
        )
    solved_graph.graph["verification"] = report
    if method is not None:
        solved_graph.graph["solver_method"] = method
    return solved_graph


def solve_multidigraph(
    graph: nx.MultiDiGraph,
    *,
    length_attr: str = "length",
    position_attr: str = "pos",
    method: str = "scipy",
    attempts: int = 100,
    seed: int = 0,
    timeout: float = 30.0,
    tolerance: float = 1e-6,
) -> tuple[nx.MultiDiGraph | None, dict]:
    problem = problem_from_multidigraph(graph, length_attr=length_attr)
    if method == "scipy":
        from .scipy_solver import solve as solve_scipy

        coordinates, report = solve_scipy(problem, attempts, seed, tolerance)
    elif method == "z3":
        from .z3_solver import solve as solve_z3

        coordinates, report = solve_z3(problem, int(timeout * 1000), tolerance)
    else:
        raise ValueError(f"unknown method {method!r}")

    if coordinates is None:
        return None, report
    return graph_with_solution(
        graph,
        coordinates,
        report,
        position_attr=position_attr,
        method=method,
    ), report
