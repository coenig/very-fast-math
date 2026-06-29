from __future__ import annotations

from itertools import combinations

import z3

from .geometry import verify
from .model import Problem


def _orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _same_strict_side(first, second):
    return z3.Or(z3.And(first > 0, second > 0), z3.And(first < 0, second < 0))


def _as_float(value: z3.ArithRef) -> float:
    if z3.is_rational_value(value):
        return float(value.numerator_as_long()) / float(value.denominator_as_long())
    decimal = value.as_decimal(30)
    if decimal.endswith("?"):
        decimal = decimal[:-1]
    if decimal.endswith("..."):
        decimal = decimal[:-3]
    return float(decimal)


def solve(
    problem: Problem, timeout_ms: int = 30_000, tolerance: float = 1e-6
) -> tuple[dict[str, tuple[float, float]] | None, dict]:
    solver = z3.Solver()
    solver.set(timeout=timeout_ms)
    points = {
        node: (z3.Real(f"x_{index}"), z3.Real(f"y_{index}"))
        for index, node in enumerate(problem.nodes)
    }

    for edge in problem.edges:
        a, b = points[edge.u], points[edge.v]
        length = z3.RealVal(str(edge.length))
        solver.add((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 == length**2)

    if problem.nodes:
        solver.add(points[problem.nodes[0]][0] == 0, points[problem.nodes[0]][1] == 0)
    if len(problem.nodes) > 1:
        solver.add(points[problem.nodes[1]][1] == 0, points[problem.nodes[1]][0] >= 0)

    for first, second in combinations(problem.nodes, 2):
        solver.add(z3.Or(points[first][0] != points[second][0], points[first][1] != points[second][1]))

    for node in problem.nodes:
        p = points[node]
        for edge in problem.edges:
            if node in (edge.u, edge.v):
                continue
            a, b = points[edge.u], points[edge.v]
            outside = (p[0] - a[0]) * (p[0] - b[0]) + (p[1] - a[1]) * (p[1] - b[1]) > 0
            solver.add(z3.Or(_orient(a, b, p) != 0, outside))

    for first, second in combinations(problem.edges, 2):
        shared = {first.u, first.v} & {second.u, second.v}
        if shared:
            common = next(iter(shared))
            q = first.v if first.u == common else first.u
            r = second.v if second.u == common else second.u
            p0, p1, p2 = points[common], points[q], points[r]
            dot = (p1[0] - p0[0]) * (p2[0] - p0[0]) + (p1[1] - p0[1]) * (p2[1] - p0[1])
            solver.add(z3.Or(_orient(p0, p1, p2) != 0, dot <= 0))
            continue

        a, b = points[first.u], points[first.v]
        c, d = points[second.u], points[second.v]
        solver.add(
            z3.Or(
                _same_strict_side(_orient(a, b, c), _orient(a, b, d)),
                _same_strict_side(_orient(c, d, a), _orient(c, d, b)),
            )
        )

    status = solver.check()
    if status != z3.sat:
        reason = solver.reason_unknown() if status == z3.unknown else "constraints are unsatisfiable"
        return None, {"valid": False, "status": str(status), "error": reason}

    model = solver.model()
    coordinates = {
        node: (
            _as_float(model.eval(point[0], model_completion=True)),
            _as_float(model.eval(point[1], model_completion=True)),
        )
        for node, point in points.items()
    }
    report = verify(problem, coordinates, tolerance)
    report["status"] = "sat"
    return coordinates, report
