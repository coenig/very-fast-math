from __future__ import annotations

import math
from itertools import combinations

from .model import Edge, Problem


Point = tuple[float, float]


def orient(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def point_on_segment(point: Point, a: Point, b: Point, tolerance: float) -> bool:
    scale = max(1.0, math.dist(a, b))
    if abs(orient(a, b, point)) > tolerance * scale:
        return False
    return (
        min(a[0], b[0]) - tolerance <= point[0] <= max(a[0], b[0]) + tolerance
        and min(a[1], b[1]) - tolerance
        <= point[1]
        <= max(a[1], b[1]) + tolerance
    )


def segments_conflict(
    first: Edge,
    second: Edge,
    coordinates: dict[str, Point],
    tolerance: float,
) -> bool:
    a, b = coordinates[first.u], coordinates[first.v]
    c, d = coordinates[second.u], coordinates[second.v]
    shared = {first.u, first.v} & {second.u, second.v}

    if shared:
        common = next(iter(shared))
        p = coordinates[common]
        q = coordinates[first.v if first.u == common else first.u]
        r = coordinates[second.v if second.u == common else second.u]
        cross = abs(orient(p, q, r))
        dot = (q[0] - p[0]) * (r[0] - p[0]) + (q[1] - p[1]) * (r[1] - p[1])
        return cross <= tolerance * max(1.0, math.dist(p, q), math.dist(p, r)) and dot > tolerance

    o1, o2 = orient(a, b, c), orient(a, b, d)
    o3, o4 = orient(c, d, a), orient(c, d, b)
    proper = (
        ((o1 > tolerance and o2 < -tolerance) or (o1 < -tolerance and o2 > tolerance))
        and ((o3 > tolerance and o4 < -tolerance) or (o3 < -tolerance and o4 > tolerance))
    )
    if proper:
        return True
    return any(
        (
            point_on_segment(c, a, b, tolerance),
            point_on_segment(d, a, b, tolerance),
            point_on_segment(a, c, d, tolerance),
            point_on_segment(b, c, d, tolerance),
        )
    )


def verify(
    problem: Problem,
    coordinates: dict[str, Point],
    length_tolerance: float = 1e-6,
    geometry_tolerance: float = 1e-9,
) -> dict:
    missing = [node for node in problem.nodes if node not in coordinates]
    if missing:
        return {"valid": False, "error": f"missing coordinates for {missing}"}

    errors = []
    for edge in problem.edges:
        actual = math.dist(coordinates[edge.u], coordinates[edge.v])
        errors.append(
            {
                "edge": [edge.u, edge.v],
                "target": edge.length,
                "actual": actual,
                "absolute_error": abs(actual - edge.length),
            }
        )

    coincident_vertices = []
    for first, second in combinations(problem.nodes, 2):
        if math.dist(coordinates[first], coordinates[second]) <= geometry_tolerance:
            coincident_vertices.append([first, second])

    conflicts = []
    for first, second in combinations(problem.edges, 2):
        if segments_conflict(first, second, coordinates, geometry_tolerance):
            conflicts.append([[first.u, first.v], [second.u, second.v]])

    vertex_on_edge = []
    for node in problem.nodes:
        for edge in problem.edges:
            if node in (edge.u, edge.v):
                continue
            if point_on_segment(
                coordinates[node],
                coordinates[edge.u],
                coordinates[edge.v],
                geometry_tolerance,
            ):
                vertex_on_edge.append([node, [edge.u, edge.v]])

    max_error = max((item["absolute_error"] for item in errors), default=0.0)
    return {
        "valid": (
            max_error <= length_tolerance
            and not coincident_vertices
            and not conflicts
            and not vertex_on_edge
        ),
        "max_length_error": max_error,
        "edge_errors": errors,
        "coincident_vertices": coincident_vertices,
        "edge_conflicts": conflicts,
        "vertices_on_edges": vertex_on_edge,
    }
