from __future__ import annotations

import math

import networkx as nx
import numpy as np
from scipy.optimize import least_squares

from .geometry import verify
from .model import Problem, coordinates_from_vector


def _residual(problem: Problem, values: np.ndarray) -> np.ndarray:
    points = values.reshape((-1, 2))
    indexes = {node: index for index, node in enumerate(problem.nodes)}
    residuals = []
    for edge in problem.edges:
        delta = points[indexes[edge.u]] - points[indexes[edge.v]]
        residuals.append((np.dot(delta, delta) - edge.length**2) / edge.length**2)

    residuals.extend((points[0, 0], points[0, 1]))
    if len(points) > 1:
        residuals.append(points[1, 1])
    return np.asarray(residuals)


def _initial_layout(problem: Problem, rng: np.random.Generator, attempt: int) -> np.ndarray:
    if attempt == 0 and len(problem.initial) == len(problem.nodes):
        return np.asarray([problem.initial[node] for node in problem.nodes], dtype=float).ravel()

    graph = nx.Graph()
    graph.add_nodes_from(problem.nodes)
    graph.add_edges_from((edge.u, edge.v) for edge in problem.edges)
    scale = max((edge.length for edge in problem.edges), default=1.0) * max(1, len(problem.nodes))
    if attempt == 0:
        _, embedding = nx.check_planarity(graph)
        layout = nx.combinatorial_embedding_to_pos(embedding)
        values = np.asarray([layout[node] for node in problem.nodes], dtype=float)
        values -= values[0]
        norm = np.max(np.linalg.norm(values, axis=1))
        if norm:
            values *= scale / norm
        return values.ravel()
    return rng.normal(0.0, scale, size=(len(problem.nodes), 2)).ravel()


def solve(
    problem: Problem,
    attempts: int = 100,
    seed: int = 0,
    tolerance: float = 1e-6,
) -> tuple[dict[str, tuple[float, float]] | None, dict]:
    if not problem.nodes:
        return {}, verify(problem, {}, tolerance)

    rng = np.random.default_rng(seed)
    best_coordinates = None
    best_report = None
    best_score = math.inf

    for attempt in range(attempts):
        initial = _initial_layout(problem, rng, attempt)
        result = least_squares(
            lambda values: _residual(problem, values),
            initial,
            method="trf",
            max_nfev=5000,
            ftol=1e-12,
            xtol=1e-12,
            gtol=1e-12,
        )
        coordinates = coordinates_from_vector(problem, result.x)
        report = verify(problem, coordinates, tolerance)
        score = (
            report["max_length_error"]
            + len(report["coincident_vertices"]) * 1e6
            + len(report["edge_conflicts"]) * 1e6
            + len(report["vertices_on_edges"]) * 1e6
        )
        if score < best_score:
            best_coordinates, best_report, best_score = coordinates, report, score
        if report["valid"]:
            report["attempt"] = attempt + 1
            report["optimizer_cost"] = float(result.cost)
            return coordinates, report

    assert best_report is not None
    best_report["attempts"] = attempts
    best_report["error"] = "no verified straight-line solution found"
    return best_coordinates, best_report
