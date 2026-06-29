from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import networkx as nx


@dataclass(frozen=True)
class Edge:
    u: str
    v: str
    length: float


@dataclass(frozen=True)
class Problem:
    nodes: tuple[str, ...]
    edges: tuple[Edge, ...]
    initial: dict[str, tuple[float, float]]


def problem_from_multidigraph(
    graph: nx.MultiDiGraph,
    *,
    length_attr: str = "length",
    initial_attr: str | None = "pos",
    duplicate_tolerance: float = 1e-9,
) -> Problem:
    """Build a fixed-length layout problem from a NetworkX MultiDiGraph.

    Edge direction is ignored by the geometry solver. Parallel edges are
    accepted only when they impose the same length constraint; in that case the
    duplicate constraints are collapsed into one undirected segment.
    """
    nodes = tuple(str(node) for node in graph.nodes)
    if len(nodes) != len(set(nodes)):
        raise ValueError("node names must be unique after string conversion")

    node_set = set(nodes)
    edges: list[Edge] = []
    lengths_by_pair: dict[frozenset[str], float] = {}

    for u, v, key, data in graph.edges(keys=True, data=True):
        edge_u = str(u)
        edge_v = str(v)
        if edge_u not in node_set or edge_v not in node_set:
            raise ValueError(f"edge {edge_u}-{edge_v} references an unknown node")
        if edge_u == edge_v:
            continue
            # raise ValueError("self-loops are not supported")
        if length_attr not in data:
            raise ValueError(f"edge {edge_u}-{edge_v} key {key!r} is missing {length_attr!r}")

        length = float(data[length_attr])
        if not math.isfinite(length) or length <= 0:
            raise ValueError(f"edge {edge_u}-{edge_v} key {key!r} needs a positive finite length")

        pair = frozenset((edge_u, edge_v))
        previous = lengths_by_pair.get(pair)
        if previous is not None:
            if not math.isclose(previous, length, rel_tol=0.0, abs_tol=duplicate_tolerance):
                raise ValueError(
                    f"parallel edge {edge_u}-{edge_v} key {key!r} has length {length}, "
                    f"but the same node pair already has length {previous}"
                )
            continue

        lengths_by_pair[pair] = length
        edges.append(Edge(edge_u, edge_v, length))

    initial: dict[str, tuple[float, float]] = {}
    if initial_attr is not None:
        for node, data in graph.nodes(data=True):
            if initial_attr not in data:
                continue
            point = data[initial_attr]
            initial[str(node)] = (float(point[0]), float(point[1]))

    simple_graph = nx.Graph()
    simple_graph.add_nodes_from(nodes)
    simple_graph.add_edges_from((edge.u, edge.v) for edge in edges)
    planar, _ = nx.check_planarity(simple_graph)
    if not planar:
        raise ValueError(
            "the abstract graph is nonplanar, so no crossing-free drawing exists"
        )

    return Problem(nodes, tuple(edges), initial)


def load_problem(path: str | Path) -> Problem:
    data = json.loads(Path(path).read_text())
    nodes = tuple(str(node) for node in data["nodes"])
    if len(nodes) != len(set(nodes)):
        raise ValueError("node names must be unique")

    node_set = set(nodes)
    edges: list[Edge] = []
    pairs: set[frozenset[str]] = set()
    for item in data["edges"]:
        edge = Edge(str(item["u"]), str(item["v"]), float(item["length"]))
        if edge.u not in node_set or edge.v not in node_set:
            raise ValueError(f"edge {edge.u}-{edge.v} references an unknown node")
        if edge.u == edge.v:
            raise ValueError("self-loops are not supported")
        if not math.isfinite(edge.length) or edge.length <= 0:
            raise ValueError(f"edge {edge.u}-{edge.v} needs a positive finite length")
        pair = frozenset((edge.u, edge.v))
        if pair in pairs:
            raise ValueError(f"duplicate edge {edge.u}-{edge.v}")
        pairs.add(pair)
        edges.append(edge)

    initial = {
        str(node): (float(point[0]), float(point[1]))
        for node, point in data.get("initial", {}).items()
    }
    if not set(initial) <= node_set:
        raise ValueError("initial contains an unknown node")

    graph = nx.Graph()
    graph.add_nodes_from(nodes)
    graph.add_edges_from((edge.u, edge.v) for edge in edges)
    planar, _ = nx.check_planarity(graph)
    if not planar:
        raise ValueError(
            "the abstract graph is nonplanar, so no crossing-free drawing exists"
        )
    return Problem(nodes, tuple(edges), initial)


def write_solution(
    path: str | Path,
    problem: Problem,
    coordinates: dict[str, tuple[float, float]],
    method: str,
    report: dict[str, Any],
) -> None:
    result = {
        "method": method,
        "coordinates": {
            node: [float(coordinates[node][0]), float(coordinates[node][1])]
            for node in problem.nodes
        },
        "verification": report,
    }
    Path(path).write_text(json.dumps(result, indent=2) + "\n")


def coordinates_from_vector(
    problem: Problem, values: np.ndarray
) -> dict[str, tuple[float, float]]:
    points = values.reshape((-1, 2))
    return {
        node: (float(points[index, 0]), float(points[index, 1]))
        for index, node in enumerate(problem.nodes)
    }
