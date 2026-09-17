from __future__ import annotations

from itertools import combinations
from typing import Iterator

import networkx as nx


def max_simple_planar_edges(node_count: int) -> int:
    if node_count < 0:
        raise ValueError("node_count must be non-negative")
    if node_count < 3:
        return node_count * (node_count - 1) // 2
    return 3 * node_count - 6


def iter_labeled_planar_graphs(
    node_count: int,
    edge_count: int,
    *,
    connected: bool = False,
) -> Iterator[nx.Graph]:
    """Yield all labeled simple planar graphs with the given size.

    Nodes are the integers ``0 .. node_count - 1``. Graphs are "labeled", so
    two isomorphic graphs with different vertex labels are yielded separately.
    The iterator is lazy, but the search space is still
    ``binomial(node_count * (node_count - 1) / 2, edge_count)`` before
    planarity filtering.
    """
    if node_count < 0:
        raise ValueError("node_count must be non-negative")
    if edge_count < 0:
        raise ValueError("edge_count must be non-negative")

    all_edges = list(combinations(range(node_count), 2))
    if edge_count > len(all_edges):
        return
    if edge_count > max_simple_planar_edges(node_count):
        return
    if connected and node_count > 0 and edge_count < node_count - 1:
        return

    for chosen_edges in combinations(all_edges, edge_count):
        graph = nx.Graph()
        graph.add_nodes_from(range(node_count))
        graph.add_edges_from(chosen_edges)
        if connected and not nx.is_connected(graph):
            continue
        planar, _ = nx.check_planarity(graph)
        if planar:
            yield graph
