from __future__ import annotations

from pathlib import Path

import networkx as nx
from pyvis.network import Network

from graph_constraints.cex import build_multidigraph_from_xml_cex


class GraphBuilder:
    def build_from_xml_cex(self, cex_file: str | Path) -> nx.MultiDiGraph:
        return build_multidigraph_from_xml_cex(cex_file, include_none_edge=True)


def dump_graph(graph: nx.MultiDiGraph, html_file: str = "digraph.html") -> None:
    network = Network(notebook=True, directed=True, cdn_resources="remote")
    network.from_nx(graph)
    network.show_buttons(filter_=["physics"])
    network.show(html_file)


def dump_graph_with_lengths(
    graph: nx.MultiDiGraph,
    html_file: str = "digraph_with_lengths.html",
) -> None:
    network = Network(notebook=True, directed=True, cdn_resources="remote")
    for node in graph.nodes:
        if node != "NONE":
            network.add_node(node, label=str(node))

    for start, end, _, data in graph.edges(keys=True, data=True):
        if start == "NONE" or end == "NONE":
            continue
        length = data.get("length")
        kind = data.get("kind", "edge")
        label = kind if length is None else f"{length:g}"
        title = (
            f"{start} -> {end}"
            if length is None
            else f"{kind}: {start} -> {end}, length={length:g}"
        )
        network.add_edge(
            start,
            end,
            label=label,
            title=title,
            arrows="to",
            physics=True,
        )

    network.show_buttons(filter_=["physics"])
    network.show(html_file)


def geometric_length_graph(graph: nx.MultiDiGraph) -> nx.MultiDiGraph:
    geometry = nx.MultiDiGraph()
    for start, end, key, data in graph.edges(keys=True, data=True):
        if start == "NONE" or end == "NONE" or "length" not in data:
            continue
        geometry.add_edge(start, end, key=key, **data)
    return geometry
