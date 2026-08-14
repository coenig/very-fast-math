"""Generate and check a VFM model for one caller-supplied road graph.

Unlike the graph-search approaches, this module makes every road-topology and
road-length variable a singleton in the generated SMV model.  Consequently
nuXmv checks vehicle behaviour on the supplied graph; it does not search for a
graph as part of the counterexample.

The most convenient input format is::

    graph = nx.MultiDiGraph()
    graph.add_node("entry", length=300)
    graph.add_node("junction", length=180)
    graph.add_edge("entry", "junction", length=25)

Node names may be arbitrary hashable Python objects.  They are assigned SMV
section indexes in insertion order.  A connection ``length`` applies to all
lanes; use ``lane_lengths=[25, 27]`` to provide one value per lane.

The bipartite format used by ``approaches.approach_3`` is accepted as well:
section edges are marked ``kind="section", section=N`` and connection edges
are marked ``kind="connection", start_section=N, end_section=M``.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Hashable, Mapping, Sequence

import networkx as nx

from model_checking import Morty
from scenario_generation import build_property_spec, generate_scenario


@dataclass(frozen=True)
class FixedRoadGraph:
    """SMV-ready, validated representation of a road graph."""

    section_lengths: tuple[int, ...]
    connections: tuple[tuple[int, int, tuple[int, ...]], ...]
    node_to_section: Mapping[Hashable, int]

    @property
    def section_count(self) -> int:
        return len(self.section_lengths)

    @property
    def lane_count(self) -> int:
        return len(self.connections[0][2]) if self.connections else 1

    @property
    def max_outgoing(self) -> int:
        counts = [0] * self.section_count
        for start, _, _ in self.connections:
            counts[start] += 1
        # The VFM template expects at least one slot even for a terminal road.
        return max(1, max(counts, default=0))


def _positive_integer(value: object, description: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{description} must be a positive integer, got {value!r}")
    integer = int(value)
    if integer != value or integer <= 0:
        raise ValueError(f"{description} must be a positive integer, got {value!r}")
    return integer


def _edge_lane_lengths(data: Mapping, lanes: int, description: str) -> tuple[int, ...]:
    values = data.get("lane_lengths")
    if values is None:
        length = _positive_integer(data.get("length"), f"{description} length")
        return (length,) * lanes
    if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
        raise TypeError(f"{description} lane_lengths must be a sequence")
    if len(values) != lanes:
        raise ValueError(
            f"{description} has {len(values)} lane lengths, expected {lanes}"
        )
    return tuple(
        _positive_integer(value, f"{description} lane {lane} length")
        for lane, value in enumerate(values)
    )


def normalize_road_graph(graph: nx.MultiDiGraph, *, lanes: int = 1) -> FixedRoadGraph:
    """Validate ``graph`` and convert it to contiguous VFM section indexes."""
    if not isinstance(graph, nx.MultiDiGraph):
        raise TypeError("graph must be a networkx.MultiDiGraph")
    if graph.number_of_nodes() == 0:
        raise ValueError("graph must contain at least one road section")
    if lanes < 1:
        raise ValueError("lanes must be positive")

    section_edges = [
        (start, end, data)
        for start, end, data in graph.edges(data=True)
        if data.get("kind") == "section"
    ]
    if section_edges:
        ids_and_lengths = []
        for start, end, data in section_edges:
            section = data.get("section")
            if not isinstance(section, int) or section < 0:
                raise ValueError(f"section edge {start!r}->{end!r} needs integer section")
            ids_and_lengths.append(
                (section, _positive_integer(data.get("length"), f"section {section} length"))
            )
        ids_and_lengths.sort()
        ids = [section for section, _ in ids_and_lengths]
        if ids != list(range(len(ids))):
            raise ValueError(f"section indexes must be contiguous from zero, got {ids}")
        section_lengths = tuple(length for _, length in ids_and_lengths)
        node_map: dict[Hashable, int] = {}
        connection_data = []
        for start, end, data in graph.edges(data=True):
            if data.get("kind") != "connection":
                continue
            source = data.get("start_section")
            target = data.get("end_section")
            connection_data.append((source, target, data, f"connection {start!r}->{end!r}"))
    else:
        nodes = list(graph.nodes)
        node_map = {node: section for section, node in enumerate(nodes)}
        section_lengths = tuple(
            _positive_integer(graph.nodes[node].get("length"), f"section {node!r} length")
            for node in nodes
        )
        connection_data = [
            (node_map[start], node_map[end], data, f"connection {start!r}->{end!r}")
            for start, end, data in graph.edges(data=True)
        ]

    connections = []
    seen_pairs = set()
    for source, target, data, description in connection_data:
        if not isinstance(source, int) or not 0 <= source < len(section_lengths):
            raise ValueError(f"{description} has invalid start section {source!r}")
        if not isinstance(target, int) or not 0 <= target < len(section_lengths):
            raise ValueError(f"{description} has invalid end section {target!r}")
        if source == target:
            raise ValueError(f"{description} is a self-connection, unsupported by VFM")
        if (source, target) in seen_pairs:
            raise ValueError(
                f"parallel connections from section {source} to {target} are unsupported"
            )
        seen_pairs.add((source, target))
        connections.append(
            (source, target, _edge_lane_lengths(data, lanes, description))
        )
    connections.sort(key=lambda item: (item[0], item[1]))
    return FixedRoadGraph(section_lengths, tuple(connections), node_map)


def _replace_singleton_domain(
    text: str, variable: str, value: int, *, declaration_required: bool = True
) -> str:
    pattern = re.compile(
        rf"(?m)^(?P<prefix>[ \t]*(?:FROZENVAR[ \t]+)?)"
        rf"{re.escape(variable)}\s*:\s*"
        r"-?\d+\s*\.\.\s*-?\d+\s*;"
    )
    replacement = rf"\g<prefix>{variable} : {value} .. {value};"
    text, count = pattern.subn(replacement, text, count=1)
    if declaration_required and count != 1:
        raise ValueError(f"generated EnvModel has no declaration for {variable}")
    return text


def specialize_env_model(
    env_model: str | Path,
    road: FixedRoadGraph,
    *,
    lanes: int,
    output_file: str | Path | None = None,
) -> Path:
    """Write an EnvModel whose graph-related domains all have one value."""
    source = Path(env_model)
    text = source.read_text()
    targets = {section: [] for section in range(road.section_count)}
    lengths = {}
    for start, end, lane_lengths in road.connections:
        targets[start].append(end)
        lengths[(start, end)] = lane_lengths

    for section, length in enumerate(road.section_lengths):
        text = _replace_singleton_domain(text, f"section_{section}_end", length)
        values = sorted(targets[section])
        values.extend([-1] * (road.max_outgoing - len(values)))
        for slot, target in enumerate(values):
            text = _replace_singleton_domain(
                text, f"outgoing_connection_{slot}_of_section_{section}", target
            )

    for start in range(road.section_count):
        for end in range(road.section_count):
            if start == end:
                continue
            lane_lengths = lengths.get((start, end), (-1,) * lanes)
            for lane, length in enumerate(lane_lengths):
                text = _replace_singleton_domain(
                    text,
                    f"arclength_from_sec_{start}_to_sec_{end}_on_lane_{lane}",
                    length,
                )

    destination = Path(output_file) if output_file else source.with_name("EnvModel_FixedGraph.smv")
    destination.write_text(
        "-- Fixed road graph: topology and metric domains specialized by fixed_graph_model.py\n"
        + text
    )
    return destination


def generate_fixed_graph_model(
    graph: nx.MultiDiGraph,
    *,
    nonegos: int = 2,
    lanes: int = 1,
    property_name: str = "vehicle_order",
    property_smv: str | None = None,
    input_config: str | Path = "main_template.json",
    experiments_dir: str | Path = "SMV_GEN",
) -> Morty:
    """Generate a scenario specialized to exactly ``graph``.

    The returned ``Morty`` can be passed to :func:`check_fixed_graph_model` or
    used directly.  Its target directory contains ``main_fixed_graph.smv``.
    """
    road = normalize_road_graph(graph, lanes=lanes)
    config = {
        "#007": nonegos,
        "NONEGOS": nonegos,
        "#008": lanes,
        "NUMLANES": lanes,
        "#010": road.section_count,
        "SECTIONS": road.section_count,
        "SECTIONSMINLENGTH": min(road.section_lengths),
        "SECTIONSMAXLENGTH": max(road.section_lengths),
        "MINDISTCONNECTIONS": min(
            (length for _, _, values in road.connections for length in values),
            default=1,
        ),
        "MAXDISTCONNECTIONS": max(
            (length for _, _, values in road.connections for length in values),
            default=1,
        ),
        "MAXOUTGOINGCONNECTIONS": road.max_outgoing,
        "MODEL_INTERSECTION_GEOMETRY": 0,
        "SPEC": property_smv or build_property_spec(property_name, nonegos),
    }
    morty = generate_scenario(
        config,
        input_config=input_config,
        experiments_dir=experiments_dir,
        scenario_suffix="fixed_graph",
    )
    target = morty.get_target_path()
    # The native generator currently prefixes its working directory even when
    # passed an absolute output path.  Keep absolute ``experiments_dir`` useful
    # by detecting that legacy behaviour and teaching Morty the actual path.
    if target.is_absolute() and not target.exists():
        generated_target = Path.cwd() / str(target).lstrip("/")
        if generated_target.exists():
            target = generated_target
            morty.args[5] = str(target)
    fixed_env = specialize_env_model(
        target / "EnvModel.smv", road, lanes=lanes
    )
    main_text = (target / "main.smv").read_text()
    include = '#include "EnvModel.smv"'
    if main_text.count(include) != 1:
        raise ValueError("generated main.smv does not contain one EnvModel include")
    (target / "main_fixed_graph.smv").write_text(
        main_text.replace(include, f'#include "{fixed_env.name}"')
    )
    return morty


def check_fixed_graph_model(
    morty: Morty,
    *,
    cex_file: str = "cex_fixed_graph.xml",
    bound: int = 100,
) -> Path | None:
    """Check the property and return its counterexample path, if one exists."""
    try:
        return morty.model_check(
            cex_file, 0, model_file="main_fixed_graph.smv", bound=bound
        )
    finally:
        morty.close_model_checker()


if __name__ == "__main__":
    example = nx.MultiDiGraph()
    example.add_node("west", length=300)
    example.add_node("east", length=250)
    example.add_edge("west", "east", length=25)
    example.add_edge("east", "west", length=25)
    generated = generate_fixed_graph_model(example, nonegos=3)
    print(f"Fixed-graph model written to {generated.get_target_path()}")
    counterexample = check_fixed_graph_model(generated)
    print(f"Counterexample: {counterexample}" if counterexample else "No counterexample found")
