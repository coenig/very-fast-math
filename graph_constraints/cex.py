from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import networkx as nx


_OUTGOING_RE = re.compile(r"(?:.*\.)?outgoing_connection_(\d+)_of_section_(\d+)$")
_SECTION_BEGIN_RE = re.compile(r"(?:.*\.)?section_(\d+)_segment_0_pos_begin$")
_SECTION_END_RE = re.compile(r"(?:.*\.)?section_(\d+)_end$")
_ARCLENGTH_RE = re.compile(
    r"(?:.*\.)?arclength_from_sec_(\d+)_to_sec_(\d+)_on_lane_0$"
)


def _as_number(value: str | None, variable: str) -> float:
    if value is None:
        raise ValueError(f"{variable} has no value")
    return float(value)


def build_multidigraph_from_xml_cex(
    cex_file: str | Path,
    *,
    include_none_edge: bool = False,
) -> nx.MultiDiGraph:
    tree = ET.parse(cex_file)
    root = tree.getroot()
    node = root.find("node")
    if node is None:
        raise ValueError("counterexample XML is missing a node element")
    state = node.find("state")
    if state is None:
        raise ValueError("counterexample XML is missing a state element")

    section_begins: dict[str, float] = {}
    section_ends: dict[str, float] = {}
    arclengths: dict[tuple[str, str], float] = {}
    outgoing: list[tuple[str, str, str]] = []
    minus_ones: list[tuple[str, str]] = []

    for child in state:
        variable = child.get("variable")
        if variable is None:
            continue

        if match := _SECTION_BEGIN_RE.fullmatch(variable):
            section_begins[match.group(1)] = _as_number(child.text, variable)
            continue

        if match := _SECTION_END_RE.fullmatch(variable):
            section_ends[match.group(1)] = _as_number(child.text, variable)
            continue

        if match := _ARCLENGTH_RE.fullmatch(variable):
            start, end = match.groups()
            arclengths[(start, end)] = _as_number(child.text, variable)
            continue

        if match := _OUTGOING_RE.fullmatch(variable):
            conn_number, start = match.groups()
            end = child.text
            if end is None:
                raise ValueError(f"{variable} has no value")
            if end == "-1":
                minus_ones.append((conn_number, start))
            else:
                outgoing.append((conn_number, start, end))

    graph = nx.MultiDiGraph()
    processed_sections: set[str] = set()

    for section in sorted(section_ends, key=int):
        begin = section_begins.get(section)
        end = section_ends[section]
        if begin is None:
            raise ValueError(f"section {section} is missing segment_0_pos_begin")
        length = end - begin
        if length <= 0:
            raise ValueError(f"section {section} has non-positive length {length}")
        graph.add_edge(
            f"S_{section}",
            f"E_{section}",
            length=length,
            kind="section",
            section=section,
        )
        processed_sections.add(section)

    for conn_number, start, end in outgoing:
        if start not in processed_sections:
            raise ValueError(f"outgoing connection references unknown section {start}")
        if end not in processed_sections:
            raise ValueError(f"outgoing connection references unknown section {end}")
        try:
            length = arclengths[(start, end)]
        except KeyError as error:
            raise ValueError(
                f"missing lane-0 arclength from section {start} to section {end}"
            ) from error
        graph.add_edge(
            f"E_{start}",
            f"S_{end}",
            length=length,
            kind="connection",
            conn_number=conn_number,
            start_section=start,
            end_section=end,
        )

    graph.graph["minus_ones"] = minus_ones
    if include_none_edge:
        graph.add_edge("NONE", "NONE", data=minus_ones)
    return graph
