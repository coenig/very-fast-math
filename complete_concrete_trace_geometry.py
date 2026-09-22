#!/usr/bin/env python3
"""Add deterministic grid geometry metadata to a concrete VFM XML trace.

The parking grid models intentionally disable VFM's intersection geometry,
therefore NuSMV traces do not contain source coordinates, angles, or the
derived sine/cosine values used by the native renderer.  This utility restores
those static values from selected_grid.json without changing any vehicle state.
"""
from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def _int(value: str | None, default: int = 0) -> int:
    try:
        return int(value) if value is not None else default
    except ValueError:
        return default


def complete(trace: Path, grid: Path, output: Path, pitch: int = 40) -> None:
    topology = json.loads(grid.read_text())
    nodes = {int(n["id"]): (int(n["row"]), int(n["column"])) for n in topology["nodes"]}
    root = ET.parse(trace).getroot()
    states = list(root.iter("state"))
    if not states:
        raise ValueError(f"trace contains no states: {trace}")

    # Select a stable heading for each section from its first outgoing edge;
    # otherwise use the first graph edge or east.  Headings are quantized to
    # the 45-degree granularity used by the VFM templates.
    outgoing: dict[int, int] = {}
    for value in states[0].findall("value"):
        name = value.attrib.get("variable", "")
        if name.startswith("env.outgoing_connection_") and "_of_section_" in name:
            sec = int(name.rsplit("_of_section_", 1)[1])
            target = _int(value.text, -1)
            if target >= 0 and sec not in outgoing:
                outgoing[sec] = target
    for edge in topology.get("edges", []):
        outgoing.setdefault(int(edge["source"]), int(edge["target"]))

    geometry: dict[int, dict[str, int]] = {}
    for sec, (row, col) in nodes.items():
        target = outgoing.get(sec)
        if target in nodes:
            tr, tc = nodes[target]
            angle = round(math.degrees(math.atan2(tr - row, tc - col)) / 45) * 45
            angle %= 360
        else:
            angle = 0
        # Integer lookup values match EnvModel_Sections.tpl (sin/cos * 100).
        geometry[sec] = {
            "source.x": col * pitch,
            "source.y": -row * pitch,
            "angle_raw": int(angle // 45),
            "angle": int(angle),
            "sin": int(round(math.sin(math.radians(angle)) * 100)),
            "cos": int(round(math.cos(math.radians(angle)) * 100)),
        }

    for state in states:
        existing = {v.attrib.get("variable") for v in state.findall("value")}
        values: list[tuple[str, int]] = []
        for sec, data in geometry.items():
            # section_end is already present in concrete traces; use it for
            # the derived drain coordinates and preserve the model value.
            end_name = f"env.section_{sec}_end"
            end = next((v.text for v in state.findall("value") if v.attrib.get("variable") == end_name), None)
            length = _int(end, pitch)
            values.extend([
                (f"env.section_{sec}.source.x", data["source.x"]),
                (f"env.section_{sec}.source.y", data["source.y"]),
                (f"env.section_{sec}.angle_raw", data["angle_raw"]),
                (f"env.section_{sec}.angle", data["angle"]),
                (f"env.sin_of_section_{sec}_angle", data["sin"]),
                (f"env.cos_of_section_{sec}_angle", data["cos"]),
                (f"env.section_{sec}.drain.x", data["source.x"] + length * data["cos"] // 100),
                (f"env.section_{sec}.drain.y", data["source.y"] + length * data["sin"] // 100),
            ])
        # Connection distances are half the centre-to-centre grid spacing for
        # straight neighbouring sections (the VFM equation uses the sum of
        # the two unit direction vectors).  Emit them for every concrete
        # outgoing slot so the native geometry consumer sees a complete set.
        for value in states[0].findall("value"):
            name = value.attrib.get("variable", "")
            if not name.startswith("env.outgoing_connection_") or "_of_section_" not in name:
                continue
            slot, sec_text = name.split("_of_section_", 1)[0].rsplit("_", 1)[1], name.rsplit("_of_section_", 1)[1]
            target = _int(value.text, -1)
            if target < 0:
                continue
            values.append((f"env.dist_{slot}_of_section_{sec_text}_to_{target}", pitch // 2))
        for name, value in values:
            if name not in existing:
                ET.SubElement(state, "value", variable=name).text = str(value)

    ET.indent(root, space="\t")
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=False)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("grid", type=Path)
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--pitch", type=int, default=40)
    args = parser.parse_args()
    complete(args.trace, args.grid, args.output, args.pitch)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
