#!/usr/bin/env python3
"""Draw a concrete VFM grid from a NuXmv XML counterexample.

The concrete grid model stores the geometry as fixed variables in the trace:
``env.section_<n>.source.x``, ``source.y``, ``angle`` and ``section_<n>_end``.
The outgoing connection variables describe which section endpoints are joined.
This script deliberately does not run VFM or NuXmv; it only reads those values
and creates a small Matplotlib topology plot.

Usage::

    python drawing_grid.py path/to/trace.xml

If ``concrete_geometry.json`` is next to the trace, it is used as a fallback
for section lengths, kinds, and connections.  The XML remains the primary
source, so the script also works with a trace copied without that JSON file.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch
from matplotlib.path import Path as MatplotlibPath

from parking_geometry import connection_curve, section_endpoint


_SOURCE_RE = re.compile(r"^env\.section_(\d+)\.source\.(x|y)$")
_ANGLE_RE = re.compile(r"^env\.section_(\d+)\.angle$")
_RAW_ANGLE_RE = re.compile(r"^env\.section_(\d+)\.angle_raw$")
_END_RE = re.compile(r"^env\.section_(\d+)_end$")
_DRAIN_RE = re.compile(r"^env\.section_(\d+)\.drain\.(x|y)$")
_OUTGOING_RE = re.compile(
    r"^env\.outgoing_connection_(\d+)_of_section_(\d+)$"
)


@dataclass
class Section:
    identifier: int
    source_x: float
    source_y: float
    angle: float
    length: float
    kind: str = "road"
    drain_x: float | None = None
    drain_y: float | None = None

    @property
    def endpoint(self) -> tuple[float, float]:
        if self.drain_x is not None and self.drain_y is not None:
            return self.drain_x, self.drain_y
        return tuple(float(value) for value in section_endpoint(self.geometry()))

    def geometry(self) -> dict:
        return {
            "source_x": int(round(self.source_x)),
            "source_y": int(round(self.source_y)),
            "angle": int(round(self.angle)),
            "length": int(round(self.length)),
        }


def _number(value: str | None) -> float | None:
    if value is None:
        return None
    match = re.search(r"[-+]?\d+(?:\.\d+)?", value)
    return float(match.group(0)) if match else None


def _first_state_values(trace: Path) -> dict[str, str]:
    root = ET.parse(trace).getroot()
    state = root.find(".//state")
    if state is None:
        raise ValueError(f"no state found in trace {trace}")
    return {
        value.get("variable"): (value.text or "")
        for value in state.findall("value")
        if value.get("variable")
    }


def _load_manifest(trace: Path) -> dict:
    manifest = trace.with_name("concrete_geometry.json")
    if manifest.is_file():
        return json.loads(manifest.read_text())

    # Benchmark VFM runs use the abstract grid directly and therefore do not
    # emit concrete_geometry.json. Reconstruct a simple section layout from
    # the selected grid and its nearby VFM configuration.
    selected_grid = None
    config = None
    for parent in (trace.parent, *trace.parents):
        candidate_grid = parent / "selected_grid.json"
        candidate_config = trace.parent / "vfm_grid_parking_config.json"
        if selected_grid is None and candidate_grid.is_file():
            selected_grid = candidate_grid
        if config is None and candidate_config.is_file():
            config = candidate_config
        if selected_grid is not None and config is not None:
            break
    if selected_grid is None:
        return {}
    grid = json.loads(selected_grid.read_text())
    values = json.loads(config.read_text()) if config is not None else {}
    columns = int(values.get("columns", 1))
    pitch = float(values.get("section_length", 100) + 40)
    sections = []
    for node in grid.get("nodes", []):
        identifier = int(node["id"])
        row = int(node.get("row", identifier // columns))
        column = int(node.get("column", identifier % columns))
        sections.append({
            "id": identifier,
            "source_x": column * pitch,
            "source_y": -row * pitch,
            "angle": 0,
            "length": float(values.get("section_length", 100)),
            "kind": "road",
        })
    return {"sections": sections}


def load_trace_geometry(trace: Path) -> tuple[list[Section], dict[int, list[int]]]:
    """Read static section geometry and topology from the first XML state."""
    values = _first_state_values(trace)
    manifest = _load_manifest(trace)
    manifest_sections = {
        int(section["id"]): section for section in manifest.get("sections", [])
    }

    ids: set[int] = set()
    source: dict[int, dict[str, float]] = {}
    angles: dict[int, float] = {}
    lengths: dict[int, float] = {}
    drains: dict[int, dict[str, float]] = {}
    outgoing: dict[int, dict[int, int]] = {}

    for name, text in values.items():
        match = _SOURCE_RE.match(name)
        if match:
            identifier, coordinate = int(match.group(1)), match.group(2)
            number = _number(text)
            if number is not None:
                ids.add(identifier)
                source.setdefault(identifier, {})[coordinate] = number
            continue
        match = _ANGLE_RE.match(name)
        if match and _number(text) is not None:
            identifier = int(match.group(1))
            ids.add(identifier)
            angles[identifier] = _number(text) or 0
            continue
        match = _RAW_ANGLE_RE.match(name)
        if match and _number(text) is not None:
            identifier = int(match.group(1))
            ids.add(identifier)
            angles.setdefault(identifier, (_number(text) or 0) * 45)
            continue
        match = _END_RE.match(name)
        if match and _number(text) is not None:
            identifier = int(match.group(1))
            ids.add(identifier)
            lengths[identifier] = _number(text) or 0
            continue
        match = _DRAIN_RE.match(name)
        if match and _number(text) is not None:
            identifier, coordinate = int(match.group(1)), match.group(2)
            ids.add(identifier)
            drains.setdefault(identifier, {})[coordinate] = _number(text) or 0
            continue
        match = _OUTGOING_RE.match(name)
        if match and _number(text) is not None:
            slot, identifier = int(match.group(1)), int(match.group(2))
            target = int(_number(text) or -1)
            outgoing.setdefault(identifier, {})[slot] = target

    # A copied trace can omit static fields.  The manifest is a safe fallback
    # because it was used to generate the concrete model.
    for identifier, section in manifest_sections.items():
        ids.add(identifier)
        source.setdefault(identifier, {}).setdefault("x", float(section["source_x"]))
        source.setdefault(identifier, {}).setdefault("y", float(section["source_y"]))
        angles.setdefault(identifier, float(section["angle"]))
        lengths.setdefault(identifier, float(section["length"]))

    if not ids:
        raise ValueError("trace contains no section geometry")

    sections: list[Section] = []
    for identifier in sorted(ids):
        metadata = manifest_sections.get(identifier, {})
        point = source.get(identifier, {})
        if "x" not in point or "y" not in point:
            raise ValueError(f"missing source coordinates for section {identifier}")
        sections.append(
            Section(
                identifier=identifier,
                source_x=point["x"],
                source_y=point["y"],
                angle=angles.get(identifier, 0),
                length=lengths.get(identifier, 0),
                kind=str(metadata.get("kind", "road")),
                drain_x=drains.get(identifier, {}).get("x"),
                drain_y=drains.get(identifier, {}).get("y"),
            )
        )

    if not outgoing and manifest.get("outgoing"):
        outgoing = {
            int(identifier): {
                slot: int(target)
                for slot, target in enumerate(targets)
            }
            for identifier, targets in manifest["outgoing"].items()
        }
    normalized = {
        identifier: [target for _, target in sorted(slots.items()) if target >= 0]
        for identifier, slots in outgoing.items()
    }
    return sections, normalized


def draw_grid(
    trace: Path,
    output: Path,
    *,
    labels: bool = True,
    dpi: int = 150,
    show: bool = False,
) -> tuple[int, int]:
    sections, outgoing = load_trace_geometry(trace)
    by_id = {section.identifier: section for section in sections}
    figure, axes = plt.subplots(figsize=(10, 7))

    for section in sections:
        end_x, end_y = section.endpoint
        axes.plot(
            [section.source_x, end_x], [section.source_y, end_y],
            color="black", linewidth=3.0, solid_capstyle="butt", zorder=2,
        )
        if labels:
            middle_x = (section.source_x + end_x) / 2
            middle_y = (section.source_y + end_y) / 2
            axes.text(
                middle_x, middle_y, str(section.identifier), fontsize=8,
                ha="center", va="center", color="white", zorder=5,
                bbox={"boxstyle": "round,pad=0.15", "facecolor": "black", "alpha": 0.7},
            )

    # Junctions are explicit gaps between lane-separated road sections.
    # Draw straight connectors as blue segments and turns/U-turns as cubic
    # curves, using the same construction as the arclength compiler.
    zero_points: set[tuple[float, float]] = set()
    connection_count = 0
    for source_id, target_ids in outgoing.items():
        source_section = by_id.get(source_id)
        if source_section is None:
            continue
        source_x, source_y = source_section.endpoint
        for target_id in target_ids:
            target_section = by_id.get(target_id)
            if target_section is None:
                continue
            target_x, target_y = target_section.source_x, target_section.source_y
            connection_count += 1
            if math.isclose(source_x, target_x) and math.isclose(source_y, target_y):
                zero_points.add((source_x, source_y))
            else:
                kind, points = connection_curve(
                    source_section.geometry(), target_section.geometry()
                )
                if kind == "line":
                    arrow = FancyArrowPatch(
                        points[0], points[1], arrowstyle="-|>", mutation_scale=8,
                        color="tab:blue", linewidth=1.2, alpha=0.72, zorder=3,
                    )
                else:
                    path = MatplotlibPath(
                        list(points),
                        [MatplotlibPath.MOVETO, MatplotlibPath.CURVE4,
                         MatplotlibPath.CURVE4, MatplotlibPath.CURVE4],
                    )
                    arrow = FancyArrowPatch(
                        path=path, arrowstyle="-|>", mutation_scale=8,
                        color="tab:blue", linewidth=1.2, alpha=0.72, zorder=3,
                    )
                axes.add_patch(arrow)

    if zero_points:
        x_values, y_values = zip(*sorted(zero_points))
        axes.scatter(
            x_values, y_values, marker="s", s=42, color="tab:blue",
            edgecolors="white", linewidths=0.5, zorder=4,
        )

    axes.set_aspect("equal", adjustable="datalim")
    axes.grid(True, color="#dddddd", linewidth=0.7, alpha=0.6)
    axes.set_xlabel("x")
    axes.set_ylabel("y")
    axes.set_title(f"Concrete grid topology ({len(sections)} sections, {connection_count} connections)")
    axes.legend(handles=[
        Line2D([0], [0], color="black", linewidth=3, label="section"),
        Line2D([0], [0], color="tab:blue", linewidth=2, label="connection / turn"),
    ], loc="best")
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=dpi)
    if show:
        plt.show()
    plt.close(figure)
    return len(sections), connection_count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path, help="concrete NuXmv XML trace")
    parser.add_argument("--output", type=Path, help="PNG path (default: trace directory/grid.png)")
    parser.add_argument("--no-labels", action="store_true", help="do not label sections with IDs")
    parser.add_argument("--dpi", type=int, default=150)
    parser.add_argument("--show", action="store_true", help="also open an interactive Matplotlib window")
    args = parser.parse_args()
    output = args.output or args.trace.with_name("grid.png")
    sections, connections = draw_grid(
        args.trace, output, labels=not args.no_labels, dpi=args.dpi, show=args.show
    )
    print(f"Sections: {sections}")
    print(f"Connections: {connections}")
    print(f"Grid image: {output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
