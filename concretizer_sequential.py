#!/usr/bin/env python3
"""Sequential concrete refinement for an abstract grid counterexample.

The ordinary concretizer encodes every abstract milestone in one monitor.  This
variant keeps the generated VFM model fixed and checks one milestone at a time:
the terminal concrete state of segment *i* becomes the ``INIT`` state of
segment *i+1*, while the target is checked with ``INVARSPEC !(target)``.
"""
from __future__ import annotations

import argparse
import json
import re
import shutil
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from concretizer import (
    abstract_milestone_modes,
    abstract_milestones,
    load_abstract_run,
)
from parking_geometry import GeometryConfig, build_geometry, validate_geometry
from scalable_parking import parse_states as parse_xml_states
from vfm_grid_parking import (
    VfmGridParkingModel,
    VfmGridParkingConfig,
    concrete_milestone_expression,
    generate_vfm_grid_parking_model,
    save_topology_png,
    save_topology,
    vehicle,
    write_vfm_render_config,
)
from model_checking import Morty
import networkx as nx


def _initial_assignments(pair, modes, geometry, config) -> list[str]:
    """Translate the first abstract state into concrete vehicle INITs."""
    result: list[str] = []
    node_to_section = geometry.get("start_by_node", {})
    for car, node in enumerate(pair):
        if modes[car]:
            parking = geometry["parking_by_node"][str(node)]
            section = int(parking["inbound"] if isinstance(parking, dict) else parking)
            position = max(0, int(geometry["sections"][section]["length"]) - 5)
            velocity = 0
        else:
            section = int(node_to_section.get(str(node), node))
            position = 0
            velocity = config.initial_velocity
        result.append(f"INIT {vehicle(car, f'is_on_sec_{section}')} = 1;")
        result.append(f"INIT {vehicle(car, 'abs_pos')} = {position};")
        result.append(f"INIT {vehicle(car, 'v')} = {velocity};")
    return result


_TRANSFER_FIELDS = {
    # Keep physical longitudinal state.  Lane-change bookkeeping is model
    # policy state and must be reinitialized by the concrete model; copying it
    # can replay a value outside the regenerated MIN_TIME domain.
    "abs_pos", "prev_abs_pos", "prev_rel_pos", "a", "v",
}


def _terminal_assignments(state: dict[str, str], cars: int) -> list[str]:
    """Copy the concrete car state needed to continue the next segment.

    Derived values (``on_straight_section``, traversal flags, gap defines, and
    geometry constants) are intentionally omitted; they are recomputed by the
    model from the copied vehicle state and fixed topology.
    """
    result: list[str] = []
    for car in range(cars):
        prefix = vehicle(car, "")
        for name, value in sorted(state.items()):
            if not name.startswith(prefix):
                continue
            field = name[len(prefix):]
            if field.startswith("is_on_sec_") or field in _TRANSFER_FIELDS:
                if re.fullmatch(r"-?\d+|TRUE|FALSE", value):
                    result.append(f"INIT {name} = {value};")
    if not result:
        raise ValueError("concrete trace terminal state contained no transferable vehicle variables")
    return result


def _rewrite_segment(baseline: str, target: str, assignments: list[str], output: Path) -> None:
    text, count = re.subn(r"(?m)^LTLSPEC .*$", f"INVARSPEC !({target});", baseline, count=1)
    if count != 1:
        raise ValueError("generated model does not contain exactly one LTLSPEC")
    text += "\n-- Sequential concretization state.\n" + "\n".join(assignments) + "\n"
    output.write_text(text)


def _merge_xml(segment_paths: list[Path], output: Path) -> None:
    root = ET.Element("counter-example", {"type": "sequential", "id": "1"})
    next_id = 1
    for path in segment_paths:
        source = ET.parse(path).getroot()
        for node in source.findall("node"):
            copied = ET.fromstring(ET.tostring(node, encoding="unicode"))
            state = copied.find("state")
            if state is not None:
                state.set("id", str(next_id))
                next_id += 1
            root.append(copied)
    ET.ElementTree(root).write(output, encoding="unicode")


def run(abstract_run: Path, *, bound: int, output_dir: Path,
        section_length: int = 100, connection_length: int = 20,
        lane_offset: int = 10, junction_radius: int = 20,
        parking_lane_offset: int = 5, existing_model: Path | None = None,
        existing_geometry: Path | None = None,
        input_config: Path = Path("src/templates/envmodel_config.json")) -> Path | None:
    abstract_config, states, graph = load_abstract_run(abstract_run)
    milestones = abstract_milestones(states)
    modes = abstract_milestone_modes(states)
    config = VfmGridParkingConfig(
        rows=int(abstract_config["rows"]), columns=int(abstract_config["columns"]),
        section_length=section_length, connection_length=connection_length,
        bound=bound, cars=len(milestones[0]),
    )
    geometry_path = existing_geometry
    if geometry_path is None and existing_model is not None:
        geometry_path = existing_model.parent / "concrete_geometry.json"
    if geometry_path is not None and geometry_path.exists():
        geometry = json.loads(geometry_path.read_text())
    else:
        geometry = build_geometry(
            json.loads((abstract_run / "selected_grid.json").read_text()),
            GeometryConfig(
                grid_pitch=section_length + 2 * junction_radius,
                road_length=section_length, junction_radius=junction_radius,
                lane_offset=lane_offset, parking_lane_offset=parking_lane_offset,
            ),
        )
    validate_geometry(geometry)
    if existing_model is not None:
        model_file = existing_model.resolve()
        if not model_file.is_file():
            raise FileNotFoundError(model_file)
        morty = Morty()
        # Morty.model_check uses args[5] as the directory containing the SMV.
        morty.args = ["", "", "", "", "", str(model_file.parent)]
        model = VfmGridParkingModel(morty, config, model_file)
    else:
        output_dir.mkdir(parents=True, exist_ok=True)
        model = generate_vfm_grid_parking_model(
            config, input_config=input_config,
            experiments_dir=output_dir / "vfm", concrete_geometry=geometry
        )
    baseline = model.model_file.read_text()
    target_dir = model.target_path
    (target_dir / "concretization_sequential.json").write_text(json.dumps({
        "abstract_run": str(abstract_run.resolve()),
        "milestones": milestones, "parked_modes": modes, "bound": bound,
    }, indent=2) + "\n")
    (target_dir / "concrete_geometry.json").write_text(json.dumps(geometry, indent=2) + "\n")
    write_vfm_render_config(geometry, target_dir / "vfm_render_config.json")
    save_topology(nx.MultiDiGraph(graph), target_dir)
    save_topology_png(nx.MultiDiGraph(graph), config, target_dir / "concrete_grid.png")

    prior = _initial_assignments(milestones[0], modes[0], geometry, config)
    segments: list[Path] = []
    summary = []
    try:
        for index, target_pair in enumerate(milestones[1:], start=1):
            target = concrete_milestone_expression(target_pair, geometry, modes[index])
            segment_model = target_dir / f"segment_{index:03d}.smv"
            _rewrite_segment(baseline, target, prior, segment_model)
            started = time.perf_counter()
            cex = model.morty.model_check(
                f"segment_{index:03d}.xml", 0,
                model_file=segment_model.name, bound=bound, property_kind="invar",
            )
            elapsed = time.perf_counter() - started
            model.morty.close_model_checker()
            if cex is None:
                summary.append({"segment": index, "status": "no_witness", "seconds": elapsed})
                break
            destination = target_dir / f"segment_{index:03d}.xml"
            if cex.resolve() != destination.resolve():
                shutil.copy2(cex, destination)
            states_now = parse_xml_states(destination)
            if not states_now:
                raise RuntimeError(f"segment {index} produced an empty trace")
            prior = _terminal_assignments(states_now[-1], len(milestones[0]))
            segments.append(destination)
            summary.append({"segment": index, "status": "counterexample",
                            "seconds": elapsed, "states": len(states_now)})
            print(f"segment {index}/{len(milestones)-1}: {len(states_now)} states, {elapsed:.2f}s",
                  flush=True)
    finally:
        model.morty.close_model_checker()
    (target_dir / "segments.json").write_text(json.dumps(summary, indent=2) + "\n")
    if len(segments) != len(milestones) - 1:
        return None
    combined = target_dir / "sequential_cex.xml"
    _merge_xml(segments, combined)
    print(f"Sequential concrete CEX: {combined.resolve()}")
    return combined


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("abstract_run", type=Path)
    parser.add_argument("--bound", type=int, default=100)
    parser.add_argument("--output-dir", type=Path, default=Path("vfm-with-parking/sequential"))
    parser.add_argument(
        "--existing-model", type=Path,
        help="reuse an already generated fixed-topology SMV and skip VFM generation",
    )
    parser.add_argument(
        "--geometry", type=Path,
        help="concrete_geometry.json (defaults to the existing model directory)",
    )
    parser.add_argument(
        "--input-config", type=Path,
        default=Path("src/templates/envmodel_config.json"),
        help="scenario-generator JSON config (used only when generating an SMV)",
    )
    args = parser.parse_args()
    return 0 if run(
        args.abstract_run, bound=args.bound, output_dir=args.output_dir,
        existing_model=args.existing_model, existing_geometry=args.geometry,
        input_config=args.input_config,
    ) else 2


if __name__ == "__main__":
    raise SystemExit(main())
