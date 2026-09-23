#!/usr/bin/env python3
"""Concretize an abstract grid counterexample with the full VFM dynamics."""

from __future__ import annotations

import argparse
import json
import re
import shutil
from dataclasses import asdict
from pathlib import Path

import networkx as nx

from scalable_parking import parse_states as parse_abstract_states
from vfm_grid_parking import (
    VfmGridParkingConfig,
    generate_vfm_grid_parking_model,
    concrete_milestone_expression,
    write_vfm_render_config,
    save_topology,
    save_topology_png,
    save_trace_gif,
    vehicle,
)
from parking_geometry import GeometryConfig, build_geometry, validate_geometry


def load_abstract_run(run: Path) -> tuple[dict, list[dict[str, str]], nx.Graph]:
    # Current abstract grid runs use grid_config.json; older runs used
    # config.json.  Accept both formats so the pipeline can refine either.
    config_path = run / "grid_config.json"
    if not config_path.exists():
        config_path = run / "config.json"
    config = json.loads(config_path.read_text())
    states = parse_abstract_states(run / "cex_grid_parking.xml")
    graph_data = json.loads((run / "selected_grid.json").read_text())
    graph = nx.node_link_graph(graph_data, edges="edges")
    if not states:
        raise ValueError("the abstract counterexample contains no states")
    return config, states, graph


def _abstract_milestone_indices(states: list[dict[str, str]]) -> list[int]:
    """Return source-state indices retained by the concrete monitor."""
    if "permutation_phase" not in states[0]:
        return list(range(len(states)))
    result = [0]
    previous = int(states[0]["permutation_phase"])
    for index, state in enumerate(states[1:], start=1):
        phase = int(state.get("permutation_phase", previous))
        if phase > previous:
            result.append(index - 1)
            previous = phase
    return result


def abstract_milestones(states: list[dict[str, str]]) -> list[tuple[int, ...]]:
    """Return semantic milestones, or every state for an unphased trace.

    A progress-monitor trace changes ``permutation_phase`` one state after its
    target configuration is observed. In that case, retain the initial
    configuration and the state immediately before every phase increase. The
    concrete VFM model remains free to realize the intervening travel.
    """
    cars = 0
    while f"loc_{cars}" in states[0]:
        cars += 1
    if cars < 2:
        raise ValueError("the abstract trace must contain at least two cars")
    locations: list[tuple[int, ...]] = [
        tuple(int(state[f"loc_{car}"]) for car in range(cars)) for state in states
    ]
    result = [locations[index] for index in _abstract_milestone_indices(states)]
    if len(result) < 2:
        raise ValueError("at least two abstract states are required")
    return result


def abstract_milestone_modes(states: list[dict[str, str]]) -> list[tuple[bool, ...]]:
    """Mark final goals and explicit waits as concrete parked states."""
    cars = 0
    while f"loc_{cars}" in states[0]:
        cars += 1
    indices = _abstract_milestone_indices(states)
    if "permutation_phase" in states[0]:
        # The phase changes only after a complete staged parking
        # configuration has been reached.
        return [tuple(True for _ in range(cars)) for _ in indices]
    modes: list[tuple[bool, ...]] = []
    last = len(states) - 1
    for index in indices:
        modes.append(tuple(
            index == last or (
                index < last
                and states[index][f"loc_{car}"] == states[index + 1][f"loc_{car}"]
            )
            for car in range(cars)
        ))
    return modes


def milestone_expression(pair: tuple[int, ...]) -> str:
    return "(" + " & ".join(
        f"{vehicle(car, 'on_straight_section')} = {section}"
        for car, section in enumerate(pair)
    ) + ")"


def concrete_ltl_property(milestones: list[tuple[int, ...]]) -> str:
    """Require the abstract section states in order, allowing VFM microsteps."""
    expression = milestone_expression(milestones[-1])
    for pair in reversed(milestones[:-1]):
        target = milestone_expression(pair)
        expression = f"((!{target}) U ({target} & X ({expression})))"
    return (
        "-- Concrete refinement of the abstract counterexample.\n"
        f"LTLSPEC !({expression});"
    )


def concrete_reachability_monitor(milestones: list[tuple[int, ...]],
                                  geometry: dict | None = None,
                                  parked_modes: list[tuple[bool, ...]] | None = None) -> str:
    """Compile finite milestones to safety reachability for non-lasso VFM."""
    final = len(milestones) - 1
    parked_modes = parked_modes or [
        tuple(False for _ in milestone) for milestone in milestones
    ]
    if len(parked_modes) != len(milestones):
        raise ValueError("parked modes must align with abstract milestones")
    cases = []
    for phase, pair in enumerate(milestones[1:], start=0):
        expression = (concrete_milestone_expression(
                          pair, geometry, parked_modes[phase + 1])
                      if geometry is not None else milestone_expression(pair))
        cases.append(f"    concretization_phase = {phase} & {expression} : {phase + 1};")
    return "\n".join((
        "-- Progress monitor generated from abstract states; controls remain nondeterministic.",
        "VAR",
        f"  concretization_phase : 0..{final};",
        "ASSIGN",
        "  init(concretization_phase) := 0;",
        "  next(concretization_phase) := case",
        *cases,
        "    TRUE : concretization_phase;",
        "  esac;",
        f"INVARSPEC concretization_phase != {final};",
    ))


def _slot(source: int, target: int, columns: int) -> int:
    difference = target - source
    if difference == -columns:
        return 0  # north
    if difference == 1 and source // columns == target // columns:
        return 1  # east
    if difference == columns:
        return 2  # south
    if difference == -1 and source // columns == target // columns:
        return 3  # west
    raise ValueError(f"edge {source}-{target} is not an orthogonal grid edge")


def fixed_topology_constraints(graph: nx.Graph, rows: int, columns: int) -> str:
    """Fix every VFM north/east/south/west frozen connection slot."""
    selected: dict[tuple[int, int], int] = {}
    for first, second in graph.edges():
        first, second = int(first), int(second)
        selected[(first, _slot(first, second, columns))] = second
        selected[(second, _slot(second, first, columns))] = first
    lines = ["-- Topology concretized from the abstract counterexample."]
    for section in range(rows * columns):
        for slot in range(4):
            value = selected.get((section, slot), -1)
            lines.append(f"INIT outgoing_connection_{slot}_of_section_{section} = {value};")
    return "\n".join(lines) + "\n"


def _rewrite_main(model_file: Path, milestones: list[tuple[int, ...]],
                  config: VfmGridParkingConfig, geometry: dict | None = None,
                  parked_modes: list[tuple[bool, ...]] | None = None,
                  *, false_property: bool = False) -> None:
    text = model_file.read_text()
    specification = (
        "-- Deliberately false smoke-test property.\nINVARSPEC FALSE;"
        if false_property else
        concrete_reachability_monitor(milestones, geometry, parked_modes)
    )
    text, count = re.subn(
        r"(?m)^LTLSPEC .*$",
        specification,
        text, count=1,
    )
    if count != 1:
        raise ValueError(f"expected one LTLSPEC in {model_file}, found {count}")
    node_to_section = geometry.get("start_by_node", {}) if geometry else {}
    initial_modes = (
        tuple(False for _ in milestones[0])
        if false_property else
        parked_modes[0] if parked_modes else tuple(False for _ in milestones[0])
    )
    for car, section in enumerate(milestones[0]):
        initially_parked = bool(initial_modes[car])
        if false_property and geometry is not None:
            # A false-property run is a renderer smoke test, not a refinement
            # of the abstract initial parking state.  Keep every vehicle on
            # the root road section so the native VFM renderer can construct a
            # rooted RoadGraph before it paints the trace.
            concrete_section = 0
            initial_position = max(0, 20 - 15 * car)
            initial_velocity = config.initial_velocity
        elif geometry is not None and initially_parked:
            parking = geometry["parking_by_node"][str(section)]
            concrete_section = int(
                parking["inbound"] if isinstance(parking, dict) else parking
            )
            initial_position = max(
                0, int(geometry["sections"][concrete_section]["length"]) - 5
            )
            initial_velocity = 0
        else:
            concrete_section = int(node_to_section.get(str(section), section))
            initial_position = 0 if geometry is not None else 20
            initial_velocity = config.initial_velocity
        new = f"INIT {vehicle(car, f'is_on_sec_{concrete_section}')} = 1;"
        # The generic initial-state lines have already been commented out by
        # _replace_default_initial_state.  Append an active assignment rather
        # than replacing a substring inside a comment.
        text += f"\n{new}\n"
        # A transit observation starts at the junction-side endpoint of an
        # outgoing road.  A parked observation starts stopped at the end of
        # the inbound parking section.
        text += f"INIT {vehicle(car, 'abs_pos')} = {initial_position};\n"
        text += f"INIT {vehicle(car, 'v')} = {initial_velocity};\n"
    text += "\n-- Section-aware collision constraints for the concrete grid.\n"
    for car in range(1, config.cars):
        text += (
            f"INVAR !(env.veh_0_and_veh_{car}_on_same_seclet & "
            f"env.ego.laterally_overlapping_with_veh_{car} & "
            f"abs({vehicle(car, 'rel_pos')}) <= env.veh_length);\n"
        )
    model_file.write_text(text)


def concretize(abstract_run: Path, *, bound: int, section_length: int,
               connection_length: int, lane_offset: int, junction_radius: int,
               parking_lane_offset: int, output_dir: Path,
               false_property: bool = False):
    abstract_config, abstract_states, graph = load_abstract_run(abstract_run)
    rows = int(abstract_config["rows"])
    columns = int(abstract_config["columns"])
    milestones = abstract_milestones(abstract_states)
    parked_modes = abstract_milestone_modes(abstract_states)
    config = VfmGridParkingConfig(
        rows=rows, columns=columns, section_length=section_length,
        connection_length=connection_length, bound=bound,
        cars=len(milestones[0]),
    )
    geometry = build_geometry(
        json.loads((abstract_run / "selected_grid.json").read_text()),
        GeometryConfig(
            grid_pitch=section_length + 2 * junction_radius,
            road_length=section_length,
            junction_radius=junction_radius,
            lane_offset=lane_offset,
            parking_lane_offset=parking_lane_offset,
        ),
    )
    validate_geometry(geometry)
    model = generate_vfm_grid_parking_model(
        config, experiments_dir=output_dir, concrete_geometry=geometry
    )
    env_file = model.target_path / "EnvModel_GridParking.smv"
    _rewrite_main(
        model.model_file, milestones, config, geometry, parked_modes,
        false_property=false_property,
    )
    manifest = {
        "abstract_run": str(abstract_run.resolve()),
        "abstract_milestones": milestones,
        "parked_modes": parked_modes,
        "false_property": false_property,
        "vfm_config": asdict(config),
    }
    (model.target_path / "concretization.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    (model.target_path / "concrete_geometry.json").write_text(
        json.dumps(geometry, indent=2) + "\n"
    )
    write_vfm_render_config(
        geometry, model.target_path / "vfm_render_config.json"
    )
    shutil.copy2(abstract_run / "cex_grid_parking.xml",
                 model.target_path / "abstract_counterexample.xml")
    shutil.copy2(abstract_run / "selected_grid.json",
                 model.target_path / "abstract_selected_grid.json")
    return model, graph, milestones


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("abstract_run", type=Path,
                        help="directory produced by abstract_grid_yielding.py")
    parser.add_argument("--bound", type=int, default=100)
    parser.add_argument("--section-length", type=int, default=100)
    parser.add_argument("--connection-length", type=int, default=20)
    parser.add_argument("--lane-offset", type=int, default=10,
                        help="perpendicular offset for opposite directed road sections")
    parser.add_argument("--junction-radius", type=int, default=20,
                        help="radius of the connector area around each grid node")
    parser.add_argument("--parking-lane-offset", type=int, default=5,
                        help="half-separation of inbound/outbound parking sections")
    parser.add_argument("--output-dir", type=Path,
                        default=Path("vfm-with-parking/concretized"))
    parser.add_argument("--generate-only", action="store_true")
    parser.add_argument(
        "--false-property", action="store_true",
        help="replace refinement with INVARSPEC FALSE for trace/render smoke tests",
    )
    args = parser.parse_args()
    model, abstract_graph, milestones = concretize(
        args.abstract_run, bound=args.bound,
        section_length=args.section_length,
        connection_length=args.connection_length,
        lane_offset=args.lane_offset,
        junction_radius=args.junction_radius,
        parking_lane_offset=args.parking_lane_offset,
        output_dir=args.output_dir,
        false_property=args.false_property,
    )
    save_topology(nx.MultiDiGraph(abstract_graph), model.target_path)
    save_topology_png(nx.MultiDiGraph(abstract_graph), model.config,
                      model.target_path / "concretized_topology.png")
    print(f"Abstract milestones: {milestones}")
    print(f"Concrete model: {model.model_file.resolve()}")
    if args.generate_only:
        model.morty.close_model_checker()
        return 0
    try:
        counterexample = model.morty.model_check(
            "cex_concretized_vfm.xml", 0, model_file=model.model_file.name,
            bound=model.config.bound, property_kind="invar",
        )
    finally:
        model.morty.close_model_checker()
    if counterexample is None:
        print(f"No concrete witness found through bound {args.bound}")
        return 2
    if args.false_property:
        from drawing_grid import draw_grid

        topology = model.target_path / "concrete_grid.png"
        draw_grid(counterexample, topology)
        print(f"Concrete counterexample: {counterexample}")
        print(f"Text trace: {(model.target_path / 'debug_trace_array.txt').resolve()}")
        print(f"Concrete topology: {topology.resolve()}")
        return 0
    gif = model.target_path / "concretized_vfm.gif"
    save_trace_gif(counterexample, nx.MultiDiGraph(abstract_graph), model.config, gif)
    print(f"Concrete counterexample: {counterexample}")
    print(f"Animation: {gif.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
