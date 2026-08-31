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
    save_topology,
    save_topology_png,
    save_trace_gif,
    vehicle,
)


def load_abstract_run(run: Path) -> tuple[dict, list[dict[str, str]], nx.Graph]:
    config = json.loads((run / "config.json").read_text())
    states = parse_abstract_states(run / "cex_grid_parking.xml")
    graph_data = json.loads((run / "selected_grid.json").read_text())
    graph = nx.node_link_graph(graph_data, edges="edges")
    if not states:
        raise ValueError("the abstract counterexample contains no states")
    return config, states, graph


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
    locations = [
        tuple(int(state[f"loc_{car}"]) for car in range(cars)) for state in states
    ]
    if "permutation_phase" in states[0]:
        result = [locations[0]]
        previous = int(states[0]["permutation_phase"])
        for index, state in enumerate(states[1:], start=1):
            phase = int(state.get("permutation_phase", previous))
            if phase > previous:
                result.append(locations[index - 1])
                previous = phase
    else:
        result = locations
    if len(result) < 2:
        raise ValueError("at least two abstract states are required")
    return result


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


def concrete_reachability_monitor(milestones: list[tuple[int, ...]]) -> str:
    """Compile finite milestones to safety reachability for non-lasso VFM."""
    final = len(milestones) - 1
    cases = []
    for phase, pair in enumerate(milestones[1:], start=0):
        cases.append(
            f"    concretization_phase = {phase} & {milestone_expression(pair)} : {phase + 1};"
        )
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
                  config: VfmGridParkingConfig) -> None:
    text = model_file.read_text()
    text, count = re.subn(
        r"(?m)^LTLSPEC .*$", concrete_reachability_monitor(milestones), text, count=1
    )
    if count != 1:
        raise ValueError(f"expected one LTLSPEC in {model_file}, found {count}")
    for car, section in enumerate(milestones[0]):
        old = f"INIT {vehicle(car, 'is_on_sec_0')} = 1;"
        new = f"INIT {vehicle(car, f'is_on_sec_{section}')} = 1;"
        if old in text:
            text = text.replace(old, new)
        else:
            text += f"\n{new}\nINIT {vehicle(car, 'abs_pos')} = 20;\n"
            text += f"INIT {vehicle(car, 'v')} = {config.initial_velocity};\n"
    text += "\n-- Concrete traces must refine the maneuver without a collision.\nINVAR !env.crash;\n"
    model_file.write_text(text)


def concretize(abstract_run: Path, *, bound: int, section_length: int,
               connection_length: int, output_dir: Path):
    abstract_config, abstract_states, graph = load_abstract_run(abstract_run)
    rows = int(abstract_config["rows"])
    columns = int(abstract_config["columns"])
    milestones = abstract_milestones(abstract_states)
    config = VfmGridParkingConfig(
        rows=rows, columns=columns, section_length=section_length,
        connection_length=connection_length, bound=bound,
        cars=len(milestones[0]),
    )
    model = generate_vfm_grid_parking_model(config, experiments_dir=output_dir)
    env_file = model.target_path / "EnvModel_GridParking.smv"
    with env_file.open("a") as stream:
        stream.write("\n" + fixed_topology_constraints(graph, rows, columns))
    _rewrite_main(model.model_file, milestones, config)
    manifest = {
        "abstract_run": str(abstract_run.resolve()),
        "abstract_milestones": milestones,
        "vfm_config": asdict(config),
    }
    (model.target_path / "concretization.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
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
    parser.add_argument("--output-dir", type=Path,
                        default=Path("vfm-with-parking/concretized"))
    parser.add_argument("--generate-only", action="store_true")
    args = parser.parse_args()
    model, abstract_graph, milestones = concretize(
        args.abstract_run, bound=args.bound,
        section_length=args.section_length,
        connection_length=args.connection_length,
        output_dir=args.output_dir,
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
    gif = model.target_path / "concretized_vfm.gif"
    save_trace_gif(counterexample, nx.MultiDiGraph(abstract_graph), model.config, gif)
    print(f"Concrete counterexample: {counterexample}")
    print(f"Animation: {gif.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
