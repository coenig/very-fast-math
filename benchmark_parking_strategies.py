#!/usr/bin/env python3
"""Benchmark topology enumeration, abstraction/refinement, and direct VFM search."""

from __future__ import annotations

import argparse
import contextlib
import csv
import itertools
import json
import multiprocessing as mp
import os
import re
import resource
import shutil
import signal
import time
import traceback
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict
from pathlib import Path

import networkx as nx

from concretizer import (
    _slot, abstract_milestones, concrete_ltl_property,
    concrete_reachability_monitor, concrete_milestone_expression,
    abstract_milestone_modes, _anchor_geometry_at_section_zero,
)
from generate_scenarios import (
    _at_least, _distance, render_role_smv, save_gif as save_abstract_gif,
    selected_graph, trace_used_graph,
)
from parking_benchmark_scenarios import (
    BenchmarkCase, load_benchmark_cases, stages_to_monitor,
)
from scalable_grid_parking import check_model, edge_name, grid_edges
from scalable_parking import parse_states as parse_abstract_states
from vfm_grid_parking import (
    VfmGridParkingConfig, generate_vfm_grid_parking_model, parse_states,
    save_trace_gif, vehicle, write_vfm_render_config, _edge_topology_constraints,
)
from parking_geometry import (
    GeometryConfig, build_candidate_geometry, restrict_geometry,
)


ROOT = Path(__file__).resolve().parent
DEFAULT_CATALOG = ROOT / "parking_strategy_benchmarks.toml"
DEFAULT_OUTPUT = ROOT / "vfm-with-parking" / "three-way-benchmarks"
METHODS = ("enumerated_vfm", "abstract_concretized", "symbolic_vfm")


def _edge_slots(first: int, second: int, columns: int) -> tuple[str, str]:
    return (
        f"env.outgoing_connection_{_slot(first, second, columns)}_of_section_{first}",
        f"env.outgoing_connection_{_slot(second, first, columns)}_of_section_{second}",
    )


def edge_enabled(first: int, second: int, columns: int) -> str:
    forward, reverse = _edge_slots(first, second, columns)
    return f"({forward} = {second} & {reverse} = {first})"


def symbolic_topology_constraints(case: BenchmarkCase) -> str:
    spec = case.spec
    edges = grid_edges(spec)  # type: ignore[arg-type]
    terms = []
    lines = ["-- Bidirectional optional partial-grid topology."]
    for first, second in edges:
        forward, reverse = _edge_slots(first, second, spec.columns)
        lines.append(
            f"INIT (({forward} = {second} & {reverse} = {first}) | "
            f"({forward} = -1 & {reverse} = -1));"
        )
        terms.append(f"(case {edge_enabled(first, second, spec.columns)} : 1; TRUE : 0; esac)")
    lines.extend((
        "DEFINE",
        "  benchmark_enabled_edge_count := " + " + ".join(terms) + ";",
        f"INVAR benchmark_enabled_edge_count <= {spec.maximum_edges};",
    ))
    return "\n".join(lines)


def fixed_topology_constraints(case: BenchmarkCase,
                               enabled: set[tuple[int, int]]) -> str:
    spec = case.spec
    selected: dict[tuple[int, int], int] = {}
    for first, second in enabled:
        selected[(first, _slot(first, second, spec.columns))] = second
        selected[(second, _slot(second, first, spec.columns))] = first
    lines = ["-- Externally fixed bidirectional partial-grid topology."]
    for section in range(spec.nodes):
        for slot in range(4):
            lines.append(
                f"INIT env.outgoing_connection_{slot}_of_section_{section} = "
                f"{selected.get((section, slot), -1)};"
            )
    return "\n".join(lines)


def concrete_role_constraints(case: BenchmarkCase,
                              fixed_roles: dict[str, int] | None = None) -> str:
    spec, prop = case.spec, case.prop
    edges = grid_edges(spec)  # type: ignore[arg-type]
    incident: dict[int, list[str]] = {node: [] for node in range(spec.nodes)}
    for first, second in edges:
        enabled = edge_enabled(first, second, spec.columns)
        incident[first].append(enabled)
        incident[second].append(enabled)
    lines = ["FROZENVAR"]
    lines.extend(f"  {role.name} : 0..{spec.nodes - 1};" for role in prop.roles)
    # ``first_row`` is a shared benchmark condition, so roles denoting an
    # initial parking position are the corresponding concrete start cell.
    lines.extend(f"INIT {role.name} = {spec.cars - 1 - role.initial_car};"
                 for role in prop.roles if role.initial_car is not None)
    if fixed_roles is not None:
        lines.extend(f"INIT {name} = {value};" for name, value in sorted(fixed_roles.items()))
    for index, first in enumerate(prop.roles):
        for second in prop.roles[index + 1:]:
            lines.append(f"INIT {first.name} != {second.name};")
    initial_roles = {role.initial_car: role.name for role in prop.roles
                     if role.initial_car is not None}
    fixed_starts = {car: car for car in range(spec.cars) if car not in initial_roles}
    for role in prop.roles:
        if role.exclude_initial:
            for start in range(spec.cars):
                lines.append(f"INIT {role.name} != {start};")
        if role.minimum_start_distance:
            for cell in range(spec.nodes):
                if any(_distance(cell, start, spec.columns) < role.minimum_start_distance
                       for start in fixed_starts.values()):
                    lines.append(f"INIT {role.name} != {cell};")
            for initial_role in initial_roles.values():
                if initial_role == role.name:
                    continue
                for first in range(spec.nodes):
                    for second in range(spec.nodes):
                        if _distance(first, second, spec.columns) < role.minimum_start_distance:
                            lines.append(
                                f"INIT !({role.name} = {first} & {initial_role} = {second});"
                            )
        if role.minimum_degree:
            for cell in range(spec.nodes):
                lines.append(
                    f"INIT {role.name} = {cell} -> "
                    f"{_at_least(incident[cell], role.minimum_degree)};"
                )
    for distance in prop.role_distances:
        for first in range(spec.nodes):
            for second in range(spec.nodes):
                if _distance(first, second, spec.columns) < distance.minimum:
                    lines.append(
                        f"INIT !({distance.first} = {first} & {distance.second} = {second});"
                    )
    return "\n".join(lines)


def specialize_vfm_main(model_file: Path, case: BenchmarkCase, *,
                        topology: str, fixed_roles: dict[str, int] | None = None,
                        refinement: str = "monitor",
                        milestones: list[tuple[int, ...]] | None = None,
                        section_length: int = 100) -> None:
    text = model_file.read_text()
    for car in range(case.spec.cars):
        prefix = re.escape(f"env.veh___6{car}9___")
        text = re.sub(rf"(?m)^INIT {prefix}\.is_on_sec_\d+\s*=.*;$",
                      lambda match: "-- benchmark replaced: " + match.group(0), text)
        text = re.sub(rf"(?m)^INIT {prefix}\.abs_pos\s*=.*;$",
                      lambda match: "-- benchmark replaced: " + match.group(0), text)
        text = re.sub(rf"(?m)^INIT {prefix}\.v\s*=.*;$",
                      lambda match: "-- benchmark replaced: " + match.group(0), text)
    if refinement not in {"stages", "monitor", "ltl"}:
        raise ValueError(f"unknown refinement mode: {refinement}")
    if refinement == "ltl":
        if not milestones:
            raise ValueError("LTL refinement requires abstract milestones")
        property_text = concrete_ltl_property(milestones)
    elif refinement == "monitor" and milestones:
        property_text = concrete_reachability_monitor(milestones)
    else:
        property_text = stages_to_monitor(case.stages)
    additions = ["", concrete_role_constraints(case, fixed_roles), topology,
                 "-- Benchmark initial state and admissibility."]
    front_position = max(2, int(round(0.6 * section_length)))
    rear_position = max(1, int(round(0.2 * section_length)))
    if rear_position >= front_position:
        rear_position = max(0, front_position - 1)
    for car in range(case.spec.cars):
        additions.extend((
            f"INIT {vehicle(car, f'is_on_sec_{case.spec.cars - 1 - car}')} = 1;",
            f"INIT {vehicle(car, 'abs_pos')} = {front_position if car == 0 else rear_position};",
            f"INIT {vehicle(car, 'v')} = 5;",
            f"INVAR {vehicle(car, 'v')} >= 0;",
        ))
    additions.append("INVAR !env.crash;")
    # The generated VFM model has one original LTLSPEC.  For monitor mode the
    # replacement contains VAR/ASSIGN/INVARSPEC; for LTL mode it is LTLSPEC.
    text, count = re.subn(r"(?m)^LTLSPEC .*$", property_text, text, count=1)
    if count != 1:
        raise ValueError(f"expected one LTLSPEC in {model_file}, found {count}")
    model_file.write_text(text + "\n" + "\n".join(additions) + "\n")


def render_abstract_model(case: BenchmarkCase) -> str:
    text = render_role_smv(case.spec, case.prop)
    # VFM's established first-row convention numbers the front car first,
    # hence car 0 starts at the highest occupied cell.
    for car in range(case.spec.cars):
        role = next((item.name for item in case.prop.roles
                     if item.initial_car == car), None)
        old = role or str(car)
        start = case.spec.cars - 1 - car
        text = text.replace(f"INIT loc_{car} = {old};", f"INIT loc_{car} = {start};")
    fixed_initial_roles = [
        f"INIT {role.name} = {case.spec.cars - 1 - role.initial_car};"
        for role in case.prop.roles if role.initial_car is not None
    ]
    return text + "\n" + "\n".join(fixed_initial_roles) + "\n"


def generate_concrete_base(case: BenchmarkCase, run_dir: Path,
                           section_length: int):
    initial_a_position = max(2, int(round(0.6 * section_length)))
    initial_b_position = max(1, int(round(0.2 * section_length)))
    if initial_b_position >= initial_a_position:
        initial_b_position = max(0, initial_a_position - 1)
    config = VfmGridParkingConfig(
        rows=case.spec.rows, columns=case.spec.columns, cars=case.spec.cars,
        section_length=section_length, connection_length=20, bound=case.spec.bound,
        initial_a_position=initial_a_position,
        initial_b_position=initial_b_position,
    )
    # VFM's generator treats an absolute output path as if it were relative
    # (prefixing it with ``./``), while the kratos post-pass does not.  Passing
    # one cwd-relative path keeps both halves of generation in the same folder.
    relative_output = Path(os.path.relpath(run_dir.resolve(), ROOT))
    return generate_vfm_grid_parking_model(config, experiments_dir=relative_output)


def check_concrete(model, model_file: Path, cex_name: str, *, property_kind: str = "invar") -> Path | None:
    try:
        return model.morty.model_check(
            cex_name, 0, model_file=model_file.name,
            bound=model.config.bound, property_kind=property_kind,
        )
    finally:
        model.morty.close_model_checker()


def graph_from_edges(case: BenchmarkCase, enabled: set[tuple[int, int]]) -> nx.Graph:
    graph = nx.Graph()
    for node in range(case.spec.nodes):
        row, column = divmod(node, case.spec.columns)
        graph.add_node(node, row=row, column=column)
    graph.add_edges_from(enabled)
    return graph


def _rich_graph_from_state(case: BenchmarkCase, geometry: dict,
                           state: dict[str, str]) -> nx.Graph:
    graph = graph_from_edges(case, set())
    for index, edge in enumerate(geometry.get("candidate_edges", [])):
        if state.get(f"rich_edge_{index}") == "TRUE":
            graph.add_edge(int(edge["source"]), int(edge["target"]))
    return graph


def iter_topologies(case: BenchmarkCase):
    edges = tuple(tuple(sorted(edge)) for edge in grid_edges(case.spec))  # type: ignore[arg-type]
    index = 0
    for count in range(min(case.spec.maximum_edges, len(edges)) + 1):
        for chosen in itertools.combinations(edges, count):
            yield index, set(chosen)
            index += 1


def candidate_structurally_possible(case: BenchmarkCase,
                                    enabled: set[tuple[int, int]]) -> bool:
    """Soundly skip grids that cannot assign reachable semantic roles.

    This is not a scenario search: it only checks static role domains,
    distances, degrees, and undirected reachability.  Every candidate that
    survives is still fixed and checked by full VFM.
    """
    graph = graph_from_edges(case, enabled)
    starts = {car: case.spec.cars - 1 - car for car in range(case.spec.cars)}
    referenced_by: dict[str, set[int]] = {role.name: set() for role in case.prop.roles}
    for stage in case.stages:
        for atom in (*stage.event, *stage.hold):
            referenced_by[atom.role].add(atom.car)
    domains: dict[str, list[int]] = {}
    initial_cells = set(starts.values())
    for role in case.prop.roles:
        if role.initial_car is not None:
            domains[role.name] = [starts[role.initial_car]]
            continue
        values = []
        for cell in range(case.spec.nodes):
            if role.exclude_initial and cell in initial_cells:
                continue
            if role.minimum_degree and graph.degree(cell) < role.minimum_degree:
                continue
            if role.minimum_start_distance and any(
                _distance(cell, start, case.spec.columns) < role.minimum_start_distance
                for start in initial_cells
            ):
                continue
            if any(not nx.has_path(graph, starts[car], cell)
                   for car in referenced_by[role.name]):
                continue
            values.append(cell)
        domains[role.name] = values
        if not values:
            return False
    distances = case.prop.role_distances
    ordered = sorted(case.prop.roles, key=lambda role: len(domains[role.name]))

    def assign(index: int, chosen: dict[str, int]) -> bool:
        if index == len(ordered):
            return True
        name = ordered[index].name
        for cell in domains[name]:
            if cell in chosen.values():
                continue
            valid = True
            for constraint in distances:
                other = (constraint.second if constraint.first == name else
                         constraint.first if constraint.second == name else None)
                if other in chosen and _distance(cell, chosen[other], case.spec.columns) < constraint.minimum:
                    valid = False
                    break
            if valid:
                chosen[name] = cell
                if assign(index + 1, chosen):
                    return True
                del chosen[name]
        return False

    return assign(0, {})


def _base_result(case: BenchmarkCase, method: str, run_dir: Path) -> dict:
    return {
        "case": case.identity, "tier": case.tier, "method": method,
        "rows": case.spec.rows, "columns": case.spec.columns,
        "cars": case.spec.cars, "bound": case.spec.bound,
        "maximum_edges": case.spec.maximum_edges, "status": "error",
        "total_time_to_concrete_xml_seconds": "",
        "abstract_time_to_xml_seconds": "", "concrete_generation_seconds": "",
        "concrete_checking_seconds": "", "visualization_seconds": "",
        "abstract_visualization_seconds": "",
        "candidates_checked": 0, "nuxmv_invocations": 0,
        "candidates_enumerated": 0, "candidates_pruned_structurally": 0,
        "abstract_states": "", "concrete_states": "", "selected_edges": "",
        "smv_lines": "", "smv_bytes": "", "peak_child_rss_kb": "",
        "run_directory": str(run_dir.resolve()), "error": "",
        "concrete_layout": "legacy",
    }


def _save_graph(graph: nx.Graph, run_dir: Path) -> None:
    nx.write_graphml(graph, run_dir / "selected_grid.graphml")
    (run_dir / "selected_grid.json").write_text(
        json.dumps(nx.node_link_data(graph, edges="edges"), indent=2) + "\n"
    )


def _finish_concrete(result: dict, cex: Path, graph: nx.Graph, model,
                     run_dir: Path, geometry: dict | None = None) -> None:
    target = run_dir / "concrete_counterexample.xml"
    shutil.copy2(cex, target)
    result["concrete_states"] = len(parse_states(target))
    result["selected_edges"] = graph.number_of_edges()
    _save_graph(graph, run_dir)
    visualization_started = time.perf_counter()
    try:
        save_trace_gif(target, nx.MultiDiGraph(graph), model.config,
                       run_dir / "concrete_scenario.gif", geometry=geometry)
    except Exception as error:
        (run_dir / "visualization_error.txt").write_text(str(error) + "\n")
    result["visualization_seconds"] = time.perf_counter() - visualization_started


def _rich_model(case: BenchmarkCase, run_dir: Path, geometry: dict,
                section_length: int):
    """Generate a VFM model over the directed-section geometry universe."""
    run_dir.mkdir(parents=True, exist_ok=True)
    config = VfmGridParkingConfig(
        rows=case.spec.rows, columns=case.spec.columns, cars=case.spec.cars,
        section_length=section_length, connection_length=20,
        bound=case.spec.bound,
        initial_a_position=max(2, int(round(0.6 * section_length))),
        initial_b_position=max(1, int(round(0.2 * section_length))),
    )
    relative = Path(os.path.relpath(run_dir.resolve(), ROOT))
    generation_config = run_dir / "rich_generation_config.json"
    write_vfm_render_config(geometry, generation_config)
    (run_dir / "concrete_geometry.json").write_text(
        json.dumps(geometry, indent=2) + "\n"
    )
    return generate_vfm_grid_parking_model(
        config, input_config=generation_config,
        experiments_dir=relative, concrete_geometry=geometry,
    )


def _rich_universe(case: BenchmarkCase, section_length: int) -> dict:
    grid = {"nodes": [
        {"id": node, "row": node // case.spec.columns,
         "column": node % case.spec.columns}
        for node in range(case.spec.nodes)
    ]}
    geometry = build_candidate_geometry(
        grid, GeometryConfig(
            grid_pitch=section_length + 40, road_length=section_length,
        )
    )
    _anchor_geometry_at_section_zero(geometry)
    return geometry


def _specialize_rich_main(model_file: Path, case: BenchmarkCase,
                          geometry: dict, milestones: list[tuple[int, ...]],
                          refinement: str,
                          parked_modes: list[tuple[bool, ...]]) -> None:
    """Install a monitor/LTL refinement and concrete initial states."""
    text = model_file.read_text()
    if refinement == "ltl":
        property_text = concrete_ltl_property(milestones)
    else:
        property_text = concrete_reachability_monitor(
            milestones, geometry, parked_modes
        )
    text, count = re.subn(r"(?m)^LTLSPEC .*$", property_text, text, count=1)
    if count != 1:
        raise ValueError(f"expected one LTLSPEC in {model_file}, found {count}")
    # The rich model has no default initial vehicle assignment.  Translate the
    # first abstract state to a road/parking section using the geometry map.
    initial = parked_modes[0]
    for car, node in enumerate(milestones[0]):
        if initial[car]:
            section = int(geometry["parking_by_node"][str(node)]["inbound"])
            position = max(0, int(geometry["sections"][section]["length"]) - 5)
            velocity = 0
        else:
            section = int(geometry["start_by_node"].get(str(node),
                           geometry["parking_by_node"][str(node)]["inbound"]))
            position = 0
            velocity = 5
        text += (
            f"\nINIT {vehicle(car, f'is_on_sec_{section}')} = 1;"
            f"\nINIT {vehicle(car, 'abs_pos')} = {position};"
            f"\nINIT {vehicle(car, 'v')} = {velocity};"
            f"\nINVAR {vehicle(car, 'v')} >= 0;"
        )
    text += "\nINVAR !env.crash;\n"
    model_file.write_text(text)


def _specialize_rich_stage_main(model_file: Path, case: BenchmarkCase,
                                geometry: dict,
                                fixed_roles: dict[str, int] | None = None) -> None:
    """Install a role-stage property on a rich concrete model."""
    text = model_file.read_text()
    property_text = _rich_stage_property(case, geometry, fixed_roles)
    text, count = re.subn(r"(?m)^LTLSPEC .*$", property_text, text, count=1)
    if count != 1:
        raise ValueError(f"expected one LTLSPEC in {model_file}, found {count}")
    if fixed_roles is None:
        text += _rich_symbolic_roles(case, geometry)
    starts = {
        role.initial_car: int(value)
        for role in case.prop.roles
        if role.initial_car is not None
        for value in [case.spec.cars - 1 - role.initial_car]
    }
    for car in range(case.spec.cars):
        node = starts.get(car, case.spec.cars - 1 - car)
        section = geometry["start_by_node"].get(str(node))
        if section is None:
            section = geometry["parking_by_node"][str(node)]["inbound"]
        text += (
            f"\nINIT {vehicle(car, f'is_on_sec_{int(section)}')} = 1;"
            f"\nINIT {vehicle(car, 'abs_pos')} = 0;"
            f"\nINIT {vehicle(car, 'v')} = 5;"
            f"\nINVAR {vehicle(car, 'v')} >= 0;"
        )
    text += "\nINVAR !env.crash;\n"
    model_file.write_text(text)


def _rich_role_atom(atom, geometry: dict, fixed_roles: dict[str, int] | None = None) -> str:
    """Translate a symbolic role atom to concrete section observations."""
    role_value = fixed_roles.get(atom.role) if fixed_roles is not None else None
    nodes = [role_value] if role_value is not None else [
        int(node["id"]) for node in geometry["nodes"]
    ]
    terms = []
    for node in nodes:
        observation = concrete_milestone_expression(
            (node,), geometry, (False,)
        )
        if role_value is not None:
            terms.append(observation if not atom.negated else f"!{observation}")
        else:
            terms.append(f"({atom.role} = {node} & "
                         f"{observation if not atom.negated else '!' + observation})")
    return "(" + " | ".join(terms) + ")" if terms else "FALSE"


def _rich_stage_property(case: BenchmarkCase, geometry: dict,
                         fixed_roles: dict[str, int] | None = None) -> str:
    """Build the benchmark's role-based history over rich concrete sections."""
    final = len(case.stages)
    dead = final + 1
    cases = []
    for index, stage in enumerate(case.stages):
        event = "(" + " & ".join(
            _rich_role_atom(atom, geometry, fixed_roles) for atom in stage.event
        ) + ")"
        cases.append(f"    parking_phase = {index} & {event} : {index + 1};")
        if stage.hold:
            hold = "(" + " & ".join(
                _rich_role_atom(atom, geometry, fixed_roles) for atom in stage.hold
            ) + ")"
            cases.append(f"    parking_phase = {index} & !{hold} : {dead};")
    return "\n".join((
        "-- Rich concrete observer compiled from symbolic role stages.",
        "VAR", f"  parking_phase : 0..{dead};", "ASSIGN",
        "  init(parking_phase) := 0;", "  next(parking_phase) := case",
        *cases, "    TRUE : parking_phase;", "  esac;",
        f"INVARSPEC parking_phase != {final};",
    ))


def _rich_symbolic_roles(case: BenchmarkCase, geometry: dict) -> str:
    """Declare symbolic role locations for the rich direct-search model."""
    lines = ["-- Symbolic role locations over abstract grid nodes.", "FROZENVAR"]
    for role in case.prop.roles:
        lines.append(f"  {role.name} : 0..{case.spec.nodes - 1};")
    for role in case.prop.roles:
        if role.initial_car is not None:
            lines.append(f"INIT {role.name} = {case.spec.cars - 1 - role.initial_car};")
        if role.exclude_initial:
            for car in range(case.spec.cars):
                lines.append(f"INIT {role.name} != {case.spec.cars - 1 - car};")
    for first_index, first in enumerate(case.prop.roles):
        for second in case.prop.roles[first_index + 1:]:
            lines.append(f"INIT {first.name} != {second.name};")
    candidates = [tuple(sorted((int(edge["source"]), int(edge["target"]))))
                  for edge in geometry.get("candidate_edges", [])]
    for role in case.prop.roles:
        if not role.minimum_degree:
            continue
        for node in range(case.spec.nodes):
            incident = [
                f"rich_edge_{index}"
                for index, edge in enumerate(candidates) if node in edge
            ]
            lines.append(
                f"INIT {role.name} = {node} -> {_at_least(incident, role.minimum_degree)};"
            )
    return "\n".join(lines) + "\n"


def _rewrite_rich_topology(model_file: Path, constraints: str) -> None:
    text = model_file.read_text()
    marker = "-- Exact edge-based grid topology"
    if marker in text:
        text = text.split(marker, 1)[0]
    model_file.write_text(text + constraints)


def _rich_topology_constraints(geometry: dict, *, symbolic: bool = False,
                               case: BenchmarkCase | None = None) -> str:
    """Emit fixed or paired symbolic connector assignments for rich sections."""
    if not symbolic:
        return _edge_topology_constraints(geometry)
    if case is None:
        raise ValueError("symbolic rich topology requires a benchmark case")
    lines = ["-- Symbolic topology over the directed rich section universe.",
             "FROZENVAR"]
    candidates = [
        tuple(sorted((int(edge["source"]), int(edge["target"]))))
        for edge in geometry.get("candidate_edges", [])
    ]
    for index, (source, target) in enumerate(candidates):
        lines.append(f"  rich_edge_{index} : boolean;")
    for index in range(len(candidates)):
        lines.append(f"INIT rich_edge_{index} = TRUE | rich_edge_{index} = FALSE;")
    edge_count = " + ".join(
        f"case rich_edge_{index} : 1; TRUE : 0; esac"
        for index in range(len(candidates))
    )
    lines.append(f"INVAR ({edge_count}) <= {case.spec.maximum_edges};")
    # Connector activation is encoded per slot.  The generated model receives
    # no exact topology assignments in symbolic mode.
    by_edge = {edge: index for index, edge in enumerate(candidates)}
    sections = geometry["sections"]
    roads = {
        (int(section["edge"][0]), int(section["edge"][1])): int(section["id"])
        for section in sections if section.get("kind") == "road"
    }
    all_edges = set(candidates)
    def connector(slot: int, section: int) -> str:
        return f"env.outgoing_connection_{slot}_of_section_{section}"

    for section in sections:
        sid = int(section["id"])
        targets: list[tuple[str, int]] = []
        if section.get("kind") == "road":
            source, target = (int(value) for value in section["edge"])
            own = by_edge[tuple(sorted((source, target)))]
            exits = sorted(
                (edge, roads[(target, other)])
                for edge in all_edges if target in edge
                for other in [edge[1] if edge[0] == target else edge[0]]
                if edge != tuple(sorted((source, target)))
            )
            targets = [(f"rich_edge_{by_edge[edge]}", target_id)
                       for edge, target_id in exits]
            targets.append((f"rich_edge_{own}", int(geometry["parking_by_node"][str(target)]["inbound"])))
            guard = f"rich_edge_{own}"
        elif section.get("kind") == "parking_in":
            targets = [("TRUE", int(geometry["parking_by_node"][str(section["grid_node"])]["outbound"]))]
            guard = "TRUE"
        else:
            node = int(section["grid_node"])
            exits = sorted(
                (edge, roads[(node, other)])
                for edge in all_edges if node in edge
                for other in [edge[1] if edge[0] == node else edge[0]]
            )
            targets = [(f"rich_edge_{by_edge[edge]}", target_id)
                       for edge, target_id in exits]
            guard = "TRUE"
        for slot in range(int(geometry.get("max_outgoing", 5))):
            if slot < len(targets):
                edge_guard, target_id = targets[slot]
                lines.append(
                    f"INIT ({guard} & {edge_guard}) -> "
                    f"{connector(slot, sid)} = {target_id};")
                lines.append(
                    f"INIT (!({guard} & {edge_guard})) -> "
                    f"{connector(slot, sid)} = -1;")
            else:
                lines.append(
                    f"INIT {connector(slot, sid)} = -1;")
    return "\n".join(lines) + "\n"


def execute_abstract_concretized(case: BenchmarkCase, run_dir: Path,
                                 refinement: str = "monitor",
                                 section_length: int = 100,
                                 concrete_layout: str = "legacy") -> dict:
    if refinement not in {"monitor", "ltl"}:
        raise ValueError(f"unknown concrete refinement mode: {refinement}")
    result = _base_result(case, "abstract_concretized", run_dir)
    result["refinement"] = refinement
    result["concrete_layout"] = concrete_layout
    total_started = time.perf_counter()
    abstract_dir = run_dir / "abstract"
    abstract_dir.mkdir(parents=True, exist_ok=True)
    abstract_model = abstract_dir / "model.smv"
    abstract_model.write_text(render_abstract_model(case))
    result["nuxmv_invocations"] = 1
    abstract_cex = check_model(abstract_model, case.spec.bound, temporal=True, progress=True)
    if abstract_cex is None:
        result.update(status="abstract_no_witness_within_bound",
                      abstract_time_to_xml_seconds=time.perf_counter() - total_started)
        return result
    result["abstract_time_to_xml_seconds"] = time.perf_counter() - total_started
    abstract_target = abstract_dir / "counterexample.xml"
    shutil.move(abstract_cex, abstract_target)
    abstract_states = parse_abstract_states(abstract_target)
    result["abstract_states"] = len(abstract_states)
    milestones = abstract_milestones(abstract_states)
    result["abstract_milestones"] = milestones
    graph = selected_graph(case.spec, abstract_states[0])
    used_graph = trace_used_graph(case.spec, abstract_states)
    roles = {role.name: int(abstract_states[0][role.name]) for role in case.prop.roles}
    (abstract_dir / "selected_roles.json").write_text(json.dumps(roles, indent=2) + "\n")
    _save_graph(graph, abstract_dir)
    abstract_visualization_started = time.perf_counter()
    try:
        # Render immediately so an abstract witness remains useful even when
        # VFM generation/checking later times out or finds no refinement.
        save_abstract_gif(abstract_target, used_graph, roles, case.spec, case.prop,
                          abstract_dir / "abstract_scenario.gif")
    except Exception as error:
        (abstract_dir / "visualization_error.txt").write_text(str(error) + "\n")
    result["abstract_visualization_seconds"] = (
        time.perf_counter() - abstract_visualization_started
    )
    tick = time.perf_counter()
    if concrete_layout == "rich":
        geometry = _rich_universe(case, section_length)
        geometry = restrict_geometry(geometry, set(graph.edges()))
        model = _rich_model(case, run_dir / "vfm", geometry, section_length)
        (run_dir / "vfm" / "concrete_geometry.json").write_text(
            json.dumps(geometry, indent=2) + "\n"
        )
    else:
        geometry = None
        model = generate_concrete_base(case, run_dir / "vfm", section_length)
    result["concrete_generation_seconds"] = time.perf_counter() - tick
    if concrete_layout == "rich":
        _specialize_rich_main(
            model.model_file, case, geometry, milestones, refinement,
            abstract_milestone_modes(abstract_states),
        )
    else:
        specialize_vfm_main(
            model.model_file, case,
            topology=fixed_topology_constraints(case, set(graph.edges())),
            fixed_roles=roles, refinement=refinement, milestones=milestones,
            section_length=section_length,
        )
    tick = time.perf_counter()
    result["nuxmv_invocations"] = 2
    cex = check_concrete(
        model, model.model_file, "cex_benchmark.xml",
        property_kind="ltl" if refinement == "ltl" else "invar",
    )
    result["concrete_checking_seconds"] = time.perf_counter() - tick
    if cex is None:
        result["status"] = "concrete_no_witness_within_bound"
        return result
    result["total_time_to_concrete_xml_seconds"] = (
        time.perf_counter() - total_started
        - float(result["abstract_visualization_seconds"] or 0)
    )
    result["status"] = "witness"
    _finish_concrete(result, cex, graph, model, run_dir, geometry)
    return result


def execute_symbolic_vfm(case: BenchmarkCase, run_dir: Path,
                        section_length: int = 100,
                        concrete_layout: str = "legacy") -> dict:
    result = _base_result(case, "symbolic_vfm", run_dir)
    result["concrete_layout"] = concrete_layout
    started = time.perf_counter()
    tick = time.perf_counter()
    if concrete_layout == "rich":
        geometry = _rich_universe(case, section_length)
        model = _rich_model(case, run_dir / "vfm", geometry, section_length)
        env_path = model.target_path / "EnvModel_GridParking.smv"
        env_text = env_path.read_text()
        env_path.write_text(env_text.split(
            "-- Exact edge-based grid topology", 1
        )[0])
        _rewrite_rich_topology(
            model.model_file,
            _rich_topology_constraints(geometry, symbolic=True, case=case),
        )
    else:
        geometry = None
        model = generate_concrete_base(case, run_dir / "vfm", section_length)
    result["concrete_generation_seconds"] = time.perf_counter() - tick
    if concrete_layout == "rich":
        _specialize_rich_stage_main(model.model_file, case, geometry)
    else:
        specialize_vfm_main(model.model_file, case,
                            topology=symbolic_topology_constraints(case),
                            section_length=section_length)
    tick = time.perf_counter()
    result["nuxmv_invocations"] = 1
    cex = check_concrete(model, model.model_file, "cex_benchmark.xml")
    result["concrete_checking_seconds"] = time.perf_counter() - tick
    if cex is None:
        result["status"] = "concrete_no_witness_within_bound"
        return result
    result["total_time_to_concrete_xml_seconds"] = time.perf_counter() - started
    states = parse_states(cex)
    enabled = set()
    first = states[0]
    for edge in grid_edges(case.spec):  # type: ignore[arg-type]
        source, target = edge
        forward, reverse = _edge_slots(source, target, case.spec.columns)
        if first.get(forward) == str(target) and first.get(reverse) == str(source):
            enabled.add(tuple(sorted(edge)))
    graph = (_rich_graph_from_state(case, geometry, first)
             if concrete_layout == "rich" else graph_from_edges(case, enabled))
    result["status"] = "witness"
    _finish_concrete(result, cex, graph, model, run_dir, geometry)
    return result


def execute_enumerated_vfm(case: BenchmarkCase, run_dir: Path,
                           section_length: int = 100,
                           concrete_layout: str = "legacy") -> dict:
    result = _base_result(case, "enumerated_vfm", run_dir)
    result["concrete_layout"] = concrete_layout
    started = time.perf_counter()
    tick = time.perf_counter()
    if concrete_layout == "rich":
        geometry = _rich_universe(case, section_length)
        model = _rich_model(case, run_dir / "vfm", geometry, section_length)
    else:
        geometry = None
        model = generate_concrete_base(case, run_dir / "vfm", section_length)
    result["concrete_generation_seconds"] = time.perf_counter() - tick
    base_text = model.model_file.read_text()
    rich_env_base = None
    if concrete_layout == "rich":
        env_path = model.target_path / "EnvModel_GridParking.smv"
        rich_env_base = env_path.read_text()
    checking = 0.0
    for index, enabled in iter_topologies(case):
        result["candidates_enumerated"] = index + 1
        if not candidate_structurally_possible(case, enabled):
            result["candidates_pruned_structurally"] += 1
            continue
        result["candidates_checked"] += 1
        candidate = model.target_path / "main_benchmark_candidate.smv"
        if concrete_layout == "rich":
            restricted = restrict_geometry(geometry, enabled)
            env_candidate = model.target_path / "EnvModel_BenchmarkCandidate.smv"
            env_prefix = rich_env_base.split("-- Exact edge-based grid topology", 1)[0]
            env_candidate.write_text(env_prefix + _edge_topology_constraints(restricted))
            candidate.write_text(base_text.replace(
                '#include "EnvModel_GridParking.smv"',
                f'#include "{env_candidate.name}"',
            ))
            _specialize_rich_stage_main(candidate, case, restricted)
        else:
            candidate.write_text(base_text)
            specialize_vfm_main(candidate, case,
                                topology=fixed_topology_constraints(case, enabled),
                                section_length=section_length)
        tick = time.perf_counter()
        result["nuxmv_invocations"] += 1
        cex = check_concrete(model, candidate, "cex_benchmark_candidate.xml")
        checking += time.perf_counter() - tick
        if cex is not None:
            result["concrete_checking_seconds"] = checking
            result["total_time_to_concrete_xml_seconds"] = time.perf_counter() - started
            result["status"] = "witness"
            shutil.copy2(candidate, run_dir / "winning_fixed_main.smv")
            _finish_concrete(result, cex, graph_from_edges(case, enabled), model,
                             run_dir, geometry if concrete_layout == "rich" else None)
            return result
    result.update(status="exhaustive_no_witness", concrete_checking_seconds=checking)
    return result


def execute(case: BenchmarkCase, method: str, run_dir: Path,
                        refinement: str | None = None, section_length: int = 100,
            concrete_layout: str = "legacy") -> dict:
    if method == "abstract_concretized":
        result = execute_abstract_concretized(
            case, run_dir, refinement or "monitor", section_length,
            concrete_layout)
    else:
        result = {
            "enumerated_vfm": execute_enumerated_vfm,
            "symbolic_vfm": execute_symbolic_vfm,
        }[method](case, run_dir, section_length, concrete_layout)
    model_files = list(run_dir.rglob("*.smv"))
    result["smv_bytes"] = sum(path.stat().st_size for path in model_files)
    result["smv_lines"] = sum(sum(1 for _ in path.open(errors="ignore")) for path in model_files)
    result["peak_child_rss_kb"] = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
    return result


def worker(case: BenchmarkCase, method: str, run_dir: Path,
           refinement: str | None = None, section_length: int = 100,
           concrete_layout: str = "legacy") -> None:
    os.setsid()
    with (run_dir / "stdout.log").open("w") as stdout, \
         (run_dir / "stderr.log").open("w") as stderr:
        # VFM writes from its C++ shared library directly to file descriptors;
        # redirecting only Python's sys.stdout would flood the benchmark TTY.
        os.dup2(stdout.fileno(), 1)
        os.dup2(stderr.fileno(), 2)
        try:
            with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                result = execute(case, method, run_dir, refinement, section_length,
                                 concrete_layout)
        except BaseException as error:
            traceback.print_exc(file=stderr)
            result = _base_result(case, method, run_dir)
            if refinement is not None:
                result["refinement"] = refinement
            result["concrete_layout"] = concrete_layout
            result["error"] = f"{type(error).__name__}: {error}"
    (run_dir / "result.json").write_text(json.dumps(result, indent=2) + "\n")


def run_timed(case: BenchmarkCase, method: str, run_dir: Path, timeout: int,
              refinement: str | None = None, section_length: int = 100,
              concrete_layout: str = "legacy") -> dict:
    # Resume may encounter a directory left by an interrupted worker before it
    # managed to checkpoint result.json. Reuse it and retain the partial logs.
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "case.json").write_text(json.dumps({
        "spec": asdict(case.spec), "property": asdict(case.prop),
        "stages": asdict(case)["stages"], "method": method,
        "refinement": refinement, "concrete_layout": concrete_layout,
    }, indent=2) + "\n")
    process = mp.Process(
        target=worker,
        args=(case, method, run_dir, refinement, section_length, concrete_layout),
    )
    started = time.perf_counter()
    process.start()
    deadline = started + timeout
    while process.is_alive() and time.perf_counter() < deadline:
        process.join(min(15, max(0, deadline - time.perf_counter())))
        if process.is_alive():
            elapsed = time.perf_counter() - started
            print(f"[{case.identity}/{method}] still running: {elapsed:.0f}s / {timeout}s",
                  flush=True)
    if process.is_alive():
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        process.join(5)
        if process.is_alive():
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.join()
        result = _base_result(case, method, run_dir)
        result.update(status="timeout", refinement=refinement,
                      error=f"exceeded {timeout}-second whole-pipeline timeout",
                      elapsed_at_timeout_seconds=time.perf_counter() - started)
        (run_dir / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        return result
    result_file = run_dir / "result.json"
    if not result_file.exists():
        result = _base_result(case, method, run_dir)
        result.update(error=f"worker exited {process.exitcode} without result.json")
        return result
    return json.loads(result_file.read_text())


def write_results(results: list[dict], root: Path) -> None:
    (root / "results.jsonl").write_text("".join(json.dumps(item) + "\n" for item in results))
    columns = sorted({key for result in results for key in result})
    with (root / "results.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(results)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--only", action="append", help="scenario name (repeatable)")
    parser.add_argument("--method", action="append", choices=METHODS)
    parser.add_argument("--timeout", type=int, help="override whole-pipeline timeout")
    parser.add_argument(
        "--section-length", type=int, default=100,
        help="road-section length used by the concrete VFM model",
    )
    parser.add_argument(
        "--concrete-layout", choices=("legacy", "rich"), default="legacy",
        help="concrete representation: node sections or directed-edge geometry",
    )
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--smoke", action="store_true", help="run the first scenario only")
    parser.add_argument(
        "--parallel", type=int, default=1,
        help="number of independent benchmark jobs to run concurrently (default: 1)",
    )
    parser.add_argument(
        "--refinement", choices=("monitor", "ltl", "both"), default="monitor",
        help="concrete refinement encoding for abstract_concretized",
    )
    args = parser.parse_args()
    if args.parallel < 1:
        raise SystemExit("--parallel must be at least 1")
    cases = list(load_benchmark_cases(args.catalog))
    if args.only:
        cases = [case for case in cases if case.identity in set(args.only)]
    if args.smoke:
        cases = cases[:1]
    if not cases:
        raise SystemExit("no scenarios selected")
    methods = tuple(args.method or METHODS)
    jobs = [(method, refinement) for method in methods
            for refinement in (("monitor", "ltl") if
                               method == "abstract_concretized" and args.refinement == "both"
                               else ((args.refinement,) if method == "abstract_concretized"
                                     else (None,)))]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if args.resume:
        roots = sorted(path for path in args.output_dir.glob("run_*") if path.is_dir())
        root = roots[-1] if roots else args.output_dir / f"run_{time.time_ns()}"
    else:
        root = args.output_dir / f"run_{time.time_ns()}"
    root.mkdir(parents=True, exist_ok=True)
    shutil.copy2(args.catalog, root / "catalog.toml")
    results = []
    if args.resume:
        for result_file in sorted(root.rglob("result.json")):
            try:
                results.append(json.loads(result_file.read_text()))
            except (OSError, json.JSONDecodeError):
                pass
    pending = []
    for case in cases:
        for method, refinement in jobs:
            run_dir = root / case.identity / method
            if refinement is not None and args.refinement == "both":
                run_dir = run_dir / refinement
            result_file = run_dir / "result.json"
            if args.resume and result_file.exists():
                existing = json.loads(result_file.read_text())
                # Interrupted workers can leave a directory and an error
                # record before the underlying case was actually attempted.
                # Resume retries those records, while retaining real witness,
                # timeout, and no-witness outcomes.
                matches_refinement = (
                    existing.get("refinement", "monitor") == (refinement or "monitor")
                )
                matches_layout = existing.get("concrete_layout", "legacy") == args.concrete_layout
                if existing.get("status") != "error" and matches_refinement and matches_layout:
                    result = existing
                    results = [item for item in results
                               if not (item.get("case") == case.identity and
                                       item.get("method") == method and
                                       item.get("refinement", "monitor") == (refinement or "monitor"))]
                    results.append(result)
                    continue
                else:
                    pass
            pending.append((case, method, refinement, run_dir,
                            args.timeout or case.spec.timeout_seconds,
                            args.section_length, args.concrete_layout))

    # Each job owns a separate directory and child process, so independent
    # methods/cases can safely run concurrently.  Checkpointing remains in the
    # coordinator thread as jobs complete.
    with ThreadPoolExecutor(max_workers=args.parallel) as executor:
        futures = {
            executor.submit(
                run_timed, case, method, run_dir, timeout, refinement, section_length,
                concrete_layout
            ):
            (case, method, refinement)
            for case, method, refinement, run_dir, timeout, section_length, concrete_layout in pending
        }
        for future in as_completed(futures):
            case, method, refinement = futures[future]
            result = future.result()
            results = [item for item in results
                       if not (item.get("case") == case.identity and
                               item.get("method") == method and
                               item.get("refinement", "monitor") == (refinement or "monitor"))]
            results.append(result)
            write_results(results, root)
            label = f"{method}/{refinement}" if refinement else method
            print(f"{case.identity}/{label}: {result['status']}", flush=True)
    write_results(results, root)
    print(f"Results: {root.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
