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
from dataclasses import asdict
from pathlib import Path

import networkx as nx

from concretizer import _slot
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
    save_trace_gif, vehicle,
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
                        topology: str, fixed_roles: dict[str, int] | None = None) -> None:
    text = model_file.read_text()
    text, count = re.subn(r"(?m)^LTLSPEC .*$", stages_to_monitor(case.stages), text, count=1)
    if count != 1:
        raise ValueError(f"expected one LTLSPEC in {model_file}, found {count}")
    for car in range(case.spec.cars):
        prefix = re.escape(f"env.veh___6{car}9___")
        text = re.sub(rf"(?m)^INIT {prefix}\.is_on_sec_\d+\s*=.*;$",
                      lambda match: "-- benchmark replaced: " + match.group(0), text)
        text = re.sub(rf"(?m)^INIT {prefix}\.abs_pos\s*=.*;$",
                      lambda match: "-- benchmark replaced: " + match.group(0), text)
        text = re.sub(rf"(?m)^INIT {prefix}\.v\s*=.*;$",
                      lambda match: "-- benchmark replaced: " + match.group(0), text)
    additions = ["", concrete_role_constraints(case, fixed_roles), topology,
                 "-- Benchmark initial state and admissibility."]
    for car in range(case.spec.cars):
        additions.extend((
            f"INIT {vehicle(car, f'is_on_sec_{case.spec.cars - 1 - car}')} = 1;",
            f"INIT {vehicle(car, 'abs_pos')} = {60 if car == 0 else 20};",
            f"INIT {vehicle(car, 'v')} = 5;",
            f"INVAR {vehicle(car, 'v')} >= 0;",
        ))
    additions.append("INVAR !env.crash;")
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


def generate_concrete_base(case: BenchmarkCase, run_dir: Path):
    config = VfmGridParkingConfig(
        rows=case.spec.rows, columns=case.spec.columns, cars=case.spec.cars,
        section_length=100, connection_length=20, bound=case.spec.bound,
    )
    # VFM's generator treats an absolute output path as if it were relative
    # (prefixing it with ``./``), while the kratos post-pass does not.  Passing
    # one cwd-relative path keeps both halves of generation in the same folder.
    relative_output = Path(os.path.relpath(run_dir.resolve(), ROOT))
    return generate_vfm_grid_parking_model(config, experiments_dir=relative_output)


def check_concrete(model, model_file: Path, cex_name: str) -> Path | None:
    try:
        return model.morty.model_check(
            cex_name, 0, model_file=model_file.name,
            bound=model.config.bound, property_kind="invar",
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
    }


def _save_graph(graph: nx.Graph, run_dir: Path) -> None:
    nx.write_graphml(graph, run_dir / "selected_grid.graphml")
    (run_dir / "selected_grid.json").write_text(
        json.dumps(nx.node_link_data(graph, edges="edges"), indent=2) + "\n"
    )


def _finish_concrete(result: dict, cex: Path, graph: nx.Graph, model,
                     run_dir: Path) -> None:
    target = run_dir / "concrete_counterexample.xml"
    shutil.copy2(cex, target)
    result["concrete_states"] = len(parse_states(target))
    result["selected_edges"] = graph.number_of_edges()
    _save_graph(graph, run_dir)
    visualization_started = time.perf_counter()
    try:
        save_trace_gif(target, nx.MultiDiGraph(graph), model.config,
                       run_dir / "concrete_scenario.gif")
    except Exception as error:
        (run_dir / "visualization_error.txt").write_text(str(error) + "\n")
    result["visualization_seconds"] = time.perf_counter() - visualization_started


def execute_abstract_concretized(case: BenchmarkCase, run_dir: Path) -> dict:
    result = _base_result(case, "abstract_concretized", run_dir)
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
    model = generate_concrete_base(case, run_dir / "vfm")
    result["concrete_generation_seconds"] = time.perf_counter() - tick
    specialize_vfm_main(model.model_file, case,
                        topology=fixed_topology_constraints(case, set(graph.edges())),
                        fixed_roles=roles)
    tick = time.perf_counter()
    result["nuxmv_invocations"] = 2
    cex = check_concrete(model, model.model_file, "cex_benchmark.xml")
    result["concrete_checking_seconds"] = time.perf_counter() - tick
    if cex is None:
        result["status"] = "concrete_no_witness_within_bound"
        return result
    result["total_time_to_concrete_xml_seconds"] = (
        time.perf_counter() - total_started
        - float(result["abstract_visualization_seconds"] or 0)
    )
    result["status"] = "witness"
    _finish_concrete(result, cex, graph, model, run_dir)
    return result


def execute_symbolic_vfm(case: BenchmarkCase, run_dir: Path) -> dict:
    result = _base_result(case, "symbolic_vfm", run_dir)
    started = time.perf_counter()
    tick = time.perf_counter()
    model = generate_concrete_base(case, run_dir / "vfm")
    result["concrete_generation_seconds"] = time.perf_counter() - tick
    specialize_vfm_main(model.model_file, case,
                        topology=symbolic_topology_constraints(case))
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
    graph = graph_from_edges(case, enabled)
    result["status"] = "witness"
    _finish_concrete(result, cex, graph, model, run_dir)
    return result


def execute_enumerated_vfm(case: BenchmarkCase, run_dir: Path) -> dict:
    result = _base_result(case, "enumerated_vfm", run_dir)
    started = time.perf_counter()
    tick = time.perf_counter()
    model = generate_concrete_base(case, run_dir / "vfm")
    result["concrete_generation_seconds"] = time.perf_counter() - tick
    base_text = model.model_file.read_text()
    checking = 0.0
    for index, enabled in iter_topologies(case):
        result["candidates_enumerated"] = index + 1
        if not candidate_structurally_possible(case, enabled):
            result["candidates_pruned_structurally"] += 1
            continue
        result["candidates_checked"] += 1
        candidate = model.target_path / "main_benchmark_candidate.smv"
        candidate.write_text(base_text)
        specialize_vfm_main(candidate, case,
                            topology=fixed_topology_constraints(case, enabled))
        tick = time.perf_counter()
        result["nuxmv_invocations"] += 1
        cex = check_concrete(model, candidate, "cex_benchmark_candidate.xml")
        checking += time.perf_counter() - tick
        if cex is not None:
            result["concrete_checking_seconds"] = checking
            result["total_time_to_concrete_xml_seconds"] = time.perf_counter() - started
            result["status"] = "witness"
            shutil.copy2(candidate, run_dir / "winning_fixed_main.smv")
            _finish_concrete(result, cex, graph_from_edges(case, enabled), model, run_dir)
            return result
    result.update(status="exhaustive_no_witness", concrete_checking_seconds=checking)
    return result


def execute(case: BenchmarkCase, method: str, run_dir: Path) -> dict:
    result = {
        "enumerated_vfm": execute_enumerated_vfm,
        "abstract_concretized": execute_abstract_concretized,
        "symbolic_vfm": execute_symbolic_vfm,
    }[method](case, run_dir)
    model_files = list(run_dir.rglob("*.smv"))
    result["smv_bytes"] = sum(path.stat().st_size for path in model_files)
    result["smv_lines"] = sum(sum(1 for _ in path.open(errors="ignore")) for path in model_files)
    result["peak_child_rss_kb"] = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
    return result


def worker(case: BenchmarkCase, method: str, run_dir: Path) -> None:
    os.setsid()
    with (run_dir / "stdout.log").open("w") as stdout, \
         (run_dir / "stderr.log").open("w") as stderr:
        # VFM writes from its C++ shared library directly to file descriptors;
        # redirecting only Python's sys.stdout would flood the benchmark TTY.
        os.dup2(stdout.fileno(), 1)
        os.dup2(stderr.fileno(), 2)
        try:
            with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                result = execute(case, method, run_dir)
        except BaseException as error:
            traceback.print_exc(file=stderr)
            result = _base_result(case, method, run_dir)
            result["error"] = f"{type(error).__name__}: {error}"
    (run_dir / "result.json").write_text(json.dumps(result, indent=2) + "\n")


def run_timed(case: BenchmarkCase, method: str, run_dir: Path, timeout: int) -> dict:
    # Resume may encounter a directory left by an interrupted worker before it
    # managed to checkpoint result.json. Reuse it and retain the partial logs.
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "case.json").write_text(json.dumps({
        "spec": asdict(case.spec), "property": asdict(case.prop),
        "stages": asdict(case)["stages"], "method": method,
    }, indent=2) + "\n")
    process = mp.Process(target=worker, args=(case, method, run_dir))
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
        result.update(status="timeout", error=f"exceeded {timeout}-second whole-pipeline timeout",
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
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--smoke", action="store_true", help="run the first scenario only")
    args = parser.parse_args()
    cases = list(load_benchmark_cases(args.catalog))
    if args.only:
        cases = [case for case in cases if case.identity in set(args.only)]
    if args.smoke:
        cases = cases[:1]
    if not cases:
        raise SystemExit("no scenarios selected")
    methods = tuple(args.method or METHODS)
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
        for result_file in sorted(root.glob("*/*/result.json")):
            try:
                results.append(json.loads(result_file.read_text()))
            except (OSError, json.JSONDecodeError):
                pass
    for case in cases:
        for method in methods:
            run_dir = root / case.identity / method
            result_file = run_dir / "result.json"
            if args.resume and result_file.exists():
                existing = json.loads(result_file.read_text())
                # Interrupted workers can leave a directory and an error
                # record before the underlying case was actually attempted.
                # Resume retries those records, while retaining real witness,
                # timeout, and no-witness outcomes.
                if existing.get("status") != "error":
                    result = existing
                else:
                    result = run_timed(case, method, run_dir,
                                       args.timeout or case.spec.timeout_seconds)
            else:
                result = run_timed(case, method, run_dir,
                                   args.timeout or case.spec.timeout_seconds)
            results = [item for item in results
                       if not (item.get("case") == case.identity and
                               item.get("method") == method)]
            results.append(result)
            write_results(results, root)
            print(f"{case.identity}/{method}: {result['status']}", flush=True)
    print(f"Results: {root.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
