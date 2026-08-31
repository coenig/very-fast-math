#!/usr/bin/env python3
"""Generate and check role-based abstract partial-grid LTL scenarios from TOML."""

from __future__ import annotations

import argparse
import contextlib
import csv
import itertools
import json
import multiprocessing as mp
import os
import re
import shutil
import signal
import time
import tomllib
import traceback
from dataclasses import asdict, dataclass
from pathlib import Path

import networkx as nx

from scalable_grid_parking import check_model, edge_name, grid_edges
from scalable_parking import parse_states


ROOT = Path(__file__).resolve().parent
DEFAULT_TOML = ROOT / "scenario_specifications.toml"
DEFAULT_OUTPUT = ROOT / "vfm-with-parking" / "generated-scenarios"
NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")


@dataclass(frozen=True)
class RoleSpec:
    name: str
    exclude_initial: bool = False
    initial_car: int | None = None
    minimum_start_distance: int = 0
    minimum_degree: int = 0


@dataclass(frozen=True)
class RoleDistance:
    first: str
    second: str
    minimum: int


@dataclass(frozen=True)
class LtlProperty:
    name: str
    explanation: str
    formula: str
    roles: tuple[RoleSpec, ...]
    role_distances: tuple[RoleDistance, ...] = ()


@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    rows: int
    columns: int
    cars: int
    bound: int
    timeout_seconds: int
    initial_layout: str
    properties: tuple[LtlProperty, ...]
    maximum_edges: int | None = None

    @property
    def nodes(self) -> int:
        return self.rows * self.columns


def _name(value: object, context: str) -> str:
    if not isinstance(value, str) or not NAME_PATTERN.fullmatch(value):
        raise ValueError(f"{context} must match {NAME_PATTERN.pattern}")
    return value


def _positive_int(value: object, context: str, *, allow_zero: bool = False) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise ValueError(f"{context} must be an integer")
    if value < (0 if allow_zero else 1):
        raise ValueError(f"{context} must be {'non-negative' if allow_zero else 'positive'}")
    return value


def _parse_roles(raw: object, cars: int, context: str) -> tuple[RoleSpec, ...]:
    if not isinstance(raw, list) or not raw:
        raise ValueError(f"{context}.roles must be a non-empty array")
    roles = []
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ValueError(f"{context}.roles[{index}] must be a table")
        initial_car = item.get("initial_car")
        if initial_car is not None and (not isinstance(initial_car, int)
                                        or not 0 <= initial_car < cars):
            raise ValueError(f"{context}.roles[{index}].initial_car is out of range")
        role = RoleSpec(
            name=_name(item.get("name"), f"{context}.roles[{index}].name"),
            exclude_initial=bool(item.get("exclude_initial", False)),
            initial_car=initial_car,
            minimum_start_distance=_positive_int(
                item.get("minimum_start_distance", 0),
                f"{context}.roles[{index}].minimum_start_distance", allow_zero=True),
            minimum_degree=_positive_int(
                item.get("minimum_degree", 0),
                f"{context}.roles[{index}].minimum_degree", allow_zero=True),
        )
        if role.minimum_degree > 4:
            raise ValueError(f"{context}.{role.name} minimum_degree cannot exceed 4")
        roles.append(role)
    names = [role.name for role in roles]
    if len(names) != len(set(names)):
        raise ValueError(f"{context} contains duplicate role names")
    assigned = [role.initial_car for role in roles if role.initial_car is not None]
    if len(assigned) != len(set(assigned)):
        raise ValueError(f"{context} assigns more than one role to an initial car")
    return tuple(roles)


def _normalize_formula(formula: object, roles: tuple[RoleSpec, ...], cars: int,
                       context: str) -> str:
    if not isinstance(formula, str) or not formula.strip():
        raise ValueError(f"{context}.formula must be a non-empty string")
    formula = " ".join(formula.split())
    if ";" in formula or re.search(r"\b(?:MODULE|LTLSPEC|INVARSPEC)\b", formula,
                                    re.IGNORECASE):
        raise ValueError(f"{context}.formula must contain only an LTL expression")
    for car in re.findall(r"\bloc_(\d+)\b", formula):
        if int(car) >= cars:
            raise ValueError(f"{context}.formula references car {car}, but cars={cars}")
    identifiers = set(re.findall(r"\b[A-Za-z_][A-Za-z0-9_]*\b", formula))
    allowed = {role.name for role in roles} | {"X", "U", "F", "G", "TRUE", "FALSE"}
    allowed.update(f"loc_{car}" for car in range(cars))
    unknown = sorted(identifiers - allowed)
    if unknown:
        raise ValueError(f"{context}.formula contains unknown identifiers: {unknown}")
    return formula


def load_specifications(path: Path) -> tuple[ScenarioSpec, ...]:
    with path.open("rb") as source:
        document = tomllib.load(source)
    defaults = document.get("defaults", {})
    raw_specs = document.get("specification")
    if not isinstance(raw_specs, list) or not raw_specs:
        raise ValueError("TOML must contain at least one [[specification]]")
    specifications = []
    for spec_index, raw in enumerate(raw_specs):
        context = f"specification[{spec_index}]"
        name = _name(raw.get("name"), f"{context}.name")
        grid = raw.get("grid_size")
        if not isinstance(grid, list) or len(grid) != 2:
            raise ValueError(f"{context}.grid_size must be [rows, columns]")
        rows = _positive_int(grid[0], f"{context}.grid_size[0]")
        columns = _positive_int(grid[1], f"{context}.grid_size[1]")
        if rows < 2 or columns < 2:
            raise ValueError(f"{context} grid must be at least 2x2")
        cars = _positive_int(raw.get("n_cars"), f"{context}.n_cars")
        if cars > rows * columns:
            raise ValueError(f"{context} has more cars than grid cells")
        bound = _positive_int(raw.get("bound", defaults.get("bound", 250)),
                              f"{context}.bound", allow_zero=True)
        timeout = _positive_int(raw.get("timeout_seconds",
                                        defaults.get("timeout_seconds", 900)),
                                f"{context}.timeout_seconds")
        initial_layout = raw.get("initial_layout", "first_row")
        if initial_layout != "first_row":
            raise ValueError(f"{context}.initial_layout must be first_row")
        if cars > columns:
            raise ValueError(f"{context} first_row layout needs n_cars <= columns")
        maximum_edges = raw.get("maximum_edges")
        possible_edges = rows * (columns - 1) + (rows - 1) * columns
        if maximum_edges is not None:
            maximum_edges = _positive_int(maximum_edges, f"{context}.maximum_edges",
                                          allow_zero=True)
            if maximum_edges > possible_edges:
                raise ValueError(
                    f"{context}.maximum_edges exceeds the grid's {possible_edges} edges"
                )
        raw_properties = raw.get("ltl_properties")
        if not isinstance(raw_properties, list) or not raw_properties:
            raise ValueError(f"{context} needs [[specification.ltl_properties]]")
        properties = []
        for property_index, item in enumerate(raw_properties):
            prop_context = f"{context}.ltl_properties[{property_index}]"
            roles = _parse_roles(item.get("roles"), cars, prop_context)
            role_names = {role.name for role in roles}
            distances = []
            for distance_index, distance in enumerate(item.get("role_distances", [])):
                first, second = distance.get("from"), distance.get("to")
                if first not in role_names or second not in role_names or first == second:
                    raise ValueError(f"{prop_context}.role_distances[{distance_index}] "
                                     "must reference two distinct declared roles")
                distances.append(RoleDistance(
                    first, second, _positive_int(distance.get("minimum"),
                                                 f"{prop_context}.role_distances.minimum")))
            properties.append(LtlProperty(
                name=_name(item.get("name"), f"{prop_context}.name"),
                explanation=str(item.get("explanation", "")).strip(),
                formula=_normalize_formula(item.get("formula"), roles, cars, prop_context),
                roles=roles, role_distances=tuple(distances),
            ))
        property_names = [item.name for item in properties]
        if len(property_names) != len(set(property_names)):
            raise ValueError(f"{context} contains duplicate property names")
        specifications.append(ScenarioSpec(
            name, rows, columns, cars, bound, timeout,
            initial_layout, tuple(properties), maximum_edges,
        ))
    names = [spec.name for spec in specifications]
    if len(names) != len(set(names)):
        raise ValueError("duplicate specification names")
    return tuple(specifications)


def _at_least(expressions: list[str], minimum: int) -> str:
    if minimum == 0:
        return "TRUE"
    if minimum > len(expressions):
        return "FALSE"
    return "(" + " | ".join(
        "(" + " & ".join(selected) + ")"
        for selected in itertools.combinations(expressions, minimum)
    ) + ")"


def _distance(first: int, second: int, columns: int) -> int:
    first_row, first_column = divmod(first, columns)
    second_row, second_column = divmod(second, columns)
    return abs(first_row - second_row) + abs(first_column - second_column)


def render_role_smv(spec: ScenarioSpec, prop: LtlProperty) -> str:
    edges = grid_edges(spec)  # type: ignore[arg-type]
    incident: dict[int, list[str]] = {cell: [] for cell in range(spec.nodes)}
    for edge in edges:
        name = edge_name(*edge)
        incident[edge[0]].append(name)
        incident[edge[1]].append(name)
    lines = ["MODULE main", "", "FROZENVAR"]
    lines.extend(f"  {edge_name(*edge)} : boolean;" for edge in edges)
    lines.extend(f"  {role.name} : 0..{spec.nodes - 1};" for role in prop.roles)
    if spec.maximum_edges is not None:
        edge_terms = [
            f"(case {edge_name(*edge)} : 1; TRUE : 0; esac)" for edge in edges
        ]
        lines.extend((
            "", "DEFINE",
            "  enabled_edge_count := " + " + ".join(edge_terms) + ";",
            f"INVAR enabled_edge_count <= {spec.maximum_edges};",
        ))
    lines.extend(("", "-- Semantic locations are symbolic and trace-constant."))
    for first_index, first in enumerate(prop.roles):
        for second in prop.roles[first_index + 1:]:
            lines.append(f"INIT {first.name} != {second.name};")
    initial_role_by_car = {role.initial_car: role.name for role in prop.roles
                           if role.initial_car is not None}
    fixed_starts = {car: car for car in range(spec.cars)
                    if car not in initial_role_by_car}
    for role in prop.roles:
        if role.exclude_initial:
            for start in range(spec.cars):
                lines.append(f"INIT {role.name} != {start};")
        if role.minimum_start_distance:
            for cell in range(spec.nodes):
                if any(_distance(cell, start, spec.columns) < role.minimum_start_distance
                       for start in fixed_starts.values()):
                    lines.append(f"INIT {role.name} != {cell};")
            for initial_role in initial_role_by_car.values():
                if initial_role == role.name:
                    continue
                for first in range(spec.nodes):
                    for second in range(spec.nodes):
                        if _distance(first, second, spec.columns) < role.minimum_start_distance:
                            lines.append(
                                f"INIT !({role.name} = {first} & "
                                f"{initial_role} = {second});"
                            )
        if role.minimum_degree:
            for cell in range(spec.nodes):
                lines.append(
                    f"INIT {role.name} = {cell} -> "
                    f"{_at_least(incident[cell], role.minimum_degree)};"
                )
    for constraint in prop.role_distances:
        for first in range(spec.nodes):
            for second in range(spec.nodes):
                if _distance(first, second, spec.columns) < constraint.minimum:
                    lines.append(
                        f"INIT !({constraint.first} = {first} & "
                        f"{constraint.second} = {second});"
                    )
    lines.extend(("", "VAR"))
    for car in range(spec.cars):
        lines.append(f"  loc_{car} : 0..{spec.nodes - 1};")
    initial_roles = {role.initial_car: role.name for role in prop.roles
                     if role.initial_car is not None}
    for car in range(spec.cars):
        start = initial_roles.get(car, str(car))
        lines.extend(("", f"INIT loc_{car} = {start};"))
        moves = [f"next(loc_{car}) = loc_{car}"]
        for first, second in edges:
            enabled = edge_name(first, second)
            moves.extend((
                f"({enabled} & loc_{car} = {first} & next(loc_{car}) = {second})",
                f"({enabled} & loc_{car} = {second} & next(loc_{car}) = {first})",
            ))
        lines.append("TRANS " + " |\n  ".join(moves) + ";")
    lines.extend(("", "-- Unit-capacity cells and no instantaneous exchanges."))
    for first in range(spec.cars):
        for second in range(first + 1, spec.cars):
            lines.append(f"INVAR loc_{first} != loc_{second};")
            lines.append(
                f"TRANS !(next(loc_{first}) = loc_{second} & "
                f"next(loc_{second}) = loc_{first});"
            )
    lines.extend(("", f"-- {prop.explanation}", f"LTLSPEC !({prop.formula});", ""))
    return "\n".join(lines)


def selected_graph(spec: ScenarioSpec, state: dict[str, str]) -> nx.Graph:
    graph = nx.Graph()
    for cell in range(spec.nodes):
        row, column = divmod(cell, spec.columns)
        graph.add_node(cell, row=row, column=column)
    graph.add_edges_from(edge for edge in grid_edges(spec)  # type: ignore[arg-type]
                         if state.get(edge_name(*edge), "FALSE").upper() == "TRUE")
    return graph


def trace_used_graph(spec: ScenarioSpec, states: list[dict[str, str]]) -> nx.Graph:
    """Return only grid edges traversed by at least one car in the witness."""
    graph = nx.Graph()
    for cell in range(spec.nodes):
        row, column = divmod(cell, spec.columns)
        graph.add_node(cell, row=row, column=column)
    for current, following in zip(states, states[1:]):
        for car in range(spec.cars):
            first = int(current[f"loc_{car}"])
            second = int(following[f"loc_{car}"])
            if first != second:
                graph.add_edge(*sorted((first, second)))
    return graph


def save_gif(counterexample: Path, graph: nx.Graph, roles: dict[str, int],
             spec: ScenarioSpec, prop: LtlProperty, output: Path) -> None:
    from matplotlib import pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    states = parse_states(counterexample)
    positions = {cell: (data["column"], -data["row"])
                 for cell, data in graph.nodes(data=True)}
    figure, axes = plt.subplots(figsize=(max(8, 1.3 * spec.columns),
                                         max(5, 1.2 * spec.rows)))
    colors = ["#ffe29a" if cell in roles.values() else "#dcecff"
              for cell in graph.nodes]
    nx.draw_networkx_nodes(graph, positions, node_color=colors, node_size=600, ax=axes)
    nx.draw_networkx_labels(graph, positions, font_size=7, ax=axes)
    nx.draw_networkx_edges(graph, positions, width=2.4, edge_color="#788da8", ax=axes)
    for role, cell in roles.items():
        axes.annotate(role, positions[cell], xytext=(0, -19), textcoords="offset points",
                      ha="center", fontsize=7)
    palette = plt.get_cmap("tab10")
    dots = [axes.scatter([], [], s=170, color=palette(car % 10),
                         label=f"car {car}", zorder=5) for car in range(spec.cars)]
    title = axes.set_title("")
    axes.legend(loc="lower center", ncol=min(spec.cars, 5))
    axes.axis("off")

    def update(frame: int):
        state = states[frame]
        for car, dot in enumerate(dots):
            dot.set_offsets([positions[int(state[f"loc_{car}"])]])
        title.set_text(f"{prop.name} | step {frame}/{len(states)-1}")
        return [*dots, title]

    FuncAnimation(figure, update, frames=len(states), interval=400).save(
        output, writer=PillowWriter(fps=3)
    )
    plt.close(figure)


def save_topology_png(graph: nx.Graph, roles: dict[str, int], output: Path) -> None:
    from matplotlib import pyplot as plt

    positions = {cell: (data["column"], -data["row"])
                 for cell, data in graph.nodes(data=True)}
    figure, axes = plt.subplots(figsize=(9, 6))
    colors = ["#ffe29a" if cell in roles.values() else "#dcecff"
              for cell in graph.nodes]
    nx.draw(graph, positions, with_labels=True, node_color=colors, node_size=560,
            width=2.2, edge_color="#788da8", font_size=7, ax=axes)
    for role, cell in roles.items():
        axes.annotate(role, positions[cell], xytext=(0, -19), textcoords="offset points",
                      ha="center", fontsize=7)
    axes.axis("off")
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)


def _result(spec: ScenarioSpec, prop: LtlProperty, run_dir: Path) -> dict:
    resolved_run_dir = run_dir.resolve()
    try:
        displayed_run_dir = str(resolved_run_dir.relative_to(ROOT))
    except ValueError:
        displayed_run_dir = str(resolved_run_dir)
    return {
        "specification": spec.name, "property": prop.name, "status": "error",
        "rows": spec.rows, "columns": spec.columns, "cars": spec.cars,
        "bound": spec.bound, "generation_seconds": 0.0, "time_to_xml_seconds": "",
        "visualization_seconds": "", "counterexample_states": "",
        "selected_edges": "", "visualized_edges": "",
        "run_directory": displayed_run_dir,
        "error": "",
    }


def execute(spec: ScenarioSpec, prop: LtlProperty, run_dir: Path,
            *, generate_only: bool, make_gif: bool, bmc_progress: bool) -> dict:
    result = _result(spec, prop, run_dir)
    started = time.perf_counter()
    model = run_dir / "model.smv"
    model.write_text(render_role_smv(spec, prop))
    (run_dir / "property.ltl").write_text(f"LTLSPEC !({prop.formula});\n")
    (run_dir / "config.json").write_text(json.dumps({
        "specification": asdict(spec), "property": asdict(prop)}, indent=2) + "\n")
    result["generation_seconds"] = time.perf_counter() - started
    if generate_only:
        result["status"] = "generated"
        return result
    counterexample = check_model(
        model, spec.bound, temporal=True, progress=bmc_progress
    )
    if counterexample is None:
        result["time_to_xml_seconds"] = time.perf_counter() - started
        result["status"] = "no_witness_within_bound"
        return result
    result["time_to_xml_seconds"] = time.perf_counter() - started
    target = run_dir / "counterexample.xml"
    shutil.move(counterexample, target)
    states = parse_states(target)
    graph = selected_graph(spec, states[0])
    used_graph = trace_used_graph(spec, states)
    roles = {role.name: int(states[0][role.name]) for role in prop.roles}
    nx.write_graphml(graph, run_dir / "selected_grid.graphml")
    (run_dir / "selected_grid.json").write_text(
        json.dumps(nx.node_link_data(graph, edges="edges"), indent=2) + "\n")
    (run_dir / "selected_roles.json").write_text(json.dumps(roles, indent=2) + "\n")
    nx.write_graphml(used_graph, run_dir / "trace_used_grid.graphml")
    (run_dir / "trace_used_grid.json").write_text(
        json.dumps(nx.node_link_data(used_graph, edges="edges"), indent=2) + "\n")
    save_topology_png(used_graph, roles, run_dir / "selected_topology.png")
    result.update(status="witness", counterexample_states=len(states),
                  selected_edges=graph.number_of_edges(),
                  visualized_edges=used_graph.number_of_edges())
    if make_gif:
        visualization_started = time.perf_counter()
        save_gif(target, used_graph, roles, spec, prop, run_dir / "scenario.gif")
        result["visualization_seconds"] = time.perf_counter() - visualization_started
    return result


def worker(spec: ScenarioSpec, prop: LtlProperty, run_dir_text: str,
           generate_only: bool, make_gif: bool, bmc_progress: bool) -> None:
    os.setsid()
    run_dir = Path(run_dir_text)
    with (run_dir / "stdout.log").open("w") as stdout, \
         (run_dir / "stderr.log").open("w") as stderr, \
         contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
        try:
            result = execute(spec, prop, run_dir, generate_only=generate_only,
                             make_gif=make_gif, bmc_progress=bmc_progress)
        except BaseException as error:
            traceback.print_exc()
            result = _result(spec, prop, run_dir)
            result["error"] = f"{type(error).__name__}: {error}"
    (run_dir / "result.json").write_text(json.dumps(result, indent=2) + "\n")


def run_one(spec: ScenarioSpec, prop: LtlProperty, run_dir: Path, timeout: int,
            *, generate_only: bool, make_gif: bool, bmc_progress: bool) -> dict:
    run_dir.mkdir(parents=True, exist_ok=False)
    process = mp.Process(target=worker, args=(spec, prop, str(run_dir),
                                             generate_only, make_gif, bmc_progress))
    process.start()
    process.join(timeout)
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
        result = _result(spec, prop, run_dir)
        result.update(status="timeout", error=f"exceeded {timeout}-second timeout")
        (run_dir / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        return result
    result_file = run_dir / "result.json"
    if result_file.exists():
        return json.loads(result_file.read_text())
    result = _result(spec, prop, run_dir)
    result["error"] = f"worker exited {process.exitcode} without result.json"
    return result


def write_summary(results: list[dict], output: Path) -> None:
    (output / "results.jsonl").write_text(
        "".join(json.dumps(result) + "\n" for result in results))
    fields = list(results[0]) if results else []
    with (output / "results.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(results)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("toml", nargs="?", type=Path, default=DEFAULT_TOML)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--only", help="run only specification/property")
    parser.add_argument("--generate-only", action="store_true")
    parser.add_argument("--no-gif", action="store_true")
    parser.add_argument(
        "--quiet-bmc", action="store_true",
        help="do not print each attempted BMC bound with elapsed time",
    )
    parser.add_argument("--timeout", type=int, help="override per-property timeout")
    args = parser.parse_args()
    specifications = load_specifications(args.toml)
    output = args.output_dir / f"run_{time.time_ns()}"
    output.mkdir(parents=True, exist_ok=False)
    shutil.copy2(args.toml, output / "scenario_specifications.toml")
    results = []
    for spec in specifications:
        for prop in spec.properties:
            identity = f"{spec.name}/{prop.name}"
            if args.only and args.only != identity:
                continue
            run_dir = output / f"{spec.name}__{prop.name}"
            result = run_one(spec, prop, run_dir, args.timeout or spec.timeout_seconds,
                             generate_only=args.generate_only, make_gif=not args.no_gif,
                             bmc_progress=not args.quiet_bmc)
            results.append(result)
            write_summary(results, output)
            print(f"{identity}: {result['status']}")
    if args.only and not results:
        raise SystemExit(f"No property matched --only {args.only!r}")
    print(f"Results: {output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
