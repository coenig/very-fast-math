"""Structured symbolic parking-lot topology search for VFM/nuXmv.

The aisle is fixed, while nuXmv chooses where ordered parking spots attach::

    A0 -> A1 -> ... -> An
           \\-> Pi -/

For every spot ``Pi``, ``parking_attachment_i`` selects exactly one aisle gap.
The same nuXmv run chooses these frozen topology variables and the vehicle
strategy.  It does not enumerate arbitrary directed graphs.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import time
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from pathlib import Path

import networkx as nx

from fixed_graph_model import _replace_singleton_domain
from model_checking import Morty
from scenario_generation import generate_scenario


def vehicle(index: int, field: str) -> str:
    return f"env.veh___6{index}9___.{field}"


@dataclass(frozen=True)
class FuelBudgetPolicy:
    """Hard per-car caps on accumulated absolute longitudinal displacement."""

    budgets: tuple[int, ...]

    def validate(self, cars: int) -> None:
        if len(self.budgets) != cars:
            raise ValueError(f"expected {cars} fuel budgets, got {len(self.budgets)}")
        if any(isinstance(value, bool) or not isinstance(value, int) or value < 0 for value in self.budgets):
            raise ValueError("fuel budgets must be non-negative integers")

    def smv(self) -> str:
        declarations = [f"  fuel_used_{car} : 0 .. {budget + 1};" for car, budget in enumerate(self.budgets)]
        initialization = []
        transitions = []
        invariants = []
        for car, budget in enumerate(self.budgets):
            initialization.append(f"INIT fuel_used_{car} = 0;")
            transitions.append(
                f"TRANS next(fuel_used_{car}) = min({budget + 1}, "
                f"fuel_used_{car} + abs(next({vehicle(car, 'v')})));"
            )
            invariants.append(f"INVAR fuel_used_{car} <= {budget};")
        return "\n".join((
            "-- FuelBudgetPolicy: observer only; vehicle choices remain symbolic.",
            "VAR", *declarations, *initialization, *transitions, *invariants, "",
        ))


@dataclass(frozen=True)
class ParkingSearchConfig:
    aisle_sections: int
    parking_spots: int
    initial_positions: tuple[int, ...]
    assignments: tuple[int, ...]
    fuel_budgets: tuple[int, ...]
    aisle_length: int = 80
    spot_length: int = 30
    connection_length: int = 20
    parking_tolerance: int = 5
    bmc_bound: int = 100
    require_collision_free: bool = False

    @property
    def cars(self) -> int:
        return len(self.initial_positions)

    @property
    def sections(self) -> int:
        return self.aisle_sections + self.parking_spots

    def validate(self) -> None:
        if self.aisle_sections < 2:
            raise ValueError("aisle_sections must be at least 2")
        if self.parking_spots < 1:
            raise ValueError("parking_spots must be positive")
        if self.parking_spots > self.aisle_sections - 1:
            raise ValueError("ordered spots require at least one distinct aisle gap per spot")
        if self.cars < 1:
            raise ValueError("at least one car is required")
        if len(self.assignments) != self.cars:
            raise ValueError("assignments must contain one spot index per car")
        if len(set(self.assignments)) != len(self.assignments):
            raise ValueError("car assignments must use distinct parking spots")
        if any(not 0 <= spot < self.parking_spots for spot in self.assignments):
            raise ValueError("assignment references an unknown parking spot")
        if any(not 0 <= position <= self.aisle_length for position in self.initial_positions):
            raise ValueError("initial positions must lie on A0")
        positive = (self.aisle_length, self.spot_length, self.connection_length)
        if any(isinstance(value, bool) or not isinstance(value, int) or value <= 0 for value in positive):
            raise ValueError("section and connection lengths must be positive integers")
        if not 0 <= self.parking_tolerance < self.spot_length:
            raise ValueError("parking_tolerance must be non-negative and shorter than a spot")
        if self.bmc_bound < 0:
            raise ValueError("bmc_bound must be non-negative")
        FuelBudgetPolicy(self.fuel_budgets).validate(self.cars)


@dataclass
class SymbolicParkingModel:
    morty: Morty
    config: ParkingSearchConfig
    model_file: Path
    metadata_file: Path

    @property
    def target_path(self) -> Path:
        return self.morty.get_target_path()


def parking_property(config: ParkingSearchConfig) -> str:
    clauses = []
    for car, spot in enumerate(config.assignments):
        section = config.aisle_sections + spot
        clauses.extend((
            f"{vehicle(car, 'on_straight_section')} = {section}",
            f"{vehicle(car, 'abs_pos')} >= env.section_{section}_end - {config.parking_tolerance}",
            f"{vehicle(car, 'v')} = 0",
            f"fuel_used_{car} <= {config.fuel_budgets[car]}",
        ))
    return "INVARSPEC !(" + " & ".join(f"({clause})" for clause in clauses) + ");"


def _specialize_metrics(source: Path, config: ParkingSearchConfig) -> Path:
    text = source.read_text()
    for section in range(config.sections):
        length = config.aisle_length if section < config.aisle_sections else config.spot_length
        text = _replace_singleton_domain(text, f"section_{section}_end", length)
    for start in range(config.sections):
        for end in range(config.sections):
            if start != end:
                text = _replace_singleton_domain(
                    text, f"arclength_from_sec_{start}_to_sec_{end}_on_lane_0",
                    config.connection_length,
                )
    output = source.with_name("EnvModel_ParkingSymbolic.smv")
    output.write_text("-- Fixed metrics; structured topology remains symbolic.\n" + text)
    return output


def _topology_smv(config: ParkingSearchConfig) -> str:
    lines = [
        "-- Structured symbolic parking topology.",
        "FROZENVAR",
    ]
    gaps = config.aisle_sections - 1
    for spot in range(config.parking_spots):
        lines.append(f"  parking_attachment_{spot} : 0 .. {gaps - 1};")
    for spot in range(config.parking_spots - 1):
        lines.append(f"INIT parking_attachment_{spot} < parking_attachment_{spot + 1};")

    # With strict attachment ordering, at most one spot belongs to an aisle gap.
    for aisle in range(config.aisle_sections):
        direct = aisle + 1 if aisle < config.aisle_sections - 1 else -1
        lines.append(f"INIT env.outgoing_connection_0_of_section_{aisle} = {direct};")
        if aisle < config.aisle_sections - 1:
            choices = "\n".join(
                f"    parking_attachment_{spot} = {aisle} : {config.aisle_sections + spot};"
                for spot in range(config.parking_spots)
            )
            lines.extend((
                f"INIT env.outgoing_connection_1_of_section_{aisle} = case",
                choices, "    TRUE : -1;", "  esac;",
            ))
        else:
            lines.append(f"INIT env.outgoing_connection_1_of_section_{aisle} = -1;")
    for spot in range(config.parking_spots):
        section = config.aisle_sections + spot
        lines.extend((
            f"INIT env.outgoing_connection_0_of_section_{section} = parking_attachment_{spot} + 1;",
            f"INIT env.outgoing_connection_1_of_section_{section} = -1;",
        ))
    if config.require_collision_free:
        lines.append("INVAR !env.crash;")
    return "\n".join(lines) + "\n"


def _replace_default_initial_state(text: str, config: ParkingSearchConfig) -> str:
    for car in range(config.cars):
        prefix = re.escape(f"env.veh___6{car}9___")
        for field in (r"is_on_sec_\d+", "abs_pos"):
            text = re.sub(
                rf"(?m)^INIT {prefix}\.{field}\s*=.*;$",
                lambda match: "-- Replaced by symbolic parking search: " + match.group(0),
                text,
            )
    lines = ["", "-- Symbolic parking initial ordering on aisle section A0."]
    for car, position in enumerate(config.initial_positions):
        lines.extend((
            f"INIT {vehicle(car, 'is_on_sec_0')} = 1;",
            f"INIT {vehicle(car, 'abs_pos')} = {position};",
            f"INIT {vehicle(car, 'v')} = 5;",
        ))
    return text + "\n".join(lines) + "\n"


def generate_symbolic_parking_model(
    config: ParkingSearchConfig,
    *,
    input_config: str | Path = "main_template.json",
    experiments_dir: str | Path = "SMV_GEN",
) -> SymbolicParkingModel:
    """Generate one model where nuXmv chooses topology and driving behavior."""
    config.validate()
    generation_config = {
        "#007": config.cars, "NONEGOS": config.cars,
        "#008": 1, "NUMLANES": 1,
        "#010": config.sections, "SECTIONS": config.sections,
        "SECTIONSMINLENGTH": min(config.aisle_length, config.spot_length),
        "SECTIONSMAXLENGTH": max(config.aisle_length, config.spot_length),
        "MINDISTCONNECTIONS": config.connection_length,
        "MAXDISTCONNECTIONS": config.connection_length,
        "MAXOUTGOINGCONNECTIONS": 2,
        "MODEL_INTERSECTION_GEOMETRY": 0,
        "SPEC": parking_property(config),
    }
    morty = generate_scenario(
        generation_config, input_config=input_config,
        experiments_dir=experiments_dir, scenario_suffix="symbolic_parking",
    )
    target = morty.get_target_path()
    if target.is_absolute() and not target.exists():
        actual = Path.cwd() / str(target).lstrip("/")
        if actual.exists():
            target = actual
            morty.args[5] = str(actual)
    planner_smv = target / "planner.cpp_combined.k2.smv"
    if not planner_smv.exists():
        # The native generator prefixes cwd to absolute experiment paths, but
        # Morty's first kratos invocation still uses the unprefixed path.
        subprocess.run(
            [
                "external/linux64/kratos",
                "-trans_output_format=nuxmv-module",
                "-trans_enum_mode=symbolic",
                f"-output_file={planner_smv}",
                str(target / "planner.cpp_combined.k2"),
            ],
            check=True,
        )
    env_model = _specialize_metrics(target / "EnvModel.smv", config)
    main_text = (target / "main.smv").read_text().replace(
        '#include "EnvModel.smv"', f'#include "{env_model.name}"'
    )
    main_text = _replace_default_initial_state(main_text, config)
    main_text += "\n" + _topology_smv(config) + "\n" + FuelBudgetPolicy(config.fuel_budgets).smv()
    model_file = target / "main_parking_symbolic.smv"
    model_file.write_text(main_text)
    metadata_file = target / "parking_search_config.json"
    metadata_file.write_text(json.dumps(asdict(config), indent=2) + "\n")
    return SymbolicParkingModel(morty, config, model_file, metadata_file)


def check_symbolic_parking_model(
    model: SymbolicParkingModel,
    *,
    cex_file: str = "cex_symbolic_parking.xml",
) -> Path | None:
    try:
        return model.morty.model_check(
            cex_file, 0, model_file=model.model_file.name,
            bound=model.config.bmc_bound,
        )
    finally:
        model.morty.close_model_checker()


def _first_state(path: Path) -> dict[str, str]:
    state = ET.parse(path).getroot().find("./node/state")
    if state is None:
        raise ValueError("counterexample has no initial state")
    return {
        value.get("variable"): value.text
        for value in state.findall("value")
        if value.get("variable") and value.text is not None
    }


def reconstruct_selected_topology(
    cex_file: str | Path, config: ParkingSearchConfig
) -> nx.MultiDiGraph:
    state = _first_state(Path(cex_file))
    graph = nx.MultiDiGraph()
    for aisle in range(config.aisle_sections):
        graph.add_node(f"A{aisle}", kind="aisle", length=config.aisle_length, section=aisle)
        if aisle + 1 < config.aisle_sections:
            graph.add_edge(f"A{aisle}", f"A{aisle + 1}", length=config.connection_length)
    for spot in range(config.parking_spots):
        key = f"parking_attachment_{spot}"
        if key not in state:
            raise ValueError(f"counterexample does not report {key}")
        attachment = int(state[key])
        name = f"P{spot}"
        graph.add_node(name, kind="parking", length=config.spot_length,
                       section=config.aisle_sections + spot, attachment=attachment)
        graph.add_edge(f"A{attachment}", name, length=config.connection_length)
        graph.add_edge(name, f"A{attachment + 1}", length=config.connection_length)
    return graph


def save_selected_topology(graph: nx.MultiDiGraph, target: Path) -> tuple[Path, Path]:
    graphml = target / "selected_parking_topology.graphml"
    json_file = target / "selected_parking_topology.json"
    nx.write_graphml(graph, graphml)
    json_file.write_text(json.dumps(nx.node_link_data(graph, edges="edges"), indent=2) + "\n")
    return graphml, json_file


def parse_counterexample_states(path: str | Path) -> list[dict[str, str]]:
    states: list[dict[str, str]] = []
    current: dict[str, str] = {}
    for node in ET.parse(path).getroot().findall("node"):
        state = node.find("state")
        if state is None:
            continue
        current = current.copy()
        for value in state.findall("value"):
            name = value.get("variable")
            if name and value.text is not None:
                current[name] = value.text
        states.append(current)
    return states


def print_counterexample_trace(path: Path, config: ParkingSearchConfig) -> None:
    names = {
        **{index: f"A{index}" for index in range(config.aisle_sections)},
        **{config.aisle_sections + index: f"P{index}" for index in range(config.parking_spots)},
    }
    for step, state in enumerate(parse_counterexample_states(path)):
        cars = []
        for car, budget in enumerate(config.fuel_budgets):
            section = int(state.get(vehicle(car, "on_straight_section"), -1))
            if section >= 0:
                location = names[section]
            else:
                start = int(state.get(vehicle(car, "traversion_from"), -1))
                end = int(state.get(vehicle(car, "traversion_to"), -1))
                location = f"{names.get(start, '?')}->{names.get(end, '?')}"
            used = state.get(f"fuel_used_{car}", "?")
            remaining = "?" if used == "?" else str(budget - int(used))
            cars.append(
                f"{chr(65 + car)}:{location:>6} "
                f"pos={state.get(vehicle(car, 'abs_pos'), '?'):>3} "
                f"v={state.get(vehicle(car, 'v'), '?'):>3} "
                f"fuel={used}/{budget} rem={remaining}"
            )
        print(f"{step:>3} | " + " | ".join(cars))


def _drawing_lines(
    graph: nx.MultiDiGraph, config: ParkingSearchConfig
) -> dict[str, tuple[tuple[float, float], tuple[float, float]]]:
    lines = {
        f"A{aisle}": ((aisle * 110.0, 0.0), (aisle * 110.0 + 75.0, 0.0))
        for aisle in range(config.aisle_sections)
    }
    for spot in range(config.parking_spots):
        attachment = graph.nodes[f"P{spot}"]["attachment"]
        center = attachment * 110.0 + 92.5
        lines[f"P{spot}"] = ((center - 10.0, 45.0), (center + 10.0, 45.0))
    return lines


def save_counterexample_gif(
    path: Path,
    graph: nx.MultiDiGraph,
    config: ParkingSearchConfig,
    output: Path,
    *,
    delay: float = 0.15,
) -> None:
    from matplotlib import pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    states = parse_counterexample_states(path)
    lines = _drawing_lines(graph, config)
    sections = {data["section"]: node for node, data in graph.nodes(data=True)}
    figure, axes = plt.subplots(figsize=(12, 3.8))
    axes.set_aspect("equal")
    axes.axis("off")
    axes.set_xlim(-25, (config.aisle_sections - 1) * 110 + 105)
    axes.set_ylim(-30, 80)
    for name, (start, end) in lines.items():
        axes.plot((start[0], end[0]), (start[1], end[1]), color="#555", linewidth=8)
        axes.text((start[0] + end[0]) / 2, start[1] + 11, name, ha="center")
    for start_name, end_name in graph.edges():
        axes.annotate(
            "", xy=lines[end_name][0], xytext=lines[start_name][1],
            arrowprops={"arrowstyle": "->", "color": "#888", "linewidth": 2},
        )
    palette = plt.get_cmap("tab10")
    dots = [
        axes.scatter([], [], s=190, color=palette(car),
                     label=f"{chr(65 + car)} → P{spot}", zorder=5)
        for car, spot in enumerate(config.assignments)
    ]
    title = axes.set_title("")
    axes.legend(loc="lower right")

    def car_point(state: dict[str, str], car: int) -> tuple[float, float]:
        position = float(state.get(vehicle(car, "abs_pos"), 0))
        section = int(state.get(vehicle(car, "on_straight_section"), -1))
        if section >= 0:
            name = sections[section]
            start, end = lines[name]
            length = graph.nodes[name]["length"]
        else:
            source = sections[int(state[vehicle(car, "traversion_from")])]
            target = sections[int(state[vehicle(car, "traversion_to")])]
            start, end = lines[source][1], lines[target][0]
            length = config.connection_length
        fraction = max(0.0, min(1.0, position / length))
        return tuple(a + fraction * (b - a) for a, b in zip(start, end))

    def update(frame: int):
        state = states[frame]
        for car, dot in enumerate(dots):
            dot.set_offsets([car_point(state, car)])
        fuel = ", ".join(
            f"{chr(65+car)} {state.get(f'fuel_used_{car}', '?')}/{budget}"
            for car, budget in enumerate(config.fuel_budgets)
        )
        title.set_text(f"Symbolic parking — step {frame}/{len(states)-1} — {fuel}")
        return (*dots, title)

    animation = FuncAnimation(
        figure, update, frames=len(states), interval=max(1, int(delay * 1000))
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    animation.save(
        output, writer=PillowWriter(fps=max(1, round(1 / max(delay, 0.01))))
    )
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--aisle-sections", type=int, default=4)
    parser.add_argument("--spots", type=int, default=2)
    parser.add_argument("--positions", nargs="+", type=int, default=[20, 5])
    parser.add_argument("--assignments", nargs="+", type=int, default=[1, 0])
    parser.add_argument("--fuel-budgets", nargs="+", type=int, default=[120, 65])
    parser.add_argument("--aisle-length", type=int, default=30)
    parser.add_argument("--spot-length", type=int, default=20)
    parser.add_argument("--connection-length", type=int, default=10)
    parser.add_argument("--parking-tolerance", type=int, default=5)
    parser.add_argument(
        "--require-collision-free", action="store_true",
        help="also require !env.crash (experimental for non-linear road graphs)",
    )
    parser.add_argument("--bound", type=int, default=50)
    parser.add_argument("--output-dir", default="SMV_GEN")
    parser.add_argument("--generate-only", action="store_true")
    parser.add_argument("--gif", nargs="?", const="", metavar="FILE")
    parser.add_argument("--delay", type=float, default=0.15)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.delay < 0:
        raise SystemExit("--delay must be non-negative")
    config = ParkingSearchConfig(
        aisle_sections=args.aisle_sections, parking_spots=args.spots,
        initial_positions=tuple(args.positions), assignments=tuple(args.assignments),
        fuel_budgets=tuple(args.fuel_budgets), aisle_length=args.aisle_length,
        spot_length=args.spot_length, connection_length=args.connection_length,
        parking_tolerance=args.parking_tolerance, bmc_bound=args.bound,
        require_collision_free=args.require_collision_free,
    )
    model = generate_symbolic_parking_model(config, experiments_dir=args.output_dir)
    print(f"Model: {model.model_file}")
    if args.generate_only:
        return 0
    cex = check_symbolic_parking_model(model)
    if cex is None:
        print("No topology and parking strategy found within the bound.")
        return 1
    graph = reconstruct_selected_topology(cex, config)
    outputs = save_selected_topology(graph, model.target_path)
    print(f"Counterexample: {cex}")
    print("Attachments:", {node: data["attachment"] for node, data in graph.nodes(data=True)
                           if data["kind"] == "parking"})
    print(f"Topology: {outputs[0]} and {outputs[1]}")
    print_counterexample_trace(cex, config)
    if args.gif is not None:
        gif = Path(args.gif) if args.gif else model.target_path / "symbolic_parking.gif"
        save_counterexample_gif(cex, graph, config, gif.resolve(), delay=args.delay)
        print(f"Animation: {gif.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
