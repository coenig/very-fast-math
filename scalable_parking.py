"""Scalable section-level parking topology search and visualization.

This deliberately small SMV model keeps the parking problem's graph semantics
without expanding VFM's all-pairs road geometry.  Aisle sections are connected
in a directed horizontal chain.  Parking sections attach to symbolic, ordered
aisle gaps and are either terminal vertical spots, return loops, or a symbolic
mixture of both.
"""

from __future__ import annotations

import argparse
import json
import time
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from pathlib import Path

import networkx as nx

from model_checking import NuXmvConn


@dataclass(frozen=True)
class ScalableParkingConfig:
    cars: int = 10
    aisle_sections: int = 22
    topology: str = "mixed"
    assignments: tuple[int, ...] = ()
    fuel_budgets: tuple[int, ...] = ()
    bound: int = 200
    aisle_cost: int = 10
    parking_cost: int = 6
    return_cost: int = 4
    parking_spots: int = 0
    property_mode: str = "basic"

    @property
    def spots(self) -> int:
        return self.parking_spots or self.cars

    @property
    def nodes(self) -> int:
        return self.aisle_sections + self.spots

    def normalized(self) -> "ScalableParkingConfig":
        spots = self.parking_spots or self.cars
        if self.assignments:
            assignments = self.assignments
        elif self.property_mode == "yield":
            assignments = (spots - 1, *range(1, self.cars))
        else:
            assignments = tuple(reversed(range(self.cars)))
        budgets = self.fuel_budgets or tuple(220 for _ in range(self.cars))
        result = ScalableParkingConfig(
            self.cars, self.aisle_sections, self.topology,
            tuple(assignments), tuple(budgets), self.bound,
            self.aisle_cost, self.parking_cost, self.return_cost,
            spots, self.property_mode,
        )
        result.validate()
        return result

    def validate(self) -> None:
        if self.cars < 2:
            raise ValueError("at least two cars are required")
        if self.aisle_sections < 2 * self.cars + 1:
            raise ValueError("the aisle needs at least 2*cars+1 sections")
        if self.topology not in {"vertical", "loops", "mixed"}:
            raise ValueError("topology must be vertical, loops, or mixed")
        if len(self.assignments) != self.cars or len(set(self.assignments)) != self.cars:
            raise ValueError("cars must have distinct parking assignments")
        if any(not 0 <= spot < self.spots for spot in self.assignments):
            raise ValueError("assignment references an unknown parking spot")
        if self.property_mode not in {"basic", "yield"}:
            raise ValueError("property_mode must be basic or yield")
        if self.property_mode == "yield":
            if self.spots < self.cars + 1:
                raise ValueError("yield mode requires at least one unassigned spot")
            if 0 in self.assignments or self.assignments[0] == 0:
                raise ValueError("P0 must be an unassigned, non-goal waiting bay")
        if len(self.fuel_budgets) != self.cars or any(x <= 0 for x in self.fuel_budgets):
            raise ValueError("one positive fuel budget is required per car")
        if self.bound < 0:
            raise ValueError("bound must be non-negative")


def _or(parts: list[str]) -> str:
    return " |\n    ".join(parts)


def render_smv(config: ScalableParkingConfig) -> str:
    config = config.normalized()
    last_aisle = config.aisle_sections - 1
    lines = [
        "MODULE main", "", "FROZENVAR",
    ]
    for spot in range(config.spots):
        lines.append(f"  attachment_{spot} : 0..{last_aisle - 1};")
        if config.topology == "mixed":
            lines.append(f"  returns_{spot} : boolean;")
    lines.extend(("", "VAR"))
    for car, budget in enumerate(config.fuel_budgets):
        lines.extend((
            f"  loc_{car} : 0..{config.nodes - 1};",
            f"  fuel_{car} : 0..{budget};",
        ))
    if config.property_mode == "yield":
        lines.extend((
            "  yield_seen : boolean;",
            "  yield_waited : boolean;",
            "  pass_seen : boolean;",
            "  rejoined_seen : boolean;",
        ))
    lines.extend(("", "-- Ordered symbolic parking attachments."))
    for spot in range(config.spots - 1):
        lines.append(f"INVAR attachment_{spot} < attachment_{spot + 1};")
    lines.append(f"INVAR attachment_0 >= {config.cars};")
    if config.topology == "mixed":
        lines.append("INVAR " + " | ".join(f"returns_{s}" for s in range(config.spots)) + ";")
        lines.append("INVAR " + " | ".join(f"!returns_{s}" for s in range(config.spots)) + ";")
        if config.property_mode == "yield":
            lines.append("INVAR returns_0;")

    lines.extend(("", "-- Initially the cars occupy consecutive aisle sections."))
    for car in range(config.cars):
        lines.extend((f"INIT loc_{car} = {config.cars - 1 - car};", f"INIT fuel_{car} = 0;"))

    for car, budget in enumerate(config.fuel_budgets):
        moves = [
            f"next(loc_{car}) = loc_{car}",
            f"(loc_{car} < {last_aisle} & next(loc_{car}) = loc_{car} + 1)",
        ]
        for spot in range(config.spots):
            node = config.aisle_sections + spot
            moves.append(f"(loc_{car} = attachment_{spot} & next(loc_{car}) = {node})")
            if config.topology == "loops":
                moves.append(f"(loc_{car} = {node} & next(loc_{car}) = attachment_{spot} + 1)")
            elif config.topology == "mixed":
                moves.append(
                    f"(returns_{spot} & loc_{car} = {node} & "
                    f"next(loc_{car}) = attachment_{spot} + 1)"
                )
            elif config.property_mode == "yield" and spot == 0:
                moves.append(f"(loc_{car} = {node} & next(loc_{car}) = attachment_{spot} + 1)")
        lines.extend(("", f"TRANS {_or(moves)};"))
        parking_targets = " | ".join(
            f"next(loc_{car}) = {config.aisle_sections + spot}"
            for spot in range(config.spots)
        )
        return_sources = " | ".join(
            f"loc_{car} = {config.aisle_sections + spot}"
            for spot in range(config.spots)
        )
        lines.append(
            f"TRANS next(fuel_{car}) = fuel_{car} + case\n"
            f"  next(loc_{car}) = loc_{car} : 0;\n"
            f"  {parking_targets} : {config.parking_cost};\n"
            f"  {return_sources} : {config.return_cost};\n"
            f"  TRUE : {config.aisle_cost};\n"
            "esac;"
        )

    lines.extend(("", "-- One car per section, including parking sections."))
    for first in range(config.cars):
        for second in range(first + 1, config.cars):
            lines.append(f"INVAR loc_{first} != loc_{second};")
    goals = [
        f"loc_{car} = {config.aisle_sections + spot}"
        for car, spot in enumerate(config.assignments)
    ]
    if config.property_mode == "yield":
        waiting_node = config.aisle_sections
        follower_goal = config.aisle_sections + config.assignments[1]
        lines.extend((
            "", "-- History monitor for a non-greedy yielding maneuver.",
            "ASSIGN",
            "  init(yield_seen) := FALSE;",
            f"  next(yield_seen) := yield_seen | next(loc_0) = {waiting_node};",
            "  init(yield_waited) := FALSE;",
            f"  next(yield_waited) := yield_waited | "
            f"(yield_seen & loc_0 = {waiting_node} & next(loc_0) = {waiting_node});",
            "  init(pass_seen) := FALSE;",
            f"  next(pass_seen) := pass_seen | (yield_waited & loc_0 = {waiting_node} & "
            f"(((loc_1 > attachment_0) & (loc_1 < {config.aisle_sections})) | "
            f"loc_1 = {follower_goal}));",
            "  init(rejoined_seen) := FALSE;",
            f"  next(rejoined_seen) := rejoined_seen | (pass_seen & loc_0 = {waiting_node} & "
            "next(loc_0) = attachment_0 + 1);",
        ))
        goals.extend(("yield_waited", "pass_seen", "rejoined_seen"))
    lines.extend(("", "-- A counterexample is a state in which all cars are parked.",
                  "INVARSPEC !(" + " & ".join(goals) + ");", ""))
    return "\n".join(lines)


def generate_model(config: ScalableParkingConfig, output_dir: Path) -> tuple[Path, ScalableParkingConfig]:
    config = config.normalized()
    target = output_dir / f"scenario{time.time_ns()}_{config.topology}_{config.cars}cars"
    target.mkdir(parents=True, exist_ok=False)
    model = target / "parking_scalable.smv"
    model.write_text(render_smv(config))
    (target / "parking_config.json").write_text(json.dumps(asdict(config), indent=2) + "\n")
    return model, config


def check_model(model: Path, bound: int) -> Path | None:
    connection = NuXmvConn(["nuXmv", "-quiet", "-int", model.name], cwd=model.parent)
    try:
        connection._send("go_msat")
        setup = connection._read_response()
        if "error" in setup.lower():
            raise RuntimeError(setup)
        connection._send(f"msat_check_invar_bmc -a falsification -k {bound} -n 0")
        output = connection._read_response()
        if "counter-example of size" not in output.lower() and "trace description:" not in output.lower():
            return None
        counterexample = (model.parent / "cex_scalable_parking.xml").resolve()
        connection._send(f"show_traces -p 6 -o {counterexample} 1")
        connection._read_response()
        return counterexample if counterexample.exists() else None
    finally:
        connection.close()


def parse_states(counterexample: Path) -> list[dict[str, str]]:
    result, current = [], {}
    for node in ET.parse(counterexample).getroot().findall("node"):
        state = node.find("state")
        if state is None:
            continue
        current = current.copy()
        for value in state.findall("value"):
            if value.get("variable") and value.text is not None:
                current[value.get("variable")] = value.text
        result.append(current)
    return result


def selected_graph(state: dict[str, str], config: ScalableParkingConfig) -> nx.MultiDiGraph:
    graph = nx.MultiDiGraph()
    for aisle in range(config.aisle_sections):
        graph.add_node(f"A{aisle}", kind="aisle", section=aisle)
        if aisle + 1 < config.aisle_sections:
            graph.add_edge(f"A{aisle}", f"A{aisle + 1}")
    for spot in range(config.spots):
        attachment = int(state[f"attachment_{spot}"])
        returns = (config.property_mode == "yield" and spot == 0) or config.topology == "loops" or (
            config.topology == "mixed" and state.get(f"returns_{spot}") == "TRUE"
        )
        name = f"P{spot}"
        graph.add_node(name, kind="parking", section=config.aisle_sections + spot,
                       attachment=attachment, returns=returns)
        graph.add_edge(f"A{attachment}", name)
        if returns:
            graph.add_edge(name, f"A{attachment + 1}")
    return graph


def save_gif(counterexample: Path, graph: nx.MultiDiGraph,
             config: ScalableParkingConfig, output: Path) -> None:
    from matplotlib import pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    states = parse_states(counterexample)
    width = max(12, config.aisle_sections * 0.85)
    figure, axes = plt.subplots(figsize=(width, 4.8))
    positions = {f"A{i}": (i, 0.0) for i in range(config.aisle_sections)}
    for spot in range(config.spots):
        attachment = graph.nodes[f"P{spot}"]["attachment"]
        positions[f"P{spot}"] = (attachment + 0.35, 1.0)
    nx.draw_networkx_edges(graph, positions, ax=axes, arrows=True, edge_color="#888")
    nx.draw_networkx_nodes(graph, positions, ax=axes, node_color=[
        "#dcecff" if graph.nodes[node]["kind"] == "aisle" else
        ("#ffe0a8" if graph.nodes[node]["returns"] else "#ccebc5")
        for node in graph.nodes
    ], node_size=650)
    nx.draw_networkx_labels(graph, positions, ax=axes, font_size=8)
    palette = plt.get_cmap("tab10")
    dots = [axes.scatter([], [], s=130, color=palette(i % 10), zorder=5)
            for i in range(config.cars)]
    section_names = {data["section"]: node for node, data in graph.nodes(data=True)}
    title = axes.set_title("")
    axes.axis("off")

    def update(frame: int):
        state = states[frame]
        for car, dot in enumerate(dots):
            dot.set_offsets([positions[section_names[int(state[f"loc_{car}"])]]])
        fuel = ", ".join(f"{chr(65+i)}={state[f'fuel_{i}']}" for i in range(config.cars))
        title.set_text(f"{config.topology} | step {frame}/{len(states)-1} | fuel: {fuel}")
        return [*dots, title]

    animation = FuncAnimation(figure, update, frames=len(states), interval=350, blit=False)
    animation.save(output, writer=PillowWriter(fps=3))
    plt.close(figure)


def fuel_profile(name: str, cars: int) -> tuple[int, ...]:
    if name == "tight":
        return tuple(115 + 8 * (cars - 1 - car) for car in range(cars))
    if name == "balanced":
        return tuple(190 for _ in range(cars))
    if name == "generous":
        return tuple(300 for _ in range(cars))
    raise ValueError(name)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cars", type=int, default=10)
    parser.add_argument("--aisle-sections", type=int, default=22)
    parser.add_argument("--topology", choices=("vertical", "loops", "mixed"), default="mixed")
    parser.add_argument("--fuel-profile", choices=("tight", "balanced", "generous"), default="balanced")
    parser.add_argument("--spots", type=int, default=0,
                        help="parking spot count; zero means one per car")
    parser.add_argument("--property", dest="property_mode", choices=("basic", "yield"), default="basic")
    parser.add_argument("--bound", type=int, default=200)
    parser.add_argument("--output-dir", type=Path, default=Path("SMV_GEN/scalable_parking"))
    parser.add_argument("--gif", action="store_true")
    args = parser.parse_args()
    config = ScalableParkingConfig(
        cars=args.cars, aisle_sections=args.aisle_sections, topology=args.topology,
        fuel_budgets=fuel_profile(args.fuel_profile, args.cars), bound=args.bound,
        parking_spots=args.spots, property_mode=args.property_mode,
    )
    model, config = generate_model(config, args.output_dir)
    print(f"Model: {model}")
    counterexample = check_model(model, config.bound)
    if counterexample is None:
        print(f"No witness found through bound {config.bound}")
        return 2
    states = parse_states(counterexample)
    graph = selected_graph(states[0], config)
    nx.write_graphml(graph, model.parent / "selected_topology.graphml")
    (model.parent / "selected_topology.json").write_text(
        json.dumps(nx.node_link_data(graph, edges="edges"), indent=2) + "\n"
    )
    print(f"Counterexample: {counterexample} ({len(states)} states)")
    if args.gif:
        gif = model.parent / "parking.gif"
        save_gif(counterexample, graph, config, gif)
        print(f"Animation: {gif.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
