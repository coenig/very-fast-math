#!/usr/bin/env python3
"""Generate long symbolic-grid parking permutations for nuXmv."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import networkx as nx

from scalable_grid_parking import check_model, edge_name, grid_edges
from scalable_parking import parse_states


@dataclass(frozen=True)
class PermutationConfig:
    rows: int = 3
    columns: int = 4
    cars: int = 4
    rotations: int = 2
    min_role_distance: int = 2
    bound: int = 120
    fuel_budget: int = 300
    edge_cost: int = 10
    check_mode: str = "monitor"

    @property
    def nodes(self) -> int:
        return self.rows * self.columns

    @property
    def role_names(self) -> tuple[str, ...]:
        return tuple(f"park_{car}" for car in range(self.cars)) + ("buffer",)

    def validate(self) -> None:
        if self.rows < 2 or self.columns < 2:
            raise ValueError("the grid must be at least 2x2")
        if self.cars != 4:
            raise ValueError("the first permutation scenario requires exactly four cars")
        if self.nodes < self.cars + 1:
            raise ValueError("the grid needs one distinct cell per car plus a buffer")
        if self.rotations < 1 or self.min_role_distance < 1:
            raise ValueError("rotations and minimum role distance must be positive")
        if self.bound < 0 or self.fuel_budget <= 0 or self.edge_cost <= 0:
            raise ValueError("bound must be non-negative and fuel values positive")
        if self.check_mode not in {"ltl", "monitor"}:
            raise ValueError("check_mode must be ltl or monitor")


def permutation_phases(config: PermutationConfig) -> list[tuple[str, ...]]:
    """Return every complete configuration for repeated buffer rotations."""
    roles = [f"park_{car}" for car in range(config.cars)]
    locations = roles.copy()
    phases = [tuple(locations)]
    for rotation in range(config.rotations):
        first = rotation % config.cars
        old_locations = locations.copy()
        locations[first] = "buffer"
        phases.append(tuple(locations))
        for offset in range(1, config.cars):
            car = (first + offset) % config.cars
            previous_car = (car - 1) % config.cars
            locations[car] = old_locations[previous_car]
            phases.append(tuple(locations))
        last = (first - 1) % config.cars
        locations[first] = old_locations[last]
        phases.append(tuple(locations))
    return phases


def phase_expression(phase: tuple[str, ...]) -> str:
    return "(" + " & ".join(
        f"loc_{car} = {role}" for car, role in enumerate(phase)
    ) + ")"


def chained_history(phases: list[tuple[str, ...]]) -> str:
    expression = phase_expression(phases[-1])
    for phase in reversed(phases[:-1]):
        target = phase_expression(phase)
        expression = f"((!{target}) U ({target} & X ({expression})))"
    return expression


def progress_monitor(phases: list[tuple[str, ...]]) -> str:
    final = len(phases) - 1
    cases = [
        f"    permutation_phase = {index - 1} & {phase_expression(phase)} : {index};"
        for index, phase in enumerate(phases[1:], start=1)
    ]
    return "\n".join((
        "ASSIGN",
        "  init(permutation_phase) := 0;",
        "  next(permutation_phase) := case",
        *cases,
        "    TRUE : permutation_phase;",
        "  esac;",
        f"INVARSPEC permutation_phase != {final};",
    ))


def role_constraints(config: PermutationConfig) -> list[str]:
    lines: list[str] = []
    roles = config.role_names
    for first_index, first in enumerate(roles):
        for second in roles[first_index + 1:]:
            lines.append(f"INIT {first} != {second};")
            for first_cell in range(config.nodes):
                first_row, first_column = divmod(first_cell, config.columns)
                for second_cell in range(config.nodes):
                    second_row, second_column = divmod(second_cell, config.columns)
                    distance = abs(first_row - second_row) + abs(first_column - second_column)
                    if 0 < distance < config.min_role_distance:
                        lines.append(
                            f"INIT !({first} = {first_cell} & {second} = {second_cell});"
                        )
    return lines


def render_smv(config: PermutationConfig) -> str:
    config.validate()
    edges = grid_edges(config)  # type: ignore[arg-type]
    lines = ["MODULE main", "", "FROZENVAR"]
    lines.extend(f"  {edge_name(*edge)} : boolean;" for edge in edges)
    lines.extend(f"  {role} : 0..{config.nodes - 1};" for role in config.role_names)
    lines.extend(("", "-- Symbolic, distinct, well-separated parking roles."))
    lines.extend(role_constraints(config))
    lines.extend(("", "VAR"))
    for car in range(config.cars):
        lines.extend((
            f"  loc_{car} : 0..{config.nodes - 1};",
            f"  fuel_{car} : 0..{config.fuel_budget};",
        ))
    phases = permutation_phases(config)
    if config.check_mode == "monitor":
        lines.append(f"  permutation_phase : 0..{len(phases) - 1};")
    for car in range(config.cars):
        lines.extend(("", f"INIT loc_{car} = park_{car};", f"INIT fuel_{car} = 0;"))
        moves = [f"next(loc_{car}) = loc_{car}"]
        for start, end in edges:
            enabled = edge_name(start, end)
            moves.extend((
                f"({enabled} & loc_{car} = {start} & next(loc_{car}) = {end})",
                f"({enabled} & loc_{car} = {end} & next(loc_{car}) = {start})",
            ))
        lines.append("TRANS " + " |\n  ".join(moves) + ";")
        lines.append(
            f"TRANS next(fuel_{car}) = fuel_{car} + case "
            f"next(loc_{car}) = loc_{car} : 0; TRUE : {config.edge_cost}; esac;"
        )
    lines.extend(("", "-- Unit-capacity cells and no direct exchanges."))
    for first in range(config.cars):
        for second in range(first + 1, config.cars):
            lines.append(f"INVAR loc_{first} != loc_{second};")
            lines.append(
                f"TRANS !(next(loc_{first}) = loc_{second} & next(loc_{second}) = loc_{first});"
            )
    lines.extend(("", f"-- {len(phases)} configurations for {config.rotations} rotations."))
    if config.check_mode == "ltl":
        lines.append(f"LTLSPEC !({chained_history(phases)});")
    else:
        lines.append(progress_monitor(phases))
    lines.append("")
    return "\n".join(lines)


def selected_graph(config: PermutationConfig, state: dict[str, str]) -> nx.Graph:
    graph = nx.Graph()
    for cell in range(config.nodes):
        row, column = divmod(cell, config.columns)
        graph.add_node(cell, row=row, column=column)
    for edge in grid_edges(config):  # type: ignore[arg-type]
        if state.get(edge_name(*edge), "FALSE").upper() == "TRUE":
            graph.add_edge(*edge)
    return graph


def selected_roles(config: PermutationConfig, state: dict[str, str]) -> dict[str, int]:
    return {role: int(state[role]) for role in config.role_names}


def save_gif(counterexample: Path, graph: nx.Graph, roles: dict[str, int],
             config: PermutationConfig, output: Path) -> None:
    from matplotlib import pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    states = parse_states(counterexample)
    positions = {node: (data["column"], -data["row"])
                 for node, data in graph.nodes(data=True)}
    colors = []
    role_cells = set(roles.values())
    for node in graph.nodes:
        if node == roles["buffer"]:
            colors.append("#ffd166")
        elif node in role_cells:
            colors.append("#b7e4c7")
        else:
            colors.append("#e5ecf4")
    figure, axes = plt.subplots(figsize=(2.2 * config.columns, 2 * config.rows))
    nx.draw_networkx_nodes(graph, positions, node_color=colors, node_size=780, ax=axes)
    nx.draw_networkx_labels(graph, positions, ax=axes)
    nx.draw_networkx_edges(graph, positions, width=2.5, edge_color="#748cab", ax=axes)
    for role, cell in roles.items():
        axes.annotate(role, positions[cell], xytext=(0, -22), textcoords="offset points",
                      ha="center", fontsize=8)
    palette = plt.get_cmap("tab10")
    dots = [axes.scatter([], [], s=190, color=palette(car), label=chr(65 + car), zorder=5)
            for car in range(config.cars)]
    title = axes.set_title("")
    axes.legend(loc="lower center", ncol=config.cars)
    axes.axis("off")

    def update(frame: int):
        state = states[frame]
        for car, dot in enumerate(dots):
            dot.set_offsets([positions[int(state[f"loc_{car}"])]] )
        fuels = ", ".join(state[f"fuel_{car}"] for car in range(config.cars))
        title.set_text(f"Two parking rotations | step {frame}/{len(states)-1} | fuel [{fuels}]")
        return [*dots, title]

    FuncAnimation(figure, update, frames=len(states), interval=400).save(
        output, writer=PillowWriter(fps=3)
    )
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rows", type=int, default=3)
    parser.add_argument("--columns", type=int, default=4)
    parser.add_argument("--cars", type=int, default=4)
    parser.add_argument("--rotations", type=int, default=2)
    parser.add_argument("--min-role-distance", type=int, default=2)
    parser.add_argument("--bound", type=int, default=120)
    parser.add_argument("--fuel", type=int, default=300)
    parser.add_argument("--edge-cost", type=int, default=10)
    parser.add_argument("--check-mode", choices=("monitor", "ltl"), default="monitor")
    parser.add_argument("--output-dir", type=Path,
                        default=Path("vfm-with-parking/permutations"))
    args = parser.parse_args()
    config = PermutationConfig(
        args.rows, args.columns, args.cars, args.rotations,
        args.min_role_distance, args.bound, args.fuel, args.edge_cost, args.check_mode,
    )
    config.validate()
    target = args.output_dir / f"scenario{time.time_ns()}_grid_permutation"
    target.mkdir(parents=True, exist_ok=False)
    model = target / "abstract_grid_permutation.smv"
    model.write_text(render_smv(config))
    (target / "permutation_property.ltl").write_text(
        "LTLSPEC !(" + chained_history(permutation_phases(config)) + ");\n"
    )
    (target / "config.json").write_text(json.dumps(asdict(config), indent=2) + "\n")
    print(f"Model: {model.resolve()}")
    counterexample = check_model(model, config.bound, temporal=config.check_mode == "ltl")
    if counterexample is None:
        print(f"No counterexample through bound {config.bound}")
        return 2
    states = parse_states(counterexample)
    graph = selected_graph(config, states[0])
    roles = selected_roles(config, states[0])
    nx.write_graphml(graph, target / "selected_grid.graphml")
    (target / "selected_grid.json").write_text(
        json.dumps(nx.node_link_data(graph, edges="edges"), indent=2) + "\n"
    )
    (target / "selected_roles.json").write_text(json.dumps(roles, indent=2) + "\n")
    gif = target / "abstract_grid_permutation.gif"
    save_gif(counterexample, graph, roles, config, gif)
    print(f"Counterexample: {counterexample} ({len(states)} states)")
    print(f"Roles: {roles}")
    print(f"Animation: {gif.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
