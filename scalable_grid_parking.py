"""Symbolic sparse 2-D grid parking scenarios for nuXmv.

The grid has a fixed serpentine spanning path and symbolic optional orthogonal
edges.  Consequently every generated graph is connected, but it is not a full
four-neighbour grid.  nuXmv chooses the useful optional edges and all vehicle
movements subject to section capacity and fuel budgets.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

import networkx as nx

from model_checking import NuXmvConn
from scalable_parking import parse_states


@dataclass(frozen=True)
class GridParkingConfig:
    rows: int = 5
    columns: int = 6
    cars: int = 10
    bound: int = 300
    fuel_budgets: tuple[int, ...] = ()
    edge_cost: int = 10
    swap_rounds: int = 0
    grid_mode: str = "symbolic"

    @property
    def nodes(self) -> int:
        return self.rows * self.columns

    def normalized(self) -> "GridParkingConfig":
        result = GridParkingConfig(
            self.rows, self.columns, self.cars, self.bound,
            self.fuel_budgets or tuple(400 for _ in range(self.cars)),
            self.edge_cost, self.swap_rounds, self.grid_mode,
        )
        result.validate()
        return result

    def validate(self) -> None:
        if self.rows < 2 or self.columns < 2:
            raise ValueError("the grid must be at least 2x2")
        if self.cars < 2 or self.nodes < 2 * self.cars:
            raise ValueError("the grid needs at least two distinct nodes per car")
        if len(self.fuel_budgets) != self.cars or any(x <= 0 for x in self.fuel_budgets):
            raise ValueError("one positive fuel budget is required per car")
        if self.swap_rounds < 0:
            raise ValueError("swap_rounds must be non-negative")
        if self.grid_mode not in {"symbolic", "corridor"}:
            raise ValueError("grid_mode must be symbolic or corridor")


def node(row: int, column: int, config: GridParkingConfig) -> int:
    return row * config.columns + column


def snake(config: GridParkingConfig) -> list[int]:
    result = []
    for row in range(config.rows):
        columns = range(config.columns) if row % 2 == 0 else range(config.columns - 1, -1, -1)
        result.extend(node(row, column, config) for column in columns)
    return result


def grid_edges(config: GridParkingConfig) -> list[tuple[int, int]]:
    edges = []
    for row in range(config.rows):
        for column in range(config.columns):
            here = node(row, column, config)
            if column + 1 < config.columns:
                edges.append((here, node(row, column + 1, config)))
            if row + 1 < config.rows:
                edges.append((here, node(row + 1, column, config)))
    return edges


def edge_name(start: int, end: int) -> str:
    first, second = sorted((start, end))
    return f"edge_{first}_{second}"


def topology(config: GridParkingConfig):
    path = snake(config)
    backbone = {tuple(sorted(pair)) for pair in zip(path, path[1:])}
    all_edges = {tuple(sorted(edge)) for edge in grid_edges(config)}
    return path, backbone, sorted(all_edges - backbone)


def starts_and_goals(config: GridParkingConfig) -> tuple[list[int], list[int]]:
    path = snake(config)
    starts = path[:2 * config.cars:2]
    goals = list(reversed(path[-config.cars:]))
    return starts, goals


def parking_phases(config: GridParkingConfig) -> list[list[int]]:
    """Build staged pair swaps; each swap contributes three LTL milestones."""
    path = snake(config)
    _, goals = starts_and_goals(config)
    assignment = list(goals)
    phases = [assignment.copy()]
    for _ in range(config.swap_rounds):
        for car in range(0, config.cars - 1, 2):
            staging = path[-config.cars - 1 - car // 2]
            first_goal, second_goal = assignment[car], assignment[car + 1]
            staged = assignment.copy()
            staged[car] = staging
            phases.append(staged)
            yielded = staged.copy()
            yielded[car + 1] = first_goal
            phases.append(yielded)
            assignment[car], assignment[car + 1] = assignment[car + 1], assignment[car]
            phases.append(assignment.copy())
    return phases


def phase_expression(phase: list[int]) -> str:
    return "(" + " & ".join(
        f"loc_{car} = {goal}" for car, goal in enumerate(phase)
    ) + ")"


def chained_until(phases: list[list[int]]) -> str:
    """Require each complete parking configuration to occur in order."""
    expression = phase_expression(phases[-1])
    for phase in reversed(phases[:-1]):
        target = phase_expression(phase)
        expression = f"((!{target}) U ({target} & X ({expression})))"
    return expression


def render_smv(config: GridParkingConfig) -> str:
    config = config.normalized()
    _, backbone, optional = topology(config)
    starts, goals = starts_and_goals(config)
    incident = {n: [] for n in range(config.nodes)}
    for edge in grid_edges(config):
        canonical = tuple(sorted(edge))
        incident[edge[0]].append(canonical)
        incident[edge[1]].append(canonical)
    optional_set = set(optional)
    lines = ["MODULE main", "", "FROZENVAR"]
    lines.extend(f"  {edge_name(*edge)} : boolean;" for edge in optional)
    lines.extend(("", "VAR"))
    for car, budget in enumerate(config.fuel_budgets):
        lines.extend((f"  loc_{car} : 0..{config.nodes - 1};", f"  fuel_{car} : 0..{budget};"))
    lines.extend(("", "-- Sparse cells keep only their guaranteed backbone edges."))
    if config.grid_mode == "corridor":
        upper_row = config.rows - 2
        backbone_column = config.columns - 1 if upper_row % 2 == 0 else 0
        gate_column = 0 if backbone_column != 0 else config.columns - 1
        gate = tuple(sorted((node(upper_row, gate_column, config),
                             node(upper_row + 1, gate_column, config))))
        for edge in optional:
            lines.append(
                f"INVAR {' ' if edge == gate else '!'}{edge_name(*edge)};"
            )
    else:
        disabled = set()
        for grid_node, edges in incident.items():
            row, column = divmod(grid_node, config.columns)
            if (row + column) % 4 == 0:
                disabled.update(edge for edge in edges if edge in optional_set)
        for edge in sorted(disabled):
            lines.append(f"INVAR !{edge_name(*edge)};")
        selectable = [edge for edge in optional if edge not in disabled]
        if selectable:
            lines.append("INVAR " + " | ".join(edge_name(*edge) for edge in selectable) + ";")
        for row in range(config.rows - 1):
            gates = [
                edge for edge in selectable
                if {edge[0] // config.columns, edge[1] // config.columns} == {row, row + 1}
            ]
            if gates:
                lines.append("INVAR " + " | ".join(edge_name(*edge) for edge in gates) + ";")
    for grid_node, edges in incident.items():
        if len(edges) == 4:
            conditions = ["TRUE" if edge in backbone else edge_name(*edge) for edge in edges]
            lines.append("INVAR !(" + " & ".join(conditions) + ");")

    for car, (start, budget) in enumerate(zip(starts, config.fuel_budgets)):
        lines.extend(("", f"INIT loc_{car} = {start};", f"INIT fuel_{car} = 0;"))
        moves = [f"next(loc_{car}) = loc_{car}"]
        for start_node, end_node in grid_edges(config):
            canonical = tuple(sorted((start_node, end_node)))
            enabled = "TRUE" if canonical in backbone else edge_name(*canonical)
            moves.extend((
                f"({enabled} & loc_{car} = {start_node} & next(loc_{car}) = {end_node})",
                f"({enabled} & loc_{car} = {end_node} & next(loc_{car}) = {start_node})",
            ))
        lines.append("TRANS " + " |\n  ".join(moves) + ";")
        lines.append(
            f"TRANS next(fuel_{car}) = fuel_{car} + "
            f"case next(loc_{car}) = loc_{car} : 0; TRUE : {config.edge_cost}; esac;"
        )
    lines.extend(("", "-- A grid cell has capacity one."))
    for first in range(config.cars):
        for second in range(first + 1, config.cars):
            lines.append(f"INVAR loc_{first} != loc_{second};")
            lines.append(
                f"TRANS !(next(loc_{first}) = loc_{second} & "
                f"next(loc_{second}) = loc_{first});"
            )
    if config.swap_rounds:
        desired = chained_until(parking_phases(config))
        lines.extend((
            "", "-- Counterexamples satisfy the chained parking/swap history.",
            f"LTLSPEC !({desired});", "",
        ))
    else:
        lines.extend(("", "-- The counterexample parks every car at its assigned grid goal.",
                      "INVARSPEC !(" + " & ".join(
                          f"loc_{car} = {goal}" for car, goal in enumerate(goals)
                      ) + ");", ""))
    return "\n".join(lines)


def check_model(model: Path, bound: int, *, temporal: bool = False,
                progress: bool = False) -> Path | None:
    connection = NuXmvConn(["nuXmv", "-quiet", "-int", model.name], cwd=model.parent)
    try:
        connection._send("go_msat")
        setup = connection._read_response()
        if "error" in setup.lower():
            raise RuntimeError(setup)
        command = (
            f"msat_check_ltlspec_sbmc_inc -k {bound} -n 0"
            if temporal else
            f"msat_check_invar_bmc -a falsification -k {bound} -n 0"
        )
        connection._send(command)
        checking_started = time.perf_counter()

        def report(line: str) -> None:
            if not re.match(
                r"-- (?:no counterexample found with bound|terminating with bound)",
                line,
                re.IGNORECASE,
            ):
                return
            elapsed = time.perf_counter() - checking_started
            timestamp = datetime.now().astimezone().isoformat(timespec="seconds")
            message = f"[BMC {timestamp} +{elapsed:.3f}s] {line}"
            print(message, flush=True)
            if sys.stdout is not sys.__stdout__ and sys.__stdout__ is not None:
                print(message, file=sys.__stdout__, flush=True)

        result = connection._read_response(report if progress else None)
        if "counter-example of size" not in result.lower() and "trace description:" not in result.lower():
            return None
        output = (model.parent / "cex_grid_parking.xml").resolve()
        connection._send(f"show_traces -p 6 -o {output} 1")
        connection._read_response()
        return output if output.exists() else None
    finally:
        connection.close()


def selected_graph(config: GridParkingConfig, state: dict[str, str]) -> nx.Graph:
    _, backbone, optional = topology(config)
    graph = nx.Graph()
    starts, goals = starts_and_goals(config)
    for grid_node in range(config.nodes):
        row, column = divmod(grid_node, config.columns)
        graph.add_node(grid_node, row=row, column=column,
                       start=grid_node in starts, goal=grid_node in goals)
    graph.add_edges_from(backbone, kind="backbone")
    graph.add_edges_from(
        (edge for edge in optional if state.get(edge_name(*edge)) == "TRUE"),
        kind="symbolic",
    )
    return graph


def save_gif(counterexample: Path, graph: nx.Graph, config: GridParkingConfig,
             output: Path) -> None:
    from matplotlib import pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    states = parse_states(counterexample)
    positions = {n: (data["column"], -data["row"]) for n, data in graph.nodes(data=True)}
    figure, axes = plt.subplots(figsize=(10, 7))
    colors = ["#b6e3a8" if data["goal"] else "#dcecff" for _, data in graph.nodes(data=True)]
    nx.draw_networkx_nodes(graph, positions, node_color=colors, node_size=520, ax=axes)
    nx.draw_networkx_labels(graph, positions, font_size=7, ax=axes)
    nx.draw_networkx_edges(
        graph, positions, ax=axes, width=[1.5 if data["kind"] == "backbone" else 3
                                         for *_, data in graph.edges(data=True)],
        edge_color=["#999" if data["kind"] == "backbone" else "#e07a5f"
                    for *_, data in graph.edges(data=True)],
    )
    palette = plt.get_cmap("tab10")
    dots = [axes.scatter([], [], s=150, color=palette(car % 10), zorder=5)
            for car in range(config.cars)]
    title = axes.set_title("")
    axes.axis("off")

    def update(frame):
        state = states[frame]
        for car, dot in enumerate(dots):
            dot.set_offsets([positions[int(state[f"loc_{car}"])]])
        fuels = ", ".join(str(state[f"fuel_{car}"]) for car in range(config.cars))
        title.set_text(f"Sparse grid parking | step {frame}/{len(states)-1} | fuel [{fuels}]")
        return [*dots, title]

    FuncAnimation(figure, update, frames=len(states), interval=350).save(
        output, writer=PillowWriter(fps=3)
    )
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rows", type=int, default=5)
    parser.add_argument("--columns", type=int, default=6)
    parser.add_argument("--cars", type=int, default=10)
    parser.add_argument("--bound", type=int, default=300)
    parser.add_argument("--fuel", type=int, default=400)
    parser.add_argument("--swap-rounds", type=int, default=0,
                        help="number of staged pair-swap rounds encoded with chained U")
    parser.add_argument("--grid-mode", choices=("symbolic", "corridor"), default="symbolic")
    parser.add_argument("--output-dir", type=Path, default=Path("SMV_GEN/scalable_grid_parking"))
    parser.add_argument("--gif", action="store_true")
    args = parser.parse_args()
    config = GridParkingConfig(
        args.rows, args.columns, args.cars, args.bound,
        tuple(args.fuel for _ in range(args.cars)), swap_rounds=args.swap_rounds,
        grid_mode=args.grid_mode,
    ).normalized()
    target = args.output_dir / f"scenario{time.time_ns()}_grid_{args.rows}x{args.columns}_{args.cars}cars"
    target.mkdir(parents=True, exist_ok=False)
    model = target / "grid_parking.smv"
    model.write_text(render_smv(config))
    (target / "grid_config.json").write_text(json.dumps(asdict(config), indent=2) + "\n")
    print(f"Model: {model}")
    counterexample = check_model(model, config.bound, temporal=bool(config.swap_rounds))
    if counterexample is None:
        print(f"No witness found through bound {config.bound}")
        return 2
    states = parse_states(counterexample)
    graph = selected_graph(config, states[0])
    nx.write_graphml(graph, target / "selected_grid.graphml")
    (target / "selected_grid.json").write_text(
        json.dumps(nx.node_link_data(graph, edges="edges"), indent=2) + "\n"
    )
    print(f"Counterexample: {counterexample} ({len(states)} states)")
    if args.gif:
        save_gif(counterexample, graph, config, target / "grid_parking.gif")
        print(f"Animation: {(target / 'grid_parking.gif').resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
