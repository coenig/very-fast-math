"""Search jointly for an optional grid topology and a two-car yielding trace."""

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
class YieldingGridConfig:
    rows: int = 3
    columns: int = 4
    bound: int = 40
    fuel_budget: int = 100
    edge_cost: int = 10
    cars: int = 2

    @property
    def nodes(self) -> int:
        return self.rows * self.columns

    def validate(self) -> None:
        if self.rows < 2 or self.columns < 2 or self.nodes < 4:
            raise ValueError("the grid must contain at least four cells")
        if self.cars < 2 or self.cars > self.nodes:
            raise ValueError("cars must be between two and the number of cells")
        if self.bound < 0 or self.fuel_budget <= 0 or self.edge_cost <= 0:
            raise ValueError("bound must be non-negative and fuel values positive")


def _or(expressions: list[str]) -> str:
    return "(" + " | ".join(f"({item})" for item in expressions) + ")"


def yielding_history(config: YieldingGridConfig) -> str:
    """Relational action history, symmetric over yield and merge locations."""
    histories: list[str] = []
    for yielding in range(2, config.nodes):
        for merge in range(2, config.nodes):
            if yielding == merge:
                continue
            waiting = f"loc_0 = {yielding}"
            passed = f"loc_0 = {yielding} & loc_1 = {merge}"
            rejoined = f"loc_0 = {merge} & loc_1 != {merge}"
            rejoined_later = f"((!({rejoined})) U ({rejoined}))"
            wait_until_passed = (
                f"(({waiting}) U (({passed}) & X ({rejoined_later})))"
            )
            histories.append(
                f"((!({waiting})) U (({waiting}) & X ({wait_until_passed})))"
            )
    return _or(histories)


def render_smv(
    config: YieldingGridConfig,
    fixed_edges: set[tuple[int, int]] | None = None,
) -> str:
    """Render the yielding model with symbolic or externally fixed edges.

    ``fixed_edges=None`` preserves the original joint topology-and-trace
    search. Passing a set makes every orthogonal edge a Boolean constant, so
    nuXmv searches only for the vehicle trace on that particular topology.
    """
    config.validate()
    edges = grid_edges(config)  # type: ignore[arg-type]
    canonical_fixed = None
    if fixed_edges is not None:
        canonical_fixed = {tuple(sorted(edge)) for edge in fixed_edges}
        unknown = canonical_fixed - {tuple(sorted(edge)) for edge in edges}
        if unknown:
            raise ValueError(f"fixed topology contains non-grid edges: {sorted(unknown)}")
    lines = ["MODULE main", ""]
    if canonical_fixed is None:
        lines.extend((
            "-- Every orthogonal edge is optional and trace-constant.",
            "FROZENVAR",
        ))
        lines.extend(f"  {edge_name(*edge)} : boolean;" for edge in edges)
    else:
        lines.extend((
            "-- Every orthogonal edge is fixed by the external enumerator.",
            "DEFINE",
        ))
        lines.extend(
            f"  {edge_name(*edge)} := "
            f"{'TRUE' if tuple(sorted(edge)) in canonical_fixed else 'FALSE'};"
            for edge in edges
        )
    lines.extend(("", "VAR"))
    for car in range(config.cars):
        lines.extend((
            f"  loc_{car} : 0..{config.nodes - 1};",
            f"  fuel_{car} : 0..{config.fuel_budget};",
        ))
    lines.extend((
        "", "-- Car 0 starts ahead of car 1; neither route nor topology is fixed.",
        "INIT loc_0 = 1;", "INIT loc_1 = 0;",
    ))
    for car in range(config.cars):
        if car >= 2:
            lines.append(f"INIT loc_{car} = {car};")
        lines.append(f"INIT fuel_{car} = 0;")
        moves = [f"next(loc_{car}) = loc_{car}"]
        for start, end in edges:
            enabled = edge_name(start, end)
            moves.extend((
                f"({enabled} & loc_{car} = {start} & next(loc_{car}) = {end})",
                f"({enabled} & loc_{car} = {end} & next(loc_{car}) = {start})",
            ))
        lines.extend((
            "", "TRANS " + " |\n  ".join(moves) + ";",
            f"TRANS next(fuel_{car}) = fuel_{car} + case "
            f"next(loc_{car}) = loc_{car} : 0; TRUE : {config.edge_cost}; esac;",
        ))
    lines.extend(("", "-- Unit-capacity cells and no instantaneous position exchange."))
    for first in range(config.cars):
        for second in range(first + 1, config.cars):
            lines.append(f"INVAR loc_{first} != loc_{second};")
            lines.append(
                f"TRANS !(next(loc_{first}) = loc_{second} & next(loc_{second}) = loc_{first});"
            )
    lines.extend((
        "", "-- A counterexample discovers yield cell, merge cell, topology, and route.",
        f"LTLSPEC !({yielding_history(config)});", "",
    ))
    return "\n".join(lines)


def selected_graph(config: YieldingGridConfig, state: dict[str, str]) -> nx.Graph:
    graph = nx.Graph()
    for cell in range(config.nodes):
        row, column = divmod(cell, config.columns)
        graph.add_node(cell, row=row, column=column)
    for edge in grid_edges(config):  # type: ignore[arg-type]
        if state.get(edge_name(*edge), "FALSE").upper() == "TRUE":
            graph.add_edge(*edge)
    return graph


def save_gif(counterexample: Path, graph: nx.Graph, config: YieldingGridConfig,
             output: Path) -> None:
    from matplotlib import pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    states = parse_states(counterexample)
    positions = {cell: (data["column"], -data["row"])
                 for cell, data in graph.nodes(data=True)}
    figure, axes = plt.subplots(figsize=(8, 5))
    nx.draw_networkx_nodes(graph, positions, node_color="#dcecff", node_size=650, ax=axes)
    nx.draw_networkx_labels(graph, positions, ax=axes)
    nx.draw_networkx_edges(graph, positions, width=3, edge_color="#788da8", ax=axes)
    palette = plt.get_cmap("tab10")
    dots = [axes.scatter([], [], s=190, color=palette(car), zorder=5)
            for car in range(config.cars)]
    title = axes.set_title("")
    axes.axis("off")

    def update(frame: int):
        state = states[frame]
        for car, dot in enumerate(dots):
            dot.set_offsets([positions[int(state[f"loc_{car}"])]] )
        fuels = ", ".join(state[f"fuel_{car}"] for car in range(config.cars))
        title.set_text(f"Symbolic grid yielding | step {frame}/{len(states)-1} | fuel [{fuels}]")
        return [*dots, title]

    FuncAnimation(figure, update, frames=len(states), interval=500).save(
        output, writer=PillowWriter(fps=2)
    )
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rows", type=int, default=3)
    parser.add_argument("--columns", type=int, default=4)
    parser.add_argument("--bound", type=int, default=40)
    parser.add_argument("--fuel", type=int, default=100)
    parser.add_argument("--edge-cost", type=int, default=10)
    parser.add_argument("--cars", type=int, default=2)
    parser.add_argument("--output-dir", type=Path,
                        default=Path("vfm-with-parking/abstraction"))
    args = parser.parse_args()
    config = YieldingGridConfig(
        rows=args.rows, columns=args.columns, bound=args.bound,
        fuel_budget=args.fuel, edge_cost=args.edge_cost, cars=args.cars,
    )
    config.validate()
    target = args.output_dir / f"scenario{time.time_ns()}_abstract_grid_yielding"
    target.mkdir(parents=True, exist_ok=False)
    model = target / "abstract_grid_yielding.smv"
    model.write_text(render_smv(config))
    (target / "config.json").write_text(json.dumps(asdict(config), indent=2) + "\n")
    print(f"Model: {model.resolve()}")
    counterexample = check_model(model, config.bound, temporal=True)
    if counterexample is None:
        print(f"No counterexample through bound {config.bound}")
        return 2
    states = parse_states(counterexample)
    graph = selected_graph(config, states[0])
    nx.write_graphml(graph, target / "selected_grid.graphml")
    (target / "selected_grid.json").write_text(
        json.dumps(nx.node_link_data(graph, edges="edges"), indent=2) + "\n"
    )
    gif = target / "abstract_grid_yielding.gif"
    save_gif(counterexample, graph, config, gif)
    print(f"Counterexample: {counterexample} ({len(states)} states)")
    print(f"Selected topology: {(target / 'selected_grid.json').resolve()}")
    print(f"Animation: {gif.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
