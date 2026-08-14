"""Run fixed-graph parking examples selected by named topology functions."""

from __future__ import annotations

import argparse
import re
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import networkx as nx

from fixed_graph_model import check_fixed_graph_model, generate_fixed_graph_model


CONNECTION_LENGTH = 20
PARKING_TOLERANCE = 5


@dataclass(frozen=True)
class ParkingExample:
    graph: nx.MultiDiGraph
    initial_positions: tuple[int, ...]
    assignments: tuple[str, ...]
    drawing: str
    section_lines: dict[str, tuple[tuple[float, float], tuple[float, float]]]


def easy_loop() -> ParkingExample:
    """Return the topology-forced two-car loop.

    Topology::

        R0 -> P0 -> R1 -> P1
         ^           |
         +--- L0 <---+

    A starts ahead and is assigned P0.  Because P0 is the only path forward,
    A must leave its own target, traverse R1 -> L0 -> R0, and return after B
    has passed toward P1.
    """
    graph = nx.MultiDiGraph()
    for name, length in (("R0", 50), ("P0", 25), ("R1", 50), ("P1", 25), ("L0", 30)):
        graph.add_node(name, length=length)
    for start, end in (("R0", "P0"), ("P0", "R1"), ("R1", "P1"), ("R1", "L0"), ("L0", "R0")):
        graph.add_edge(start, end, length=CONNECTION_LENGTH)
    drawing = """
                         A's target              B's target
                            [ P0 ] -----> [ R1 ] -----> [ P1 ]
                              ^              |
                              |              v
    start -----> [ R0 ] ------+            [ L0 ]
                    ^                          |
                    +--------------------------+
    """.strip()
    lines = {
        "R0": ((0, 0), (100, 0)), "P0": ((130, 0), (180, 0)),
        "R1": ((210, 0), (310, 0)), "P1": ((340, 0), (390, 0)),
        "L0": ((280, -60), (100, -60)),
    }
    return ParkingExample(graph, (30, 10), ("P0", "P1"), drawing, lines)


def vertical_spots() -> ParkingExample:
    """Return a three-car aisle with three vertical parking extensions.

    Topology::

             P0              P1      P2
             ^|              ^|      ^|
             |v              |v      |v
        R0 --------> R1 -----------------> R2

    P0 is attached between R0 and R1.  P1 and P2 are alternative vertical
    extensions between R1 and R2.  Cars A, B and C start front-to-back on R0
    and are assigned P2, P1 and P0 respectively.  This is intentionally a
    comparison topology: even with three cars it admits a greedy solution,
    because each car can bypass earlier branches and enter its own spot.
    """
    graph = nx.MultiDiGraph()
    for index in range(3):
        graph.add_node(f"R{index}", length=70)
    for index in range(3):
        graph.add_node(f"P{index}", length=25)
    for start, end in (
        ("R0", "R1"), ("R0", "P0"), ("P0", "R1"),
        ("R1", "R2"), ("R1", "P1"), ("P1", "R2"),
        ("R1", "P2"), ("P2", "R2"),
    ):
        graph.add_edge(start, end, length=CONNECTION_LENGTH)
    drawing = """
                         [ P0 ]            [ P1 ]  [ P2 ]
                         /    \\            /  \\    /  \\
    start --> [ R0 ] ---------> [ R1 ] -------------> [ R2 ]

    Initial order: C ---- B ---- A --->
    Assignments:   A -> P2, B -> P1, C -> P0
    """.strip()
    lines = {
        "R0": ((0, 0), (100, 0)), "R1": ((140, 0), (240, 0)),
        "R2": ((330, 0), (430, 0)), "P0": ((110, 45), (130, 45)),
        "P1": ((255, 45), (275, 45)), "P2": ((290, 45), (310, 45)),
    }
    return ParkingExample(graph, (50, 30, 10), ("P2", "P1", "P0"), drawing, lines)


def vertical_non_trivial() -> ParkingExample:
    """Return a forward-only three-car topology that forces revisiting goals.

    Topology::

                  P0 ---------> P1 ---------> P2 (C target)
                  ^              |             final
                  |              v
        start --> R0            L0
                  ^              |
                  +--------------+

    There are deliberately no bypasses around P0 and P1: both parking sections
    are single-lane choke points.  A starts in front
    and is assigned P1, B starts in the middle and is assigned P0, and rear car
    C is assigned P2.  A and B must leave their respective goals so C can pass,
    take the forward-only P1->L0->R0 return loop, and then come back to park.
    The reachability property contains only the final assignments.
    """
    graph = nx.MultiDiGraph()
    for name, length in (
        ("R0", 50), ("P0", 25), ("P1", 25), ("P2", 25), ("L0", 30),
    ):
        graph.add_node(name, length=length)
    for start, end in (
        ("R0", "P0"), ("P0", "P1"),
        ("P1", "P2"), ("P1", "L0"), ("L0", "R0"),
    ):
        graph.add_edge(start, end, length=CONNECTION_LENGTH)
    drawing = """
                         [ P0 ] -----> [ P1 ] -----> [ P2: C ]
                           ^              |
                           |              v
    start --> [ R0 ] ------+            [ L0 ]
                 ^                          |
                 +--------------------------+

    Initial order: C ---- B ---- A --->
    Assignments:   A -> P1, B -> P0, C -> P2
    """.strip()
    lines = {
        "R0": ((0, 0), (90, 0)), "P0": ((115, 45), (155, 45)),
        "P1": ((185, 45), (225, 45)), "P2": ((260, 45), (305, 45)),
        "L0": ((210, -55), (65, -55)),
    }
    return ParkingExample(graph, (40, 25, 10), ("P1", "P0", "P2"), drawing, lines)


TOPOLOGIES = {
    "easy_loop": easy_loop,
    "vertical_spots": vertical_spots,
    "vertical_non_trivial": vertical_non_trivial,
}


def vehicle(index: int, field: str) -> str:
    return f"env.veh___6{index}9___.{field}"


def final_property(example: ParkingExample, section_ids: dict[str, int]) -> str:
    clauses = []
    for car, target in enumerate(example.assignments):
        section = section_ids[target]
        clauses.extend((
            f"{vehicle(car, 'on_straight_section')} = {section}",
            f"{vehicle(car, 'abs_pos')} >= env.section_{section}_end - {PARKING_TOLERANCE}",
            f"{vehicle(car, 'v')} = 0",
        ))
    return "INVARSPEC !(" + " & ".join(f"({clause})" for clause in clauses) + ");"


def add_initial_state(model: Path, section_ids: dict[str, int], positions: tuple[int, ...]) -> None:
    text = model.read_text()
    for car in range(len(positions)):
        prefix = re.escape(f"env.veh___6{car}9___")
        for field in (r"is_on_sec_\d+", "abs_pos"):
            text = re.sub(
                rf"(?m)^INIT {prefix}\.{field}\s*=.*;$",
                lambda match: "-- Replaced by parking topology example: " + match.group(0),
                text,
            )
    additions = ["", "-- Fixed starting order; the driving strategy remains symbolic."]
    r0 = section_ids["R0"]
    for car, position in enumerate(positions):
        additions.extend((
            f"INIT {vehicle(car, f'is_on_sec_{r0}')} = 1;",
            f"INIT {vehicle(car, 'abs_pos')} = {position};",
            f"INIT {vehicle(car, 'v')} = 5;",
        ))
    model.write_text(text + "\n".join(additions) + "\n")


def parse_states(path: Path) -> list[dict[str, str]]:
    result, current = [], {}
    for node in ET.parse(path).getroot().findall("node"):
        state = node.find("state")
        if state is None:
            continue
        current = current.copy()
        for value in state.findall("value"):
            name = value.get("variable")
            if name and value.text is not None:
                current[name] = value.text
        result.append(current)
    return result


def location(state: dict[str, str], car: int, names: dict[int, str]) -> str:
    section = int(state.get(vehicle(car, "on_straight_section"), -1))
    if section >= 0:
        return names[section]
    start = int(state.get(vehicle(car, "traversion_from"), -1))
    end = int(state.get(vehicle(car, "traversion_to"), -1))
    return f"{names.get(start, '?')}->{names.get(end, '?')}"


def print_trace(path: Path, example: ParkingExample, ids: dict[str, int], animate: bool, delay: float) -> None:
    names = {index: name for name, index in ids.items()}
    for step, state in enumerate(parse_states(path)):
        if animate:
            print("\033[2J\033[H" + example.drawing + "\n")
        fields = []
        for car in range(len(example.assignments)):
            fields.append(
                f"{chr(65 + car)}: {location(state, car, names):>6} "
                f"pos={state.get(vehicle(car, 'abs_pos'), '?'):>4} "
                f"v={state.get(vehicle(car, 'v'), '?'):>3}"
            )
        print(f"{step:>3} | " + " | ".join(fields), flush=True)
        if animate:
            time.sleep(delay)


def point(state: dict[str, str], car: int, example: ParkingExample, names: dict[int, str]):
    section = int(state.get(vehicle(car, "on_straight_section"), -1))
    position = float(state.get(vehicle(car, "abs_pos"), 0))
    if section >= 0:
        name = names[section]
        start, end = example.section_lines[name]
        fraction = position / example.graph.nodes[name]["length"]
    else:
        start_name = names[int(state[vehicle(car, "traversion_from")])]
        end_name = names[int(state[vehicle(car, "traversion_to")])]
        start, end = example.section_lines[start_name][1], example.section_lines[end_name][0]
        fraction = position / CONNECTION_LENGTH
    fraction = max(0.0, min(1.0, fraction))
    return tuple(a + fraction * (b - a) for a, b in zip(start, end))


def save_gif(path: Path, example: ParkingExample, ids: dict[str, int], output: Path, delay: float) -> None:
    from matplotlib import pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    states = parse_states(path)
    names = {index: name for name, index in ids.items()}
    figure, axes = plt.subplots(figsize=(11, 4))
    axes.set_aspect("equal"); axes.axis("off")
    all_points = [point for line in example.section_lines.values() for point in line]
    axes.set_xlim(min(x for x, _ in all_points) - 25, max(x for x, _ in all_points) + 25)
    axes.set_ylim(min(y for _, y in all_points) - 30, max(y for _, y in all_points) + 35)
    for name, (start, end) in example.section_lines.items():
        axes.plot((start[0], end[0]), (start[1], end[1]), color="#555", linewidth=8)
        axes.text((start[0] + end[0]) / 2, (start[1] + end[1]) / 2 + 10, name, ha="center")
    for start_name, end_name in example.graph.edges():
        axes.annotate("", xy=example.section_lines[end_name][0], xytext=example.section_lines[start_name][1],
                      arrowprops={"arrowstyle": "->", "color": "#888", "linewidth": 2})
    colors = ("#1976d2", "#e53935", "#43a047")
    dots = [axes.scatter([], [], s=190, color=colors[i], label=f"{chr(65+i)} → {target}", zorder=5)
            for i, target in enumerate(example.assignments)]
    title = axes.set_title(""); axes.legend(loc="lower right")

    def update(frame):
        for car, dot in enumerate(dots):
            dot.set_offsets([point(states[frame], car, example, names)])
        title.set_text(f"{frame}/{len(states)-1}: model-checker parking strategy")
        return (*dots, title)

    animation = FuncAnimation(figure, update, frames=len(states), interval=max(1, int(delay * 1000)))
    output.parent.mkdir(parents=True, exist_ok=True)
    animation.save(output, writer=PillowWriter(fps=max(1, round(1 / max(delay, .01)))))
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topology", choices=TOPOLOGIES, default="vertical_spots")
    parser.add_argument("--output-dir", default="SMV_GEN")
    parser.add_argument("--bound", type=int, default=100)
    parser.add_argument("--generate-only", action="store_true")
    parser.add_argument("--animate", action="store_true")
    parser.add_argument("--delay", type=float, default=.15)
    parser.add_argument("--gif", nargs="?", const="", metavar="FILE")
    args = parser.parse_args()
    if args.bound < 0 or args.delay < 0:
        parser.error("--bound and --delay must be non-negative")

    example = TOPOLOGIES[args.topology]()
    ids = {node: index for index, node in enumerate(example.graph.nodes)}
    print(example.drawing)
    morty = generate_fixed_graph_model(
        example.graph, nonegos=len(example.assignments),
        property_smv=final_property(example, ids), experiments_dir=args.output_dir,
    )
    model = morty.get_target_path() / "main_fixed_graph.smv"
    add_initial_state(model, ids, example.initial_positions)
    print(f"\nModel: {model}")
    if args.generate_only:
        return 0
    cex = check_fixed_graph_model(morty, bound=args.bound)
    if cex is None:
        print("No parking strategy found within the bound.")
        return 1
    print(f"\nCounterexample: {cex}\n")
    print_trace(cex, example, ids, args.animate, args.delay)
    if args.gif is not None:
        gif = Path(args.gif) if args.gif else cex.with_name(f"{args.topology}.gif")
        save_gif(cex, example, ids, gif.resolve(), args.delay)
        print(f"\nAnimation: {gif.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
