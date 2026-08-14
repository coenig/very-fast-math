"""Parking example where the topology itself forces a non-greedy maneuver.

P0 is a mandatory single-lane choke point.  Car A starts ahead and is assigned
to P0, while car B must reach P1 beyond it.  For both cars to park, A has to
leave its own destination, take the return loop, and come back after B passes.
There is deliberately no maneuver monitor in the SMV property.
"""

from __future__ import annotations

import argparse
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import networkx as nx

from fixed_graph_model import check_fixed_graph_model, generate_fixed_graph_model


CAR_NAMES = ("A", "B")
PARKING_TOLERANCE = 5
CONNECTION_LENGTH = 20


def vehicle(index: int, field: str) -> str:
    return f"env.veh___6{index}9___.{field}"


def build_forced_loop_lot() -> nx.MultiDiGraph:
    graph = nx.MultiDiGraph()
    graph.add_node("R0", length=50)
    graph.add_node("P0", length=25)
    graph.add_node("R1", length=50)
    graph.add_node("P1", length=25)
    graph.add_node("L0", length=30)

    # The only forward route passes through A's assigned P0.
    graph.add_edge("R0", "P0", length=CONNECTION_LENGTH)
    graph.add_edge("P0", "R1", length=CONNECTION_LENGTH)
    graph.add_edge("R1", "P1", length=CONNECTION_LENGTH)
    # A can return to P0 only by completing this directed loop.
    graph.add_edge("R1", "L0", length=CONNECTION_LENGTH)
    graph.add_edge("L0", "R0", length=CONNECTION_LENGTH)
    return graph


def print_lot() -> None:
    print(
        """
                         A's target              B's target
                            [ P0 ] -----> [ R1 ] -----> [ P1 ]
                              ^              |
                              |              v
    start -----> [ R0 ] ------+            [ L0 ]
                    ^                          |
                    +--------------------------+

    Initial order on R0:  B ---- A --->
    Final assignment:     A -> P0, B -> P1
    """.strip()
    )


def final_parking_property(section_ids: dict[str, int]) -> str:
    goals = []
    for car, target in ((0, "P0"), (1, "P1")):
        section = section_ids[target]
        goals.extend(
            (
                f"{vehicle(car, 'on_straight_section')} = {section}",
                f"{vehicle(car, 'abs_pos')} >= env.section_{section}_end - {PARKING_TOLERANCE}",
                f"{vehicle(car, 'v')} = 0",
            )
        )
    return "INVARSPEC !(" + " & ".join(f"({goal})" for goal in goals) + ");"


def add_initial_state(model: Path, section_ids: dict[str, int]) -> None:
    import re

    text = model.read_text()
    for index in range(2):
        prefix = re.escape(f"env.veh___6{index}9___")
        for field_pattern in (r"is_on_sec_\d+", "abs_pos"):
            text = re.sub(
                rf"(?m)^INIT {prefix}\.{field_pattern}\s*=.*;$",
                lambda match: "-- Replaced by forced-loop example: " + match.group(0),
                text,
            )
    r0 = section_ids["R0"]
    text += f"""

-- Fixed starting order only; route, acceleration and braking remain symbolic.
INIT {vehicle(0, f'is_on_sec_{r0}')} = 1;
INIT {vehicle(1, f'is_on_sec_{r0}')} = 1;
INIT {vehicle(0, 'abs_pos')} = 30;
INIT {vehicle(1, 'abs_pos')} = 10;
INIT {vehicle(0, 'v')} = 5;
INIT {vehicle(1, 'v')} = 5;
"""
    model.write_text(text)


def parse_states(path: Path) -> list[dict[str, str]]:
    states = []
    current: dict[str, str] = {}
    for node in ET.parse(path).getroot().findall("node"):
        state = node.find("state")
        if state is None:
            continue
        current = current.copy()
        for value in state.findall("value"):
            if value.get("variable") and value.text is not None:
                current[value.get("variable")] = value.text
        states.append(current)
    return states


def car_location(state: dict[str, str], car: int, names: dict[int, str]) -> str:
    section = int(state.get(vehicle(car, "on_straight_section"), "-1"))
    if section >= 0:
        return names[section]
    start = int(state.get(vehicle(car, "traversion_from"), "-1"))
    end = int(state.get(vehicle(car, "traversion_to"), "-1"))
    return f"{names.get(start, '?')}->{names.get(end, '?')}"


def print_trace(path: Path, section_ids: dict[str, int], *, animate: bool, delay: float) -> None:
    names = {index: name for name, index in section_ids.items()}
    for step, state in enumerate(parse_states(path)):
        if animate:
            print("\033[2J\033[H", end="")
            print_lot()
            print()
        cars = []
        for car, label in enumerate(CAR_NAMES):
            cars.append(
                f"{label}: {car_location(state, car, names):>6} "
                f"pos={state.get(vehicle(car, 'abs_pos'), '?'):>4} "
                f"v={state.get(vehicle(car, 'v'), '?'):>3}"
            )
        print(f"{step:>3} | " + " | ".join(cars), flush=True)
        if animate:
            time.sleep(delay)


SECTION_LINES = {
    "R0": ((0.0, 0.0), (100.0, 0.0)),
    "P0": ((130.0, 0.0), (180.0, 0.0)),
    "R1": ((210.0, 0.0), (310.0, 0.0)),
    "P1": ((340.0, 0.0), (390.0, 0.0)),
    "L0": ((280.0, -60.0), (100.0, -60.0)),
}
SECTION_LENGTHS = {"R0": 50, "P0": 25, "R1": 50, "P1": 25, "L0": 30}


def car_point(state: dict[str, str], car: int, names: dict[int, str]) -> tuple[float, float]:
    section = int(state.get(vehicle(car, "on_straight_section"), "-1"))
    position = float(state.get(vehicle(car, "abs_pos"), "0"))
    if section >= 0:
        name = names[section]
        start, end = SECTION_LINES[name]
        fraction = max(0.0, min(1.0, position / SECTION_LENGTHS[name]))
    else:
        start_name = names[int(state[vehicle(car, "traversion_from")])]
        end_name = names[int(state[vehicle(car, "traversion_to")])]
        start = SECTION_LINES[start_name][1]
        end = SECTION_LINES[end_name][0]
        fraction = max(0.0, min(1.0, position / CONNECTION_LENGTH))
    return tuple(a + fraction * (b - a) for a, b in zip(start, end))


def save_gif(path: Path, section_ids: dict[str, int], output: Path, delay: float) -> None:
    from matplotlib import pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    states = parse_states(path)
    names = {index: name for name, index in section_ids.items()}
    graph = build_forced_loop_lot()
    figure, axes = plt.subplots(figsize=(11, 4))
    axes.set_aspect("equal")
    axes.axis("off")
    axes.set_xlim(-20, 410)
    axes.set_ylim(-90, 35)

    for name, (start, end) in SECTION_LINES.items():
        axes.plot([start[0], end[0]], [start[1], end[1]], color="#555", linewidth=8)
        axes.text((start[0] + end[0]) / 2, start[1] + 10, name, ha="center")
    for start_name, end_name in graph.edges():
        start = SECTION_LINES[start_name][1]
        end = SECTION_LINES[end_name][0]
        axes.annotate(
            "", xy=end, xytext=start,
            arrowprops={"arrowstyle": "->", "color": "#888", "linewidth": 2},
        )

    a_dot = axes.scatter([], [], s=190, color="#1976d2", label="A → P0", zorder=5)
    b_dot = axes.scatter([], [], s=190, color="#e53935", label="B → P1", zorder=5)
    title = axes.set_title("")
    axes.legend(loc="lower right")

    def update(frame: int):
        a_dot.set_offsets([car_point(states[frame], 0, names)])
        b_dot.set_offsets([car_point(states[frame], 1, names)])
        title.set_text(f"Topology-forced parking strategy — step {frame}/{len(states)-1}")
        return a_dot, b_dot, title

    animation = FuncAnimation(figure, update, frames=len(states), interval=max(1, int(delay * 1000)))
    output.parent.mkdir(parents=True, exist_ok=True)
    animation.save(output, writer=PillowWriter(fps=max(1, round(1 / max(delay, 0.01)))))
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="SMV_GEN")
    parser.add_argument("--bound", type=int, default=100)
    parser.add_argument("--generate-only", action="store_true")
    parser.add_argument("--animate", action="store_true")
    parser.add_argument("--delay", type=float, default=0.15)
    parser.add_argument(
        "--gif", nargs="?", const="", metavar="FILE",
        help="save a GIF (default: alongside the counterexample)",
    )
    args = parser.parse_args()
    if args.bound < 0 or args.delay < 0:
        parser.error("--bound and --delay must be non-negative")
    return args


def main() -> int:
    args = parse_args()
    graph = build_forced_loop_lot()
    section_ids = {node: index for index, node in enumerate(graph.nodes)}
    print_lot()
    morty = generate_fixed_graph_model(
        graph,
        nonegos=2,
        property_smv=final_parking_property(section_ids),
        experiments_dir=args.output_dir,
    )
    model = morty.get_target_path() / "main_fixed_graph.smv"
    add_initial_state(model, section_ids)
    print(f"\nModel: {model}")
    if args.generate_only:
        return 0
    counterexample = check_fixed_graph_model(morty, bound=args.bound)
    if counterexample is None:
        print("No parking strategy found within the bound.")
        return 1
    print(f"\nCounterexample: {counterexample}\n")
    print_trace(counterexample, section_ids, animate=args.animate, delay=args.delay)
    if args.gif is not None:
        gif = Path(args.gif) if args.gif else counterexample.with_name("parking_loop.gif")
        save_gif(counterexample, section_ids, gif.resolve(), args.delay)
        print(f"\nAnimation: {gif.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
