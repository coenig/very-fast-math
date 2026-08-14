"""Small fixed-graph parking-lot counterexample example.

Car A starts in front of car B, but A's assigned space is farther away.  The
requested trace requires A to enter P0 temporarily, B to pass and park in P1,
and A to rejoin the aisle and park in P2.
"""

from __future__ import annotations

import argparse
import re
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import networkx as nx

from fixed_graph_model import check_fixed_graph_model, generate_fixed_graph_model


ROAD_LENGTH = 100
SPACE_LENGTH = 30
CONNECTION_LENGTH = 20
PARKING_TOLERANCE = 5
CAR_NAMES = ("A", "B")


def vehicle(index: int, field: str) -> str:
    return f"env.veh___6{index}9___.{field}"


def build_parking_lot() -> nx.MultiDiGraph:
    """Return R0->R1->R2->R3 with side pockets P0, P1 and P2."""
    graph = nx.MultiDiGraph()
    for index in range(4):
        graph.add_node(f"R{index}", length=ROAD_LENGTH)
    for index in range(3):
        graph.add_node(f"P{index}", length=SPACE_LENGTH)
        graph.add_edge(f"R{index}", f"R{index + 1}", length=CONNECTION_LENGTH)
        graph.add_edge(f"R{index}", f"P{index}", length=CONNECTION_LENGTH)
        graph.add_edge(f"P{index}", f"R{index + 1}", length=CONNECTION_LENGTH)
    return graph


def print_parking_lot() -> None:
    print(
        """
                         [ P0 ]            [ P1 ]            [ P2 ]
                         /    \\            /    \\            /    \\
    start --> [ R0 ] ---------> [ R1 ] ---------> [ R2 ] ---------> [ R3 ]

    Initial order:  B ---- A --->
    Assignments:    A -> P2, B -> P1
    Required yield: A visits P0, B passes, A rejoins, then both park.
""".strip()
    )


def parking_property(section_ids: dict[str, int]) -> str:
    a_target = section_ids["P2"]
    b_target = section_ids["P1"]
    parked_a = (
        f"{vehicle(0, 'on_straight_section')} = {a_target} & "
        f"{vehicle(0, 'abs_pos')} >= env.section_{a_target}_end - {PARKING_TOLERANCE} & "
        f"{vehicle(0, 'v')} = 0"
    )
    parked_b = (
        f"{vehicle(1, 'on_straight_section')} = {b_target} & "
        f"{vehicle(1, 'abs_pos')} >= env.section_{b_target}_end - {PARKING_TOLERANCE} & "
        f"{vehicle(1, 'v')} = 0"
    )
    return f"INVARSPEC !(parking_phase = 3 & ({parked_a}) & ({parked_b}));"


def scenario_constraints(section_ids: dict[str, int]) -> str:
    r0 = section_ids["R0"]
    p0 = section_ids["P0"]
    r1 = section_ids["R1"]
    p1 = section_ids["P1"]
    p2 = section_ids["P2"]
    return f"""

-- Parking-lot initial state and maneuver monitor.
INIT {vehicle(0, f'is_on_sec_{r0}')} = 1;
INIT {vehicle(1, f'is_on_sec_{r0}')} = 1;
INIT {vehicle(0, 'abs_pos')} = 60;
INIT {vehicle(1, 'abs_pos')} = 20;
INIT {vehicle(0, 'v')} = 5;
INIT {vehicle(1, 'v')} = 5;

VAR parking_phase : 0 .. 3;
ASSIGN
  init(parking_phase) := 0;
  next(parking_phase) := case
    parking_phase = 0 & {vehicle(0, 'on_straight_section')} = {p0} : 1;
    parking_phase = 1 & (
      {vehicle(0, 'on_straight_section')} = {p0} &
      {vehicle(1, 'on_straight_section')} = {r1}
    ) : 2;
    parking_phase = 2 & (
      {vehicle(0, 'on_straight_section')} = {r1} |
      {vehicle(0, 'on_straight_section')} = {p2}
    ) : 3;
    TRUE : parking_phase;
  esac;
"""


def add_scenario_constraints(model: Path, section_ids: dict[str, int]) -> None:
    text = model.read_text()
    # main.tpl supplies a demonstration placement for all non-ego cars.  Replace
    # it rather than conjoining our positions with those incompatible defaults.
    for index in range(2):
        prefix = re.escape(f"env.veh___6{index}9___")
        text = re.sub(
            rf"(?m)^INIT {prefix}\.is_on_sec_\d+\s*=.*;$",
            lambda match: "-- Replaced by parking example: " + match.group(0),
            text,
        )
        text = re.sub(
            rf"(?m)^INIT {prefix}\.abs_pos\s*=.*;$",
            lambda match: "-- Replaced by parking example: " + match.group(0),
            text,
        )
    model.write_text(text)
    with model.open("a") as output:
        output.write(scenario_constraints(section_ids))


def _parse_counterexample(path: Path) -> list[dict[str, str]]:
    states = []
    current: dict[str, str] = {}
    for node in ET.parse(path).getroot().findall("node"):
        state = node.find("state")
        if state is None:
            continue
        current = current.copy()
        for value in state.findall("value"):
            variable = value.get("variable")
            if variable and value.text is not None:
                current[variable] = value.text
        states.append(current)
    return states


def _car_summary(
    state: dict[str, str], index: int, section_names: dict[int, str]
) -> str:
    section_text = state.get(vehicle(index, "on_straight_section"), "-1")
    try:
        section = section_names.get(int(section_text), f"connection/{section_text}")
    except ValueError:
        section = section_text
    if section_text == "-1":
        start = state.get(vehicle(index, "traversion_from"), "?")
        end = state.get(vehicle(index, "traversion_to"), "?")
        try:
            section = f"{section_names.get(int(start), start)}->{section_names.get(int(end), end)}"
        except ValueError:
            section = f"{start}->{end}"
    position = state.get(vehicle(index, "abs_pos"), "?")
    velocity = state.get(vehicle(index, "v"), "?")
    return f"{section:>6} pos={position:>4} v={velocity:>3}"


def print_trace(
    counterexample: Path,
    section_ids: dict[str, int],
    *,
    animate: bool = False,
    delay: float = 0.15,
) -> None:
    states = _parse_counterexample(counterexample)
    names = {index: name for name, index in section_ids.items()}
    previous = None
    for step, state in enumerate(states):
        row = (
            f"{step:>3} | phase={state.get('parking_phase', '?')} | "
            f"A: {_car_summary(state, 0, names)} | "
            f"B: {_car_summary(state, 1, names)}"
        )
        # The XML can contain intermediate nodes without a changed relevant value.
        signature = tuple(state.get(key) for key in (
            "parking_phase",
            vehicle(0, "on_straight_section"), vehicle(0, "abs_pos"), vehicle(0, "v"),
            vehicle(0, "traversion_from"), vehicle(0, "traversion_to"),
            vehicle(1, "on_straight_section"), vehicle(1, "abs_pos"), vehicle(1, "v"),
            vehicle(1, "traversion_from"), vehicle(1, "traversion_to"),
        ))
        if signature == previous:
            continue
        previous = signature
        if animate:
            print("\033[2J\033[H", end="")
            print_parking_lot()
            print("\nCounterexample playback\n")
        print(row, flush=True)
        if animate:
            time.sleep(delay)


def _section_point(name: str, position: float) -> tuple[float, float]:
    fraction = max(0.0, min(1.0, position / (SPACE_LENGTH if name[0] == "P" else ROAD_LENGTH)))
    index = int(name[1:])
    if name[0] == "R":
        return index * 140 + fraction * 100, 0.0
    return index * 140 + 110 + fraction * 20, 40.0


def _car_point(
    state: dict[str, str], index: int, section_names: dict[int, str]
) -> tuple[float, float]:
    position = float(state.get(vehicle(index, "abs_pos"), "0"))
    section = int(state.get(vehicle(index, "on_straight_section"), "-1"))
    if section >= 0:
        return _section_point(section_names[section], position)
    start = section_names[int(state[vehicle(index, "traversion_from")])]
    end = section_names[int(state[vehicle(index, "traversion_to")])]
    start_point = _section_point(start, SPACE_LENGTH if start[0] == "P" else ROAD_LENGTH)
    end_point = _section_point(end, 0)
    fraction = max(0.0, min(1.0, position / CONNECTION_LENGTH))
    return tuple(a + fraction * (b - a) for a, b in zip(start_point, end_point))


def save_trace_gif(
    counterexample: Path,
    section_ids: dict[str, int],
    output: Path,
    *,
    delay: float = 0.15,
) -> None:
    """Render the counterexample as a small dependency-light animated GIF."""
    from matplotlib import pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    states = _parse_counterexample(counterexample)
    names = {index: name for name, index in section_ids.items()}
    figure, axes = plt.subplots(figsize=(12, 3.5))
    axes.set_aspect("equal")
    axes.axis("off")
    axes.set_xlim(-15, 535)
    axes.set_ylim(-25, 75)

    for index in range(4):
        x = index * 140
        axes.plot([x, x + 100], [0, 0], color="#555", linewidth=9, solid_capstyle="butt")
        axes.text(x + 50, -12, f"R{index}", ha="center")
    for index in range(3):
        road_end = (index * 140 + 100, 0)
        next_start = ((index + 1) * 140, 0)
        pocket_start = (index * 140 + 110, 40)
        pocket_end = (index * 140 + 130, 40)
        axes.plot(
            [road_end[0], next_start[0]], [0, 0],
            color="#888", linewidth=3, linestyle="--",
        )
        axes.plot(
            [road_end[0], pocket_start[0], pocket_end[0], next_start[0]],
            [road_end[1], pocket_start[1], pocket_end[1], next_start[1]],
            color="#555", linewidth=5,
        )
        axes.text(index * 140 + 120, 52, f"P{index}", ha="center")

    car_a = axes.scatter([], [], s=180, color="#1976d2", label="A → P2", zorder=5)
    car_b = axes.scatter([], [], s=180, color="#e53935", label="B → P1", zorder=5)
    axes.legend(loc="lower right")
    title = axes.set_title("")

    def update(frame: int):
        state = states[frame]
        car_a.set_offsets([_car_point(state, 0, names)])
        car_b.set_offsets([_car_point(state, 1, names)])
        title.set_text(
            f"Parking counterexample — step {frame}/{len(states) - 1}, "
            f"phase {state.get('parking_phase', '?')}"
        )
        return car_a, car_b, title

    animation = FuncAnimation(
        figure, update, frames=len(states), interval=max(1, int(delay * 1000)), blit=False
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    animation.save(output, writer=PillowWriter(fps=max(1, round(1 / max(delay, 0.01)))))
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="SMV_GEN")
    parser.add_argument("--generate-only", action="store_true")
    parser.add_argument("--animate", action="store_true")
    parser.add_argument("--delay", type=float, default=0.15)
    parser.add_argument("--bound", type=int, default=100)
    parser.add_argument(
        "--gif",
        nargs="?",
        const="",
        metavar="FILE",
        help="save an animated GIF (default: alongside the counterexample)",
    )
    args = parser.parse_args()
    if args.delay < 0:
        parser.error("--delay must be non-negative")
    if args.bound < 0:
        parser.error("--bound must be non-negative")
    return args


def main() -> int:
    args = parse_args()
    graph = build_parking_lot()
    section_ids = {node: index for index, node in enumerate(graph.nodes)}
    print_parking_lot()
    print("\nGenerating the fixed parking-lot model...")
    morty = generate_fixed_graph_model(
        graph,
        nonegos=2,
        property_smv=parking_property(section_ids),
        experiments_dir=args.output_dir,
    )
    model = morty.get_target_path() / "main_fixed_graph.smv"
    add_scenario_constraints(model, section_ids)
    print(f"Model: {model}")
    if args.generate_only:
        return 0

    print("Checking for the yield/pass/park counterexample...")
    counterexample = check_fixed_graph_model(morty, bound=args.bound)
    if counterexample is None:
        print("No counterexample was found within the configured BMC bound.")
        return 1
    print(f"Counterexample: {counterexample}\n")
    print("step | monitor | car states")
    print_trace(
        counterexample,
        section_ids,
        animate=args.animate,
        delay=args.delay,
    )
    if args.gif is not None:
        gif = (
            Path(args.gif)
            if args.gif
            else counterexample.with_name("parking_counterexample.gif")
        )
        save_trace_gif(
            counterexample, section_ids, gif.resolve(), delay=args.delay
        )
        print(f"\nAnimation: {gif.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
