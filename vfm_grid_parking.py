#!/usr/bin/env python3
"""Search a neighbour-only grid topology and a yielding trace with full VFM."""

from __future__ import annotations

import argparse
import json
import math
import re
import time
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from pathlib import Path

import networkx as nx

from fixed_graph_model import _replace_singleton_domain
from graph_constraints.cex import build_multidigraph_from_xml_cex
from model_checking import Morty
from scenario_generation import generate_scenario


def vehicle(index: int, field: str) -> str:
    return f"env.veh___6{index}9___.{field}"


@dataclass(frozen=True)
class VfmGridParkingConfig:
    rows: int = 2
    columns: int = 4
    section_length: int = 100
    connection_length: int = 20
    initial_a_position: int = 60
    initial_b_position: int = 20
    initial_velocity: int = 5
    interior_tolerance: int = 5
    parking_tolerance: int = 5
    bound: int = 100
    cars: int = 2

    @property
    def sections(self) -> int:
        return self.rows * self.columns

    def validate(self) -> None:
        if self.rows < 2 or self.columns < 2:
            raise ValueError("the grid must contain at least two rows and columns")
        if self.sections < 4:
            raise ValueError("the relational scenario needs at least four sections")
        if self.cars < 2 or self.cars > self.sections:
            raise ValueError("cars must be between two and the number of sections")
        positive = (self.section_length, self.connection_length, self.initial_velocity)
        if any(isinstance(value, bool) or not isinstance(value, int) or value <= 0
               for value in positive):
            raise ValueError("lengths and initial velocity must be positive integers")
        if not 0 <= self.initial_b_position < self.initial_a_position <= self.section_length:
            raise ValueError("initial positions must satisfy 0 <= B < A <= section length")
        if not 0 <= self.interior_tolerance * 2 < self.section_length:
            raise ValueError("interior tolerance leaves no interior waiting position")
        if not 0 <= self.parking_tolerance < self.section_length:
            raise ValueError("parking tolerance must be shorter than a section")
        if self.bound < 0:
            raise ValueError("bound must be non-negative")


@dataclass
class VfmGridParkingModel:
    morty: Morty
    config: VfmGridParkingConfig
    model_file: Path

    @property
    def target_path(self) -> Path:
        return self.morty.get_target_path()


def _or(clauses: list[str]) -> str:
    return "(" + " | ".join(f"({clause})" for clause in clauses) + ")"


def relational_ltl_property(config: VfmGridParkingConfig) -> str:
    """Describe yielding by observations, without assigning layout roles."""
    a_section = vehicle(0, "on_straight_section")
    b_section = vehicle(1, "on_straight_section")
    a_position = vehicle(0, "abs_pos")
    b_position = vehicle(1, "abs_pos")
    a_velocity = vehicle(0, "v")
    b_velocity = vehicle(1, "v")

    waiting = _or([
        f"{a_section} = {section} & {a_velocity} = 0 & "
        f"{a_position} >= {config.interior_tolerance} & "
        f"{a_position} <= env.section_{section}_end - {config.interior_tolerance}"
        for section in range(1, config.sections)
    ])
    histories = []
    for merge in range(1, config.sections):
        departed = f"{waiting} & {b_section} = {merge} & {a_section} != {merge}"
        rejoined = f"{a_section} = {merge} & {b_section} != {merge}"
        rejoined_later = f"((!({rejoined})) U ({rejoined}))"
        wait_until_departed = (
            f"(({waiting}) U (({departed}) & X ({rejoined_later})))"
        )
        histories.append(
            f"((!({waiting})) U (({waiting}) & X ({wait_until_departed})))"
        )
    history = _or(histories)
    return (
        "-- The counterexample discovers both the grid layout and yielding route.\n"
        f"LTLSPEC !({history});"
    )


def _replace_default_initial_state(text: str, config: VfmGridParkingConfig,
                                   *, include_defaults: bool = True) -> str:
    for car in range(config.cars):
        prefix = re.escape(f"env.veh___6{car}9___")
        text = re.sub(
            rf"(?m)^INIT {prefix}\.is_on_sec_\d+\s*=.*;$",
            lambda match: "-- Replaced by VFM grid parking: " + match.group(0), text,
        )
        text = re.sub(
            rf"(?m)^INIT {prefix}\.abs_pos\s*=.*;$",
            lambda match: "-- Replaced by VFM grid parking: " + match.group(0), text,
        )
    if not include_defaults:
        return text
    return text + f"""

-- Relational parking initial state; no maneuver is prescribed.
INIT {vehicle(0, 'is_on_sec_0')} = 1;
INIT {vehicle(1, 'is_on_sec_0')} = 1;
INIT {vehicle(0, 'abs_pos')} = {config.initial_a_position};
INIT {vehicle(1, 'abs_pos')} = {config.initial_b_position};
INIT {vehicle(0, 'v')} = {config.initial_velocity};
INIT {vehicle(1, 'v')} = {config.initial_velocity};

-- Scenario admissibility: passing must not be achieved by reversing.
INVAR {vehicle(0, 'v')} >= 0;
INVAR {vehicle(1, 'v')} >= 0;
"""


def grid_topology_constraints(config: VfmGridParkingConfig) -> str:
    """Constrain the four existing VFM connection slots to grid neighbours."""
    constraints = [
        "-- Neighbour-only grid grammar; slots are north/east/south/west.",
    ]
    for section in range(config.sections):
        row, column = divmod(section, config.columns)
        neighbours = (
            section - config.columns if row > 0 else None,
            section + 1 if column + 1 < config.columns else None,
            section + config.columns if row + 1 < config.rows else None,
            section - 1 if column > 0 else None,
        )
        for slot, neighbour in enumerate(neighbours):
            variable = f"outgoing_connection_{slot}_of_section_{section}"
            if neighbour is None:
                constraints.append(f"INIT {variable} = -1;")
            else:
                constraints.append(
                    f"INIT {variable} = -1 | {variable} = {neighbour};"
                )
    return "\n".join(constraints) + "\n"


def _edge_topology_constraints(geometry: dict) -> str:
    """Exact topology assignments usable with geometry-agnostic EnvModel."""
    lines = ["-- Exact edge-based grid topology (geometry is fixed below as metadata)."]
    max_slots = int(geometry.get("max_outgoing", 5))
    for section in geometry["sections"]:
        index = int(section["id"])
        targets = [int(target) for target in geometry["outgoing"].get(str(index), [])]
        targets.extend([-1] * (max_slots - len(targets)))
        if len(targets) > max_slots:
            raise ValueError(f"section {index} needs more than five outgoing slots")
        for slot, target in enumerate(targets):
            lines.append(f"INIT outgoing_connection_{slot}_of_section_{index} = {target};")
    return "\n".join(lines) + "\n"


def _geometry_metadata(geometry: dict) -> str:
    """Emit the same static source/angle/drain fields as geometry mode.

    The generic geometry template expands all section pairs and becomes
    prohibitively large for a concrete grid.  Once topology is fixed, these
    fields are constants, so retaining the fields as frozen/defined metadata
    gives the native renderer the complete geometry without reopening a
    symbolic geometry search.
    """
    lines = ["-- Fixed grid geometry metadata for native trace rendering.", "FROZENVAR"]
    for section in geometry["sections"]:
        index = int(section["id"])
        lines.extend((
            f"  section_{index}.source.x : {int(section['source_x'])} .. {int(section['source_x'])};",
            f"  section_{index}.source.y : {int(section['source_y'])} .. {int(section['source_y'])};",
            f"  section_{index}.angle_raw : {int(section['angle']) // 45} .. {int(section['angle']) // 45};",
        ))
    lines.append("DEFINE")
    for section in geometry["sections"]:
        index = int(section["id"])
        angle = int(section["angle"])
        sine = int(round(math.sin(math.radians(angle)) * 100))
        cosine = int(round(math.cos(math.radians(angle)) * 100))
        lines.extend((
            f"  section_{index}.angle := section_{index}.angle_raw * 45;",
            f"  sin_of_section_{index}_angle := {sine};",
            f"  cos_of_section_{index}_angle := {cosine};",
            f"  section_{index}.drain.x := section_{index}.source.x + (section_{index}_end * cos_of_section_{index}_angle) / 100;",
            f"  section_{index}.drain.y := section_{index}.source.y + (section_{index}_end * sin_of_section_{index}_angle) / 100;",
        ))
    return "\n".join(lines) + "\n"


def _fixed_macro_list(values: list[int]) -> str:
    """Encode a list in the VFM fixed-array macro syntax."""
    return "@(" + ")@@(".join(str(value) for value in values) + ")@"


def _fixed_macro_connectors(
    outgoing: dict[str, list[int]], section_count: int, max_outgoing: int = 5,
) -> str:
    """Encode section IDs and their outgoing connector slots for VFM."""
    entries: list[str] = []
    for section in range(section_count):
        targets = [int(target) for target in outgoing.get(str(section), [])]
        targets = (targets + [-1] * max_outgoing)[:max_outgoing]
        inner = _fixed_macro_list(targets)
        # A connector map entry is ``@(section)@@(target-list)@)``;
        # the final ``@)`` closes the entry in the outer list.
        entries.append(f"@({section})@@({inner})@)")
    return "@(" + "@@(".join(entries) + "@"


def write_vfm_render_config(geometry: dict, output: Path,
                            *, base: str | Path = "src/templates/envmodel_config.json") -> None:
    """Write a VFM CEX-renderer config matching a concrete geometry manifest.

    The CEX renderer does not infer topology from the trace alone; it reads
    the fixed section arrays from the environment JSON.  Keeping this config
    beside a concretized trace prevents it from accidentally using the
    repository's unrelated example topology.
    """
    config = json.loads(Path(base).read_text())
    values = config.setdefault("_config", {})
    sections = sorted(geometry["sections"], key=lambda item: int(item["id"]))
    section_count = len(sections)
    outgoing = geometry["outgoing"]
    values.update({
        "SECTIONS": section_count,
        "MAXOUTGOINGCONNECTIONS": int(geometry.get("max_outgoing", 1)),
        "NONEGOS": 2,
        "NUMLANES": 1,
        "ANGLEGRANULARITY": 45,
        "ALLOW_ZEROLENGTH_SECTIONS": False,
        "MODEL_INTERSECTION_GEOMETRY": False,
        "FIXED_SECTION_IDs": _fixed_macro_list([int(s["id"]) for s in sections]),
        "FIXED_SECTION_ANGLEs": _fixed_macro_list([int(s["angle"]) for s in sections]),
        "FIXED_SECTION_LENGTHs": _fixed_macro_list([int(s["length"]) for s in sections]),
        "FIXED_SECTION_SOURCE_Xs": _fixed_macro_list([int(s["source_x"]) for s in sections]),
        "FIXED_SECTION_SOURCE_Ys": _fixed_macro_list([int(s["source_y"]) for s in sections]),
        "FIXED_SECTION_CONNECTORS": _fixed_macro_connectors(
            outgoing, section_count, int(geometry.get("max_outgoing", 5))
        ),
        "_CEX_FILE_NAME": "debug_trace_array.txt",
    })
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(config, indent=2) + "\n")


def _node_observation_expression(node: int, geometry: dict, car: int) -> str:
    """Observe a car crossing a concrete junction for an abstract node.

    Grid nodes are connections rather than sections, so a node observation
    includes the end of an incoming edge and the beginning of an outgoing
    edge. Parking observations are handled separately so reaching a goal
    means actually reaching the end of the parking-in section.
    """
    clauses: list[str] = []
    sections = geometry["sections"]
    for section in sections:
        if section.get("kind") != "road":
            continue
        edge = tuple(int(value) for value in section["edge"])
        if edge[1] == node:
            clauses.append(
                f"({vehicle(car, 'on_straight_section')} = {int(section['id'])} & "
                f"{vehicle(car, 'abs_pos')} >= {int(section['length']) - 5})"
            )
        if edge[0] == node:
            clauses.append(
                f"({vehicle(car, 'on_straight_section')} = {int(section['id'])} & "
                f"{vehicle(car, 'abs_pos')} <= 5)"
            )
    if not clauses:
        return "FALSE"
    return "(" + " | ".join(clauses) + ")"


def _parked_observation_expression(node: int, geometry: dict, car: int) -> str:
    parking = geometry.get("parking_by_node", {}).get(str(node))
    if parking is None:
        return "FALSE"
    # New manifests use a directional parking pair. Accept the old integer
    # schema so existing artifacts remain readable.
    inbound = int(parking["inbound"] if isinstance(parking, dict) else parking)
    section = geometry["sections"][inbound]
    threshold = max(0, int(section["length"]) - 5)
    return (
        f"({vehicle(car, 'on_straight_section')} = {inbound} & "
        f"{vehicle(car, 'abs_pos')} >= {threshold} & "
        f"{vehicle(car, 'v')} = 0)"
    )


def concrete_milestone_expression(
    pair: tuple[int, ...], geometry: dict,
    parked: tuple[bool, ...] | None = None,
) -> str:
    parked = parked or tuple(False for _ in pair)
    if len(parked) != len(pair):
        raise ValueError("one parked/transit mode is required per car")
    return "(" + " & ".join(
        (_parked_observation_expression(int(node), geometry, car)
         if parked[car] else
         _node_observation_expression(int(node), geometry, car))
        for car, node in enumerate(pair)
    ) + ")"


def _specialize_grid_env(source: Path, config: VfmGridParkingConfig,
                         geometry: dict | None = None) -> Path:
    text = source.read_text()
    section_count = len(geometry["sections"]) if geometry is not None else config.sections
    # The generic template's slot-compaction rule is incompatible with fixed
    # north/east/south/west slots, where an absent north edge must not disable
    # an east edge.
    text, removed = re.subn(
        r"(?m)^\s*INIT outgoing_connection_\d+_of_section_\d+ = -1 -> "
        r"outgoing_connection_\d+_of_section_\d+ = -1;.*\n?",
        "", text,
    )
    max_slots = int(geometry.get("max_outgoing", 5)) if geometry is not None else 4
    expected = section_count * (max_slots - 1)
    # FIXED_TOPOLOGY makes the template omit these rules at expansion time;
    # older cached/generated templates still contain them and are stripped
    # here for compatibility.
    if removed not in {0, expected}:
        raise ValueError(f"expected to remove {expected} slot-compaction rules, removed {removed}")
    for section in range(section_count):
        length = config.section_length
        if geometry is not None:
            length = int(geometry["sections"][section]["length"])
        text = _replace_singleton_domain(text, f"section_{section}_end", length)
    for start in range(section_count):
        for end in range(section_count):
            if start != end:
                if geometry is not None:
                    value = int(geometry.get("connection_lengths", {}).get(
                        str(start), {}).get(str(end), config.connection_length
                    ))
                else:
                    value = config.connection_length
                variable = f"arclength_from_sec_{start}_to_sec_{end}_on_lane_0"
                if re.search(rf"(?m)^\s*{re.escape(variable)}\s*:", text):
                    text = _replace_singleton_domain(text, variable, value)
    output = source.with_name("EnvModel_GridParking.smv")
    if geometry is not None:
        # These generic assumptions are useful for arbitrary road networks,
        # but incompatible with a grid: equal cardinal angles are intentional
        # and the lane-separated roads share junction regions.
        text = re.sub(
            r"(?m)^\s*INIT section_\d+\.angle != section_\d+\.angle;\s*\n?", "", text
        )
        text = re.sub(
            r"(?ms)^\s*INIT \( max\( abs\( section_.*?^\s*>= 5; -- Use only farther away coordinate as lower bound\.\s*\n",
            "", text,
        )
        # The legacy highway model checks initial longitudinal separation by
        # comparing only local abs_pos values.  Its own comment states that
        # rel_pos is valid only when both cars are on the same section.  Grid
        # cars commonly occupy distinct sections at the same local coordinate
        # (for example, the ends of two parking sections), so the old INIT
        # constraint would incorrectly make those states inconsistent.  The
        # concrete main model adds a section-aware collision constraint.
        text, removed_collision_checks = re.subn(
            r"(?m)^\s*INIT abs\(veh___6\d+9___\.rel_pos\) > veh_length "
            r"\| !ego\.laterally_overlapping_with_veh_\d+;\s*\n?",
            "", text,
        )
        expected_collision_checks = max(0, config.cars - 1)
        if removed_collision_checks != expected_collision_checks:
            raise ValueError(
                "expected to remove "
                f"{expected_collision_checks} section-unaware initial collision "
                f"checks, removed {removed_collision_checks}"
            )
    metadata = _geometry_metadata(geometry) if geometry is not None else ""
    if metadata:
        # Keep declarations before the model's VAR/ASSIGN/INIT sections.
        marker = text.find("\nVAR")
        if marker < 0:
            raise ValueError("generated EnvModel has no VAR declaration")
        text = text[:marker] + "\n" + metadata + text[marker:]
    output.write_text(
        "-- Full VFM with neighbour-only symbolic grid topology.\n"
        + text + "\n"
        + (_edge_topology_constraints(geometry)
           if geometry is not None else grid_topology_constraints(config))
    )
    return output


def generate_vfm_grid_parking_model(
    config: VfmGridParkingConfig,
    *,
    input_config: str | Path = "main_template.json",
    experiments_dir: str | Path = "vfm-with-parking",
    concrete_geometry: dict | None = None,
) -> VfmGridParkingModel:
    config.validate()
    model_sections = len(concrete_geometry["sections"]) if concrete_geometry else config.sections
    model_lengths = (
        [int(section["length"]) for section in concrete_geometry["sections"]]
        if concrete_geometry else [config.section_length]
    )
    model_min_length = min(model_lengths, default=config.section_length)
    model_max_length = max(model_lengths, default=config.section_length)
    generation_config = {
        "#007": config.cars, "NONEGOS": config.cars,
        "#008": 1, "NUMLANES": 1,
        "#010": model_sections, "SECTIONS": model_sections,
        "GRID_COLUMNS": config.columns,
        "SECTIONSMINLENGTH": model_min_length,
        "SECTIONSMAXLENGTH": model_max_length if concrete_geometry else config.section_length,
        "MINDISTCONNECTIONS": 1 if concrete_geometry else config.connection_length,
        "MAXDISTCONNECTIONS": (
            int(concrete_geometry.get("max_connection_length", 0))
            if concrete_geometry else config.connection_length
        ),
        "MAXOUTGOINGCONNECTIONS": (
            int(concrete_geometry.get("max_outgoing", 5)) if concrete_geometry else 4
        ),
        "ALLOW_ZEROLENGTH_SECTIONS": False,
        # A fixed grid does not need the generic all-pairs geometry search.
        # _specialize_grid_env adds equivalent frozen geometry metadata after
        # topology is fixed, keeping concrete generation tractable.
        "MODEL_INTERSECTION_GEOMETRY": 0,
        # The grid topology is fixed by _edge_topology_constraints below.
        # This is a template-generation switch, not an SMV runtime variable:
        # omit symbolic topology consistency checks from the expanded model.
        "FIXED_TOPOLOGY": concrete_geometry is not None,
        "SPEC": relational_ltl_property(config),
    }
    morty = generate_scenario(
        generation_config, input_config=input_config,
        experiments_dir=experiments_dir, scenario_suffix="vfm_grid_parking",
    )
    target = morty.get_target_path()
    if target.is_absolute() and not target.exists():
        actual = Path.cwd() / str(target).lstrip("/")
        if actual.exists():
            target = actual
            morty.args[5] = str(actual)
    env_model = _specialize_grid_env(target / "EnvModel.smv", config, concrete_geometry)
    main_text = (target / "main.smv").read_text().replace(
        '#include "EnvModel.smv"', f'#include "{env_model.name}"'
    )
    if concrete_geometry is not None:
        # The legacy example add-on asserts pairwise-distinct angles.  A grid
        # intentionally reuses 0/90/180/270 degrees, so retain the model's
        # geometry equations but remove that unrelated example constraint.
        main_text = re.sub(
            r"(?m)^INIT env\.section_\d+\.angle != env\.section_\d+\.angle;\s*\n?",
            "", main_text,
        )
    main_text = _replace_default_initial_state(
        main_text, config, include_defaults=concrete_geometry is None
    )
    model_file = target / "main_vfm_grid_parking.smv"
    model_file.write_text(main_text)
    (target / "vfm_grid_parking_config.json").write_text(
        json.dumps(asdict(config), indent=2) + "\n"
    )
    return VfmGridParkingModel(morty, config, model_file)


def check_vfm_grid_parking_model(model: VfmGridParkingModel) -> Path | None:
    try:
        return model.morty.model_check(
            "cex_vfm_grid_parking.xml", 0, model_file=model.model_file.name,
            bound=model.config.bound, property_kind="ltl",
        )
    finally:
        model.morty.close_model_checker()


def parse_states(path: str | Path) -> list[dict[str, str]]:
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


def selected_topology(counterexample: str | Path, config: VfmGridParkingConfig) -> nx.MultiDiGraph:
    detailed = build_multidigraph_from_xml_cex(counterexample)
    graph = nx.MultiDiGraph()
    for section in range(config.sections):
        row, column = divmod(section, config.columns)
        graph.add_node(section, row=row, column=column, length=config.section_length)
    for _, _, data in detailed.edges(data=True):
        if data.get("kind") == "connection":
            graph.add_edge(
                int(data["start_section"]), int(data["end_section"]),
                length=data["length"], slot=int(data["conn_number"]),
            )
    return graph


def save_topology(graph: nx.MultiDiGraph, target: Path) -> None:
    nx.write_graphml(graph, target / "selected_grid.graphml")
    (target / "selected_grid.json").write_text(
        json.dumps(nx.node_link_data(graph, edges="edges"), indent=2) + "\n"
    )


def _layout(config: VfmGridParkingConfig) -> dict[int, tuple[float, float]]:
    return {
        section: (float(section % config.columns), float(-(section // config.columns)))
        for section in range(config.sections)
    }


def save_topology_png(graph: nx.MultiDiGraph, config: VfmGridParkingConfig, output: Path) -> None:
    from matplotlib import pyplot as plt

    figure, axes = plt.subplots(figsize=(2.2 * config.columns, 2.0 * config.rows))
    positions = _layout(config)
    nx.draw_networkx_nodes(graph, positions, ax=axes, node_color="#e8eef7", node_size=900)
    nx.draw_networkx_labels(graph, positions, ax=axes)
    nx.draw_networkx_edges(graph, positions, ax=axes, arrows=True, arrowsize=18,
                           connectionstyle="arc3,rad=0.08")
    axes.set_aspect("equal")
    axes.axis("off")
    figure.tight_layout()
    figure.savefig(output, dpi=150)
    plt.close(figure)


def _car_point(state: dict[str, str], car: int, positions: dict[int, tuple[float, float]],
               config: VfmGridParkingConfig) -> tuple[float, float]:
    section = int(state.get(vehicle(car, "on_straight_section"), "-1"))
    distance = float(state.get(vehicle(car, "abs_pos"), "0"))
    if section >= 0:
        # A grid node represents an entire road section, not a geometrically
        # horizontal lane. Keep the marker on that node; otherwise using
        # abs_pos as an x-offset makes cars on vertical parts leave the road.
        # Concrete VFM traces also contain generated parking sections, which
        # are not present in the abstract grid layout.  Keep rendering robust
        # if an older caller does not provide a concrete layout.
        return positions.get(section, positions.get(0, (0.0, 0.0)))
    start = int(state.get(vehicle(car, "traversion_from"), "0"))
    end = int(state.get(vehicle(car, "traversion_to"), "0"))
    fraction = max(0.0, min(1.0, distance / config.connection_length))
    return tuple(a + fraction * (b - a) for a, b in zip(positions[start], positions[end]))


def save_trace_gif(counterexample: Path, graph: nx.MultiDiGraph,
                   config: VfmGridParkingConfig, output: Path,
                   geometry: dict | None = None) -> None:
    from matplotlib import pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    states = parse_states(counterexample)
    if geometry is None:
        positions = _layout(config)
        display_graph = graph
    else:
        # A concrete grid has directed road and parking sections, while the
        # abstract graph contains only grid nodes.  Render the concrete
        # section graph whenever it is available.
        positions = {
            int(section["id"]): (
                float(section["source_x"]) / 100.0,
                float(section["source_y"]) / 100.0,
            )
            for section in geometry.get("sections", [])
        }
        display_graph = nx.MultiDiGraph()
        display_graph.add_nodes_from(positions)
        for source, targets in geometry.get("outgoing", {}).items():
            for target in targets:
                if int(target) >= 0:
                    display_graph.add_edge(int(source), int(target))
        if not positions:
            positions = _layout(config)
            display_graph = graph
    figure, axes = plt.subplots(figsize=(2.4 * config.columns, 2.2 * config.rows))
    nx.draw_networkx_nodes(display_graph, positions, ax=axes, node_color="#edf2f8", node_size=950)
    nx.draw_networkx_labels(display_graph, positions, ax=axes)
    nx.draw_networkx_edges(display_graph, positions, ax=axes, arrows=True, arrowsize=18,
                           connectionstyle="arc3,rad=0.08")
    palette = plt.get_cmap("tab10")
    dots = [
        axes.scatter([], [], s=180, color=palette(car % 10),
                     label=chr(ord("A") + car), zorder=5)
        for car in range(config.cars)
    ]
    title = axes.set_title("")
    axes.legend(loc="lower center", ncol=2)
    axes.set_aspect("equal")
    axes.axis("off")

    def update(frame: int):
        state = states[frame]
        for car, dot in enumerate(dots):
            dot.set_offsets([_car_point(state, car, positions, config)])
        speeds = " ".join(
            f"v{chr(ord('A') + car)}={state.get(vehicle(car, 'v'), '?')}"
            for car in range(config.cars)
        )
        title.set_text(
            f"Full-VFM grid parking — step {frame}/{len(states)-1} | "
            f"{speeds}"
        )
        return *dots, title

    animation = FuncAnimation(figure, update, frames=len(states), interval=180, blit=False)
    animation.save(output, writer=PillowWriter(fps=6))
    plt.close(figure)


def print_ascii(config: VfmGridParkingConfig) -> None:
    for row in range(config.rows):
        print("   ".join(f"[{row * config.columns + column:>2}]" for column in range(config.columns)))
        if row + 1 < config.rows:
            print("     ".join(" ? " for _ in range(config.columns)))
    print("Every orthogonal directed connection is optional; '?' is symbolic.")


def print_trace(counterexample: Path) -> None:
    print("step | A(section,pos,v) | B(section,pos,v)")
    for step, state in enumerate(parse_states(counterexample)):
        cars = []
        for car in range(2):
            section = state.get(vehicle(car, "on_straight_section"), "?")
            if section == "-1":
                section = (f"{state.get(vehicle(car, 'traversion_from'), '?')}->"
                           f"{state.get(vehicle(car, 'traversion_to'), '?')}")
            cars.append(
                f"{section},{state.get(vehicle(car, 'abs_pos'), '?')},"
                f"{state.get(vehicle(car, 'v'), '?')}"
            )
        print(f"{step:>4} | {cars[0]:>18} | {cars[1]:>18}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rows", type=int, default=2)
    parser.add_argument("--columns", type=int, default=4)
    parser.add_argument("--section-length", type=int, default=100)
    parser.add_argument("--connection-length", type=int, default=20)
    parser.add_argument("--bound", type=int, default=100)
    parser.add_argument("--output-dir", type=Path, default=Path("vfm-with-parking"))
    parser.add_argument("--generate-only", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = VfmGridParkingConfig(
        rows=args.rows, columns=args.columns, section_length=args.section_length,
        connection_length=args.connection_length, bound=args.bound,
    )
    print_ascii(config)
    print("\nGenerating the full-VFM neighbour-grid model...")
    model = generate_vfm_grid_parking_model(config, experiments_dir=args.output_dir)
    print(f"Model: {model.model_file}")
    if args.generate_only:
        return 0
    counterexample = check_vfm_grid_parking_model(model)
    if counterexample is None:
        print(f"No counterexample found through bound {config.bound}.")
        return 1
    graph = selected_topology(counterexample, config)
    save_topology(graph, model.target_path)
    png = model.target_path / "selected_grid.png"
    gif = model.target_path / "vfm_grid_parking.gif"
    save_topology_png(graph, config, png)
    save_trace_gif(counterexample, graph, config, gif)
    print_trace(counterexample)
    print(f"Counterexample: {counterexample}")
    print(f"Selected topology: {model.target_path / 'selected_grid.json'}")
    print(f"Topology image: {png}")
    print(f"Animation: {gif}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
