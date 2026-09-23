"""Compile an abstract grid into fixed, directed VFM road geometry.

Abstract nodes are junctions. Every undirected abstract edge becomes two
parallel directed road sections, and every node owns a two-section parking
bay: one section enters the spot and one returns to the junction.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True)
class GeometryConfig:
    # A 40-unit road between two 20-unit junction radii gives an 80-unit pitch.
    grid_pitch: int = 80
    road_length: int = 40
    junction_radius: int = 20
    lane_offset: int = 10
    parking_length: int = 20
    parking_angle: int = 45
    parking_lane_offset: int = 5
    # Retained in manifests for compatibility. Junctions are connector curves.
    junction_length: int = 0

    def validate(self) -> None:
        integers = (
            self.grid_pitch, self.road_length, self.junction_radius,
            self.lane_offset, self.parking_length, self.parking_angle,
            self.parking_lane_offset,
        )
        if any(isinstance(value, bool) or not isinstance(value, int) for value in integers):
            raise ValueError("geometry values must be integers")
        if self.road_length <= 0 or self.parking_length <= 0:
            raise ValueError("road and parking lengths must be positive")
        if self.junction_radius <= self.lane_offset:
            raise ValueError("junction_radius must be larger than lane_offset")
        if self.lane_offset <= 0 or not 0 < self.parking_lane_offset < self.junction_radius:
            raise ValueError("lane offsets must be positive and fit inside the junction")
        if self.grid_pitch != self.road_length + 2 * self.junction_radius:
            raise ValueError("grid_pitch must equal road_length + 2 * junction_radius")
        if self.parking_angle % 45 or self.parking_angle % 90 == 0:
            raise ValueError("parking_angle must be a non-cardinal multiple of 45 degrees")


@dataclass(frozen=True)
class ConcreteSection:
    id: int
    kind: str
    source_x: int
    source_y: int
    angle: int
    length: int
    grid_node: int | None = None
    edge: tuple[int, int] | None = None


def _scaled_direction(angle: int) -> tuple[int, int]:
    radians = math.radians(angle)
    return (
        int(round(math.cos(radians) * 100)),
        int(round(math.sin(radians) * 100)),
    )


def _smv_scaled_product(length: int, scaled_direction: int) -> int:
    """Match integer division in the generated NuXmv drain expression."""
    return math.trunc(length * scaled_direction / 100)


def section_endpoint(section: dict) -> tuple[int, int]:
    """Return the exact endpoint emitted by the SMV geometry metadata."""
    cosine, sine = _scaled_direction(int(section["angle"]))
    return (
        int(section["source_x"]) + _smv_scaled_product(int(section["length"]), cosine),
        int(section["source_y"]) + _smv_scaled_product(int(section["length"]), sine),
    )


def connection_curve(
    source: dict, target: dict,
) -> tuple[str, tuple[tuple[float, float], ...]]:
    """Return ``line`` or ``curve`` and its endpoint/control points."""
    p0 = tuple(float(value) for value in section_endpoint(source))
    p3 = (float(target["source_x"]), float(target["source_y"]))
    turn = (int(target["angle"]) - int(source["angle"])) % 360
    if turn == 0:
        return "line", (p0, p3)
    gap = math.dist(p0, p3)
    tangent = gap / 3.0
    incoming = math.radians(float(source["angle"]))
    outgoing = math.radians(float(target["angle"]))
    p1 = (p0[0] + tangent * math.cos(incoming),
          p0[1] + tangent * math.sin(incoming))
    p2 = (p3[0] - tangent * math.cos(outgoing),
          p3[1] - tangent * math.sin(outgoing))
    return "curve", (p0, p1, p2, p3)


def _bezier_length(points: tuple[tuple[float, float], ...], samples: int = 64) -> float:
    p0, p1, p2, p3 = points
    previous = p0
    total = 0.0
    for index in range(1, samples + 1):
        t = index / samples
        u = 1.0 - t
        point = (
            u**3 * p0[0] + 3 * u**2 * t * p1[0] + 3 * u * t**2 * p2[0] + t**3 * p3[0],
            u**3 * p0[1] + 3 * u**2 * t * p1[1] + 3 * u * t**2 * p2[1] + t**3 * p3[1],
        )
        total += math.dist(previous, point)
        previous = point
    return total


def connection_path_length(source: dict, target: dict) -> int:
    """Compute the integer arclength used by the concrete SMV model."""
    kind, points = connection_curve(source, target)
    length = math.dist(*points) if kind == "line" else _bezier_length(points)
    return max(1, int(round(length)))


def _node_center(node: dict, config: GeometryConfig) -> tuple[int, int]:
    return (
        int(node["column"]) * config.grid_pitch,
        -int(node["row"]) * config.grid_pitch,
    )


def _road_sections(nodes: dict[int, dict], edges: set[tuple[int, int]],
                   config: GeometryConfig) -> list[ConcreteSection]:
    sections: list[ConcreteSection] = []
    next_id = 0
    for source, target in sorted(edges):
        source_node, target_node = nodes[source], nodes[target]
        canonical_dx = int(target_node["column"]) - int(source_node["column"])
        canonical_dy = int(source_node["row"]) - int(target_node["row"])
        if math.hypot(canonical_dx, canonical_dy) != 1:
            raise ValueError(f"edge {source}-{target} is not between neighbouring grid nodes")
        normal_x, normal_y = -canonical_dy, canonical_dx
        for start, end, side in ((source, target, 1), (target, source, -1)):
            start_node, end_node = nodes[start], nodes[end]
            dx = int(end_node["column"]) - int(start_node["column"])
            dy = int(start_node["row"]) - int(end_node["row"])
            angle = int(round(math.degrees(math.atan2(dy, dx)) / 45.0) * 45) % 360
            center_x, center_y = _node_center(start_node, config)
            sections.append(ConcreteSection(
                next_id, "road",
                center_x + dx * config.junction_radius
                + side * normal_x * config.lane_offset,
                center_y + dy * config.junction_radius
                + side * normal_y * config.lane_offset,
                angle, config.road_length, edge=(start, end),
            ))
            next_id += 1
    return sections


def _parking_sections(nodes: dict[int, dict], first_id: int,
                      config: GeometryConfig) -> list[ConcreteSection]:
    sections: list[ConcreteSection] = []
    cosine, sine = _scaled_direction(config.parking_angle)
    radial_x = _smv_scaled_product(config.junction_radius, cosine)
    radial_y = _smv_scaled_product(config.junction_radius, sine)
    radians = math.radians(config.parking_angle)
    lateral_x = int(round(-math.sin(radians) * config.parking_lane_offset))
    lateral_y = int(round(math.cos(radians) * config.parking_lane_offset))
    travel_x = _smv_scaled_product(config.parking_length, cosine)
    travel_y = _smv_scaled_product(config.parking_length, sine)
    next_id = first_id
    for node, data in sorted(nodes.items()):
        center_x, center_y = _node_center(data, config)
        port_x, port_y = center_x + radial_x, center_y + radial_y
        sections.append(ConcreteSection(
            next_id, "parking_in", port_x - lateral_x, port_y - lateral_y,
            config.parking_angle, config.parking_length, grid_node=node,
        ))
        next_id += 1
        sections.append(ConcreteSection(
            next_id, "parking_out",
            port_x + travel_x + lateral_x,
            port_y + travel_y + lateral_y,
            (config.parking_angle + 180) % 360,
            config.parking_length, grid_node=node,
        ))
        next_id += 1
    return sections


def build_geometry(grid: dict, config: GeometryConfig = GeometryConfig()) -> dict:
    """Build fixed roads, junction connectors, and two-way parking bays."""
    config.validate()
    nodes = {int(node["id"]): node for node in grid["nodes"]}
    edges = {
        (min(int(edge["source"]), int(edge["target"])),
         max(int(edge["source"]), int(edge["target"])))
        for edge in grid.get("edges", [])
    }
    roads = _road_sections(nodes, edges, config)
    parking_sections = _parking_sections(nodes, len(roads), config)
    sections = roads + parking_sections
    section_dicts = [asdict(section) for section in sections]
    roads_dict = [section for section in section_dicts if section["kind"] == "road"]
    parking_by_node: dict[int, dict[str, int]] = {}
    for section in section_dicts:
        if section["kind"] in {"parking_in", "parking_out"}:
            parking_by_node.setdefault(int(section["grid_node"]), {})[
                "inbound" if section["kind"] == "parking_in" else "outbound"
            ] = int(section["id"])

    outgoing: dict[str, list[int]] = {str(section["id"]): [] for section in section_dicts}
    start_by_node: dict[str, int] = {}
    for node in sorted(nodes):
        incoming = sorted(int(road["id"]) for road in roads_dict
                          if int(road["edge"][1]) == node)
        exits = sorted(int(road["id"]) for road in roads_dict
                       if int(road["edge"][0]) == node)
        parking_in = parking_by_node[node]["inbound"]
        parking_out = parking_by_node[node]["outbound"]
        if exits:
            start_by_node[str(node)] = exits[0]
        for road in incoming:
            outgoing[str(road)] = exits + [parking_in]
        outgoing[str(parking_in)] = [parking_out]
        outgoing[str(parking_out)] = exits

    by_id = {int(section["id"]): section for section in section_dicts}
    connection_lengths = {
        source: {
            str(target): connection_path_length(by_id[int(source)], by_id[target])
            for target in targets
        }
        for source, targets in outgoing.items()
    }
    manifest = {
        "config": asdict(config),
        "sections": section_dicts,
        "nodes": grid["nodes"],
        "edges": grid.get("edges", []),
        "outgoing": outgoing,
        "start_by_node": start_by_node,
        "parking_by_node": {str(node): value for node, value in parking_by_node.items()},
        "max_outgoing": max((len(targets) for targets in outgoing.values()), default=1),
        "connection_lengths": connection_lengths,
        "max_connection_length": max(
            (length for targets in connection_lengths.values() for length in targets.values()),
            default=0,
        ),
    }
    validate_geometry(manifest)
    return manifest


def write_geometry(grid_file: Path, output: Path,
                   config: GeometryConfig = GeometryConfig()) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(
        build_geometry(json.loads(grid_file.read_text()), config), indent=2
    ) + "\n")


def validate_geometry(manifest: dict) -> None:
    """Validate the directed grid/parking structure and its embedding."""
    config = GeometryConfig(**manifest["config"])
    config.validate()
    sections = manifest["sections"]
    by_id = {int(section["id"]): section for section in sections}
    if set(by_id) != set(range(len(sections))):
        raise ValueError("section IDs must be contiguous")
    roads = [section for section in sections if section["kind"] == "road"]
    node_ids = {int(node["id"]) for node in manifest["nodes"]}
    parking = manifest["parking_by_node"]
    if set(map(int, parking)) != node_ids:
        raise ValueError("each grid node must have one parking bay")
    for node in node_ids:
        pair = parking[str(node)]
        if set(pair) != {"inbound", "outbound"}:
            raise ValueError(f"node {node} does not have both parking directions")
        if by_id[int(pair["inbound"])]["kind"] != "parking_in":
            raise ValueError("parking inbound ID has the wrong section kind")
        if by_id[int(pair["outbound"])]["kind"] != "parking_out":
            raise ValueError("parking outbound ID has the wrong section kind")

    expected_edges = {(int(edge["source"]), int(edge["target"]))
                      for edge in manifest["edges"]}
    actual_edges = {(int(section["edge"][0]), int(section["edge"][1]))
                    for section in roads}
    if actual_edges != expected_edges | {(b, a) for a, b in expected_edges}:
        raise ValueError("road sections do not contain both directions of every edge")
    if len(sections) != 2 * len(expected_edges) + 2 * len(node_ids):
        raise ValueError("unexpected number of concrete sections")
    if any(int(section["length"]) <= 0 for section in sections):
        raise ValueError("all generated sections must have positive length")
    if int(manifest["max_outgoing"]) > 5:
        raise ValueError("a degree-four junction may have at most five exits")

    outgoing = {int(source): list(map(int, targets))
                for source, targets in manifest["outgoing"].items()}
    for node in node_ids:
        incoming = {int(road["id"]) for road in roads if int(road["edge"][1]) == node}
        exits = {int(road["id"]) for road in roads if int(road["edge"][0]) == node}
        parking_in = int(parking[str(node)]["inbound"])
        parking_out = int(parking[str(node)]["outbound"])
        for road in incoming:
            if set(outgoing[road]) != exits | {parking_in}:
                raise ValueError(f"incoming section {road} has incomplete junction access")
        if outgoing[parking_in] != [parking_out]:
            raise ValueError("parking inbound must connect to parking outbound")
        if set(outgoing[parking_out]) != exits:
            raise ValueError("parking outbound must reach every outgoing road")

    segments: set[tuple[tuple[int, int], tuple[int, int]]] = set()
    for section in sections:
        start = (int(section["source_x"]), int(section["source_y"]))
        end = section_endpoint(section)
        segment = tuple(sorted((start, end)))
        if segment in segments:
            raise ValueError("two concrete sections geometrically overlap")
        segments.add(segment)
    lengths = manifest["connection_lengths"]
    for source, targets in outgoing.items():
        if set(map(int, lengths[str(source)])) != set(targets):
            raise ValueError("connection arclength table does not match topology")
        if any(int(value) <= 0 for value in lengths[str(source)].values()):
            raise ValueError("every legal junction connector must have positive length")
