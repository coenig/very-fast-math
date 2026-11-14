"""Geometry and initialization helpers for the four-arm straight-through UCD junction."""

from dataclasses import dataclass
import math

import numpy as np
from highway_env.road.lane import LineType, StraightLane
from highway_env.road.road import RoadNetwork


ARM_LENGTH = 100.0
JUNCTION_GOAL_DISTANCE = 40.0
EASTBOUND = "eastbound"
SOUTHBOUND = "southbound"
MAX_BRAKING_DECELERATION = 3.0
SIMULATION_STEP_SECONDS = 0.5
SPAWN_GAP = 25.0
SPAWN_JITTER = 2.0
SPAWN_SAFETY_MARGIN = 3.0
SPAWN_UPSTREAM_MARGIN = 3.0


@dataclass(frozen=True)
class RouteFrame:
    name: str
    source: str
    destination: str
    heading: float
    tangent: tuple[float, float]
    normal: tuple[float, float]

    @property
    def nodes(self):
        return self.source, self.destination


ROUTES = {
    EASTBOUND: RouteFrame(
        EASTBOUND, "west", "east", 0.0, (1.0, 0.0), (0.0, 1.0)
    ),
    SOUTHBOUND: RouteFrame(
        SOUTHBOUND, "north", "south", -math.pi / 2, (0.0, -1.0), (1.0, 0.0)
    ),
}


def route_for_vehicle(vehicle_id):
    """Even vehicle IDs travel west-to-east; odd IDs travel north-to-south."""
    return ROUTES[EASTBOUND if vehicle_id % 2 == 0 else SOUTHBOUND]


def _validate_junction_vehicle_count(vehicle_count):
    if (
        not isinstance(vehicle_count, int)
        or isinstance(vehicle_count, bool)
        or vehicle_count < 2
    ):
        raise ValueError("the junction profile requires at least two vehicles")


def world_to_route(point, route):
    """Project a global position into route station and lateral coordinates."""
    x, y = point
    tx, ty = route.tangent
    nx, ny = route.normal
    return x * tx + y * ty, x * nx + y * ny


def vector_to_route(vector, route):
    """Project a global velocity/vector into route tangent and normal components."""
    return world_to_route(vector, route)


def route_to_world(station, lateral, route):
    """Convert route station/lateral coordinates into global world coordinates."""
    tx, ty = route.tangent
    nx, ny = route.normal
    return station * tx + lateral * nx, station * ty + lateral * ny


def relative_heading(heading, route):
    """Return a wrapped heading error relative to a route's forward direction."""
    return (heading - route.heading + math.pi) % (2 * math.pi) - math.pi


def section_source_y(section_id, lane_count, lane_width=4.0):
    """Return the y offset which centers the section's lane band on the junction."""
    lateral_center = lane_width * (lane_count - 1) / 2
    centerline_y = 100.0 if section_id == 2 else 0.0
    return centerline_y - lateral_center


def build_junction_road_network(lane_count, lane_width=4.0):
    """Build the two straight, multi-lane routes meeting at the world origin."""
    if not isinstance(lane_count, int) or isinstance(lane_count, bool) or lane_count < 1:
        raise ValueError("lane_count must be a positive integer")
    if lane_width <= 0:
        raise ValueError("lane_width must be positive")

    lateral_center = lane_width * (lane_count - 1) / 2
    network = RoadNetwork()
    lane_markings = (LineType.CONTINUOUS, LineType.STRIPED)
    for lane_index in range(lane_count):
        lateral = -lateral_center + lane_index * lane_width
        network.add_lane(
            "west",
            "east",
            StraightLane(
                [-ARM_LENGTH, lateral],
                [ARM_LENGTH, lateral],
                width=lane_width,
                line_types=lane_markings,
                speed_limit=70,
            ),
        )
        network.add_lane(
            "north",
            "south",
            StraightLane(
                [lateral, ARM_LENGTH],
                [lateral, -ARM_LENGTH],
                width=lane_width,
                line_types=lane_markings,
                speed_limit=70,
            ),
        )
    return network


def junction_approach_occupancy_distance(
    lane_count,
    lane_width=4.0,
    model_vehicle_length_m=7.0,
    model_vehicle_width_cm=220,
    distance_scale_factor=4.0,
):
    """Return the inbound station where the model first marks a car as crossing-relevant."""
    values = (lane_width, model_vehicle_length_m, model_vehicle_width_cm, distance_scale_factor)
    if lane_count < 1 or any(not math.isfinite(float(value)) or value <= 0 for value in values):
        raise ValueError("junction model geometry and distance scale must be positive and finite")

    def to_model_units(meters):
        return math.floor(meters * distance_scale_factor + 0.5)

    model_vehicle_width_m = model_vehicle_width_cm / 100.0
    crossing_reach = (
        to_model_units(lane_count * lane_width + model_vehicle_width_m)
        + to_model_units(model_vehicle_length_m)
        + 2
    )
    vehicle_length_units = to_model_units(model_vehicle_length_m)
    return (crossing_reach + vehicle_length_units) / distance_scale_factor


def initialize_junction_vehicles(
    env,
    lane_count,
    seed,
    min_speed,
    max_speed,
    lane_width=4.0,
    model_vehicle_length_m=7.0,
    model_vehicle_width_cm=220,
    distance_scale_factor=4.0,
):
    """Place all controlled vehicles safely on their assigned inbound approaches."""
    vehicles = env.unwrapped.controlled_vehicles
    _validate_junction_vehicle_count(len(vehicles))
    if lane_count < 1:
        raise ValueError("lane_count must be positive")

    network = build_junction_road_network(lane_count, lane_width)
    env.unwrapped.road.network = network
    rng = np.random.default_rng(seed)
    if max_speed < min_speed or min_speed < 0:
        raise ValueError("junction spawn speed range must be non-negative and ordered")

    approach_occupancy_distance = junction_approach_occupancy_distance(
        lane_count,
        lane_width,
        model_vehicle_length_m,
        model_vehicle_width_cm,
        distance_scale_factor,
    )
    vehicles_per_route = (len(vehicles) + 1) // 2
    max_approach_distance = (
        ARM_LENGTH
        - SPAWN_UPSTREAM_MARGIN
        - (vehicles_per_route - 1) * SPAWN_GAP
        - SPAWN_JITTER
    )
    max_stopping_distance = (
        max_approach_distance
        - approach_occupancy_distance
        - SPAWN_SAFETY_MARGIN
    )
    if max_stopping_distance < 0:
        raise ValueError(
            "junction vehicle count leaves insufficient inbound stopping distance"
        )
    safe_speed_cap = (
        math.sqrt(
            (MAX_BRAKING_DECELERATION * SIMULATION_STEP_SECONDS) ** 2
            + 2 * MAX_BRAKING_DECELERATION * max_stopping_distance
        )
        - MAX_BRAKING_DECELERATION * SIMULATION_STEP_SECONDS
    )
    safe_max_speed = min(max_speed, safe_speed_cap)
    if safe_max_speed < min_speed:
        raise ValueError(
            "junction lanes and speed range leave insufficient inbound stopping distance"
        )
    stopping_distance = (
        safe_max_speed**2 / (2 * MAX_BRAKING_DECELERATION)
        + safe_max_speed * SIMULATION_STEP_SECONDS
    )
    nearest_spawn_distance = approach_occupancy_distance + SPAWN_SAFETY_MARGIN + stopping_distance

    route_ordinals = {EASTBOUND: 0, SOUTHBOUND: 0}
    for vehicle_id, vehicle in enumerate(vehicles):
        route = route_for_vehicle(vehicle_id)
        ordinal = route_ordinals[route.name]
        route_ordinals[route.name] += 1
        lane_index = int(rng.integers(lane_count))
        approach_distance = (
            nearest_spawn_distance
            + ordinal * SPAWN_GAP
            + rng.uniform(0.0, SPAWN_JITTER)
        )
        if approach_distance > ARM_LENGTH - SPAWN_UPSTREAM_MARGIN:
            raise ValueError(
                "junction vehicle count and speed range exceed the available approach length"
            )
        station = -approach_distance
        lateral = -lane_width * (lane_count - 1) / 2 + lane_index * lane_width

        vehicle.position = np.asarray(route_to_world(station, lateral, route), dtype=float)
        vehicle.heading = route.heading
        vehicle.speed = float(rng.uniform(min_speed, safe_max_speed))
        vehicle.crashed = False
        vehicle.on_state_update()
        lane = (route.source, route.destination, lane_index)
        vehicle.target_lane_index = lane
        vehicle.route = [lane]
        vehicle.target_speed = vehicle.speed
    return [route_for_vehicle(i) for i in range(len(vehicles))]


def junction_goal_spec(
    vehicle_count=4, distance=JUNCTION_GOAL_DISTANCE, distance_scale_factor=4
):
    """Build the parity-aware experiment-10 target in model distance units."""
    _validate_junction_vehicle_count(vehicle_count)
    scaled_distance = int(distance * distance_scale_factor)
    clauses = []
    for vehicle_id in range(vehicle_count):
        section_id = 1 if vehicle_id % 2 == 0 else 3
        clauses.append(
            f"(env.veh___6{vehicle_id}9___.is_on_sec_{section_id} = 1 "
            f"& env.veh___6{vehicle_id}9___.abs_pos > {scaled_distance})"
        )
    return "INVARSPEC !(" + " & ".join(clauses) + ");"


def junction_goal_reached(
    vehicles,
    lane_count=2,
    lane_width=4.0,
    distance=JUNCTION_GOAL_DISTANCE,
):
    """Require every car to safely reach its assigned exit within the road bounds."""
    if (
        len(vehicles) < 2
        or lane_count < 1
        or any(vehicle.crashed for vehicle in vehicles)
    ):
        return False

    road_half_width = lane_count * lane_width / 2
    for vehicle_id, vehicle in enumerate(vehicles):
        station, lateral = world_to_route(vehicle.position, route_for_vehicle(vehicle_id))
        if station <= distance or abs(lateral) > road_half_width:
            return False
    return True


def junction_plan_is_usable(selected_config_index, result, empty_counterexample):
    """A junction cycle may step only when the planner selected a nonempty plan."""
    return selected_config_index is not None and result != empty_counterexample


def vehicle_clearance(vehicle_a, vehicle_b):
    """Return signed minimum clearance between two oriented vehicle rectangles."""
    def rectangle(vehicle):
        position = np.asarray(vehicle.position, dtype=float)
        if position.shape != (2,) or not np.all(np.isfinite(position)):
            raise ValueError("vehicle position must be a finite 2D point")
        heading = float(vehicle.heading)
        if not math.isfinite(heading):
            raise ValueError("vehicle heading must be finite")
        length = float(getattr(vehicle, "length", getattr(vehicle, "LENGTH", 5.0)))
        width = float(getattr(vehicle, "width", getattr(vehicle, "WIDTH", 2.0)))
        if not math.isfinite(length) or not math.isfinite(width) or length <= 0 or width <= 0:
            raise ValueError("vehicle dimensions must be finite and positive")
        forward = np.array([math.cos(heading), math.sin(heading)])
        lateral = np.array([-forward[1], forward[0]])
        half_length = length / 2
        half_width = width / 2
        return np.array(
            [
                position + forward * half_length + lateral * half_width,
                position + forward * half_length - lateral * half_width,
                position - forward * half_length - lateral * half_width,
                position - forward * half_length + lateral * half_width,
            ]
        ), forward, lateral

    corners_a, forward_a, lateral_a = rectangle(vehicle_a)
    corners_b, forward_b, lateral_b = rectangle(vehicle_b)
    axes = (forward_a, lateral_a, forward_b, lateral_b)
    penetration = math.inf
    separated = False
    for axis in axes:
        projection_a = corners_a @ axis
        projection_b = corners_b @ axis
        overlap = min(float(projection_a.max()), float(projection_b.max())) - max(
            float(projection_a.min()), float(projection_b.min())
        )
        if overlap < 0:
            separated = True
        else:
            penetration = min(penetration, overlap)
    if not separated:
        return -penetration

    def point_segment_distance(point, start, end):
        segment = end - start
        denominator = float(segment @ segment)
        fraction = 0.0 if denominator == 0 else float(
            np.clip(((point - start) @ segment) / denominator, 0.0, 1.0)
        )
        return float(np.linalg.norm(point - (start + fraction * segment)))

    edges_a = tuple(zip(corners_a, np.roll(corners_a, -1, axis=0)))
    edges_b = tuple(zip(corners_b, np.roll(corners_b, -1, axis=0)))
    return min(
        point_segment_distance(point, start, end)
        for corners, edges in ((corners_a, edges_b), (corners_b, edges_a))
        for point in corners
        for start, end in edges
    )


class JunctionClearanceTracker:
    """Track vehicle-body clearance at highway-env's pairwise collision checks."""

    def __init__(self):
        self.minimum_clearance = math.inf
        self.minimum_pair = None
        self.pairwise_samples = 0

    def record(self, vehicle_a, vehicle_b, vehicle_ids):
        clearance = vehicle_clearance(vehicle_a, vehicle_b)
        self.pairwise_samples += 1
        if clearance < self.minimum_clearance:
            self.minimum_clearance = clearance
            self.minimum_pair = tuple(vehicle_ids)


def install_junction_clearance_tracking(vehicles):
    """Sample oriented-body clearance before highway-env resolves each collision pair."""
    tracker = JunctionClearanceTracker()
    vehicle_ids = {id(vehicle): index for index, vehicle in enumerate(vehicles)}
    for vehicle in vehicles:
        original_handle_collisions = vehicle.handle_collisions

        def track_clearance(other, dt, *, _vehicle=vehicle, _original=original_handle_collisions):
            other_id = vehicle_ids.get(id(other))
            if other_id is not None:
                tracker.record(_vehicle, other, (vehicle_ids[id(_vehicle)], other_id))
            return _original(other, dt)

        vehicle.handle_collisions = track_clearance
    return tracker
