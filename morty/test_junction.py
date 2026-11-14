import json
import math
from pathlib import Path
import re
import unittest
from types import SimpleNamespace

import gymnasium
import numpy as np
from highway_env.road.road import Road
from highway_env.vehicle.kinematics import Vehicle

from .junction import (
    JUNCTION_GOAL_DISTANCE,
    build_junction_road_network,
    initialize_junction_vehicles,
    junction_approach_occupancy_distance,
    junction_goal_reached,
    junction_goal_spec,
    junction_plan_is_usable,
    install_junction_clearance_tracking,
    route_for_vehicle,
    route_to_world,
    section_source_y,
    vector_to_route,
    vehicle_clearance,
    world_to_route,
)


def make_junction_env(vehicle_count, lane_count):
    return gymnasium.make(
        "highway-v0",
        config={
            "action": {
                "type": "MultiAgentAction",
                "action_config": {
                    "type": "ContinuousAction",
                    "acceleration_range": [-6, 6],
                    "steering_range": [-1, 1],
                    "speed_range": [0, 70],
                    "longitudinal": True,
                    "lateral": True,
                },
            },
            "observation": {
                "type": "MultiAgentObservation",
                "observation_config": {
                    "features": ["presence", "x", "y", "vx", "vy", "heading"],
                    "type": "Kinematics",
                    "normalize": False,
                },
            },
            "simulation_frequency": 60,
            "policy_frequency": 2,
            "controlled_vehicles": vehicle_count,
            "lanes_count": lane_count,
            "vehicles_count": 0,
        },
    )


class JunctionGeometryTests(unittest.TestCase):
    def test_route_position_and_vector_transforms_round_trip(self):
        for vehicle_id in range(4):
            route = route_for_vehicle(vehicle_id)
            point = (18.5, -6.25)
            local = world_to_route(point, route)
            self.assertTrue(np.allclose(route_to_world(*local, route), point))
            vector = (-3.0, 7.5)
            self.assertTrue(np.allclose(route_to_world(*vector_to_route(vector, route), route), vector))

    def test_lane_offsets_center_both_roads_at_crossing(self):
        lane_count = 3
        network = build_junction_road_network(lane_count)
        expected_laterals = [-4.0, 0.0, 4.0]
        for lane_index, lateral in enumerate(expected_laterals):
            horizontal = network.get_lane(("west", "east", lane_index))
            vertical = network.get_lane(("north", "south", lane_index))
            self.assertTrue(np.allclose(horizontal.position(100, 0), (0, lateral)))
            self.assertTrue(np.allclose(vertical.position(100, 0), (lateral, 0)))
        self.assertEqual(section_source_y(0, lane_count), -4)
        self.assertEqual(section_source_y(2, lane_count), 96)

    def test_experiment_10_spec_matches_parity_and_40m_exit_threshold(self):
        spec = junction_goal_spec(4, JUNCTION_GOAL_DISTANCE, 4)
        self.assertEqual(
            spec,
            "INVARSPEC !((env.veh___609___.is_on_sec_1 = 1 & env.veh___609___.abs_pos > 160) "
            "& (env.veh___619___.is_on_sec_3 = 1 & env.veh___619___.abs_pos > 160) "
            "& (env.veh___629___.is_on_sec_1 = 1 & env.veh___629___.abs_pos > 160) "
            "& (env.veh___639___.is_on_sec_3 = 1 & env.veh___639___.abs_pos > 160));",
        )

    def test_goal_spec_and_observation_support_two_and_odd_vehicle_counts(self):
        self.assertEqual(
            junction_goal_spec(2, JUNCTION_GOAL_DISTANCE, 4),
            "INVARSPEC !((env.veh___609___.is_on_sec_1 = 1 & env.veh___609___.abs_pos > 160) "
            "& (env.veh___619___.is_on_sec_3 = 1 & env.veh___619___.abs_pos > 160));",
        )
        self.assertIn("veh___629___", junction_goal_spec(3))

        for vehicle_count in (2, 3):
            vehicles = [
                SimpleNamespace(
                    position=route_to_world(41, 0, route_for_vehicle(i)),
                    crashed=False,
                )
                for i in range(vehicle_count)
            ]
            self.assertTrue(junction_goal_reached(vehicles))

    def test_goal_spec_rejects_vehicle_count_below_two(self):
        with self.assertRaises(ValueError):
            junction_goal_spec(1)

    def test_model_occupancy_threshold_uses_scaled_vehicle_dimensions(self):
        self.assertAlmostEqual(
            junction_approach_occupancy_distance(2, 4.0, 7.0, 220, 4.0),
            24.75,
        )
        self.assertAlmostEqual(
            junction_approach_occupancy_distance(2, 4.0, 18.0, 340, 4.0),
            48.0,
        )

    def test_goal_requires_every_car_on_its_exit_beyond_threshold(self):
        vehicles = [
            SimpleNamespace(position=route_to_world(41, -2, route_for_vehicle(i)), crashed=False)
            for i in range(4)
        ]
        self.assertTrue(junction_goal_reached(vehicles))
        vehicles[2].position = route_to_world(40, 2, route_for_vehicle(2))
        self.assertFalse(junction_goal_reached(vehicles))

    def test_goal_rejects_off_road_and_crashed_vehicles(self):
        vehicles = [
            SimpleNamespace(position=route_to_world(41, 0, route_for_vehicle(i)), crashed=False)
            for i in range(4)
        ]
        vehicles[0].position = (41, 50)
        self.assertFalse(junction_goal_reached(vehicles))

        vehicles[0].position = route_to_world(41, 0, route_for_vehicle(0))
        vehicles[1].crashed = True
        self.assertFalse(junction_goal_reached(vehicles))

    def test_failed_or_empty_plan_is_not_usable(self):
        empty = "|;|;|;|;"
        self.assertFalse(junction_plan_is_usable(None, empty, empty))
        self.assertFalse(junction_plan_is_usable(0, empty, empty))
        self.assertTrue(junction_plan_is_usable(0, "0|;0|;0|;0|;", empty))


class JunctionSimulatorTests(unittest.TestCase):
    def test_oriented_vehicle_clearance_reports_gap_and_overlap(self):
        first = SimpleNamespace(
            position=np.array([0.0, 0.0]), heading=0.0, length=5.0, width=2.0
        )
        second = SimpleNamespace(
            position=np.array([7.0, 0.0]), heading=0.0, length=5.0, width=2.0
        )
        self.assertAlmostEqual(vehicle_clearance(first, second), 2.0)

        second.position[0] = 4.9
        self.assertAlmostEqual(vehicle_clearance(first, second), -0.1)

        second.position[:] = (0.0, 0.0)
        second.heading = -np.pi / 2
        self.assertAlmostEqual(vehicle_clearance(first, second), -2.0)

    def test_clearance_tracker_samples_before_collision_resolution(self):
        first = SimpleNamespace(
            position=np.array([0.0, 0.0]),
            heading=0.0,
            length=5.0,
            width=2.0,
            handle_collisions=lambda other, dt: "collision checked",
        )
        second = SimpleNamespace(
            position=np.array([7.0, 0.0]),
            heading=0.0,
            length=5.0,
            width=2.0,
            handle_collisions=lambda other, dt: "collision checked",
        )
        tracker = install_junction_clearance_tracking([first, second])
        self.assertEqual(first.handle_collisions(second, 1 / 60), "collision checked")
        self.assertEqual(tracker.pairwise_samples, 1)
        self.assertAlmostEqual(tracker.minimum_clearance, 2.0)
        self.assertEqual(tracker.minimum_pair, (0, 1))

    def test_clearance_tracker_hooks_highway_env_60hz_pair_check(self):
        road = Road(network=build_junction_road_network(2))
        vehicles = [
            Vehicle(road, [-20.0, 2.0], heading=0.0, speed=0.0),
            Vehicle(road, [-13.0, 2.0], heading=0.0, speed=0.0),
        ]
        for vehicle in vehicles:
            vehicle.act({"acceleration": 0.0, "steering": 0.0})
        road.vehicles.extend(vehicles)
        tracker = install_junction_clearance_tracking(vehicles)

        road.step(1 / 60)

        self.assertEqual(tracker.pairwise_samples, 1)
        self.assertAlmostEqual(tracker.minimum_clearance, 2.0)

    def test_seeded_spawn_and_smoke_step_on_flexible_lanes(self):
        env = make_junction_env(4, 3)
        try:
            env.reset(seed=42)
            routes = initialize_junction_vehicles(env, 3, 42, 10, 15)
            vehicles = env.unwrapped.controlled_vehicles
            for vehicle_id, vehicle in enumerate(vehicles):
                route = routes[vehicle_id]
                station, lateral = world_to_route(vehicle.position, route)
                self.assertGreater(station, -100)
                self.assertLess(station, 0)
                self.assertGreaterEqual(
                    -station,
                    junction_approach_occupancy_distance(3)
                    + 3.0
                    + vehicle.speed**2 / (2 * 3.0)
                    + vehicle.speed * 0.5,
                )
                self.assertAlmostEqual(vehicle.heading, route.heading)
                self.assertEqual(vehicle.lane_index, (*route.nodes, vehicle.target_lane_index[2]))
                self.assertLessEqual(abs(lateral), 4.0)
            for route_name in ("eastbound", "southbound"):
                members = [
                    world_to_route(vehicle.position, route)[0]
                    for vehicle, route in zip(vehicles, routes)
                    if route.name == route_name
                ]
                self.assertGreater(abs(members[0] - members[1]), 5.0)

            observation = env.unwrapped.observation_type.observe()
            self.assertEqual(len(observation), 4)
            _, _, _, _, _ = env.step(([0, 0],) * 4)
            self.assertFalse(any(vehicle.crashed for vehicle in vehicles))
        finally:
            env.close()

    def test_seeded_spawn_and_step_for_two_and_odd_counts(self):
        for vehicle_count in (2, 3):
            with self.subTest(vehicle_count=vehicle_count):
                env = make_junction_env(vehicle_count, 2)
                try:
                    env.reset(seed=42)
                    routes = initialize_junction_vehicles(
                        env, 2, 42, 10, 15, lane_width=4.0
                    )
                    vehicles = env.unwrapped.controlled_vehicles
                    self.assertEqual(len(routes), vehicle_count)
                    for vehicle_id, (vehicle, route) in enumerate(zip(vehicles, routes)):
                        self.assertEqual(
                            route.name,
                            "eastbound" if vehicle_id % 2 == 0 else "southbound",
                        )
                        station, _ = world_to_route(vehicle.position, route)
                        self.assertGreater(station, -100)
                        self.assertLess(station, 0)
                    _, _, _, _, _ = env.step(([0, 0],) * vehicle_count)
                    self.assertFalse(any(vehicle.crashed for vehicle in vehicles))
                finally:
                    env.close()

    def test_spawn_rejects_counts_that_exceed_available_arm_length(self):
        env = make_junction_env(8, 2)
        try:
            env.reset(seed=7)
            with self.assertRaisesRegex(ValueError, "insufficient inbound stopping distance"):
                initialize_junction_vehicles(env, 2, 7, 10, 15)
        finally:
            env.close()

    def test_spawn_uses_largest_prioritized_model_vehicle_dimensions(self):
        env = make_junction_env(2, 2)
        try:
            env.reset(seed=7)
            routes = initialize_junction_vehicles(
                env,
                2,
                7,
                10,
                15,
                model_vehicle_length_m=18,
                model_vehicle_width_cm=340,
            )
            for vehicle, route in zip(env.unwrapped.controlled_vehicles, routes):
                station, _ = world_to_route(vehicle.position, route)
                expected_distance = (
                    junction_approach_occupancy_distance(2, 4.0, 18, 340)
                    + 3.0
                    + vehicle.speed**2 / 6.0
                    + vehicle.speed * 0.5
                )
                self.assertGreaterEqual(-station, expected_distance)
        finally:
            env.close()

        env = make_junction_env(4, 2)
        try:
            env.reset(seed=7)
            with self.assertRaisesRegex(ValueError, "insufficient inbound stopping distance"):
                initialize_junction_vehicles(
                    env,
                    2,
                    7,
                    10,
                    15,
                    model_vehicle_length_m=18,
                    model_vehicle_width_cm=340,
                )
        finally:
            env.close()

    def test_shipped_four_car_profile_initializes_on_worst_case_dimensions(self):
        profile_path = Path(__file__).parent / "master_templates" / "junction_tpl.json"
        template = json.loads(profile_path.read_text(encoding="utf-8"))["#TEMPLATE"]
        vehicle_count = template["NONEGOS"]
        lane_count = template["NUMLANES"]
        dimensions = [
            re.fullmatch(r"_config_vehlen=(\d+)_vehwidth=(\d+)", config_name)
            for config_name in template["UCD_CONFIG_PRIOS"].split(";")
            if config_name
        ]
        self.assertTrue(dimensions)
        self.assertTrue(all(match is not None for match in dimensions))
        self.assertTrue(template["UCD_JUNCTION"])
        self.assertEqual(template["#007"], "range(@nonegos, 4, 4);")
        self.assertEqual(template["MINSTARTSPEEDNONEGO_UCD"], 5)
        self.assertEqual(template["MAXSTARTSPEEDNONEGO_UCD"], 15)
        max_model_vehicle_length_m = max(
            math.trunc(0.1 * int(match.group(1)) ** 2 + 1.85 * int(match.group(1)) - 6.75)
            for match in dimensions
        )
        max_model_vehicle_width_cm = max(
            int(match.group(2)) * 40 - 20 for match in dimensions
        )
        self.assertEqual(vehicle_count, 4)
        self.assertEqual(lane_count, 2)
        self.assertEqual(
            (max_model_vehicle_length_m, max_model_vehicle_width_cm),
            (18, 340),
        )

        env = make_junction_env(vehicle_count, lane_count)
        try:
            env.reset(seed=19)
            routes = initialize_junction_vehicles(
                env,
                lane_count,
                19,
                template["MINSTARTSPEEDNONEGO_UCD"],
                template["MAXSTARTSPEEDNONEGO_UCD"],
                lane_width=template["LANE_WIDTH"] / 100,
                model_vehicle_length_m=max_model_vehicle_length_m,
                model_vehicle_width_cm=max_model_vehicle_width_cm,
                distance_scale_factor=template["DISTANCESCALING"] / 1000,
            )
            vehicles = env.unwrapped.controlled_vehicles
            self.assertEqual(len(routes), vehicle_count)
            occupancy_distance = junction_approach_occupancy_distance(
                lane_count,
                template["LANE_WIDTH"] / 100,
                max_model_vehicle_length_m,
                max_model_vehicle_width_cm,
                template["DISTANCESCALING"] / 1000,
            )
            for vehicle_id, (vehicle, route) in enumerate(zip(vehicles, routes)):
                self.assertEqual(
                    route.name,
                    "eastbound" if vehicle_id % 2 == 0 else "southbound",
                )
                station, _ = world_to_route(vehicle.position, route)
                self.assertLess(station, 0)
                self.assertGreaterEqual(
                    -station,
                    occupancy_distance
                    + 3.0
                    + vehicle.speed**2 / 6.0
                    + vehicle.speed * 0.5,
                )
                self.assertGreaterEqual(
                    vehicle.speed,
                    template["MINSTARTSPEEDNONEGO_UCD"],
                )
                self.assertLessEqual(vehicle.speed, 9.283)

            _, _, _, _, _ = env.step(([0, 0],) * vehicle_count)
            self.assertFalse(any(vehicle.crashed for vehicle in vehicles))
        finally:
            env.close()


if __name__ == "__main__":
    unittest.main()
