"""Solver-backed regression tests for the generated junction model."""

import os
import re
import shutil
import subprocess
import unittest
import uuid
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
VFM = ROOT / "bin" / ("vfm.exe" if os.name == "nt" else "vfm")
NUXMV = ROOT / "external" / ("win32" if os.name == "nt" else "linux64") / (
    "nuXmv.exe" if os.name == "nt" else "nuXmv"
)


@unittest.skipUnless(VFM.is_file() and NUXMV.is_file() and shutil.which("gcc"),
                     "requires built vfm, nuXmv, and gcc")
class JunctionSmvTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.work = ROOT / ("junction_smv_test_" + uuid.uuid4().hex)
        cls.work.mkdir()
        cls.addClassCleanup(shutil.rmtree, cls.work)
        cls.models = {}
        for cars, lanes, distance, time, ys in (
            (2, 1, 1000, 1000, "@(0)@@(0)@@(100)@@(0)@"),
            (2, 2, 4000, 2000, "@(-2)@@(-2)@@(98)@@(-2)@"),
            (4, 2, 4000, 2000, "@(-2)@@(-2)@@(98)@@(-2)@"),
            (4, 4, 4000, 2000, "@(-6)@@(-6)@@(94)@@(-6)@"),
        ):
            length, width = (7, 220) if (cars, lanes) == (2, 2) else (5, 200)
            settings = f"""@{{{ROOT / "src" / "templates"}}}@***********.stringToHeap[MY_PATH].nil
@{{@UCD=true; @EGOLESS=true; @UCD_JUNCTION=true; @UCD_TASK7_COMPACT=true; @SIMPLE_LC=true;
 @SECTIONS=4; @SEGMENTS=1; @NONEGOS={cars}; @NUMLANES={lanes}; @LATERAL_LC_GRANULARITY=0;
 @MODEL_INTERSECTION_GEOMETRY=false; @TIMESCALING={time}; @DISTANCESCALING={distance}; @GEOMETRYSCALING=1000;
 @MAXSPEEDNONEGO=40; @LANE_WIDTH=400; @VEHICLE_WIDTH={width}; @VEHICLE_LENGTH={length};
 @MINACCELNONEGO=-3; @MAXACCELNONEGO=3; @MAX_JUMP_OVER_TECHNICAL_LANES=1;
 @MAX_JUMP_UNTIL_VELOCITY=3; @MIN_JUMP_FROM_VELOCITY=12;
 @MIN_TIME_BETWEEN_LANECHANGES=0; @UCD_MIN_VELOCITY_TO_ALLOW_LC=1;
 @MAXOUTGOINGCONNECTIONS=1; @ANGLEGRANULARITY=90; @MINDISTCONNECTIONS=0; @MAXDISTCONNECTIONS=0;}}@***********.eval.nil
@{{@(0)@@(1)@@(2)@@(3)@}}@***********.stringToHeap[FIXED_SECTION_IDs].nil
@{{@(-100)@@(0)@@(0)@@(0)@}}@***********.stringToHeap[FIXED_SECTION_SOURCE_Xs].nil
@{{{ys}}}@***********.stringToHeap[FIXED_SECTION_SOURCE_Ys].nil
@{{@(0)@@(0)@@(270)@@(270)@}}@***********.stringToHeap[FIXED_SECTION_ANGLEs].nil
@{{@(100)@@(100)@@(100)@@(100)@}}@***********.stringToHeap[FIXED_SECTION_LENGTHs].nil
@{{@(@(0)@@(@(1)@)@)@@(@(1)@@(@(-1)@)@)@@(@(2)@@(@(3)@)@)@@(@(3)@@(@(-1)@)@)@}}@***********.stringToHeap[FIXED_SECTION_CONNECTORS].nil
@{{@(0)@@(1)@@(2)@@(3)@}}@***********.stringToHeap[FORWARD_DRIVING_CAR_IDS].nil
@{{EnvModel.tpl}}@********.include
"""
            (cls.work / "script.txt").write_text(settings, encoding="utf-8")
            run = subprocess.run([str(VFM), "--execute-script", "--rootdir", str(cls.work)],
                                 cwd=ROOT, capture_output=True, text=True, errors="replace", timeout=90)
            if run.returncode or "ERROR:" in run.stderr:
                raise AssertionError(f"template expansion failed: {run.stderr}")
            model = run.stdout.split("Script expanded to:\n--------------------\n", 1)[1]
            if "@{" in model or "#error" in model or "#Error" in model:
                raise AssertionError("template expansion left an error or unexpanded macro")
            source = cls.work / "model_input.smv"
            source.write_text("\n".join(line.split("--", 1)[0] for line in model.splitlines()))
            preprocessed = cls.work / "preprocessed.smv"
            subprocess.run(["gcc", "-E", "-P", "-x", "c", str(source), "-o", str(preprocessed)],
                           cwd=ROOT, check=True, capture_output=True, timeout=90)
            cls.models[cars, lanes] = (
                "MODULE main\nVAR env : EnvModel;\n" + preprocessed.read_text()
                + "\nDEFINE ego.gaps___609___.v_rear := 0;\n"
                + "ego.gaps___609___.s_dist_rear := 0;\n"
                + "ego.gaps___609___.i_agent_rear := -1;\n"
            )

    def solve(self, model, scenario, bound, property_="env.cnt = 0", initial_sat=True):
        smv = self.work / "case.smv"
        cmd = self.work / "case.cmd"
        smv.write_text(model + scenario)
        cmd.write_text(f'read_model -i {smv.as_posix()}\ngo_msat\n'
                       'msat_check_invar_bmc -a falsification -k 0 -p "FALSE"\n'
                       f'msat_check_invar_bmc -a falsification -k {bound} -p "{property_}"\nquit\n')
        run = subprocess.run([str(NUXMV), "-int", "-source", str(cmd)], cwd=ROOT,
                             input="quit\n", capture_output=True, text=True, errors="replace", timeout=90)
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertNotIn("aborting", run.stderr.lower())
        self.assertEqual("invariant FALSE is false, a counter-example of size 0 exists" in run.stdout,
                         initial_sat, run.stdout)
        found = bool(re.search(r"is false, a counter-example of size [1-9]\d* exists", run.stdout))
        if not found:
            self.assertIn(f"no counterexample found with bound {bound}", run.stdout)
        return found

    def test_four_car_goal_is_reachable(self):
        vehicles = (("609", 0, 300, 0), ("629", 0, 260, 2),
                    ("619", 2, 160, 0), ("639", 2, 120, 2))
        scenario = "\n".join(
            f"INIT veh___{v}___.is_on_sec_{sec} = 1 & veh___{v}___.abs_pos = {pos}"
            f" & veh___{v}___.lane_coordinate = {lane} & veh___{v}___.v = 20;"
            for v, sec, pos, lane in vehicles
        )
        goal = " & ".join(
            f"(env.veh___{v}___.is_on_sec_{sec} = 1 & env.veh___{v}___.abs_pos > 160)"
            for v, sec in (("609", 1), ("629", 1), ("619", 3), ("639", 3))
        )
        self.assertTrue(self.solve(self.models[4, 2], scenario, 30, f"!({goal})"))

    def test_two_car_goal_is_reachable(self):
        scenario = """
INIT veh___609___.is_on_sec_0 = 1 & veh___609___.abs_pos = 280 & veh___609___.v = 20 & veh___609___.lane_coordinate = 0;
INIT veh___619___.is_on_sec_2 = 1 & veh___619___.abs_pos = 120 & veh___619___.v = 20 & veh___619___.lane_coordinate = 2;
"""
        goal = ("(env.veh___609___.is_on_sec_1 = 1 & env.veh___609___.abs_pos > 160) & "
                "(env.veh___619___.is_on_sec_3 = 1 & env.veh___619___.abs_pos > 160)")
        self.assertTrue(self.solve(self.models[2, 2], scenario, 30, f"!({goal})"))

    def test_perpendicular_tunneling_is_rejected(self):
        model = self.models[2, 1]
        scenario = """
INIT veh___609___.is_on_sec_0 = 1 & veh___609___.abs_pos = 81 & veh___609___.v = 20;
INIT veh___619___.is_on_sec_2 = 1 & veh___619___.abs_pos = 81 & veh___619___.v = 20;
TRANS next(veh___609___.v) = 20 & next(veh___619___.v) = 20;
"""
        control, n = re.subn(
            r"TRANS !\(veh___619___\.junction_(?:horizontal|vertical)_route & veh___609___\.junction_(?:vertical|horizontal)_route\s*& veh___619___\.junction_(?:horizontal|vertical)_swept & veh___609___\.junction_(?:vertical|horizontal)_swept\);",
            "", model)
        self.assertEqual(n, 2)
        control, n = re.subn(
            r"INVAR !\(veh___619___\.junction_(?:horizontal|vertical)_occupied & veh___609___\.junction_(?:vertical|horizontal)_occupied\);",
            "", control)
        self.assertEqual(n, 2)
        self.assertTrue(self.solve(control, scenario, 1))
        self.assertFalse(self.solve(model, scenario, 1))

    def test_junction_position_uses_mean_velocity(self):
        model = self.models[2, 1]
        for current, following, distance in ((8, 10, 9), (10, 8, 9), (8, 9, 8)):
            with self.subTest(current=current, following=following):
                scenario = f"""
INIT veh___609___.is_on_sec_0 = 1 & veh___609___.abs_pos = 20 & veh___609___.v = {current};
TRANS next(veh___609___.v) = {following};
"""
                self.assertTrue(self.solve(
                    model, scenario, 1, f"env.veh___609___.abs_pos != {20 + distance}"))
                self.assertFalse(self.solve(
                    model, scenario, 1, f"env.veh___609___.abs_pos != {21 + distance}"))

    def test_mean_velocity_against_highway_env_substeps(self):
        try:
            from highway_env.road.road import Road
            from highway_env.vehicle.kinematics import Vehicle
            from morty.junction import build_junction_road_network
        except ImportError:
            self.skipTest("requires highway-env")

        model = self.models[4, 2]
        for current, following in ((20, 22), (22, 20), (20, 21), (21, 20)):
            with self.subTest(current=current, following=following):
                road = Road(network=build_junction_road_network(2))
                vehicle = Vehicle(road, [-80.0, 2.0], heading=0, speed=current / 2)
                vehicle.act({"acceleration": following - current, "steering": 0.0})
                for _ in range(30):
                    vehicle.step(1 / 60)
                model_position = 80 + (current + following) // 2
                scenario = f"""
INIT veh___609___.is_on_sec_0 = 1 & veh___609___.abs_pos = 80 & veh___609___.v = {current};
TRANS next(veh___609___.v) = {following};
"""
                self.assertTrue(self.solve(
                    model, scenario, 1, f"env.veh___609___.abs_pos != {model_position}"))
                self.assertAlmostEqual(vehicle.speed, following / 2)
                self.assertLessEqual(abs((vehicle.position[0] + 100) - model_position / 4), 0.25)

    def test_braking_and_acceleration_preserve_boundary_clearance(self):
        model = self.models[4, 2]
        for front_position, safe in ((21, False), (23, True)):
            with self.subTest(front_position=front_position):
                scenario = f"""
INIT veh___609___.is_on_sec_0 = 1 & veh___609___.abs_pos = 390 & veh___609___.v = 8 & veh___609___.lane_coordinate = 0;
INIT veh___629___.is_on_sec_1 = 1 & veh___629___.abs_pos = {front_position} & veh___629___.v = 8 & veh___629___.lane_coordinate = 0;
TRANS next(veh___609___.v) = 10 & next(veh___629___.v) = 6;
"""
                self.assertEqual(self.solve(model, scenario, 1), safe)

    def test_rounding_margin_at_crossing_and_same_route(self):
        model = self.models[4, 2]
        crossing = """
INIT veh___609___.is_on_sec_0 = 1 & veh___609___.abs_pos = 318;
INIT veh___619___.is_on_sec_3 = 1 & veh___619___.abs_pos = 0;
"""
        control, n = re.subn(
            r"(junction_conflict_reach := \d+ \+ veh_length) \+ 2;", r"\1 + 1;", model)
        self.assertEqual(n, 1)
        self.solve(control, crossing, 0)
        self.solve(model, crossing, 0, initial_sat=False)

        same_section = """
INIT veh___609___.is_on_sec_0 = 1 & veh___609___.abs_pos = 100 & veh___609___.lane_coordinate = 0;
INIT veh___629___.is_on_sec_0 = 1 & veh___629___.abs_pos = 130 & veh___629___.lane_coordinate = 0;
"""
        cross_section = """
INIT veh___609___.is_on_sec_0 = 1 & veh___609___.abs_pos = 399 & veh___609___.lane_coordinate = 0;
INIT veh___629___.is_on_sec_1 = 1 & veh___629___.abs_pos = 29 & veh___629___.lane_coordinate = 0;
"""
        control = model.replace("junction_longitudinal_clearance := veh_length + 8 + 2;",
                                "junction_longitudinal_clearance := veh_length + 8 + 1;")
        self.assertNotEqual(control, model)
        for scenario in (same_section, cross_section):
            self.solve(control, scenario, 0)
            self.solve(model, scenario, 0, initial_sat=False)

    def test_lateral_rounding_uses_conservative_lane_bins(self):
        model = self.models[4, 4]
        adjacent_bins = """
INIT veh___609___.is_on_sec_0 = 1 & veh___609___.abs_pos = 100 & veh___609___.lane_coordinate = 0;
INIT veh___629___.is_on_sec_0 = 1 & veh___629___.abs_pos = 100 & veh___629___.lane_coordinate = 4;
"""
        control = model.replace(" + distance_between_normalized_lanes_cm ->", " ->")
        self.assertNotEqual(control, model)
        self.solve(control, adjacent_bins, 0)
        self.solve(model, adjacent_bins, 0, initial_sat=False)

        yawed_bins = adjacent_bins.replace("lane_coordinate = 4", "lane_coordinate = 3")
        control = model.replace("<= 700 + distance_between_normalized_lanes_cm ->",
                                "<= 200 + distance_between_normalized_lanes_cm ->")
        self.assertNotEqual(control, model)
        self.solve(control, yawed_bins, 0)
        self.solve(model, yawed_bins, 0, initial_sat=False)

        separated_bins = """
INIT veh___609___.is_on_sec_0 = 1 & veh___609___.abs_pos = 100 & veh___609___.lane_coordinate = 0;
INIT veh___629___.is_on_sec_0 = 1 & veh___629___.abs_pos = 100 & veh___629___.lane_coordinate = 6;
"""
        self.solve(model, separated_bins, 0)

    def test_same_route_boundary_sweeps_and_adjacent_bins(self):
        model = self.models[4, 2]
        control, n = re.subn(
            r"INVAR \(\(veh___6\d9___\.junction_horizontal_route & veh___6\d9___\.junction_horizontal_route\).*?junction_station\);",
            "", model, flags=re.S)
        self.assertEqual(n, 6)
        control, n = re.subn(
            r"TRANS \(\(veh___6\d9___\.junction_horizontal_route & veh___6\d9___\.junction_horizontal_route\).*?junction_station\)\)\);",
            "", control, flags=re.S)
        self.assertEqual(n, 6)
        for rear, front, entry, exit_ in (("609", "629", 0, 1), ("619", "639", 2, 3)):
            for rear_lane, front_lane in ((0, 0), (2, 2), (0, 2)):
                with self.subTest(route=entry, rear_lane=rear_lane, front_lane=front_lane):
                    scenario = f"""
INIT veh___{rear}___.is_on_sec_{entry} = 1 & veh___{rear}___.abs_pos = 376 & veh___{rear}___.lane_coordinate = {rear_lane} & veh___{rear}___.v = 40;
INIT veh___{front}___.is_on_sec_{exit_} = 1 & veh___{front}___.abs_pos = 24 & veh___{front}___.lane_coordinate = {front_lane} & veh___{front}___.v = 0;
TRANS next(veh___{rear}___.v) = 40 & next(veh___{front}___.v) = 0;
"""
                    self.assertTrue(self.solve(control, scenario, 1))
                    self.assertFalse(self.solve(model, scenario, 1))

                    overlapping = f"""
INIT veh___{rear}___.is_on_sec_{entry} = 1 & veh___{rear}___.abs_pos = 396 & veh___{rear}___.lane_coordinate = {rear_lane};
INIT veh___{front}___.is_on_sec_{exit_} = 1 & veh___{front}___.abs_pos = 4 & veh___{front}___.lane_coordinate = {front_lane};
"""
                    self.solve(control, overlapping, 0)
                    self.solve(model, overlapping, 0, initial_sat=False)


if __name__ == "__main__":
    unittest.main()
