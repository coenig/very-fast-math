import ast
import contextlib
import io
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, mock_open, patch

from . import morty_helper as helper


class LiveModeTests(unittest.TestCase):
    def test_json_run_settings_preserve_legacy_defaults(self):
        self.assertEqual(helper.load_run_settings({}), {
            'num_runs': 100, 'steps_per_run': 300, 'heading_adaptation': -0.5,
            'allow_blind_steps': 1, 'allow_crashed_steps': 1, 'min_iterations': 2,
            'mc_selection': 'all',
        })
        settings = helper.load_run_settings({'UCD_MC_SELECTION': 'priority', 'UCD_NUM_RUNS': 1,
                                             'UCD_ALLOW_BLIND_STEPS': 0, 'UCD_MIN_ITERATIONS': 0})
        self.assertEqual(settings['mc_selection'], 'priority')
        self.assertEqual(settings['num_runs'], 1)
        self.assertEqual(settings['allow_blind_steps'], 0)
        self.assertEqual(settings['min_iterations'], 0)

    def test_json_run_settings_reject_invalid_values(self):
        for key, value in [('UCD_MC_SELECTION', 'unknown'), ('UCD_MC_SELECTION', None),
                           ('UCD_NUM_RUNS', True), ('UCD_STEPS_PER_RUN', -1),
                           ('UCD_ALLOW_BLIND_STEPS', 1.5), ('UCD_MIN_ITERATIONS', '0'),
                           ('UCD_HEADING_ADAPTATION', float('nan')),
                           ('UCD_HEADING_ADAPTATION', float('inf')),
                           ('UCD_HEADING_ADAPTATION', False)]:
            with self.subTest(key=key, value=value), self.assertRaisesRegex(ValueError, key):
                helper.load_run_settings({key: value})

    def test_priority_selector_prefers_larger_boxes_despite_cost_history(self):
        selector = helper.MCConfigSelector(16, policy='priority')
        selector.record(0, False, False, 100)
        selector.record(15, False, True, 0.01)
        self.assertEqual(list(selector.queries(0, None)), [(index, None) for index in range(16)])
        queries = list(selector.queries(4, 5))
        self.assertEqual(set(queries), {(index, None) for index in range(16)}
                         | {(index, point) for index in range(6) for point in range(4)})

    def parse_driver_options(self, *options, template=None):
        module = ast.parse(Path(__file__).with_name('morty.py').read_text())
        plotting_start = next(index for index, node in enumerate(module.body)
                              if isinstance(node, ast.If) and ast.unparse(node.test) == 'not args.live')
        namespace = {'__package__': 'morty',
                     'open': mock_open(read_data=json.dumps({'#TEMPLATE': template or {}}))}
        with (
            patch('sys.argv', ['morty', *options]),
            patch('shutil.which', return_value='/usr/bin/timeout'),
        ):
            exec(compile(ast.Module(body=module.body[:plotting_start], type_ignores=[]),
                         '<driver options>', 'exec'), namespace)
        return namespace['args']

    def test_plain_live_mode_remains_headless(self):
        for options in [('--live',), ('--live', '--headless')]:
            with self.subTest(options=options):
                args = self.parse_driver_options(*options)
                self.assertTrue(args.headless)
                self.assertFalse(args.force_head)
                self.assertEqual(args.mc_selection, 'all')

    def test_adaptive_selection_is_live_only_and_opt_in(self):
        for policy in ['adaptive', 'priority']:
            template = {'UCD_MC_SELECTION': policy}
            with self.subTest(policy=policy):
                self.assertEqual(self.parse_driver_options('--live', template=template).mc_selection, policy)
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    self.parse_driver_options(template=template)

    def test_behavior_settings_cannot_be_overridden_by_cli(self):
        for option, value in [('--mc_selection', 'adaptive'), ('--num_runs', '1'),
                              ('--steps_per_run', '3'), ('--heading_adaptation', '0'),
                              ('--allow_blind_steps', '0'), ('--allow_crashed_steps', '0'),
                              ('--min_iterations', '0'), ('-n', '1'), ('-s', '3'),
                              ('-a', '0'), ('-b', '0'), ('-c', '0')]:
            with self.subTest(option=option), contextlib.redirect_stderr(io.StringIO()), \
                 self.assertRaises(SystemExit):
                self.parse_driver_options('--live', option, value)

    def test_driver_applies_json_run_settings(self):
        template = {'UCD_NUM_RUNS': 3, 'UCD_STEPS_PER_RUN': 17, 'UCD_HEADING_ADAPTATION': 1.25,
                    'UCD_ALLOW_BLIND_STEPS': 0, 'UCD_ALLOW_CRASHED_STEPS': 0,
                    'UCD_MIN_ITERATIONS': 0, 'UCD_MC_SELECTION': 'priority'}
        args = self.parse_driver_options('--live', template=template)
        for name, value in helper.load_run_settings(template).items():
            self.assertEqual(getattr(args, name), value)

    def test_status_is_inactive_without_a_visible_viewer(self):
        for viewer in [None, SimpleNamespace(offscreen=True),
                       SimpleNamespace(offscreen=False, enabled=False)]:
            with self.subTest(viewer=viewer), patch.dict('sys.modules', {'pygame': None}):
                state = helper.VizState()
                env = SimpleNamespace(unwrapped=SimpleNamespace(viewer=viewer))
                state.attach_status(env)
                state.set_status(env, 'MC pipeline running', mc_running=True)
                self.assertEqual(state.status, '')
                self.assertFalse(state.mc_running)

    def test_status_is_painted_immediately_and_survives_redrawing(self):
        import pygame

        with patch.dict('os.environ', {'SDL_VIDEODRIVER': 'dummy'}):
            pygame.display.init()
            pygame.font.init()
            try:
                screen = pygame.display.set_mode((400, 180))
                viewer = SimpleNamespace(screen=screen, offscreen=False, enabled=True,
                                         display=Mock(side_effect=lambda: screen.fill((0, 0, 0))))
                env = SimpleNamespace(unwrapped=SimpleNamespace(viewer=viewer))
                state = helper.VizState()
                state.attach_status(env)
                with patch.object(pygame.display, 'update') as update:
                    state.set_status(env, 'Cycle 1 | MC pipeline running', mc_running=True)
                    update.assert_called_once()
                    self.assertEqual(tuple(screen.get_at((0, 0)))[:3], (255, 221, 92))
                    viewer.display()
                    self.assertEqual(tuple(screen.get_at((0, 0)))[:3], (255, 221, 92))
                    self.assertEqual(tuple(screen.get_at((0, 40)))[:3], (0, 0, 0))
                    state.set_status(env, 'Cycle 1 | Rendering road')
                    self.assertFalse(state.mc_running)
                    self.assertEqual(tuple(screen.get_at((0, 0)))[:3], (229, 235, 239))
            finally:
                pygame.quit()

    def test_force_head_enables_only_the_live_window(self):
        args = self.parse_driver_options('--live', '--force_head')
        self.assertTrue(args.live)
        self.assertFalse(args.headless)
        self.assertTrue(args.hide_trajectories)
        self.assertTrue(args.hide_pure_pursuit)
        self.assertTrue(args.hide_planned_positions)
        self.assertFalse(args.record_video)
        self.assertFalse(args.detailed_archive)

    def test_mc_status_brackets_the_native_call_and_preserves_output(self):
        from ctypes import create_string_buffer, sizeof

        module = ast.parse(Path(__file__).with_name('morty.py').read_text())
        functions = [node for node in module.body if isinstance(node, ast.FunctionDef)
                     and node.name in ('show_sim_status', 'run_planning_script')]
        for dryrun, headless, description in [(False, False, 'full plan'),
                                              (False, False, 'shortcut 3'),
                                              (True, False, 'full plan'),
                                              (False, True, 'full plan')]:
            with self.subTest(dryrun=dryrun, headless=headless, description=description):
                state = SimpleNamespace(set_status=Mock())
                env = object()
                phase = 'Replaying trace' if dryrun else 'MC pipeline running'

                def native_call(script, result, size):
                    if headless:
                        state.set_status.assert_not_called()
                    else:
                        state.set_status.assert_called_once_with(
                            env, f'Cycle 5 | {phase} ({description})', mc_running=not dryrun)
                    self.assertEqual(script, b'original script')
                    return b'original output'

                library = SimpleNamespace(expandScript=Mock(side_effect=native_call))
                context = Mock(side_effect=lambda **kwargs: contextlib.nullcontext(library))
                namespace = {'args': SimpleNamespace(live=True, dryrun=dryrun, headless=headless),
                             'viz_state': state, 'env': env, 'global_counter': 4,
                             'morty_script_context': context, 'sizeof': sizeof}
                exec(compile(ast.Module(body=functions, type_ignores=[]), '<planning status>', 'exec'), namespace)
                result = namespace['run_planning_script']('original script', create_string_buffer(100), description)
                self.assertEqual(result, b'original output')
                context.assert_called_once_with(live=True)
                library.expandScript.assert_called_once()
                if headless:
                    state.set_status.assert_not_called()
                else:
                    state.set_status.assert_called_with(env, 'Cycle 5 | Processing results', mc_running=False)
                    self.assertEqual(state.set_status.call_count, 2)

    def test_mc_status_is_cleared_on_failure(self):
        from ctypes import create_string_buffer, sizeof

        module = ast.parse(Path(__file__).with_name('morty.py').read_text())
        function = next(node for node in module.body if isinstance(node, ast.FunctionDef)
                        and node.name == 'run_planning_script')
        library = SimpleNamespace(expandScript=Mock(side_effect=RuntimeError('solver failed')))
        status = Mock()
        namespace = {'args': SimpleNamespace(live=True, dryrun=False), 'global_counter': 0,
                     'show_sim_status': status, 'sizeof': sizeof,
                     'morty_script_context': lambda **kwargs: contextlib.nullcontext(library)}
        exec(compile(ast.Module(body=[function], type_ignores=[]), '<planning failure>', 'exec'), namespace)
        with self.assertRaisesRegex(RuntimeError, 'solver failed'):
            namespace['run_planning_script']('script', create_string_buffer(100), 'full plan')
        status.assert_called_with('Cycle 1 | Planning interrupted or failed')

    def test_selected_query_clears_stale_traces_but_preserves_plans(self):
        from ctypes import create_string_buffer, sizeof

        module = ast.parse(Path(__file__).with_name('morty.py').read_text())
        function = next(node for node in module.body if isinstance(node, ast.FunctionDef)
                        and node.name == 'run_planning_script')
        with TemporaryDirectory() as folder:
            prefix = str(Path(folder) / 'generated')
            configs = ['_primary', '_fallback']
            for config in configs:
                package = Path(prefix + config)
                package.mkdir()
                (package / 'debug_trace_array.txt').write_text('stale counterexample')
                (package / 'future_point_2.txt').write_text('stored plan')

            def native_call(script, result, size):
                self.assertEqual(script, b'@{config.json}@.runMCJob[_fallback]')
                for config in configs:
                    self.assertEqual((Path(prefix + config) / 'debug_trace_array.txt').read_text(), '')
                    self.assertEqual((Path(prefix + config) / 'future_point_2.txt').read_text(), 'stored plan')
                return b'fresh result'

            library = SimpleNamespace(expandScript=Mock(side_effect=native_call))
            namespace = {'args': SimpleNamespace(live=True, dryrun=False), 'global_counter': 0,
                         'show_sim_status': Mock(), 'sizeof': sizeof, 'Path': Path,
                         'morty_script_context': lambda **kwargs: contextlib.nullcontext(library),
                         'ucd_config_prios_str': configs, 'generated_path_prefix': prefix}
            exec(compile(ast.Module(body=[function], type_ignores=[]), '<selected query>', 'exec'), namespace)
            result = namespace['run_planning_script']('@{config.json}@.runMCJobs[16]',
                                                     create_string_buffer(100), 'full plan', '_fallback')
            self.assertEqual(result, b'fresh result')
            library.expandScript.assert_called_once()
            with self.assertRaisesRegex(ValueError, 'Invalid selective'):
                namespace['run_planning_script']('@{config.json}@.runMCJobs[16]',
                                                 create_string_buffer(100), 'full plan', '_unknown')
            with self.assertRaisesRegex(ValueError, 'Invalid selective'):
                namespace['run_planning_script']('@{}@.prepareInputForMortyUCD[0,2,12,AUTO]@{config.json}@.runMCJobs[16]',
                                                 create_string_buffer(100), 'full plan', '_fallback')

    def test_adaptive_input_uses_previous_selected_geometry(self):
        module = ast.parse(Path(__file__).with_name('morty.py').read_text())
        cycle = next(node for node in ast.walk(module) if isinstance(node, ast.For)
                     and isinstance(node.target, ast.Name) and node.target.id == 'global_counter')
        assignments = [node for node in cycle.body if isinstance(node, ast.Assign)
                       and any(isinstance(target, ast.Name) and target.id in ('input_trace', 'MC_INPUT_SCRIPT')
                               for target in node.targets)]
        code = compile(ast.Module(body=assignments, type_ignores=[]), '<input geometry>', 'exec')
        for selector in [None, helper.MCConfigSelector(2)]:
            for selected in [None, '_fallback']:
                with self.subTest(adaptive=selector is not None, selected=selected):
                    namespace = {'config_selector': selector, 'selected_config': selected,
                                 'generated_path_prefix': './generated', 'mcinput': 'observations',
                                 'args': SimpleNamespace(heading_adaptation=2.5),
                                 'num_actual_lanes': 2, 'num_technical_lanes': 12}
                    exec(code, namespace)
                    expected = './generated_fallback/debug_trace_array.txt' if selector is not None and selected else 'AUTO'
                    self.assertEqual(namespace['input_trace'], expected)
                    self.assertIn(f'.prepareInputForMortyUCD[2.5, 2, 12, {expected}]', namespace['MC_INPUT_SCRIPT'])

    def test_adaptive_planning_prepares_before_queries_and_recovers_rare_config(self):
        from ctypes import create_string_buffer, sizeof

        module = ast.parse(Path(__file__).with_name('morty.py').read_text())
        function = next(node for node in module.body if isinstance(node, ast.FunctionDef)
                        and node.name == 'run_adaptive_planning')
        for winner in ['last_full', 'last_shortcut', 'none', 'stale']:
            with self.subTest(winner=winner):
                library = SimpleNamespace(expandScript=Mock(return_value=b''))
                queries = []
                targets = {}
                configs = ['first', 'middle', 'last']

                def query(script, result, description, config_name):
                    library.expandScript.assert_called_once()
                    self.assertEqual(library.expandScript.call_args.args[0], b'prepare geometry first')
                    queries.append((config_name, targets[config_name]))
                    success = config_name == 'last' and (
                        (winner == 'last_full' and targets[config_name] == 'original goal')
                        or (winner == 'last_shortcut' and targets[config_name] == 'INVARSPEC !(TRUE far point);'))
                    return b'fresh' if success or winner == 'stale' else b'empty'

                def select(output, priorities, empty, verbose):
                    return ('|2', 2, 'fresh') if output == 'fresh' else ('|X', None, 'empty')

                namespace = {'create_string_buffer': create_string_buffer, 'sizeof': sizeof,
                             'morty_script_context': lambda **kwargs: contextlib.nullcontext(library),
                             'config_selector': helper.MCConfigSelector(3), 'ucd_config_prios_str': configs,
                             'spec_res': 'original goal', 'run_planning_script': query,
                             'write_spec_to_main_smv': lambda name, target: targets.__setitem__(name, target),
                             'postprocess_selected_config': select, 'perf_counter': lambda: 1.0}
                exec(compile(ast.Module(body=[function], type_ignores=[]), '<adaptive planning>', 'exec'), namespace)
                if winner == 'stale':
                    with self.assertRaisesRegex(RuntimeError, 'stale result'):
                        namespace['run_adaptive_planning']('prepare geometry first', 'solve', [], 'empty', None)
                    self.assertEqual(len(queries), 1)
                    continue
                result = namespace['run_adaptive_planning']('prepare geometry first', 'solve',
                                                           ['near point', 'far point'], 'empty', 2)
                if winner == 'none':
                    self.assertEqual(result, ('|X', None, 'empty', None))
                    self.assertEqual(len(queries), 9)
                else:
                    self.assertEqual(result, ('|2', 2, 'fresh', 1 if winner == 'last_shortcut' else None))
                self.assertEqual(len(queries), len(set(queries)))

    def test_adaptive_selector_keeps_every_full_goal_fallback(self):
        selector = helper.MCConfigSelector(16)
        queries = list(selector.queries(0, None))
        self.assertEqual(queries, [(index, None) for index in range(16)])
        selector.record(12, False, True, 0.1)
        queries = list(selector.queries(0, None))
        self.assertEqual(queries[0], (12, None))
        self.assertEqual(set(queries), {(index, None) for index in range(16)})

    def test_adaptive_selector_exhausts_shortcuts_before_reporting_blind(self):
        selector = helper.MCConfigSelector(16)
        queries = list(selector.queries(15, 5))
        self.assertEqual(len(queries), len(set(queries)))
        self.assertEqual(set(queries), {(index, None) for index in range(16)}
                         | {(index, point) for index in range(6) for point in range(15)})
        first_full = next(index for index, query in enumerate(queries) if query[1] is None)
        self.assertLessEqual(first_full, 2)
        self.assertLess(queries.index((15, None)), queries.index((5, 14)))

    def test_adaptive_selector_avoids_slow_repeated_shortcuts(self):
        selector = helper.MCConfigSelector(3)
        selector.record(0, False, True, 0.1)
        for index in range(3):
            selector.record(index, True, False, 5)
        self.assertEqual(next(selector.queries(15, 2)), (0, None))
        selector.record(0, False, False, 10)
        self.assertNotEqual(next(selector.queries(0, None)), (0, None))

    def test_evaluation_policies_preserve_the_requested_candidate_pool(self):
        from .evaluate_mc_selection import EvaluationSelector

        for policy in ['adaptive', 'priority', 'cascade']:
            with self.subTest(policy=policy):
                selector = EvaluationSelector(16, [0, 5, 15], policy, Mock())
                selector.record(0, False, False, 10)
                selector.record(5, False, True, 0.1)
                full = list(selector.queries(0, None))
                self.assertEqual(full[0], (5 if policy == 'adaptive' else 0, None))
                queries = list(selector.queries(4, 5))
                self.assertEqual(len(queries), len(set(queries)))
                self.assertEqual(set(queries), {(index, None) for index in [0, 5, 15]}
                                 | {(index, point) for index in [0, 5] for point in range(4)})
                if policy == 'cascade':
                    self.assertEqual(queries[-3:], [(0, None), (5, None), (15, None)])

    def test_evaluation_does_not_hide_blind_events_or_timeouts(self):
        from .evaluate_mc_selection import summarize_trial

        record = {'blind_steps': 0, 'crashed_steps': 0, 'driver_completed': True,
                  'finished': True, 'success': False, 'solver_seconds': 1.0,
                  'seconds': 1.2, 'solver_jobs': [{}, {}]}
        result = summarize_trial([record], 0, False)
        self.assertEqual(result['outcome'], 'complete')
        self.assertFalse(result['continuous_goal'])
        record.update(blind_steps=1, driver_completed=False, finished=False)
        result = summarize_trial([record], 0, False)
        self.assertEqual(result['outcome'], 'blind')
        self.assertEqual(result['blind_events'], 1)
        self.assertEqual(summarize_trial([record], -9, True)['outcome'], 'timeout')
        self.assertEqual(summarize_trial([], 0, False)['outcome'], 'step_limit')
        self.assertEqual(summarize_trial([], 1, False)['outcome'], 'error')

    def test_evaluation_clearance_uses_rotated_physical_footprints(self):
        import importlib.util
        import math
        from highway_env.vehicle.objects import RoadObject
        from .evaluate_mc_selection import minimum_vehicle_clearance

        if importlib.util.find_spec('shapely') is None:
            self.skipTest('Optional Shapely evaluation dependency is not installed')
        vehicles = [RoadObject(None, [0, 0]), RoadObject(None, [8, 0]),
                    RoadObject(None, [100, 100]), RoadObject(None, [100, 100])]
        for vehicle in vehicles:
            vehicle.LENGTH = 5
            vehicle.WIDTH = 2
        self.assertAlmostEqual(minimum_vehicle_clearance(vehicles, {0, 1})['distance_m'], 3)
        vehicles[1].position[0] = 6
        vehicles[1].heading = math.pi / 2
        result = minimum_vehicle_clearance(vehicles, {0, 1})
        self.assertAlmostEqual(result['distance_m'], 2.5)
        self.assertEqual(result['pair'], [0, 1])
        vehicles[1].position[0] = 0
        self.assertEqual(minimum_vehicle_clearance(vehicles, {0, 1})['distance_m'], 0)

    def test_evaluation_cleans_up_interrupted_and_timed_out_trials(self):
        import signal
        import subprocess
        from . import evaluate_mc_selection as evaluation

        for failure in [subprocess.TimeoutExpired('solver', 1), KeyboardInterrupt()]:
            with self.subTest(failure=type(failure).__name__), TemporaryDirectory() as folder:
                process = Mock(pid=12345, returncode=-9)
                process.poll.return_value = None
                process.communicate.side_effect = [failure, None]
                args = SimpleNamespace(tag='trial', configs=None, steps=1, timeout=1)
                with patch.object(evaluation, 'OUTPUT', Path(folder)), \
                     patch.object(evaluation, 'prepare_trial', side_effect=lambda path, config: path.mkdir()) as prepare, \
                     patch.object(evaluation.subprocess, 'Popen', return_value=process), \
                     patch.object(evaluation.os, 'killpg') as terminate, \
                     contextlib.redirect_stdout(io.StringIO()):
                    if isinstance(failure, KeyboardInterrupt):
                        with self.assertRaises(KeyboardInterrupt):
                            evaluation.run_trial(args, {}, 'adaptive', 0)
                    else:
                        result = evaluation.run_trial(args, {}, 'adaptive', 0)
                        self.assertEqual(result['outcome'], 'timeout')
                    terminate.assert_called_once_with(12345, signal.SIGKILL)
                    self.assertEqual(process.communicate.call_count, 2)
                    template = prepare.call_args.args[1]['#TEMPLATE']
                    self.assertEqual(template['UCD_MC_SELECTION'], 'adaptive')
                    self.assertEqual(template['UCD_EVALUATION_POLICY'], 'adaptive')
                    self.assertEqual(template['UCD_EVALUATION_SEED'], 0)
                    self.assertEqual(template['UCD_EVALUATION_TIMEOUT_SECONDS'], 1)
                    self.assertEqual(template['UCD_EVALUATION_CONFIGS'], '')
                    self.assertEqual(template['UCD_STEPS_PER_RUN'], 1)
                    self.assertEqual(template['UCD_MIN_ITERATIONS'], 0)
                    self.assertEqual(template['UCD_ALLOW_BLIND_STEPS'], 0)

    def test_render_setup_preserves_each_execution_mode(self):
        module = ast.parse(Path(__file__).with_name('morty.py').read_text())
        run_loop = next(node for node in module.body if isinstance(node, ast.For)
                        and ast.unparse(node.target) == '(run_id, seedo)')
        setup = [node for node in [*module.body, *run_loop.body]
                 if (isinstance(node, ast.Assign)
                     and any(isinstance(target, ast.Name) and target.id in ('env', 'bg_image_state')
                             for target in node.targets))
                 or (isinstance(node, ast.If)
                     and any(isinstance(child, ast.Call) and isinstance(child.func, ast.Name)
                             and child.func.id in ('install_vehicle_graphics_patches', 'install_world_surface_patches')
                             for child in ast.walk(node)))]
        for options, render_mode in [(('--live',), None), (('--live', '--force_head'), 'human'),
                                     ((), 'rgb_array'), (('--headless',), 'rgb_array')]:
            with self.subTest(options=options):
                namespace = {
                    'args': self.parse_driver_options(*options),
                    'gymnasium': SimpleNamespace(make=Mock()),
                    'ACCEL_RANGE': 6, 'maxspeed': 70, 'POLICY_FREQUENCY': 2,
                    'nonegos': 10, 'num_actual_lanes': 2, 'viz_state': helper.VizState(),
                    'install_vehicle_graphics_patches': Mock(),
                    'install_world_surface_patches': Mock(),
                }
                exec(compile(ast.Module(body=setup, type_ignores=[]), '<render setup>', 'exec'), namespace)
                kwargs = namespace['gymnasium'].make.call_args.kwargs
                self.assertEqual(kwargs['render_mode'], render_mode)
                self.assertFalse(kwargs['config']['real_time_rendering'])
                self.assertEqual(namespace['bg_image_state'] is not None, render_mode is not None)
                for installer in ['install_vehicle_graphics_patches', 'install_world_surface_patches']:
                    self.assertEqual(namespace[installer].call_count, int(render_mode is not None))

    def test_hidden_trajectories_do_not_collect_drawing_history(self):
        from highway_env.vehicle.graphics import VehicleGraphics

        args = self.parse_driver_options('--live', '--force_head')
        state = helper.VizState()
        vehicle = SimpleNamespace(position=[0, 0], speed=10)
        surface = Mock()
        surface.is_visible.return_value = False
        with patch.object(VehicleGraphics, 'display', classmethod(lambda *args, **kwargs: None)):
            with patch.object(VehicleGraphics, 'get_color'):
                helper.install_vehicle_graphics_patches(args, state)
                for position in range(5):
                    vehicle.position[0] = position
                    VehicleGraphics.display(vehicle, surface)
        self.assertEqual(state.trajectories, {})

    def test_live_viewer_draws_once_per_step_without_changing_physics(self):
        import gymnasium
        import numpy as np

        module = ast.parse(Path(__file__).with_name('morty.py').read_text())
        cycle = next(node for node in ast.walk(module) if isinstance(node, ast.For)
                     and isinstance(node.target, ast.Name) and node.target.id == 'global_counter')
        step_index = next(index for index, node in enumerate(cycle.body)
                          if isinstance(node, ast.Assign) and isinstance(node.value, ast.Call)
                          and ast.unparse(node.value.func) == 'env.step')
        step_code = compile(ast.Module(body=cycle.body[step_index - 1:step_index + 1], type_ignores=[]),
                            '<control step>', 'exec')
        config = {
            'controlled_vehicles': 2, 'vehicles_count': 0, 'lanes_count': 2,
            'simulation_frequency': 60, 'policy_frequency': 2,
            'screen_width': 400, 'screen_height': 160, 'real_time_rendering': False,
            'action': {'type': 'MultiAgentAction', 'action_config': {'type': 'ContinuousAction'}},
        }
        with patch.dict('os.environ', {'SDL_VIDEODRIVER': 'dummy'}):
            plain = gymnasium.make('highway-v0', render_mode=None, config=config)
            visible = gymnasium.make('highway-v0', render_mode='human', config=config)
            try:
                plain.reset(seed=0)
                visible.reset(seed=0)
                namespace = {'args': self.parse_driver_options('--live', '--force_head'),
                             'env': visible, 'action': ([0.1, 0.0], [-0.1, 0.0])}
                viewer = visible.unwrapped.viewer
                with patch.object(viewer, 'display', wraps=viewer.display) as draw:
                    for step in range(3):
                        with self.subTest(step=step):
                            draw.reset_mock()
                            exec(step_code, namespace)
                            plain.step(namespace['action'])
                            draw.assert_called_once()
                            for shown, hidden in zip(visible.unwrapped.controlled_vehicles,
                                                     plain.unwrapped.controlled_vehicles):
                                np.testing.assert_array_equal(shown.position, hidden.position)
                                self.assertEqual(shown.speed, hidden.speed)
                                self.assertEqual(shown.heading, hidden.heading)
                self.assertIsNone(plain.unwrapped.viewer)
            finally:
                visible.close()
                plain.close()

    def test_force_head_rejects_conflicting_modes(self):
        for options in [('--force_head',), ('--live', '--force_head', '--headless')]:
            with self.subTest(options=options), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as failure:
                    self.parse_driver_options(*options)
                self.assertEqual(failure.exception.code, 2)

    def test_both_live_modes_reject_debug_recording_and_replay(self):
        for viewer_options in [(), ('--force_head',)]:
            for debug_option in ['--record_video', '--detailed_archive', '--dryrun']:
                with self.subTest(viewer_options=viewer_options, debug_option=debug_option):
                    with contextlib.redirect_stderr(io.StringIO()):
                        with self.assertRaises(SystemExit) as failure:
                            self.parse_driver_options('--live', *viewer_options, debug_option)
                    self.assertEqual(failure.exception.code, 2)

    def test_driving_engine_is_registered_without_graphics_patches(self):
        module = ast.parse(Path(__file__).with_name('morty.py').read_text())
        imports = [node for node in module.body if isinstance(node, ast.Import)
                   and any(alias.name in ('gymnasium', 'highway_env') for alias in node.names)]
        namespace = {}
        exec(compile(ast.Module(body=imports, type_ignores=[]), '<driving imports>', 'exec'), namespace)
        self.assertIn('highway-v0', namespace['gymnasium'].registry)

    def test_scheduler_keeps_seeds_without_writing_statistics(self):
        with TemporaryDirectory() as folder:
            scheduler = helper.RunScheduler(folder + '/unused', 2, False, persist=False)
            self.assertEqual(next(scheduler), (0, 0))
            self.assertEqual(next(scheduler), (0, 1))
            scheduler.commit()
            self.assertEqual(next(scheduler), (1, 2))
            scheduler.commit()
            self.assertEqual(list(scheduler), [])
            self.assertEqual(list(Path(folder).iterdir()), [])

    def test_experiment_scheduler_still_persists_seeds(self):
        with TemporaryDirectory() as folder:
            scheduler = helper.RunScheduler(folder, 1, False)
            self.assertEqual(next(scheduler), (0, 0))
            scheduler.commit()
            self.assertEqual(helper.load_run_seed_pairs(folder), [(0, 0)])

    def test_live_library_is_loaded_once(self):
        with patch.object(helper.ctypes, 'CDLL', return_value=Mock()) as loader:
            with patch.object(helper, '_live_morty_lib', None):
                with helper.morty_script_context(live=True) as first:
                    pass
                with helper.morty_script_context(live=True) as second:
                    self.assertIs(first, second)
                loader.assert_called_once()
                first.setMortyLiveMode.assert_called_once_with(True)

    def test_old_library_is_rejected(self):
        with patch.object(helper.ctypes, 'CDLL', return_value=SimpleNamespace()):
            with patch.object(helper, '_live_morty_lib', None):
                with self.assertRaisesRegex(RuntimeError, 'rebuilding'):
                    with helper.morty_script_context(live=True):
                        self.fail('An old library must not silently run in live mode')

    def test_priority_selection_is_unchanged_and_silent(self):
        module = ast.parse(Path(__file__).with_name('morty.py').read_text())
        function = next(node for node in module.body
                        if isinstance(node, ast.FunctionDef)
                        and node.name == 'postprocess_selected_config')
        namespace = {}
        exec(compile(ast.Module(body=[function], type_ignores=[]), '<selection>', 'exec'), namespace)
        select = namespace['postprocess_selected_config']
        cases = [
            ('fallback:0|;\nprimary:|;', ('|1', 1, '0|;')),
            ('fallback:0|;\nprimary:FINISHED', ('|0', 0, 'FINISHED')),
            ('fallback:|;\nprimary:|;', ('|X', None, '|;')),
        ]
        for results, expected in cases:
            with self.subTest(results=results):
                with contextlib.redirect_stdout(io.StringIO()) as output:
                    self.assertEqual(select(results, ['primary', 'fallback'], '|;', verbose=False), expected)
                self.assertEqual(output.getvalue(), '')

    def test_live_plotting_never_accesses_plot_data(self):
        module = ast.parse(Path(__file__).with_name('morty.py').read_text())
        blocks = [node for node in ast.walk(module)
                  if isinstance(node, ast.If) and ast.unparse(node.test) == 'not args.live'
                  and any(isinstance(child, ast.Call) and isinstance(child.func, ast.Name)
                          and child.func.id.startswith('plot_') for child in ast.walk(node))]
        self.assertEqual(len(blocks), 2)
        for force_head in [False, True]:
            for block in blocks:
                exec(compile(ast.Module(body=[block], type_ignores=[]), '<plots>', 'exec'),
                     {'args': SimpleNamespace(live=True, force_head=force_head)})


if __name__ == '__main__':
    unittest.main()