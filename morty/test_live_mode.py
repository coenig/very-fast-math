import ast
import contextlib
import io
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from . import morty_helper as helper


class LiveModeTests(unittest.TestCase):
    def parse_driver_options(self, *options):
        module = ast.parse(Path(__file__).with_name('morty.py').read_text())
        plotting_start = next(index for index, node in enumerate(module.body)
                              if isinstance(node, ast.If) and ast.unparse(node.test) == 'not args.live')
        namespace = {'__package__': 'morty'}
        with patch('sys.argv', ['morty', *options]), patch('shutil.which', return_value='/usr/bin/timeout'):
            exec(compile(ast.Module(body=module.body[:plotting_start], type_ignores=[]),
                         '<driver options>', 'exec'), namespace)
        return namespace['args']

    def test_plain_live_mode_remains_headless(self):
        for options in [('--live',), ('--live', '--headless')]:
            with self.subTest(options=options):
                args = self.parse_driver_options(*options)
                self.assertTrue(args.headless)
                self.assertFalse(args.force_head)

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