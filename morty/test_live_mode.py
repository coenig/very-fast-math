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
        for block in blocks:
            exec(compile(ast.Module(body=[block], type_ignores=[]), '<plots>', 'exec'),
                 {'args': SimpleNamespace(live=True)})


if __name__ == '__main__':
    unittest.main()