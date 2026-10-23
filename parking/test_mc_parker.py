import ctypes
import io
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
from types import MethodType, SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import mc_parker as parking


class ProcessCleanupTests(unittest.TestCase):
    @unittest.skipUnless(sys.platform == "linux", "Linux process-group regression")
    def test_stopping_synthesis_reaps_detached_worker(self):
        libc = ctypes.CDLL(None)
        previous_subreaper = ctypes.c_int()
        self.assertEqual(libc.prctl(37, ctypes.byref(previous_subreaper), 0, 0, 0), 0)
        self.assertEqual(libc.prctl(36, 1, 0, 0, 0), 0)
        controller, worker_pid = None, None
        try:
            with tempfile.TemporaryDirectory(prefix="vfm-cancel-") as directory:
                package = Path(directory) / "package"
                package.mkdir()
                endpoint = str(Path(directory) / "worker.sock")
                with socket.socket(socket.AF_UNIX) as listener:
                    listener.bind(endpoint)
                    listener.listen(1)
                    listener.settimeout(10)
                    controller = subprocess.Popen(
                        [sys.executable, __file__, "__controller__", str(package)],
                        env=dict(os.environ, VFM_TEST_SOCKET=endpoint), start_new_session=True,
                        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    connection, unused_address = listener.accept()
                    with connection:
                        connection.settimeout(5)
                        worker_pid = int(connection.recv(64))
                        log = io.StringIO()
                        window = SimpleNamespace(_synth_proc=controller, _synth_log_file=log,
                                                 synth_timer=mock.Mock())
                        parking.MainWindow._stop_synth(window)
                        self.assertIsNone(window._synth_proc)
                        self.assertTrue(log.closed)
                        self.assertEqual(controller.returncode, 128 + signal.SIGTERM)
                        self.assertEqual(connection.recv(1), b"")
                        with self.assertRaises(ProcessLookupError):
                            os.kill(worker_pid, 0)
                        worker_pid = None
        finally:
            parking._kill_process_group(controller)
            if worker_pid is not None:
                try:
                    os.killpg(worker_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                try:
                    os.waitpid(worker_pid, 0)
                except ChildProcessError:
                    pass
            libc.prctl(36, previous_subreaper.value, 0, 0, 0)

    @unittest.skipUnless(os.name == "posix", "POSIX process groups")
    def test_graceful_cleanup_has_forced_fallback(self):
        process = mock.Mock(pid=123)
        process.wait.side_effect = [subprocess.TimeoutExpired("worker", 5), 0]
        with mock.patch.object(parking.platform, "system", return_value="Linux"), \
                mock.patch.object(parking.os, "getpgid", return_value=456), \
                mock.patch.object(parking.os, "killpg", create=True) as kill:
            parking._kill_process_group(process, graceful=True)
        self.assertEqual(kill.call_args_list,
                         [mock.call(456, signal.SIGTERM), mock.call(456, signal.SIGKILL)])

    def test_failed_launch_restores_termination_handler(self):
        handler = signal.getsignal(signal.SIGTERM)
        with tempfile.TemporaryDirectory() as package, \
                mock.patch.object(parking.subprocess, "Popen", side_effect=OSError("launch failed")):
            with self.assertRaises(OSError):
                parking.race_first_solution(packages=[package])
        self.assertIs(signal.getsignal(signal.SIGTERM), handler)

    def test_termination_during_launch_still_cleans_up_the_worker(self):
        process = mock.Mock()
        handler = signal.getsignal(signal.SIGTERM)

        def launch(*args, **kwargs):
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
            return process

        with tempfile.TemporaryDirectory() as package, \
                mock.patch.object(parking.subprocess, "Popen", side_effect=launch), \
                mock.patch.object(parking, "_kill_process_group") as cleanup:
            with self.assertRaises(SystemExit) as stopped:
                parking.race_first_solution(packages=[package])
            self.assertEqual(stopped.exception.code, 128 + signal.SIGTERM)
            cleanup.assert_called_once_with(process)
        self.assertIs(signal.getsignal(signal.SIGTERM), handler)


class ObstacleEditingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = parking.QApplication.instance() or parking.QApplication([])

    def setUp(self):
        self.window = SimpleNamespace(
            scene=parking.QGraphicsScene(), rect_items=[], section_item=None,
            _mark_obstacles_dirty=mock.Mock(), _refresh_grid_status=mock.Mock(),
            _refresh_gran_status=mock.Mock(), _apply_visu_mode=mock.Mock(),
            _place_ego_at_start=mock.Mock(), play_timer=mock.Mock(), _race_proc=None,
            _gran_baseline=None, _grid_baseline=None, _gran_dirty=False, _grid_dirty=False,
            _section_dirty=False, _transform=(0, 0, 1))
        for name in ["add_rectangle", "_remove_obstacle", "collect_obstacle_bases_world",
                     "_capture_baseline", "reset_scene", "_write_config_from_scene"]:
            setattr(self.window, name, MethodType(getattr(parking.MainWindow, name), self.window))

        def corners():
            rectangles = [item.mapRectToScene(item.rect()) for item in self.window.rect_items]
            return [(rect.left(), rect.top(), rect.right(), rect.bottom()) for rect in rectangles]

        self.window.collect_obstacles_world = corners
        self.window.add_rectangle(1, 1, 3, 3, obstacle_offset=(1,) * 4,
                      obstacle_ranges={"RECT_OBSTACLES_SHIFT_MIN_Xs": "1",
                               "RECT_OBSTACLES_SHIFT_MAX_Xs": "3"})
        self.window.add_rectangle(15, 15, 17, 17, obstacle_offset=(5,) * 4,
                      obstacle_ranges={"RECT_OBSTACLES_SHIFT_MIN_Xs": "5",
                               "RECT_OBSTACLES_SHIFT_MAX_Xs": "7"})
        self.window._capture_baseline()

    def test_deletion_preserves_remaining_offsets(self):
        self.window._remove_obstacle(self.window.rect_items[0])
        self.assertEqual(self.window.collect_obstacle_bases_world(), [(10, 10, 12, 12)])

    def test_added_obstacle_does_not_inherit_a_deleted_offset(self):
        self.window._remove_obstacle(self.window.rect_items[0])
        self.window.add_rectangle(20, 20, 22, 22)
        self.assertEqual(self.window.collect_obstacle_bases_world(), [(10, 10, 12, 12), (20, 20, 22, 22)])
        self.assertEqual(self.window.rect_items[-1].obstacle_ranges, {})

    def test_reset_restores_offsets_and_ranges(self):
        self.window._remove_obstacle(self.window.rect_items[0])
        self.window.add_rectangle(20, 20, 22, 22)
        self.window.reset_scene()
        self.assertEqual(self.window.collect_obstacle_bases_world(), [(0, 0, 2, 2), (10, 10, 12, 12)])
        self.assertEqual([item.obstacle_ranges["RECT_OBSTACLES_SHIFT_MIN_Xs"]
                  for item in self.window.rect_items], ["1", "5"])

    def test_configuration_ranges_follow_items_across_repeated_writes(self):
        configuration = {
            "RECT_OBSTACLES_TL_Xs": "@(0)@@(10)@", "RECT_OBSTACLES_TL_Ys": "@(0)@@(10)@",
            "RECT_OBSTACLES_BR_Xs": "@(2)@@(12)@", "RECT_OBSTACLES_BR_Ys": "@(2)@@(12)@",
            "RECT_OBSTACLES_SHIFT_MIN_Xs": "@(1)@@(5)@", "RECT_OBSTACLES_SHIFT_MAX_Xs": "@(3)@@(7)@"}
        self.window._remove_obstacle(self.window.rect_items[0])
        self.window.add_rectangle(20, 20, 22, 22)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text(json.dumps({"#TEMPLATE": configuration}))
            with mock.patch.object(parking, "TPL_PATH", str(path)):
                for unused_iteration in range(2):
                    self.window._write_config_from_scene()
                    result = json.loads(path.read_text())["#TEMPLATE"]
                    self.assertEqual(result["RECT_OBSTACLES_TL_Xs"], "@(10)@@(20)@")
                    self.assertEqual(result["RECT_OBSTACLES_SHIFT_MIN_Xs"], "@(5)@@(0)@")
                    self.assertEqual(result["RECT_OBSTACLES_SHIFT_MAX_Xs"], "@(7)@@(0)@")
                self.window.reset_scene()
                self.window._write_config_from_scene()
                result = json.loads(path.read_text())["#TEMPLATE"]
                self.assertEqual(result["RECT_OBSTACLES_TL_Xs"], "@(0)@@(10)@")
                self.assertEqual(result["RECT_OBSTACLES_SHIFT_MIN_Xs"], "@(1)@@(5)@")
                self.assertEqual(result["RECT_OBSTACLES_SHIFT_MAX_Xs"], "@(3)@@(7)@")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "__worker__":
        with socket.socket(socket.AF_UNIX) as connection:
            connection.connect(os.environ["VFM_TEST_SOCKET"])
            connection.sendall(str(os.getpid()).encode())
            connection.recv(1)
    elif len(sys.argv) > 1 and sys.argv[1] == "__controller__":
        parking._MC_WORKER = str(Path(__file__).resolve())
        parking.race_first_solution(packages=[sys.argv[2]], mc_script="__worker__")
    else:
        unittest.main()