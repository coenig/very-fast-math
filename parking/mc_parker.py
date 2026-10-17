import sys
import re
import json
import math
import os
import platform
import shutil
import signal
import subprocess
import time
import ctypes
import ctypes.util
import heapq
from bisect import bisect_right
from contextlib import contextmanager
from ctypes import create_string_buffer, sizeof
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QGraphicsView, QGraphicsScene, QGraphicsRectItem,
                             QGraphicsEllipseItem, QGraphicsSimpleTextItem, QMessageBox, QProgressBar,
                             QGroupBox, QLabel, QSpinBox, QToolButton, QFileDialog)
from PyQt6.QtCore import Qt, QRectF, QTimer, QPointF, QObject, QThread, QUrl, pyqtSignal
from PyQt6.QtGui import (QPixmap, QImage, QBrush, QPen, QColor, QPainterPath, QPolygonF, QFont,
                         QDesktopServices, QTransform, QPainter)


# Repo root = parent of this parking/ folder; anchors all paths independent of the caller's cwd.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# --- Fixed locations for the parking config (see EnvModel generation into examples/gp_config). ---
TRACE_PATH = os.path.join(REPO_ROOT, "examples/gp_config/debug_trace_array.txt")
SMV_PATH = os.path.join(REPO_ROOT, "examples/gp_config/EnvModel.smv")
IMAGE_PATH = os.path.join(REPO_ROOT, "examples/gp_config/0/preview2/preview2_0.png")
PROGRESS_PATH = os.path.join(REPO_ROOT, "examples/gp")
# The config template; section-1 pose and obstacle positions are written here before regeneration.
TPL_PATH = os.path.join(REPO_ROOT, "src/templates/envmodel_config.tpl.json")
# Re-runs the model checker on the already-generated EnvModel.smv (no tpl.json regeneration).
# Path is relative to bin/, the working directory the MC runs in (see run_model_checker).
MC_SCRIPT = "@{../src/templates/envmodel_config.tpl.json}@.runMCJobs[16]"
# Regenerates EnvModel.smv/main.smv from the config; required after any config change.
ENVGEN_SCRIPT = "@{../src/templates/envmodel_config.tpl.json}@.generateEnvmodels"

# Ego sprite (639x258 px, rear bumper at x=0, front to +x); dimensions from EGO_* in activation_pose.py.
EGO_CAR_IMAGE = os.path.join(REPO_ROOT, "parking", "ego_car.png")
EGO_LENGTH_M = 4.53
EGO_ACCEL_MPS2 = 1.5
EGO_MAX_SPEED_MPS = 4.0
EGO_GEAR_CHANGE_PAUSE_S = 0.5
GIF_FPS = 20
GIF_MAX_WIDTH_PX = 1000
GIF_FINAL_HOLD_S = 1.5
# Drawn parking bays span at least this length, centred on the section middle (= car centre), so a
# short start section (quick turn-out) still shows a full-size slot.
PARKING_SLOT_LENGTH_M = 7.0


def slot_extent(section_length, slot_length):
    """(start, end) of a bay centred on a section's middle, measured along it from its source."""
    span = max(section_length, slot_length)
    return (section_length - span) / 2.0, (section_length + span) / 2.0

# runMCJobs model-checks every examples/gp* package (except the bare 'gp' prefix folder). With a
# >1 range on a '#'-variable in the tpl.json, generateEnvmodels emits one such package per value
# (cross-product for several ranges), so a single MC run races multiple configs at once.
EXAMPLES_DIR = os.path.join(REPO_ROOT, "examples")
_PACKAGE_PREFIX = "gp"
_MC_WORKER = os.path.join(REPO_ROOT, "parking", "_mc_worker.py")
_SYNTH_SCRIPT = os.path.join(REPO_ROOT, "parking", "synth_obstacles.py")
SYNTH_HISTORY_DIR = os.path.join(EXAMPLES_DIR, "synth_history")



class EnvModelWorker(QObject):
    finished = pyqtSignal(str)

    def run(self):
        result = create_string_buffer(1000000)
        prev_cwd = os.getcwd()
        os.chdir(os.path.join(REPO_ROOT, 'bin'))
        try:
            with _vfm_lib_context() as lib:
                res = lib.expandScript(ENVGEN_SCRIPT.encode('utf-8'), result, sizeof(result))
        finally:
            os.chdir(prev_cwd)
        self.finished.emit(res.decode('utf-8'))


def discover_mc_packages():
    """Folders runMCJobs will model-check: examples/gp* dirs, excluding the bare prefix dir."""
    if not os.path.isdir(EXAMPLES_DIR):
        return []
    packages = []
    for name in sorted(os.listdir(EXAMPLES_DIR)):
        full = os.path.join(EXAMPLES_DIR, name)
        if os.path.isdir(full) and name.startswith(_PACKAGE_PREFIX) and name != _PACKAGE_PREFIX:
            packages.append(full)
    return packages


def remove_mc_packages():
    """Delete every generated gp* working folder so a regen cannot leave obsolete variants behind."""
    removed = 0
    for pkg in discover_mc_packages():
        try:
            shutil.rmtree(pkg)
            removed += 1
        except OSError:
            pass
    return removed


def package_trace_path(package_dir):
    return os.path.join(package_dir, "debug_trace_array.txt")
def package_image_path(package_dir):
    return os.path.join(package_dir, "0", "preview2", "preview2_0.png")


def clear_package_trace(package_dir):
    """Delete a package's leftover debug_trace_array.txt so its content reflects THIS run only.

    debug_trace_array.txt is nuXmv's output log: a completed check always (re)writes it, with the
    counterexample state list on a CEX or only "no counterexample found ... up to N" lines on a
    blind (no-CEX) result. But runMCJobs runs with delete_old_output=false, so if a check is
    KILLED before nuXmv runs (as the race does to losers) the previous run's log lingers untouched.
    Removing it up front means a present file is necessarily from this run: CEX content => winner,
    blind content => dropout, absent => never actually checked (no stale result can masquerade).
    """
    try:
        os.remove(package_trace_path(package_dir))
    except FileNotFoundError:
        pass


def select_startup_package():
    """Pick which variant folder the GUI opens on launch.

    A first-solution run may model-check several configs at once (one gp* package each). The
    winner is the package whose trace holds a counterexample; the newest such trace wins (older
    leftovers from earlier runs carry older timestamps). The preview image is optional -- the
    roads are drawn from the trace itself -- so selection keys off the trace, not the preview.
    Falls back to any package with a readable trace, then to the base gp_config.
    """
    candidates = discover_mc_packages()
    base = os.path.join(EXAMPLES_DIR, "gp_config")
    if os.path.isdir(base) and base not in candidates:
        candidates.append(base)

    solved = [p for p in candidates if package_has_fresh_counterexample(p)]
    if solved:
        return max(solved, key=lambda p: os.path.getmtime(package_trace_path(p)))

    with_trace = [p for p in candidates if os.path.isfile(package_trace_path(p))]
    if with_trace:
        return max(with_trace, key=lambda p: os.path.getmtime(package_trace_path(p)))

    return base


def _kill_process_group(proc):
    """Kill the worker and its nuXmv children (its own process group / job tree), then reap it.

    Always targets the whole group, even if the worker itself has already exited: nuXmv
    grandchildren keep the worker's process-group id after being reparented, so killing the
    group is what actually reaps the lingering model-checker jobs of the losing variants.
    """
    if proc is None:
        return
    try:
        if platform.system() == 'Windows':
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            # getpgid fails once the worker is fully reaped; the leader pid is still the pgid.
            try:
                pgid = os.getpgid(proc.pid)
            except (ProcessLookupError, OSError):
                pgid = proc.pid
            os.killpg(pgid, signal.SIGKILL)
    except (ProcessLookupError, OSError):
        pass
    # Reap the killed worker so it doesn't linger as a zombie across many races.
    try:
        proc.wait(timeout=5)
    except (subprocess.TimeoutExpired, OSError, ValueError):
        pass


def race_first_solution(poll_interval=1.0, packages=None, mc_script=MC_SCRIPT):
    """Model-check all configured variants and stop at the FIRST complete counterexample.

    Runs the worker detached from the terminal (stdin=/dev/null, own session) so the nuXmv
    instances can never grab the tty and drop you into a shared interactive prompt, polls the
    variant folders, then kills the whole job group the moment one variant has a fresh
    counterexample. Returns the winning package dir, or None if none solved.
    """
    if packages is None:
        packages = discover_mc_packages()
    if not packages:
        print("No 'gp' packages found. Run EnvModel generation first.")
        return None

    # Clear leftover traces so a package can only win by producing a CEX in THIS run.
    for p in packages:
        clear_package_trace(p)

    print(f"Racing {len(packages)} config(s); first counterexample wins...")
    proc = subprocess.Popen(
        [sys.executable, _MC_WORKER, mc_script],
        cwd=REPO_ROOT, start_new_session=True,
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def winner():
        # A win requires a FRESH counterexample (trace newer than the just-built main.smv);
        # a leftover CEX from a previous model must not end the race.
        for p in packages:
            if package_has_fresh_counterexample(p):
                return p
        return None

    win = None
    try:
        while True:
            win = winner()
            if win:
                break
            if proc.poll() is not None:  # worker finished
                win = winner()
                break
            time.sleep(poll_interval)
    except KeyboardInterrupt:
        print("\nInterrupted; stopping all configs...")
    finally:
        _kill_process_group(proc)

    if win:
        print(f"Winner: {os.path.basename(win)} (remaining configs killed).")
    else:
        print("No variant produced a counterexample.")
    return win


_SCALING_FACTOR_RE = re.compile(r"-- (TIME|DISTANCE|GEOMETRY)SCALING\(\(\(\((-?[\d.]+)\)\)\)\)")
_SCALED_VAR_RE = re.compile(r"^--\s*([\w.]+),(\w+)\s*$")
_scaling_cache = {}


def read_scaling(smv_path):
    """Return ({'TIME'|'DISTANCE'|'GEOMETRY': factor}, {var: scale type}) from a generated EnvModel.smv.

    Mirrors vfm's ScaleDescription; factors default to 1 and the var map is empty if absent.
    """
    factors = {"TIME": 1.0, "DISTANCE": 1.0, "GEOMETRY": 1.0}
    types = {}
    try:
        mtime = os.path.getmtime(smv_path)
        if _scaling_cache.get(smv_path, (None,))[0] == mtime:
            return _scaling_cache[smv_path][1]
        with open(smv_path, "r") as f:
            content = f.read()
    except OSError:
        return factors, types
    for kind, value in _SCALING_FACTOR_RE.findall(content):
        factors[kind] = float(value)
    begin = content.find("-- SCALING DESCRIPTIONS")
    end = content.find("-- EO SCALING DESCRIPTIONS", begin)
    if begin >= 0 and end >= 0:
        for line in content[begin:end].splitlines()[1:]:
            m = _SCALED_VAR_RE.match(line.strip())
            if m:
                types[m.group(1)] = m.group(2)
    _scaling_cache[smv_path] = (mtime, (factors, types))
    return factors, types


def _model_to_world(value, scale_type, factors):
    """EnvModel -> world conversion, identical to StaticHelper::applyTimescaling."""
    t, d, g = factors["TIME"], factors["DISTANCE"], factors["GEOMETRY"]
    return {
        "time": lambda v: v / t,
        "distance": lambda v: v / d,
        "velocity": lambda v: v * t / d,
        "acceleration": lambda v: v * t * t / d,
        "geometry": lambda v: v / g,
    }.get(scale_type, lambda v: v)(value)


def _read_trace_values(trace_path):
    """Parse a debug_trace_array.txt dump into a {name: float} map in world units.

    Names keep whatever prefix they have in the file (e.g. 'env.section_0.source.x'). Values are
    scaled back via the scaling header of the sibling EnvModel.smv (if present).
    """
    values = {}
    line_re = re.compile(r'^\s*([A-Za-z_][\w.]*)\s*=\s*(-?\d+(?:\.\d+)?)\s*$')
    if not os.path.isfile(trace_path):
        return values
    factors, types = read_scaling(os.path.join(os.path.dirname(trace_path), "EnvModel.smv"))
    with open(trace_path, "r") as f:
        for line in f:
            m = line_re.match(line)
            if m:
                name = m.group(1)
                scale_type = types.get(name[4:] if name.startswith("env.") else name)
                values[name] = _model_to_world(float(m.group(2)), scale_type, factors)
    return values


def _config_values(tpl_path):
    """Trace-like {name: value} map of the fixed sections and obstacles in the tpl.json."""
    try:
        with open(tpl_path) as f:
            content = f.read()
    except OSError:
        return {}

    def numbers(key):
        m = re.search(r'"' + key + r'"\s*:\s*"([^"]*)"', content)
        out = []
        for v in _parse_paren_list(m.group(1)) if m else []:
            try:
                out.append(float(v))
            except ValueError:
                out.append(None)
        return out

    values = {}
    for sec, x, y, angle, length in zip(
            numbers("FIXED_SECTION_IDs"), numbers("FIXED_SECTION_SOURCE_Xs"),
            numbers("FIXED_SECTION_SOURCE_Ys"), numbers("FIXED_SECTION_ANGLEs"),
            numbers("FIXED_SECTION_LENGTHs")):
        if None in (sec, x, y, angle, length):
            continue
        sec = int(sec)
        values[f"section_{sec}.source.x"] = x
        values[f"section_{sec}.source.y"] = y
        values[f"section_{sec}.angle"] = angle
        values[f"section_{sec}_end"] = length
    for k, corners in enumerate(zip(
            numbers("RECT_OBSTACLES_TL_Xs"), numbers("RECT_OBSTACLES_TL_Ys"),
            numbers("RECT_OBSTACLES_BR_Xs"), numbers("RECT_OBSTACLES_BR_Ys"))):
        for field, value in zip(("tl_x", "tl_y", "br_x", "br_y"), corners):
            values[f"rect_obstacles_{field}_{k}"] = value
    return values


def _trace_getter(trace_path):
    """Return a getter accepting both 'env.'-prefixed (dump) and bare (MC trace) names.

    Without solution geometry in the trace (missing, wiped by regeneration, or blind), the
    getter serves the configured fixed sections and obstacles instead (get.from_config).
    """
    v = _read_trace_values(trace_path)
    from_config = "section_0.source.x" not in v and "env.section_0.source.x" not in v
    if from_config:
        v = _config_values(TPL_PATH)

    def get(name):
        if name in v:
            return v[name]
        return v.get("env." + name)

    get.from_config = from_config
    return get


def _ego_connected_sections(get):
    """Return the set of section indices reachable from the ego's section.

    C++ getBoundingBox(false) only spans the road graph connected to the ego via
    applyToMeAndAllMySuccessorsAndPredecessors, so disconnected sections (e.g. a
    separate parking spot with no car crossing towards it) are excluded from the fit.
    Connections are directed edges section_N -> outgoing_connection_c_of_section_N;
    traversal is undirected (successors and predecessors).
    """
    num_sections = 0
    while get(f"section_{num_sections}.source.x") is not None:
        num_sections += 1
    # The config has no connections yet, so show every configured section.
    if getattr(get, "from_config", False):
        return set(range(num_sections))
    if num_sections == 0:
        return set()

    adjacency = {i: set() for i in range(num_sections)}
    for src in range(num_sections):
        c = 0
        while True:
            target = get(f"outgoing_connection_{c}_of_section_{src}")
            if target is None:
                break
            t = int(target)
            if 0 <= t < num_sections:
                adjacency[src].add(t)
                adjacency[t].add(src)
            c += 1

    ego = get("ego.on_section")
    ego_section = int(ego) if ego is not None and 0 <= int(ego) < num_sections else 0

    component = set()
    stack = [ego_section]
    while stack:
        node = stack.pop()
        if node in component:
            continue
        component.add(node)
        stack.extend(adjacency[node] - component)
    return component


def compute_fit_transform(trace_path, image_width, image_height):
    """Derive the world<->pixel mapping of the C++ 'fit_to_roads' birdseye painter.

    The road-graph bounding box (source+drain centerlines of the sections connected to
    the ego, ghosts excluded) is centered in the canvas at a uniform scale 'ppm' (pixels
    per meter), no axis flip (env2d_simple.h getBirdseyeView / highway_image.cpp).
    Returns (center_x, center_y, ppm).
    """
    get = _trace_getter(trace_path)

    component = _ego_connected_sections(get)
    points = []
    for sec in sorted(component):
        sx = get(f"section_{sec}.source.x")
        sy = get(f"section_{sec}.source.y")
        angle_deg = get(f"section_{sec}.angle") or 0.0
        length = get(f"section_{sec}_end") or 0.0
        angle = 2.0 * math.pi * angle_deg / 360.0
        # Origin and drain point, exactly as RoadGraph::getDrainPoint() computes it.
        points.append((sx, sy))
        points.append((sx + length * math.cos(angle), sy + length * math.sin(angle)))

    if not points:
        return None

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0

    # Road-graph lane width is stored in cm in the trace (see mc_trajectory_to_gif.cpp).
    lane_width = (get("lane_width") or 400.0) / 100.0
    num_lanes = get("num_lanes") or 1.0

    # Mirrors highway_image.cpp fit_to_roads: content = max(1, bb) + 2*lateral_margin,
    # ppm fits that (plus 10% padding) into the fixed canvas getWidth()/getHeight().
    lateral_margin = num_lanes * lane_width / 2.0 + lane_width
    content_w = max(1.0, max_x - min_x) + 2.0 * lateral_margin
    content_h = max(1.0, max_y - min_y) + 2.0 * lateral_margin
    padding_factor = 1.10
    ppm = min(image_width / (content_w * padding_factor),
              image_height / (content_h * padding_factor))

    return (center_x, center_y, ppm)


def world_to_pixel(wx, wy, transform, image_width, image_height):
    center_x, center_y, ppm = transform
    return ((wx - center_x) * ppm + image_width / 2.0,
            (wy - center_y) * ppm + image_height / 2.0)


def pixel_to_world(px, py, transform, image_width, image_height):
    center_x, center_y, ppm = transform
    return ((px - image_width / 2.0) / ppm + center_x,
            (py - image_height / 2.0) / ppm + center_y)


def _snap_scene_to_world_raster(px, py, transform, image_width, image_height, step=1.0):
    """Snap a scene/pixel point onto the world raster (step in metres) the model quantises to."""
    wx, wy = pixel_to_world(px, py, transform, image_width, image_height)
    return world_to_pixel(round(wx / step) * step, round(wy / step) * step,
                          transform, image_width, image_height)


# Set by MainWindow once the world<->pixel transform for the current scene is known, so the
# draggable items can snap their pose onto the model's world raster as the target-section angle
# snaps to its granularity. None (identity) until a transform is available.
_scene_raster_snap = None  # callable(scene_x, scene_y) -> (scene_x, scene_y)


def snap_scene_point(px, py):
    """Snap a scene point onto the active world raster (identity if no transform is set)."""
    if _scene_raster_snap is None:
        return px, py
    return _scene_raster_snap(px, py)


def world_raster_step(smv_path):
    """World raster (m) of the model's GEOMETRYSCALING grid; 1 m without such a header."""
    return 1.0 / read_scaling(smv_path)[0]["GEOMETRY"]


def snap_world(value, step):
    """Round a world coordinate (m) onto the raster; the inner round() drops float noise."""
    return round(round(value / step) * step, 6)


def _fmt_world(value):
    """Compact tpl.json number: '3' instead of '3.0', '2.25' stays."""
    return f"{value:g}"


def read_obstacles_world(trace_path):
    """Return obstacles as [(tl_x, tl_y, br_x, br_y), ...] in world coords, by index order."""
    get = _trace_getter(trace_path)
    obstacles = []
    obs = 0
    while get(f"rect_obstacles_tl_x_{obs}") is not None:
        obstacles.append((
            get(f"rect_obstacles_tl_x_{obs}"), get(f"rect_obstacles_tl_y_{obs}"),
            get(f"rect_obstacles_br_x_{obs}"), get(f"rect_obstacles_br_y_{obs}"),
        ))
        obs += 1
    return obstacles


def obstacles_to_pixel_rects(trace_path, image_width, image_height):
    """Read obstacles from an MC trace and map them into preview2 pixel rectangles.

    Returns a list of (x1, y1, x2, y2) pixel tuples, one per obstacle.
    """
    transform = compute_fit_transform(trace_path, image_width, image_height)
    if transform is None:
        return []

    rects = []
    for tl_x, tl_y, br_x, br_y in read_obstacles_world(trace_path):
        tl = world_to_pixel(tl_x, tl_y, transform, image_width, image_height)
        br = world_to_pixel(br_x, br_y, transform, image_width, image_height)
        rects.append((tl[0], tl[1], br[0], br[1]))
    return rects


def road_segments_pixel(trace_path, image_width, image_height):
    """Ego-connected road sections as pixel segments [(x1, y1, x2, y2, sec, is_ego), ...].

    Uses the SAME fit transform as the obstacles, so the drawn roads and the obstacle frames
    are guaranteed to line up (the cached C++ preview is a random sample layout, not this
    counterexample, and the birdseye renderer crashes on reachability-only traces).
    """
    transform = compute_fit_transform(trace_path, image_width, image_height)
    if transform is None:
        return []

    get = _trace_getter(trace_path)
    ego = get("ego.on_section")
    ego_section = int(ego) if ego is not None else 0

    segments = []
    for sec in sorted(_ego_connected_sections(get)):
        sx = get(f"section_{sec}.source.x")
        sy = get(f"section_{sec}.source.y")
        if sx is None or sy is None:
            continue
        angle = 2.0 * math.pi * (get(f"section_{sec}.angle") or 0.0) / 360.0
        length = get(f"section_{sec}_end") or 0.0
        p1 = world_to_pixel(sx, sy, transform, image_width, image_height)
        p2 = world_to_pixel(sx + length * math.cos(angle), sy + length * math.sin(angle),
                            transform, image_width, image_height)
        segments.append((p1[0], p1[1], p2[0], p2[1], sec, sec == ego_section))
    return segments


def target_section_from_smv(smv_path):
    """Target section index from 'is_target_reachable := reach_..._of_sec_N;' (model uses sec 1)."""
    try:
        with open(smv_path, "r") as f:
            content = f.read()
    except OSError:
        return 1
    m = re.search(r"is_target_reachable\s*:=\s*reach_\d+_of_sec_(\d+)", content)
    return int(m.group(1)) if m else 1


def _cubic_bezier(t, p0, p1, p2, p3):
    mt = 1.0 - t
    a, b, c, d = mt * mt * mt, 3 * mt * mt * t, 3 * mt * t * t, t * t * t
    return (a * p0[0] + b * p1[0] + c * p2[0] + d * p3[0],
            a * p0[1] + b * p1[1] + c * p2[1] + d * p3[1])


def _section_geom(get, sec):
    """(source, drain) of a section in world coords, as RoadGraph::getDrainPoint() computes it."""
    sx = get(f"section_{sec}.source.x")
    sy = get(f"section_{sec}.source.y")
    if sx is None or sy is None:
        return None
    angle = 2.0 * math.pi * (get(f"section_{sec}.angle") or 0.0) / 360.0
    length = get(f"section_{sec}_end") or 0.0
    return (sx, sy), (sx + length * math.cos(angle), sy + length * math.sin(angle))


def _connection_bezier(src_geom, tgt_geom):
    """Control points (p0, p1, p2, p3) of the connection arc src drain -> tgt source, or None."""
    (osx, osy), odrain = src_geom
    (tsx, tsy), tdrain = tgt_geom
    p0, p3 = odrain, (tsx, tsy)
    dist = math.hypot(p3[0] - p0[0], p3[1] - p0[1])
    if dist < 1e-6:
        return None
    # dir_mine = drain - origin (of src); dir_succ = origin - drain (of tgt); both len dist/3.
    dm = (odrain[0] - osx, odrain[1] - osy)
    dm_len = math.hypot(*dm) or 1.0
    dm = (dm[0] / dm_len * dist / 3.0, dm[1] / dm_len * dist / 3.0)
    ds = (tsx - tdrain[0], tsy - tdrain[1])
    ds_len = math.hypot(*ds) or 1.0
    ds = (ds[0] / ds_len * dist / 3.0, ds[1] / ds_len * dist / 3.0)
    return p0, (p0[0] + dm[0], p0[1] + dm[1]), (p3[0] + ds[0], p3[1] + ds[1]), p3


def connection_splines_pixel(trace_path, image_width, image_height, samples=24):
    """Cubic-Bezier connection arcs between sections as [(pixel polyline, model_len, drawn_len)].

    Mirrors RoadGraph::Way::getNodesXML: each directed connection src->tgt is a cubic Bezier
    from src's drain to tgt's origin, control points offset by dist/3 along each section's
    tangent (dist = |drain_src - origin_tgt|). Only connections internal to the ego component
    are drawn, matching road_segments_pixel. model_len is the MC's lane-0 arclength (None if
    absent from the trace), drawn_len the length of the drawn Bezier (both in metres).
    """
    transform = compute_fit_transform(trace_path, image_width, image_height)
    if transform is None:
        return []

    get = _trace_getter(trace_path)
    component = _ego_connected_sections(get)

    splines = []
    for src in sorted(component):
        src_geom = _section_geom(get, src)
        if src_geom is None:
            continue
        c = 0
        while True:
            tgt = get(f"outgoing_connection_{c}_of_section_{src}")
            c += 1
            if tgt is None:
                break
            tgt = int(tgt)
            if tgt < 0 or tgt not in component:
                continue
            tgt_geom = _section_geom(get, tgt)
            if tgt_geom is None:
                continue
            bezier = _connection_bezier(src_geom, tgt_geom)
            if bezier is None:
                continue

            world_pts = [_cubic_bezier(i / samples, *bezier) for i in range(samples + 1)]
            pts = [world_to_pixel(*p, transform=transform, image_width=image_width,
                                  image_height=image_height)
                   for p in world_pts]
            drawn_len = sum(math.dist(a, b) for a, b in zip(world_pts, world_pts[1:]))
            model_len = get(f"arclength_from_sec_{src}_to_sec_{tgt}_on_lane_0")
            if model_len is not None and model_len < 0:
                model_len = None
            splines.append((pts, model_len, drawn_len))
    return splines


def target_park_in_dirs(smv_path):
    """Accepted park-in directions ('F', 'B') from 'is_target_reachable := ...' in EnvModel.smv."""
    try:
        with open(smv_path, "r") as f:
            m = re.search(r"is_target_reachable\s*:=([^;]*);", f.read())
    except OSError:
        m = None
    dirs = tuple(d for d, name in (("F", "forward"), ("B", "backward"))
                 if m and f"is_target_reachable_{name}" in m.group(1))
    return dirs or ("F", "B")


def route_states(get, target_sec, target_dirs=("F", "B")):
    """Min-move (section, 'F'|'B') state sequence from Sec. 0 to target_sec, or None.

    Mirrors EnvModel_Reachability.tpl: states seeded with (0, F) and (0, B); F follows
    connections forward, B follows them in reverse, and the gear flips only in a
    forward/backward-terminal section other than the target. Minimises gear changes first,
    then hops, so the route matches the model's move count.
    """
    num_sections = 0
    while get(f"section_{num_sections}.source.x") is not None:
        num_sections += 1
    if target_sec >= num_sections or target_sec == 0:
        return None

    succ = {s: set() for s in range(num_sections)}
    pred = {s: set() for s in range(num_sections)}
    for s in range(num_sections):
        c = 0
        while (t := get(f"outgoing_connection_{c}_of_section_{s}")) is not None:
            c += 1
            t = int(t)
            if 0 <= t < num_sections and t != s:
                succ[s].add(t)
                pred[t].add(s)

    def neighbours(sec, d):
        if d == "F":
            nxt = [((t, "F"), 0) for t in sorted(succ[sec])]
            flip = not succ[sec]
        else:
            nxt = [((m, "B"), 0) for m in sorted(pred[sec])]
            flip = not pred[sec]
        if flip and sec != target_sec:
            nxt.append(((sec, "B" if d == "F" else "F"), 1))
        return nxt

    start = [(0, "F"), (0, "B")]
    cost = {s: (0, 0) for s in start}
    prev = {s: None for s in start}
    heap = [((0, 0), s) for s in start]
    goal = None
    while heap:
        c, state = heapq.heappop(heap)
        if c > cost[state]:
            continue
        if state[0] == target_sec and state[1] in target_dirs:
            goal = state
            break
        for nxt, flips in neighbours(*state):
            nc = (c[0] + flips, c[1] + 1)
            if nxt not in cost or nc < cost[nxt]:
                cost[nxt] = nc
                prev[nxt] = state
                heapq.heappush(heap, (nc, nxt))
    if goal is None:
        return None
    states = [goal]
    while prev[states[-1]] is not None:
        states.append(prev[states[-1]])
    states.reverse()
    return states


def ego_route_world(trace_path, target_sec, target_dirs=("F", "B"), samples=48):
    """Drive legs from the middle of Sec. 0 to the middle of target_sec, as the MC reaches it.

    Follows route_states (fewest gear changes). Each gear flip happens with the car centre on
    the section middle. Returns [(points, cumulative lengths, gear), ...] with the car-centre
    path in motion order and gear +1/-1, or None if the target is unreachable.
    """
    get = _trace_getter(trace_path)
    geoms = {}
    while _section_geom(get, len(geoms)) is not None:
        geoms[len(geoms)] = _section_geom(get, len(geoms))
    states = route_states(get, target_sec, target_dirs)
    if states is None:
        return None

    def at(sec, u):
        (sx, sy), (dx, dy) = geoms[sec]
        length = math.dist((sx, sy), (dx, dy)) or 1.0
        return sx + (dx - sx) * u / length, sy + (dy - sy) * u / length

    raw_legs, pts = [], []
    for k, (sec, d) in enumerate(states):
        length = math.dist(*geoms[sec])
        mid = length / 2.0
        last = k == len(states) - 1
        flip_next = not last and states[k + 1][0] == sec
        if k == 0 or states[k - 1][0] == sec:
            u_in = mid
        else:
            u_in = 0.0 if d == "F" else length
        u_out = mid if last or flip_next else (length if d == "F" else 0.0)
        pts += [at(sec, u_in), at(sec, u_out)]
        if last or flip_next:
            raw_legs.append((pts, 1 if d == "F" else -1))
            pts = []
            continue
        nsec = states[k + 1][0]
        bezier = (_connection_bezier(geoms[sec], geoms[nsec]) if d == "F"
                  else _connection_bezier(geoms[nsec], geoms[sec]))
        if bezier is not None:
            ts = range(1, samples) if d == "F" else range(samples - 1, 0, -1)
            pts.extend(_cubic_bezier(i / samples, *bezier) for i in ts)

    legs = []
    for points, gear in raw_legs:
        cum = [0.0]
        for p, q in zip(points, points[1:]):
            cum.append(cum[-1] + math.dist(p, q))
        legs.append((points, cum, gear))
    return legs


def _polyline_point(points, cum, s):
    s = min(max(s, 0.0), cum[-1])
    i = min(max(bisect_right(cum, s) - 1, 0), len(points) - 2)
    seg = cum[i + 1] - cum[i]
    t = (s - cum[i]) / seg if seg > 0 else 0.0
    (x1, y1), (x2, y2) = points[i], points[i + 1]
    return x1 + t * (x2 - x1), y1 + t * (y2 - y1)


def polyline_pose(points, cum, s, ds=0.25):
    """(x, y, yaw) at arc length s; yaw is the local path tangent (central difference)."""
    x, y = _polyline_point(points, cum, s)
    ax, ay = _polyline_point(points, cum, s - ds)
    bx, by = _polyline_point(points, cum, s + ds)
    return x, y, math.atan2(by - ay, bx - ax)


def trapezoid_distance(t, dist, accel=EGO_ACCEL_MPS2, vmax=EGO_MAX_SPEED_MPS):
    """Distance covered after t seconds (accelerate, cruise, brake to a stop); (d, finished)."""
    if dist <= 0.0:
        return 0.0, True
    v_peak = min(vmax, math.sqrt(accel * dist))
    t_acc = v_peak / accel
    d_acc = 0.5 * accel * t_acc * t_acc
    t_cruise = (dist - 2.0 * d_acc) / v_peak
    if t < t_acc:
        return 0.5 * accel * t * t, False
    if t < t_acc + t_cruise:
        return d_acc + v_peak * (t - t_acc), False
    tb = t - t_acc - t_cruise
    if tb >= t_acc:
        return dist, True
    return d_acc + v_peak * t_cruise + v_peak * tb - 0.5 * accel * tb * tb, False


def trapezoid_duration(dist, accel=EGO_ACCEL_MPS2, vmax=EGO_MAX_SPEED_MPS):
    if dist <= 0.0:
        return 0.0
    v_peak = min(vmax, math.sqrt(accel * dist))
    return 2.0 * v_peak / accel + (dist - v_peak * v_peak / accel) / v_peak


def patch_smv_obstacles(smv_path, obstacles_world):
    """Overwrite the 'rect_obstacles_{tl,br}_{x,y}_N := <int>;' DEFINEs in EnvModel.smv.

    'obstacles_world' is a list of (tl_x, tl_y, br_x, br_y) tuples (world coords) by index.
    Only the numeric literals change, so the model stays otherwise identical. Values are written in
    the model's GEOMETRYSCALING units. For a free obstacle ('<int> + step * shift') the base
    <int> is replaced, so the MC keeps its shift range around the new box.
    """
    geometry_scaling = read_scaling(smv_path)[0]["GEOMETRY"]
    with open(smv_path, "r") as f:
        content = f.read()

    for idx, (tl_x, tl_y, br_x, br_y) in enumerate(obstacles_world):
        for field, value in (("tl_x", tl_x), ("tl_y", tl_y), ("br_x", br_x), ("br_y", br_y)):
            pattern = re.compile(r"(rect_obstacles_" + field + "_" + str(idx) + r"\s*:=\s*)-?\d+(\s*[;+])")
            content, n = pattern.subn(r"\g<1>" + str(round_half_away(value * geometry_scaling)) + r"\g<2>", content)
            if n == 0:
                raise RuntimeError(f"Could not find 'rect_obstacles_{field}_{idx}' in {smv_path}.")

    with open(smv_path, "w") as f:
        f.write(content)


_FREE_OBSTACLE_RE = re.compile(r"rect_obstacles_(tl_x|tl_y|br_x|br_y)_(\d+)\s*:=\s*(-?\d+)\s*\+")


def free_obstacle_offsets_world(trace_path, smv_path):
    """{obs: (d_tl_x, d_tl_y, d_br_x, d_br_y)}: how far the MC moved each corner of a free obstacle."""
    try:
        with open(smv_path, "r") as f:
            content = f.read()
    except OSError:
        return {}
    g = read_scaling(smv_path)[0]["GEOMETRY"]
    get = _trace_getter(trace_path)
    fields = ("tl_x", "tl_y", "br_x", "br_y")
    offsets = {}
    for field, obs, base in _FREE_OBSTACLE_RE.findall(content):
        value = get(f"rect_obstacles_{field}_{obs}")
        corner = list(offsets.get(int(obs), (0.0, 0.0, 0.0, 0.0)))
        corner[fields.index(field)] = 0.0 if value is None else value - int(base) / g
        offsets[int(obs)] = tuple(corner)
    return offsets


def _parse_paren_list(packed):
    """Split a '@(a)@@(b)@...' packed sequence into ['a', 'b', ...]."""
    return re.findall(r"@\(([^)]*)\)@", packed)


def _format_paren_list(values):
    """Pack ['a', 'b', ...] back into '@(a)@@(b)@...'."""
    return "".join(f"@({v})@" for v in values)


def _replace_tpl_string_value(content, key, new_value):
    """Replace the string value of a top-level \"key\": \"...\" entry, keeping the rest verbatim."""
    pattern = re.compile(r'("' + re.escape(key) + r'"\s*:\s*")[^"]*(")')
    new_content, n = pattern.subn(lambda m: m.group(1) + new_value + m.group(2), content, count=1)
    if n == 0:
        raise RuntimeError(f"Could not find key '{key}' in {TPL_PATH}.")
    return new_content


def target_slot_in_fixed_sections(tpl_path, target_section):
    """Array index of 'target_section' within FIXED_SECTION_IDs, or None if it is not fixed."""
    try:
        with open(tpl_path) as f:
            content = f.read()
    except OSError:
        return None
    m = re.search(r'"FIXED_SECTION_IDs"\s*:\s*"([^"]*)"', content)
    if not m:
        return None
    for i, v in enumerate(_parse_paren_list(m.group(1))):
        if v.strip() == str(target_section):
            return i
    return None


def parse_angle_granularity(tpl_path):
    """ANGLEGRANULARITY (deg) from the tpl.json; fixed-section angles must be multiples of it."""
    try:
        with open(tpl_path) as f:
            content = f.read()
    except OSError:
        return 15
    m = re.search(r'"ANGLEGRANULARITY"\s*:\s*"([^"]*)"', content)
    if m:
        d = re.search(r"-?\d+", m.group(1))
        if d:
            return int(d.group(0))
    return 15


def _variant_entry_re(var):
    """The #TEMPLATE entry defining '@var', e.g. "#012": "@anglegran = array(3); ..."."""
    return re.compile(r'("#\d+"\s*:\s*")([^"]*@' + var + r'\b[^"]*)(")')


def parse_tpl_variants(tpl_path, var):
    """Values of the '@var' template variable (one EnvModel package each), or None if absent."""
    try:
        with open(tpl_path) as f:
            content = f.read()
    except OSError:
        return None
    m = _variant_entry_re(var).search(content)
    if not m:
        return None
    body = m.group(2)
    r = re.search(r"range\(\s*@" + var + r"\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*\)", body)
    if r:
        values = list(range(int(r.group(1)), int(r.group(2)) + 1))
    else:
        values = [int(v) for v in re.findall(r"@" + var + r"\[\d+\]\s*=\s*(-?\d+)\s*;", body)]
    return values or None


def patch_tpl_variants(tpl_path, var, values):
    """Rewrite the '@var' definition as an explicit nan-terminated array of 'values'."""
    with open(tpl_path) as f:
        content = f.read()
    n = len(values)
    body = (f"@{var} = array({n + 1}); "
            + "".join(f"@{var}[{i}] = {v}; " for i, v in enumerate(values))
            + f"@{var}[{n}] = nan();")
    content, count = _variant_entry_re(var).subn(lambda m: m.group(1) + body + m.group(3), content,
                                                 count=1)
    if count == 0:
        raise RuntimeError(f"Could not find the '@{var}' definition in {tpl_path}.")
    with open(tpl_path, "w") as f:
        f.write(content)


def round_half_away(value):
    """C++ std::round semantics (Python's round() is banker's rounding: 2.5 -> 2)."""
    return int(math.copysign(math.floor(abs(value) + 0.5), value))


def modelled_world_coord(value, units_per_meter):
    """Coordinate a GEOMETRYSCALING variant stores: value * n rounded like the generator (C++)."""
    return round_half_away(value * units_per_meter) / units_per_meter


def parse_fixed_section_angles(tpl_path):
    """FIXED_SECTION_ANGLEs (deg) from the tpl.json; non-numeric entries are skipped."""
    try:
        with open(tpl_path) as f:
            content = f.read()
    except OSError:
        return []
    m = re.search(r'"FIXED_SECTION_ANGLEs"\s*:\s*"([^"]*)"', content)
    angles = []
    for v in _parse_paren_list(m.group(1)) if m else []:
        try:
            angles.append(int(round(float(v))) % 360)
        except ValueError:
            pass
    return angles


def modelled_fixed_angle(angle, gran):
    """Heading an EnvModel gives a fixed section: angle_raw = round(angle / gran), C++ std::round."""
    return (math.floor(angle / gran + 0.5) * gran) % 360


def patch_tpl_config(tpl_path, target_slot, section_x, section_y, section_angle, obstacles):
    """Write the target section's pose and the obstacle rectangles into the tpl.json in place.

    Only the affected \"key\": \"...\" values are rewritten; the fixed-section arrays keep every
    other slot (crucially section 0 at 0/0/0). 'obstacles' is a list of (tl_x, tl_y, br_x, br_y).
    Pass target_slot=None (with section_* None) to rewrite only the obstacles.
    """
    with open(tpl_path) as f:
        content = f.read()

    # target_slot is None when no draggable section exists: then only the obstacles are rewritten.
    if target_slot is not None:
        for key, value in (("FIXED_SECTION_SOURCE_Xs", section_x),
                           ("FIXED_SECTION_SOURCE_Ys", section_y),
                           ("FIXED_SECTION_ANGLEs", section_angle)):
            m = re.search(r'"' + key + r'"\s*:\s*"([^"]*)"', content)
            if not m:
                raise RuntimeError(f"Could not find key '{key}' in {tpl_path}.")
            vals = _parse_paren_list(m.group(1))
            if target_slot >= len(vals):
                raise RuntimeError(f"Target slot {target_slot} out of range for '{key}' ({vals}).")
            vals[target_slot] = _fmt_world(value)
            content = _replace_tpl_string_value(content, key, _format_paren_list(vals))

    packed = {
        "RECT_OBSTACLES_TL_Xs": [_fmt_world(o[0]) for o in obstacles],
        "RECT_OBSTACLES_TL_Ys": [_fmt_world(o[1]) for o in obstacles],
        "RECT_OBSTACLES_BR_Xs": [_fmt_world(o[2]) for o in obstacles],
        "RECT_OBSTACLES_BR_Ys": [_fmt_world(o[3]) for o in obstacles],
    }
    for key, vals in packed.items():
        content = _replace_tpl_string_value(content, key, _format_paren_list(vals))

    with open(tpl_path, "w") as f:
        f.write(content)


if platform.system() == 'Windows':
    _kernel32 = ctypes.WinDLL('kernel32', use_last_error=True)
    _kernel32.FreeLibrary.argtypes = [ctypes.c_void_p]
    _kernel32.FreeLibrary.restype = ctypes.c_int
elif platform.system() == 'Linux':
    _libc = ctypes.CDLL(ctypes.util.find_library('c'))
    _libc.dlclose.argtypes = [ctypes.c_void_p]
    _libc.dlclose.restype = ctypes.c_int


@contextmanager
def _vfm_lib_context():
    """Load libvfm.so freshly and unload it afterwards so each run starts from clean state."""
    dll_name = 'libvfm.so'
    dll_dir = os.path.join(REPO_ROOT, 'lib')
    if platform.system() == 'Windows':
        dll_name = 'VFM_MAIN_LIB.dll'
        dll_dir = os.path.join(REPO_ROOT, 'bin')
        if dll_dir not in os.environ.get('PATH', ''):
            os.environ['PATH'] = dll_dir + os.pathsep + os.environ.get('PATH', '')

    lib = ctypes.CDLL(os.path.join(dll_dir, dll_name))
    try:
        lib.expandScript.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_size_t]
        lib.expandScript.restype = ctypes.c_char_p
        yield lib
    finally:
        handle = lib._handle
        if platform.system() == 'Windows':
            _kernel32.FreeLibrary(handle)
        elif platform.system() == 'Linux':
            _libc.dlclose(handle)


def _run_vfm_script(script):
    """Run a vfm template script with bin/ as cwd and a fresh libvfm. Returns stdout text.

    The config's relative paths (../examples, ../external, ...) resolve from bin/, so the call
    must run with bin/ as the working directory.
    """
    result = create_string_buffer(1000000)
    prev_cwd = os.getcwd()
    os.chdir(os.path.join(REPO_ROOT, 'bin'))
    try:
        with _vfm_lib_context() as lib:
            res = lib.expandScript(script.encode('utf-8'), result, sizeof(result))
    finally:
        os.chdir(prev_cwd)
    return res.decode(errors="replace") if res else ""


def run_model_checker():
    """Invoke the vfm library to re-run the MC on the current EnvModel.smv. Returns stdout text."""
    return _run_vfm_script(MC_SCRIPT)


def trace_has_counterexample(trace_path):
    """A CEX result file contains the counterexample marker; a blind run does not."""
    if not os.path.isfile(trace_path):
        return False
    with open(trace_path, "r", errors="replace") as f:
        content = f.read()
    return ("Trace Type: Counterexample" in content
            or "as demonstrated by the following" in content)


def package_has_fresh_counterexample(package_dir):
    """True only if the package's trace holds a real counterexample AND postdates EnvModel.smv.

    Two independent things must hold. (1) CONTENT: trace_has_counterexample distinguishes a real
    CEX (state list) from a blind "no counterexample found ... up to N" log -- both are written to
    debug_trace_array.txt, so content, not mere presence, decides winner vs dropout. (2) FRESHNESS:
    a KILLED check (race loser) never overwrites the file, so a prior run's CEX can linger; gate
    the trace mtime against EnvModel.smv, the authoritative model INPUT (written by generation and
    patched by the GUI, always BEFORE the race). Do NOT gate against main.smv: the kratos build
    writes it during the race in the same second as the trace, so sub-second ordering would falsely
    reject a genuine fresh winner. Pre-race clear_package_trace already removes stale leftovers;
    this EnvModel.smv check is the cheap second guard.
    """
    trace_path = package_trace_path(package_dir)
    if not trace_has_counterexample(trace_path):
        return False
    model_path = os.path.join(package_dir, "EnvModel.smv")
    if not os.path.isfile(model_path):
        return True  # no model to compare against; treat the trace as authoritative
    return os.path.getmtime(trace_path) >= os.path.getmtime(model_path)

# Handle item for resizing
class ResizeHandle(QGraphicsRectItem):
    def __init__(self, parent, is_bottom_right=True):
        super().__init__(-6, -6, 12, 12, parent)
        self.parent_rect = parent
        self.is_bottom_right = is_bottom_right
        
        self.setBrush(QBrush(QColor("dodgerblue")))
        self.setPen(QPen(QColor("white"), 1))
        
        self.setFlags(
            QGraphicsRectItem.GraphicsItemFlag.ItemIgnoresTransformations |
            QGraphicsRectItem.GraphicsItemFlag.ItemSendsGeometryChanges
        )
        self.setZValue(1)  # keep handles above the rectangle so they stay grabbable
        self.setCursor(Qt.CursorShape.SizeFDiagCursor if is_bottom_right else Qt.CursorShape.SizeBDiagCursor)
        self.update_position()

    def update_position(self):
        rect = self.parent_rect.rect()
        if self.is_bottom_right:
            self.setPos(rect.right(), rect.bottom())
        else:
            self.setPos(rect.left(), rect.top())

    def mousePressEvent(self, event):
        event.accept()

    def mouseMoveEvent(self, event):
        # Resize by mapping the cursor's scene position into the rect's local space,
        # so it works correctly regardless of the view's zoom/fit scale. The dragged corner
        # snaps to the 1 m world raster (the opposite corner is already on it).
        parent = self.parent_rect
        snapped = QPointF(*snap_scene_point(event.scenePos().x(), event.scenePos().y()))
        local = parent.mapFromScene(snapped)
        rect = parent.rect()
        if self.is_bottom_right:
            new_w = max(10, local.x() - rect.left())
            new_h = max(10, local.y() - rect.top())
            parent.setRect(rect.left(), rect.top(), new_w, new_h)
        else:
            right, bottom = rect.right(), rect.bottom()
            new_left = min(local.x(), right - 10)
            new_top = min(local.y(), bottom - 10)
            parent.setRect(new_left, new_top, right - new_left, bottom - new_top)
        parent.update_handles()
        parent.notify_changed()
        event.accept()

    def mouseReleaseEvent(self, event):
        event.accept()


class DeleteHandle(QGraphicsEllipseItem):
    """Small red 'x' badge at an obstacle's top-right corner; clicking it removes the obstacle."""
    def __init__(self, parent, on_delete):
        super().__init__(-9, -9, 18, 18, parent)
        self.parent_rect = parent
        self._on_delete = on_delete
        self.setBrush(QBrush(QColor(200, 0, 0)))
        self.setPen(QPen(QColor("white"), 2))
        self.setFlag(QGraphicsEllipseItem.GraphicsItemFlag.ItemIgnoresTransformations)
        self.setZValue(3)  # above the resize handles so it stays clickable
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip("Remove this obstacle")
        cross = QGraphicsSimpleTextItem("\u00d7", self)
        cross.setBrush(QBrush(QColor("white")))
        cross.setFont(QFont("Sans", 11, QFont.Weight.Bold))
        br = cross.boundingRect()
        cross.setPos(-br.width() / 2.0, -br.height() / 2.0)
        self.update_position()

    def update_position(self):
        rect = self.parent_rect.rect()
        self.setPos(rect.right(), rect.top())

    def mousePressEvent(self, event):
        event.accept()
        if self._on_delete is not None:
            self._on_delete(self.parent_rect)


# Custom rectangle item with handles
class ResizableRectItem(QGraphicsRectItem):
    def __init__(self, x, y, w, h, on_delete=None, on_changed=None):
        super().__init__(0, 0, w, h)
        self._on_changed = on_changed
        self.setPos(x, y)
        self._updating_handles = False
        
        self.setPen(QPen(QColor("red"), 3))
        self.setBrush(QBrush(QColor(220, 40, 40, 90)))
        self.setZValue(5)
        
        self.setFlags(
            QGraphicsRectItem.GraphicsItemFlag.ItemIsMovable |
            QGraphicsRectItem.GraphicsItemFlag.ItemIsSelectable |
            QGraphicsRectItem.GraphicsItemFlag.ItemSendsGeometryChanges
        )

        self.tl_handle = None
        self.br_handle = None
        self.delete_handle = None

        self.tl_handle = ResizeHandle(self, is_bottom_right=False)
        self.br_handle = ResizeHandle(self, is_bottom_right=True)
        if on_delete is not None:
            self.delete_handle = DeleteHandle(self, on_delete)
        
        self.update_handles()

    def set_editable(self, editable):
        """Clean-view toggle: hide the resize/delete handles and lock the obstacle in place."""
        for handle in (self.tl_handle, self.br_handle, self.delete_handle):
            if handle is not None:
                handle.setVisible(editable)
        self.setFlag(QGraphicsRectItem.GraphicsItemFlag.ItemIsMovable, editable)
        self.setFlag(QGraphicsRectItem.GraphicsItemFlag.ItemIsSelectable, editable)
        if not editable:
            self.setSelected(False)

    def notify_changed(self):
        if self._on_changed is not None:
            self._on_changed()

    def update_handles(self, exclude=None):
        self._updating_handles = True
        if self.tl_handle and self.tl_handle != exclude:
            self.tl_handle.update_position()
        if self.br_handle and self.br_handle != exclude:
            self.br_handle.update_position()
        if self.delete_handle is not None:
            self.delete_handle.update_position()
        self._updating_handles = False

    def itemChange(self, change, value):
        if change == QGraphicsRectItem.GraphicsItemChange.ItemPositionChange:
            # Snap the obstacle's top-left corner onto the 1 m world raster while dragging; the
            # rect's size stays constant (already raster-aligned), so both corners land on it.
            sx, sy = snap_scene_point(value.x(), value.y())
            return QPointF(sx, sy)
        if change == QGraphicsRectItem.GraphicsItemChange.ItemPositionHasChanged:
            self.update_handles()
            self.notify_changed()
        return super().itemChange(change, value)


class SectionRotateHandle(QGraphicsRectItem):
    """Grab point at the target section's free end; dragging rotates the section about its
    source, snapping to the angle granularity."""
    def __init__(self, parent):
        super().__init__(-6, -6, 12, 12, parent)
        self.section = parent
        self.setBrush(QBrush(QColor("gold")))
        self.setPen(QPen(QColor("black"), 1))
        self.setFlags(QGraphicsRectItem.GraphicsItemFlag.ItemIgnoresTransformations)
        self.setZValue(2)
        self.setCursor(Qt.CursorShape.CrossCursor)
        self.update_position()

    def update_position(self):
        self.setPos(self.section.slot_end_px, 0)

    def mousePressEvent(self, event):
        event.accept()

    def mouseMoveEvent(self, event):
        source = self.section.pos()
        p = event.scenePos()
        angle = math.degrees(math.atan2(p.y() - source.y(), p.x() - source.x()))
        gran = self.section.angle_granularity or 1
        self.section.setRotation((round(angle / gran) * gran) % 360)
        self.section.notify_changed()
        event.accept()

    def mouseReleaseEvent(self, event):
        event.accept()


class RotatableSectionItem(QGraphicsRectItem):
    """Fixed-length target section, drawn as its parking bay: movable and rotatable (about its
    source end) but not resizable. Rotation is constrained to multiples of the angle granularity."""
    def __init__(self, source_px, length_px, angle_deg, half_width_px, angle_granularity, on_changed,
                 slot_px=0.0):
        slot_start, self.slot_end_px = slot_extent(length_px, slot_px)
        super().__init__(slot_start, -half_width_px, self.slot_end_px - slot_start, 2.0 * half_width_px)
        self.length_px = length_px
        self.angle_granularity = angle_granularity
        self._on_changed = on_changed
        self.setPos(source_px)                  # local origin (0,0) is the section's source end
        self.setTransformOriginPoint(0.0, 0.0)  # rotate about the source
        self.setRotation(angle_deg % 360)
        self.setPen(QPen(QColor("gold"), 3))
        self.setBrush(QBrush(QColor(255, 200, 0, 70)))
        self.setZValue(6)
        self.setFlags(
            QGraphicsRectItem.GraphicsItemFlag.ItemIsMovable |
            QGraphicsRectItem.GraphicsItemFlag.ItemIsSelectable |
            QGraphicsRectItem.GraphicsItemFlag.ItemSendsGeometryChanges
        )
        self.handle = SectionRotateHandle(self)

    def set_editable(self, editable):
        """Clean-view toggle: hide the rotate handle and lock the section in place."""
        if self.handle is not None:
            self.handle.setVisible(editable)
        self.setFlag(QGraphicsRectItem.GraphicsItemFlag.ItemIsMovable, editable)
        self.setFlag(QGraphicsRectItem.GraphicsItemFlag.ItemIsSelectable, editable)
        if not editable:
            self.setSelected(False)

    def notify_changed(self):
        if self._on_changed is not None:
            self._on_changed()

    def itemChange(self, change, value):
        if change == QGraphicsRectItem.GraphicsItemChange.ItemPositionChange:
            # Snap the section's source end onto the 1 m world raster while dragging, matching the
            # angle-granularity snap of the rotate handle and the raster the tpl.json quantises to.
            sx, sy = snap_scene_point(value.x(), value.y())
            return QPointF(sx, sy)
        if change == QGraphicsRectItem.GraphicsItemChange.ItemPositionHasChanged:
            self.notify_changed()
        return super().itemChange(change, value)


class VariantPanel(QGroupBox):
    """Editor for the value list of a tpl.json '#'-variable; each value becomes its own EnvModel
    package. 'on_edited' fires on every user edit; set_values() replaces the list silently."""
    def __init__(self, title, spin_range, prefix, suffix, candidates, on_edited):
        super().__init__(title)
        self.setFixedWidth(250)
        self.values = None
        self._spin_range, self._prefix, self._suffix = spin_range, prefix, suffix
        self._candidates = candidates
        self._on_edited = on_edited
        v = QVBoxLayout(self)
        self.info = QLabel()
        self.info.setWordWrap(True)
        v.addWidget(self.info)
        self._rows_layout = QVBoxLayout()
        v.addLayout(self._rows_layout)
        self._add_btn = QPushButton("Add Variant")
        self._add_btn.clicked.connect(self._add)
        v.addWidget(self._add_btn)
        self.status_labels = []

    def set_values(self, values):
        self.values = values
        self._rebuild()

    def set_status(self, index, text, color, tooltip=""):
        label = self.status_labels[index]
        label.setText(text)
        label.setStyleSheet(f"color: {color};")
        label.setToolTip(tooltip)

    def is_duplicate(self, index):
        return self.values[index] in self.values[:index]

    def _rebuild(self):
        while self._rows_layout.count():
            old = self._rows_layout.takeAt(0).widget()
            old.setParent(None)
            old.deleteLater()
        self.status_labels = []
        self._add_btn.setEnabled(self.values is not None)
        for i, value in enumerate(self.values or []):
            row = QWidget()
            h = QHBoxLayout(row)
            h.setContentsMargins(0, 0, 0, 0)
            spin = QSpinBox()
            spin.setRange(*self._spin_range)
            spin.setPrefix(self._prefix)
            spin.setSuffix(self._suffix)
            spin.setValue(value)
            spin.valueChanged.connect(lambda val, i=i: self._set(i, val))
            status = QLabel()
            remove = QToolButton()
            remove.setText("×")
            remove.setToolTip("Remove this variant")
            remove.setEnabled(len(self.values) > 1)
            remove.clicked.connect(lambda _checked=False, i=i: self._remove(i))
            h.addWidget(spin)
            h.addWidget(status, 1)
            h.addWidget(remove)
            self._rows_layout.addWidget(row)
            self.status_labels.append(status)

    def _set(self, index, value):
        self.values[index] = value
        self._on_edited()

    def _add(self):
        fallback = min(max(self.values) + 1, self._spin_range[1])
        self.values.append(next((v for v in self._candidates if v not in self.values), fallback))
        self._rebuild()
        self._on_edited()

    def _remove(self, index):
        if len(self.values) > 1:
            del self.values[index]
            self._rebuild()
            self._on_edited()


class MainWindow(QMainWindow):
    def __init__(self, image_path, trace_path, smv_path):
        super().__init__()
        self.setWindowTitle("Simple Image Annotation Tool")
        self.image_path = image_path
        self.trace_path = trace_path
        self.smv_path = smv_path
        self.rect_items = []
        self.image_w = 0
        self.image_h = 0
        self._raster_step = 1.0

        # Main Widget & Layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)

        # Setup Graphics Scene and View
        self.scene = QGraphicsScene()
        self.view = QGraphicsView(self.scene)
        
        # 💡 CRITICAL: Ensure smooth scaling behavior
        self.view.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.view.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.view.setRenderHint(self.view.renderHints().SmoothPixmapTransform)

        self.gran_panel = VariantPanel("Angle granularity variants", (1, 180), "", "°",
                                       (30, 45, 15, 90, 20, 10, 5), self._on_gran_values_changed)
        self.grid_panel = VariantPanel("Distance grid variants", (1, 20), "1/", " m",
                                       (1, 2, 4, 5, 10), self._on_grid_values_changed)
        side = QVBoxLayout()
        side.addWidget(self.gran_panel)
        side.addWidget(self.grid_panel)
        side.addStretch(1)
        top = QHBoxLayout()
        top.addLayout(side)
        top.addWidget(self.view, 1)
        layout.addLayout(top)

        self.progress_bar = QProgressBar(self)
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setVisible(False)
        self.progress_bar.setTextVisible(True)
        self.synth_status = QLabel(self)
        self.synth_status.setWordWrap(True)
        self.synth_status.setVisible(False)
        layout.addWidget(self.synth_status)
        layout.addWidget(self.progress_bar)

        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(100)
        self.poll_timer.timeout.connect(self.poll_progress_file)

        self.play_btn = QPushButton("Play")
        self.play_btn.clicked.connect(self.play_ego)
        layout.addWidget(self.play_btn)

        self.gif_btn = QPushButton("Export Drive as GIF")
        self.gif_btn.clicked.connect(self.export_gif)
        layout.addWidget(self.gif_btn)

        # Discards unregenerated edits: back to the layout the current packages were built from.
        self.reset_btn = QPushButton("Reset")
        self.reset_btn.clicked.connect(self.reset_scene)
        layout.addWidget(self.reset_btn)

        # Drops a new draggable obstacle; like moving section 1, this needs an EnvModel regen
        # (the obstacle COUNT changes, which patch_smv_obstacles cannot do on the fly).
        self.add_obstacle_btn = QPushButton("Add Obstacle")
        self.add_obstacle_btn.clicked.connect(self.add_obstacle)
        layout.addWidget(self.add_obstacle_btn)

        # Model checks every configured variant in parallel and adopts the first counterexample
        # (a single-variant config is just the degenerate one-runner case).
        # Obsolete: Rather run Env Model generation every time, and auto-trigger MC.
        # self.refresh_btn = QPushButton("Run Model Checker")
        # self.refresh_btn.clicked.connect(self.trigger_external_process)
        # layout.addWidget(self.refresh_btn)

        # Stays clickable while the MC race runs; kills every nuXmv instance at once.
        self.terminate_btn = QPushButton("Terminate Model Checker")
        self.terminate_btn.clicked.connect(self.terminate_race)
        self.terminate_btn.setEnabled(False)
        layout.addWidget(self.terminate_btn)

        self.envgen_btn = QPushButton("Run Model Checker")
        self.envgen_btn.clicked.connect(lambda: self.run_envmodel_generation())
        # Right-click = forced regeneration: wipe the EnvModel cache and rewrite the tpl.json first.
        self.envgen_btn.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.envgen_btn.customContextMenuRequested.connect(
            lambda _pos: self.run_envmodel_generation(force=True))
        self.envgen_btn.setToolTip("Right-click: clear the EnvModel cache, write all current "
                                   "obstacle positions to the tpl.json, then regenerate.")
        layout.addWidget(self.envgen_btn)

        self.synth_btn = QPushButton("Synthesize Free Obstacles (force exactly PARK_MOVES moves)")
        self.synth_btn.setToolTip("Runs parking/synth_obstacles.py and shows every candidate and "
                                  "the shorter route refuting it while the search goes on.")
        self.synth_btn.clicked.connect(self.start_synth)
        layout.addWidget(self.synth_btn)

        self.reveal_btn = QPushButton("Reveal Model Folder")
        self.reveal_btn.clicked.connect(self.reveal_model_folder)
        layout.addWidget(self.reveal_btn)

        # Clean-view toggle: hides all editing handles and locks items so the birdseye can be
        # screenshotted without resize/rotate/delete decorations. Purely visual, no config change.
        self.visu_mode = False
        self.visu_btn = QPushButton("Clean View for Screenshots (hide handles)")
        self.visu_btn.setCheckable(True)
        self.visu_btn.clicked.connect(self.toggle_visu_mode)
        layout.addWidget(self.visu_btn)

        # Race state: a killable worker process plus a poll timer watching the variant folders.
        self._race_proc = None
        self.race_timer = QTimer(self)
        self.race_timer.setInterval(400)
        self.race_timer.timeout.connect(self._poll_race)

        # Obstacle synthesis (CEGIS) state: detached CLI process plus its event-file reader.
        self._synth_proc = None
        self._synth_log_file = None
        self._synth_events_pos = 0
        self._synth_done = False
        self._synth_log = []
        self._synth_refuted = []
        self._synth_moves = None
        self.synth_timer = QTimer(self)
        self.synth_timer.setInterval(500)
        self.synth_timer.timeout.connect(self._poll_synth)

        self._ego_item = None
        self._ego_route = None
        self._play_t0 = 0.0
        self.play_timer = QTimer(self)
        self.play_timer.setInterval(30)
        self.play_timer.timeout.connect(self._play_step)

        # Target-section (section 1) editing state. Moving/rotating it invalidates the current
        # packages, so the MC re-run is blocked until the EnvModels are regenerated.
        self._section_dirty = False
        # Adding/removing obstacles also invalidates the packages (the obstacle count changes).
        self._obstacles_dirty = False
        self.section_item = None
        self._transform = None
        self._obstacle_offsets = {}
        self._target_slot = None
        self._target_section = None
        self._ppm = None
        # '@anglegran' variants: edited in the left panel, written to the tpl.json on regeneration.
        gran_values = parse_tpl_variants(TPL_PATH, "anglegran")
        self._gran_baseline = list(gran_values) if gran_values else None
        self._gran_dirty = False
        self._tpl_fixed_angles = parse_fixed_section_angles(TPL_PATH)
        # Rotation snap = gcd of all variants, so every variant's grid is reachable.
        self.angle_granularity = (math.gcd(*gran_values) if gran_values
                                  else parse_angle_granularity(TPL_PATH))
        self.gran_panel.set_values(gran_values)
        # '@geomgrid' variants (GEOMETRYSCALING in mm), shown as n = units per metre (raster 1/n m).
        grid_mm = parse_tpl_variants(TPL_PATH, "geomgrid")
        grid_values = [max(1, round(v / 1000)) for v in grid_mm] if grid_mm else None
        self._grid_baseline = list(grid_values) if grid_values else None
        self._grid_dirty = False
        self.grid_panel.set_values(grid_values)
        # Scene items re-baselined/removed once a moved layout is committed by regeneration.
        self._trajectory_items = []
        self._obstacle_ghost_items = []
        self._section_ghost_item = None
        self._baseline = None

        # Load image and obstacle rectangles from the current trace.
        self.reload_from_trace()

    def reload_from_trace(self):
        """(Re)draw the road graph and overlay obstacle rectangles read from the trace.

        The roads are drawn straight from the counterexample (same fit transform as the
        obstacles) instead of the cached C++ preview: that preview is a random sample layout,
        not this counterexample, so it does not line up with the solved obstacle positions.
        """
        pixmap = QPixmap(self.image_path)
        # Canvas size only follows the cached preview (to keep the fit transform's aspect ratio);
        # a default is used when no preview exists yet for this variant.
        if not pixmap.isNull():
            self.image_w, self.image_h = pixmap.width(), pixmap.height()
        else:
            self.image_w, self.image_h = 2200, 1650

        segments = road_segments_pixel(self.trace_path, self.image_w, self.image_h)
        if _trace_getter(self.trace_path).from_config:
            print("No solution trace; drawing the configured sections and obstacles only.")
        splines = connection_splines_pixel(self.trace_path, self.image_w, self.image_h)
        rects_data = obstacles_to_pixel_rects(self.trace_path, self.image_w, self.image_h)
        target_section = target_section_from_smv(self.smv_path)
        transform = compute_fit_transform(self.trace_path, self.image_w, self.image_h)
        ppm = transform[2] if transform else 50.0
        self._transform = transform
        self._obstacle_offsets = free_obstacle_offsets_world(self.trace_path, self.smv_path)
        self._ppm = ppm

        # Activate world-raster snapping (finest grid of the distance-grid variants) for the items.
        global _scene_raster_snap
        self._raster_step = self._grid_raster_step()
        if transform is not None:
            _scene_raster_snap = (lambda px, py, t=transform, iw=self.image_w, ih=self.image_h:
                                  _snap_scene_to_world_raster(px, py, t, iw, ih, self._raster_step))
        else:
            _scene_raster_snap = None

        # The target (section 1) is user-adjustable unless it is the pinned origin section 0.
        self._target_section = target_section
        self._target_slot = target_slot_in_fixed_sections(TPL_PATH, target_section)
        draggable_target = target_section != 0 and self._target_slot is not None
        self.section_item = None

        self.play_timer.stop()
        self._ego_item = None
        self.scene.clear()
        self.rect_items = []
        self._trajectory_items = []
        self._obstacle_ghost_items = []
        self._section_ghost_item = None

        if segments:
            self._draw_roads(segments, splines, target_section, ppm, draggable_target)
            self._draw_ego_car(transform, target_section)
            # Static ghosts of the ORIGINAL obstacle positions, so the starting layout stays
            # visible once the draggable (red) obstacles are moved during a session.
            self._draw_obstacle_ghosts(rects_data)
            self._draw_synth_refuted(transform)
            for rect in rects_data:
                self.add_rectangle(*rect)
            if draggable_target:
                self._create_section_item(segments, target_section, ppm)
            # Frame roads AND obstacles (the fit is road-only, so obstacles may sit outside the
            # image box); a gray backdrop behind everything keeps the birdseye look.
            content = self.scene.itemsBoundingRect().adjusted(-60, -60, 60, 60)
            backdrop = self.scene.addRect(content, QPen(Qt.GlobalColor.transparent),
                                          QBrush(QColor(170, 170, 170)))
            backdrop.setZValue(-10)
            self.scene.setSceneRect(content)
        else:
            if not pixmap.isNull():
                self.scene.addPixmap(pixmap)
                self.scene.setSceneRect(QRectF(pixmap.rect()))
            else:
                print(f"Warning: no road geometry in '{self.trace_path}' and no preview image.")
                self.scene.setSceneRect(0, 0, self.image_w, self.image_h)
            self._draw_obstacle_ghosts(rects_data)
            for rect in rects_data:
                self.add_rectangle(*rect)

        self._capture_baseline()
        self._refresh_gran_status()
        self._refresh_grid_status()
        self._apply_visu_mode()
        self.scale_to_fit()

    def _draw_ego_car(self, transform, target_section):
        """Ego sprite centred on the middle of Sec. 0, heading along Sec. 0 (source -> drain)."""
        image = QImage(EGO_CAR_IMAGE)
        if image.isNull() or transform is None:
            return
        # White background -> transparent; near-white too, so the anti-aliased rim leaves no halo.
        image = image.convertToFormat(QImage.Format.Format_ARGB32)
        for y in range(image.height()):
            for x in range(image.width()):
                p = image.pixel(x, y)
                if min((p >> 16) & 0xFF, (p >> 8) & 0xFF, p & 0xFF) >= 245:
                    image.setPixel(x, y, 0)
        pixmap = QPixmap.fromImage(image)
        car = self.scene.addPixmap(pixmap)
        car.setTransformationMode(Qt.TransformationMode.SmoothTransformation)
        car.setTransformOriginPoint(pixmap.width() / 2.0, pixmap.height() / 2.0)
        car.setScale(transform[2] * EGO_LENGTH_M / pixmap.width())
        car.setZValue(3.5)
        self._ego_item = car
        self._ego_route = ego_route_world(self.trace_path, target_section,
                                          target_park_in_dirs(self.smv_path))
        self._place_ego_at_start()

    def _place_ego_at_start(self):
        geom = _section_geom(_trace_getter(self.trace_path), 0)
        if self._ego_item is None or geom is None:
            return
        (sx, sy), (dx, dy) = geom
        self._set_ego_pose((sx + dx) / 2.0, (sy + dy) / 2.0, math.atan2(dy - sy, dx - sx))

    def _set_ego_pose(self, x, y, yaw):
        """Place the sprite by its centre (world coords, the MC's vehicle reference) and heading (rad)."""
        px, py = world_to_pixel(x, y, self._transform, self.image_w, self.image_h)
        origin = self._ego_item.transformOriginPoint()
        self._ego_item.setPos(px - origin.x(), py - origin.y())
        self._ego_item.setRotation(math.degrees(yaw))

    def _ego_route_ready(self):
        if self._ego_item is None:
            return False
        if self._ego_route is None:
            if _trace_getter(self.trace_path).from_config:
                QMessageBox.information(self, "No solution",
                                        "No solution loaded yet - run the model checker first.")
                return False
            QMessageBox.information(self, "No route",
                                    f"No connection chain from Sec. 0 to target Sec. "
                                    f"{self._target_section} in the current trace.")
            return False
        return True

    def play_ego(self):
        """Drive the ego from the middle of Sec. 0 to the middle of the target section."""
        if not self._ego_route_ready():
            return
        self._play_t0 = time.monotonic()
        self._play_step()
        self.play_timer.start()

    def _play_step(self):
        if self._set_ego_pose_at(time.monotonic() - self._play_t0):
            self.play_timer.stop()

    def export_gif(self):
        """Render the Play drive (start -> target) frame by frame into an animated GIF."""
        if not self._ego_route_ready():
            return
        default = os.path.join(os.path.dirname(os.path.abspath(self.trace_path)), "ego_drive.gif")
        path, _ = QFileDialog.getSaveFileName(self, "Export Drive as GIF", default,
                                              "GIF images (*.gif)")
        if not path:
            return
        from PIL import Image

        self.play_timer.stop()
        source = self.scene.sceneRect()
        scale = min(1.0, GIF_MAX_WIDTH_PX / source.width())
        w, h = max(1, round(source.width() * scale)), max(1, round(source.height() * scale))
        # Frames are rendered without editing handles, like the clean screenshot view.
        prev_visu = self.visu_mode
        self.visu_mode = True
        self._apply_visu_mode()
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        frames = []
        try:
            dt = 1.0 / GIF_FPS
            t, done = 0.0, False
            while not done:
                done = self._set_ego_pose_at(t)
                image = QImage(w, h, QImage.Format.Format_RGBA8888)
                image.fill(Qt.GlobalColor.white)
                painter = QPainter(image)
                painter.setRenderHint(QPainter.RenderHint.Antialiasing)
                painter.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
                self.scene.render(painter, QRectF(0, 0, w, h), source)
                painter.end()
                bits = image.constBits()
                bits.setsize(image.sizeInBytes())
                frames.append(Image.frombuffer("RGBA", (w, h), bytes(bits), "raw", "RGBA",
                                               image.bytesPerLine(), 1).convert("RGB"))
                t += dt
            # One shared palette avoids per-frame colour flicker of the static background.
            first = frames[0].quantize(colors=256)
            paletted = [first] + [f.quantize(palette=first, dither=Image.Dither.NONE)
                                  for f in frames[1:]]
            durations = [round(1000 * dt)] * len(paletted)
            durations[-1] = round(1000 * GIF_FINAL_HOLD_S)
            paletted[0].save(path, save_all=True, append_images=paletted[1:],
                             duration=durations, loop=0)
        except OSError as e:
            QMessageBox.warning(self, "GIF export failed", str(e))
            return
        finally:
            QApplication.restoreOverrideCursor()
            self.visu_mode = prev_visu
            self._apply_visu_mode()
            self._place_ego_at_start()
        print(f"Exported {len(frames)} frames to {path}")

    def _set_ego_pose_at(self, t):
        """Pose the ego at drive time t (s); returns True once it has reached the target."""
        last = len(self._ego_route) - 1
        for i, (points, cum, gear) in enumerate(self._ego_route):
            d, finished = trapezoid_distance(t, cum[-1])
            if not finished or i == last:
                break
            t -= trapezoid_duration(cum[-1])
            if t < EGO_GEAR_CHANGE_PAUSE_S:
                break
            t -= EGO_GEAR_CHANGE_PAUSE_S
        x, y, yaw = polyline_pose(points, cum, d)
        # In reverse the car faces against its direction of motion.
        self._set_ego_pose(x, y, yaw if gear > 0 else yaw + math.pi)
        return finished and i == last

    def _draw_obstacle_ghosts(self, rects_data):
        """Filled, non-interactive ghosts marking where the obstacles originally were."""
        ghost_pen = QPen(QColor(90, 90, 90), 2, Qt.PenStyle.DashLine)
        ghost_brush = QBrush(QColor(90, 90, 90, 90))
        for x1, y1, x2, y2 in rects_data:
            ghost = self.scene.addRect(QRectF(min(x1, x2), min(y1, y2),
                                              abs(x2 - x1), abs(y2 - y1)),
                                       ghost_pen, ghost_brush)
            ghost.setZValue(2)
            self._obstacle_ghost_items.append(ghost)

    def _draw_roads(self, segments, splines=None, target_section=None, ppm=50.0,
                    draggable_target=False):
        """Draw connection arcs, section centerlines, and white start/target parking markings."""
        road_pen = QPen(QColor(60, 60, 60), 6)
        road_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        ego_pen = QPen(QColor(0, 90, 200), 6)
        ego_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        arc_pen = QPen(QColor(120, 120, 120), 5)
        arc_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        arc_pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        for pts, model_len, drawn_len in (splines or []):
            if len(pts) < 2:
                continue
            path = QPainterPath(QPointF(pts[0][0], pts[0][1]))
            for x, y in pts[1:]:
                path.lineTo(x, y)
            self._trajectory_items.append(self.scene.addPath(path, arc_pen))
            self._draw_arclength_label(pts[len(pts) // 2], model_len, drawn_len)
        for x1, y1, x2, y2, sec, is_ego in segments:
            self._trajectory_items.append(
                self.scene.addLine(x1, y1, x2, y2, ego_pen if is_ego else road_pen))
        for x1, y1, x2, y2, sec, is_ego in segments:
            if is_ego:
                self._draw_parking_marking(x1, y1, x2, y2, "START", ppm)
            elif target_section is not None and sec == target_section and not draggable_target:
                self._draw_parking_marking(x1, y1, x2, y2, "TARGET", ppm)

    def _draw_arclength_label(self, mid, model_len, drawn_len):
        """Arclength badge at an arc's midpoint: the MC's value, else the drawn Bezier's (≈)."""
        text = f"{model_len:g} m" if model_len is not None else f"≈{drawn_len:.1f} m"
        label = self.scene.addText("", QFont("Sans", 10, QFont.Weight.Bold))
        label.setHtml(f'<span style="background-color:#303030; color:white;">&nbsp;{text}&nbsp;</span>')
        model_tip = (f"Model arclength (lane 0): {model_len:g} m" if model_len is not None
                     else "Model arclength: not in trace")
        label.setToolTip(f"{model_tip}\nDrawn Bezier length: {drawn_len:.1f} m")
        # Constant on-screen size regardless of the fit zoom, centred on the arc midpoint.
        label.setFlag(label.GraphicsItemFlag.ItemIgnoresTransformations)
        br = label.boundingRect()
        label.setTransform(QTransform.fromTranslate(-br.width() / 2.0, -br.height() / 2.0))
        label.setPos(mid[0], mid[1])
        label.setZValue(4)
        self._trajectory_items.append(label)

    def _draw_parking_marking(self, x1, y1, x2, y2, label, ppm):
        """White parking-bay outline around a section's middle, with a label (start/target indicator)."""
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy) or 1.0
        ux, uy = dx / length, dy / length
        nx, ny = -uy, ux                    # unit normal
        mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        half_len = max(length, PARKING_SLOT_LENGTH_M * ppm) / 2.0
        x1, y1 = mx - ux * half_len, my - uy * half_len
        x2, y2 = mx + ux * half_len, my + uy * half_len
        half = ppm * 1.25                    # ~2 m wide bay (1 m half-width), in world proportion
        thickness = max(3.0, ppm * 0.15)    # ~0.15 m painted line, like real markings
        corners = [
            QPointF(x1 + nx * half, y1 + ny * half),
            QPointF(x2 + nx * half, y2 + ny * half),
            QPointF(x2 - nx * half, y2 - ny * half),
            QPointF(x1 - nx * half, y1 - ny * half),
        ]
        white_pen = QPen(QColor(255, 255, 255), thickness)
        white_pen.setJoinStyle(Qt.PenJoinStyle.MiterJoin)
        bay = self.scene.addPolygon(QPolygonF(corners), white_pen,
                                    QBrush(QColor(255, 255, 255, 40)))
        bay.setZValue(3)

        text = self.scene.addText(label, QFont("Sans", 16, QFont.Weight.Bold))
        text.setDefaultTextColor(QColor(255, 255, 255))
        br = text.boundingRect()
        text.setPos(mx - br.width() / 2.0, my - br.height() / 2.0)
        text.setZValue(4)


    def scale_to_fit(self):
        """💡 Scale the scene layout to fit perfectly inside the viewport boundaries."""
        if not self.scene.sceneRect().isEmpty():
            self.view.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def toggle_visu_mode(self):
        """Flip the clean-view mode and (re)apply it to every editable item on the scene."""
        self.visu_mode = self.visu_btn.isChecked()
        self.visu_btn.setText(
            "Editing View (show handles)" if self.visu_mode
            else "Clean View for Screenshots (hide handles)")
        self._apply_visu_mode()

    def _apply_visu_mode(self):
        """Show/hide handles and lock/unlock all obstacles and the target section."""
        editable = not self.visu_mode
        for item in self.rect_items:
            item.set_editable(editable)
        if self.section_item is not None:
            self.section_item.set_editable(editable)

    def resizeEvent(self, event):
        """💡 Hook into window resize events to recalculate scale instantly."""
        super().resizeEvent(event)
        self.scale_to_fit()

    def add_rectangle(self, x1, y1, x2, y2):
        # Snap both corners onto the 1 m world raster so new/loaded obstacles start aligned.
        x1, y1 = snap_scene_point(x1, y1)
        x2, y2 = snap_scene_point(x2, y2)
        x = min(x1, x2)
        y = min(y1, y2)
        w = abs(x2 - x1)
        h = abs(y2 - y1)
        
        rect_item = ResizableRectItem(x, y, w, h, on_delete=self._remove_obstacle,
                                      on_changed=self._refresh_grid_status)
        self.scene.addItem(rect_item)
        self.rect_items.append(rect_item)

    def add_obstacle(self):
        """Drop a new obstacle in the middle of the view; needs a regen before the MC can run."""
        rect = self.scene.sceneRect()
        size = (self._ppm or 50.0) * 3.0  # ~3 m square default
        cx, cy = rect.center().x(), rect.center().y()
        self.add_rectangle(cx - size / 2.0, cy - size / 2.0,
                           cx + size / 2.0, cy + size / 2.0)
        self._mark_obstacles_dirty()
        self._refresh_grid_status()

    def _remove_obstacle(self, item):
        """Delete an obstacle from the scene; needs a regen before the MC can run again."""
        if item in self.rect_items:
            self.rect_items.remove(item)
        self.scene.removeItem(item)
        self._mark_obstacles_dirty()
        self._refresh_grid_status()

    def _mark_obstacles_dirty(self):
        """The obstacle set changed: block the MC re-run until the EnvModels are regenerated."""
        self._obstacles_dirty = True
        # self._block_mc_until_regen(
        #     "Obstacles added/removed - regenerate the EnvModels before re-running the MC.")

    def collect_obstacles_world(self):
        """Inverse-transform the current on-screen rectangles back to world coords on the model raster."""
        # Use the transform cached when the scene was drawn: the source trace file may have been
        # deleted/replaced since, but the rectangles on screen still live in that pixel space.
        transform = self._transform
        if transform is None:
            transform = compute_fit_transform(self.trace_path, self.image_w, self.image_h)
        if transform is None:
            return []

        obstacles = []
        for item in self.rect_items:
            scene_rect = item.mapRectToScene(item.rect())
            tl = pixel_to_world(scene_rect.left(), scene_rect.top(), transform, self.image_w, self.image_h)
            br = pixel_to_world(scene_rect.right(), scene_rect.bottom(), transform, self.image_w, self.image_h)
            obstacles.append(tuple(snap_world(v, self._raster_step) for v in (*tl, *br)))
        return obstacles

    def collect_obstacle_bases_world(self):
        """Drawn obstacles minus the MC's free-obstacle shift/grow, i.e. the boxes to configure."""
        bases = []
        for i, corners in enumerate(self.collect_obstacles_world()):
            offsets = self._obstacle_offsets.get(i, (0.0, 0.0, 0.0, 0.0))
            bases.append(tuple(c - d for c, d in zip(corners, offsets)))
        return bases

    def _create_section_item(self, segments, target_section, ppm):
        """Add the draggable/rotatable target section (fixed length, angle-granularity snapping)."""
        seg = next((s for s in segments if s[4] == target_section), None)
        if seg is None:
            return
        x1, y1, x2, y2, _sec, _is_ego = seg
        length_px = math.hypot(x2 - x1, y2 - y1)
        angle_deg = math.degrees(math.atan2(y2 - y1, x2 - x1))
        half_w = max(6.0, ppm * 1.25)
        self._draw_section_ghost(x1, y1, length_px, angle_deg, half_w)
        self.section_item = RotatableSectionItem(
            QPointF(x1, y1), length_px, angle_deg, half_w,
            self.angle_granularity, self._on_section_changed, PARKING_SLOT_LENGTH_M * ppm)
        self.scene.addItem(self.section_item)

    def _draw_section_ghost(self, x1, y1, length_px, angle_deg, half_w):
        """Faint outline of the target bay's original pose (like the obstacle ghosts)."""
        a = math.radians(angle_deg)
        ux, uy = math.cos(a), math.sin(a)
        nx, ny = -uy, ux
        start, end = slot_extent(length_px, PARKING_SLOT_LENGTH_M * (self._ppm or 50.0))
        x1, y1, x2, y2 = x1 + start * ux, y1 + start * uy, x1 + end * ux, y1 + end * uy
        corners = [QPointF(x1 + nx * half_w, y1 + ny * half_w),
                   QPointF(x2 + nx * half_w, y2 + ny * half_w),
                   QPointF(x2 - nx * half_w, y2 - ny * half_w),
                   QPointF(x1 - nx * half_w, y1 - ny * half_w)]
        ghost = self.scene.addPolygon(QPolygonF(corners),
                                      QPen(QColor(200, 160, 0), 2, Qt.PenStyle.DashLine),
                                      QBrush(QColor(255, 200, 0, 50)))
        ghost.setZValue(2)
        self._section_ghost_item = ghost

    def _on_section_changed(self):
        """The target section was moved/rotated: mark the config stale and block the MC re-run."""
        if not self._section_dirty:
            self._section_dirty = True
            # self._block_mc_until_regen(
            #     "Target section moved - regenerate the EnvModels before re-running the MC.")
        self._refresh_gran_status()
        self._refresh_grid_status()

    def _current_fixed_angles(self):
        angles = list(self._tpl_fixed_angles)
        if (self._section_dirty and self.section_item is not None
                and self._target_slot is not None and self._target_slot < len(angles)):
            angles[self._target_slot] = int(round(self.section_item.rotation())) % 360
        return angles

    def _refresh_gran_status(self):
        """Per variant: can it represent every fixed-section heading exactly, or does it round?"""
        panel = self.gran_panel
        if panel.values is None:
            panel.info.setText("No '@anglegran' variable in the tpl.json; "
                               f"ANGLEGRANULARITY = {self.angle_granularity}°.")
            return
        angles = self._current_fixed_angles()
        panel.info.setText(
            "One EnvModel package per value (applied on regeneration).\n"
            f"Fixed headings: {', '.join(f'{a}°' for a in angles) or '-'}\n"
            f"Target rotation snaps to {self.angle_granularity}°.")
        for i, value in enumerate(panel.values):
            off = [(a, modelled_fixed_angle(a, value)) for a in angles if a % value]
            if panel.is_duplicate(i):
                panel.set_status(i, "duplicate", "gray", "Ignored: the same value is listed above.")
            elif off:
                panel.set_status(i, "≈ " + ", ".join(f"{a}→{m}°" for a, m in off), "#c06000",
                                 f"Not a multiple of {value}°: this variant rounds the heading as "
                                 "shown, while positions keep the exact angle.")
            else:
                panel.set_status(i, "✓ exact", "green")

    def _on_gran_values_changed(self):
        self.angle_granularity = math.gcd(*self.gran_panel.values)
        if self.section_item is not None:
            self.section_item.angle_granularity = self.angle_granularity
        self._gran_dirty = True
        # self._block_mc_until_regen(
        #     "Angle granularity variants changed - regenerate the EnvModels before re-running the MC.")
        self._refresh_gran_status()

    def _grid_raster_step(self):
        """Snap raster (m): the finest grid containing every variant's grid, else the model's own."""
        values = self.grid_panel.values
        return 1.0 / math.lcm(*values) if values else world_raster_step(self.smv_path)

    def _current_world_coords(self):
        """World coords (m) the models must hold: fixed-section sources and obstacle corners."""
        coords = {k: v for k, v in _config_values(TPL_PATH).items()
                  if re.fullmatch(r"section_\d+\.source\.[xy]", k)}
        if (self._section_dirty and self.section_item is not None
                and self._transform is not None and self._target_section is not None):
            src = self.section_item.pos()
            sx, sy = pixel_to_world(src.x(), src.y(), self._transform, self.image_w, self.image_h)
            coords[f"section_{self._target_section}.source.x"] = snap_world(sx, self._raster_step)
            coords[f"section_{self._target_section}.source.y"] = snap_world(sy, self._raster_step)
        return list(coords.values()) + [c for o in self.collect_obstacles_world() for c in o]

    def _refresh_grid_status(self):
        """Per variant: can it represent every section source / obstacle corner exactly?"""
        panel = self.grid_panel
        if panel.values is None:
            panel.info.setText("No '@geomgrid' variable in the tpl.json; "
                               f"positions snap to the model's {_fmt_world(self._raster_step)} m grid.")
            return
        coords = self._current_world_coords()
        panel.info.setText(
            "GEOMETRYSCALING: one EnvModel package per value (applied on regeneration).\n"
            f"Positions snap to {_fmt_world(self._raster_step)} m.")
        for i, n in enumerate(panel.values):
            off = [(c, modelled_world_coord(c, n)) for c in coords
                   if abs(modelled_world_coord(c, n) - c) > 1e-9]
            if panel.is_duplicate(i):
                panel.set_status(i, "duplicate", "gray", "Ignored: the same value is listed above.")
            elif off:
                worst = max(abs(m - c) for c, m in off)
                panel.set_status(i, f"≈ {len(off)} rounded (≤{_fmt_world(round(worst, 3))} m)",
                                 "#c06000",
                                 f"Not on the {_fmt_world(1 / n)} m grid; this variant rounds:\n"
                                 + ", ".join(f"{_fmt_world(c)}→{_fmt_world(m)}" for c, m in off[:12]))
            else:
                panel.set_status(i, "✓ exact", "green")

    def _on_grid_values_changed(self):
        self._raster_step = self._grid_raster_step()
        self._grid_dirty = True
        # self._block_mc_until_regen(
        #     "Distance grid variants changed - regenerate the EnvModels before re-running the MC.")
        self._refresh_grid_status()

    # def _block_mc_until_regen(self, reason):
    #     """Grey out the MC re-run and explain that an EnvModel regeneration is required first."""
    #     self.refresh_btn.setEnabled(False)
    #     self.refresh_btn.setToolTip(reason)

    def _config_dirty(self):
        """True while there are un-regenerated edits (section pose, obstacle set, granularities)."""
        return self._section_dirty or self._obstacles_dirty or self._gran_dirty or self._grid_dirty

    def _write_config_from_scene(self, obstacles_only=False):
        """Hardcode the target section pose (when adjustable) and obstacles into the tpl.json."""
        if self._gran_dirty:
            patch_tpl_variants(TPL_PATH, "anglegran", list(dict.fromkeys(self.gran_panel.values)))
        if self._grid_dirty:
            patch_tpl_variants(TPL_PATH, "geomgrid",
                               [n * 1000 for n in dict.fromkeys(self.grid_panel.values)])
        if self._transform is None:
            raise RuntimeError("No transform available to write the scene into the config.")
        obstacles = self.collect_obstacle_bases_world()
        # Only a moved section is written: the drawn pose may be a variant's rounded heading.
        if (not obstacles_only and self._section_dirty and self.section_item is not None
                and self._target_slot is not None):
            src = self.section_item.pos()
            sx_w, sy_w = pixel_to_world(src.x(), src.y(), self._transform, self.image_w, self.image_h)
            angle = int(round(self.section_item.rotation())) % 360
            patch_tpl_config(TPL_PATH, self._target_slot,
                             snap_world(sx_w, self._raster_step), snap_world(sy_w, self._raster_step),
                             angle, obstacles)
        else:
            patch_tpl_config(TPL_PATH, None, None, None, None, obstacles)

    def _on_regen_success(self):
        """After a successful regeneration the drawn layout matches the config again."""
        committed = self._config_dirty()
        self._section_dirty = False
        self._obstacles_dirty = False
        if self._gran_dirty:
            self._gran_dirty = False
            self._gran_baseline = list(self.gran_panel.values)
        if self._grid_dirty:
            self._grid_dirty = False
            self._grid_baseline = list(self.grid_panel.values)
        self._tpl_fixed_angles = parse_fixed_section_angles(TPL_PATH)
        self._refresh_gran_status()
        self._refresh_grid_status()
        if committed:
            # self.refresh_btn.setEnabled(True)
            # self.refresh_btn.setToolTip("")
            self._commit_scene()

    def _commit_scene(self):
        """Lock in the moved layout: drop the now-stale trajectory and re-baseline the ghosts."""
        for item in self._trajectory_items:
            self.scene.removeItem(item)
        self._trajectory_items = []

        # Obstacle ghosts snap onto the committed (moved) obstacle positions.
        for ghost in self._obstacle_ghost_items:
            self.scene.removeItem(ghost)
        self._obstacle_ghost_items = []
        current = []
        for item in self.rect_items:
            r = item.mapRectToScene(item.rect())
            current.append((r.left(), r.top(), r.right(), r.bottom()))
        self._draw_obstacle_ghosts(current)

        # Section ghost snaps under the committed target-section pose.
        if self._section_ghost_item is not None:
            self.scene.removeItem(self._section_ghost_item)
            self._section_ghost_item = None
        if self.section_item is not None:
            pos = self.section_item.pos()
            half_w = max(6.0, (self._ppm or 50.0) * 1.0)
            self._draw_section_ghost(pos.x(), pos.y(), self.section_item.length_px,
                                     self.section_item.rotation(), half_w)
        self._capture_baseline()

    def _capture_baseline(self):
        """Remember the obstacle/section layout that matches the current packages (for Reset)."""
        rects = []
        for item in self.rect_items:
            r = item.mapRectToScene(item.rect())
            rects.append((r.left(), r.top(), r.right(), r.bottom()))
        section = None
        if self.section_item is not None:
            section = (QPointF(self.section_item.pos()), self.section_item.rotation())
        self._baseline = (rects, section)

    def reset_scene(self):
        """Undo all unregenerated edits and unblock the MC re-run (no regeneration needed)."""
        if self._baseline is None or self._race_proc is not None:
            return
        self.play_timer.stop()
        self._place_ego_at_start()
        rects, section = self._baseline
        for item in self.rect_items:
            self.scene.removeItem(item)
        self.rect_items = []
        for rect in rects:
            self.add_rectangle(*rect)
        if self.section_item is not None and section is not None:
            self.section_item.setPos(section[0])
            self.section_item.setRotation(section[1])
        # Set after restoring: moving the section item re-marks it dirty via notify_changed.
        self._section_dirty = False
        self._obstacles_dirty = False
        if self._gran_baseline is not None:
            self.angle_granularity = math.gcd(*self._gran_baseline)
            if self.section_item is not None:
                self.section_item.angle_granularity = self.angle_granularity
            self.gran_panel.set_values(list(self._gran_baseline))
        self._gran_dirty = False
        self._refresh_gran_status()
        if self._grid_baseline is not None:
            self.grid_panel.set_values(list(self._grid_baseline))
            self._raster_step = self._grid_raster_step()
        self._grid_dirty = False
        self._refresh_grid_status()
        # self.refresh_btn.setEnabled(True)
        # self.refresh_btn.setToolTip("")
        self._apply_visu_mode()

    def _set_buttons_enabled(self, enabled):
        self.gran_panel.setEnabled(enabled)
        self.grid_panel.setEnabled(enabled)
        self.envgen_btn.setEnabled(enabled)
        self.synth_btn.setEnabled(enabled)
        self.add_obstacle_btn.setEnabled(enabled)
        self.reset_btn.setEnabled(enabled)
        self.play_btn.setEnabled(enabled)
        self.gif_btn.setEnabled(enabled)
        # The MC re-run stays disabled while section 1 or the obstacle set has un-regenerated
        # edits: the current packages are stale until EnvModel regeneration writes them in.
        # self.refresh_btn.setEnabled(enabled and not self._config_dirty())

    def reveal_model_folder(self):
        """Open the folder behind the shown maneuver; also print it, as opening fails on remote."""
        folder = os.path.dirname(os.path.abspath(self.trace_path))
        print(f"Model folder: {folder}")
        opener = shutil.which("xdg-open") if platform.system() == "Linux" else None
        if opener is None:
            QDesktopServices.openUrl(QUrl.fromLocalFile(folder))
            return
        # Detached and silenced: the file manager's own GLib/D-Bus warnings (frequent over
        # remote X) would otherwise spam this terminal and tie it to the GUI's process group.
        try:
            subprocess.Popen([opener, folder], start_new_session=True,
                             stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
        except OSError as e:
            print(f"Could not open the file manager: {e}")

    def poll_progress_file(self):
        """Reads the progress file from disk and updates the GUI progress bar."""
        if os.path.exists(PROGRESS_PATH + "/progress.morty"):
            self.progress_bar.setVisible(True)
            progress_value = -1
            second = ""
            config = "NIL"
            
            try:
                with open(PROGRESS_PATH + "/progress.morty", "r") as f:
                    content = f.read().strip().split('#')
                    
                    if content[0].isdigit():
                        raw = int(content[0])
                        progress_value = (raw - 1) * 100
                    if content[1].isdigit():
                        base_max = int(content[1]) * 100
                        self.progress_bar.setRange(0, base_max)
                    
                    second = str(raw) + "/" + content[1] + "   " + content[2].replace("_config_", "").replace("_", " | ")
                    config = content[2]
            except IOError:
                pass

            try:
                if os.path.exists(PROGRESS_PATH + config + "/progress.morty"):
                    with open(PROGRESS_PATH + config + "/progress.morty", "r") as f:
                        content2 = f.read().strip().split('#')
                        
                        if content2[0].isdigit():
                            progress_value = progress_value + int(content2[0])
                        
                        second = second + " [" + content2[2] + "]"
            except IOError:
                pass
            
            if progress_value >= 0:
                self.progress_bar.setValue(progress_value)
                self.progress_bar.setFormat(second)

        else:
            self.progress_bar.setVisible(False)

    def handle_results(self, result):
        self._on_regen_success()
        self.poll_timer.stop()
        self.poll_progress_file()
        self.trigger_external_process()

    def run_envmodel_generation(self, force=False):
        # A moved target section or a changed obstacle set must be written into the config
        # BEFORE regenerating, so the new EnvModels reflect the drawn layout.
        # force: always sync the obstacles and drop the EnvModel cache so nothing is reused.
        if force or self._config_dirty():
            try:
                self._write_config_from_scene(obstacles_only=force and not self._config_dirty())
            except (RuntimeError, OSError, ValueError) as e:
                QMessageBox.critical(self, "Config update failed", str(e))
                return
        if force:
            shutil.rmtree(os.path.join(EXAMPLES_DIR, "tmp"), ignore_errors=True)

        # Wipe existing gp* variant folders first so an old parameter range can't leave obsolete
        # packages around (regeneration recreates the current set; the vfm cache keeps this fast).
        remove_mc_packages()
        
        self.thread = QThread()
        self.worker = EnvModelWorker()
        self.worker.moveToThread(self.thread)

        self.thread.started.connect(self.worker.run)
        self.worker.finished.connect(self.handle_results)

        # Clean up thread memory when done
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)

        # Start the background work AND start the GUI timer
        self.thread.start()
        self.poll_timer.start()

    def trigger_external_process(self):
        """Race all configured variants; adopt the first counterexample and kill the rest.

        Cheap variants (few sections / coarse granularity) finish first; if one already yields a
        counterexample the race ends immediately and the more expensive configs are killed off.
        Only escalation to a harder config (or all variants finishing UNSAT) resolves otherwise.
        """
        if self._race_proc is not None:
            return  # A race is already in flight.
        self._synth_refuted = []
        self.synth_status.setVisible(False)

        obstacles_world = self.collect_obstacles_world()
        # Zero obstacles is a valid scenario (empty lot); only warn when rectangles are on screen
        # but cannot be mapped to world coords (a genuine transform failure).
        if self.rect_items and not obstacles_world:
            QMessageBox.warning(self, "Obstacle mapping failed",
                                "Could not map the on-screen obstacles to world coordinates.")
            return

        packages = discover_mc_packages()
        if not packages:
            QMessageBox.warning(self, "No model packages",
                                "No generated 'gp' packages found.\n"
                                "Run EnvModel generation first.")
            return

        # Every variant has its own EnvModel.smv; patch the drawn obstacles into each of them.
        try:
            for pkg in packages:
                patch_smv_obstacles(os.path.join(pkg, "EnvModel.smv"), self.collect_obstacle_bases_world())
        except (RuntimeError, OSError) as e:
            QMessageBox.critical(self, "SMV update failed", str(e))
            return

        # Preserve the currently shown trace/image so we can restore it if no variant wins.
        self._race_trace_backup = self.trace_path + ".prev"
        self._race_image_backup = self.image_path + ".prev"
        for src, dst in ((self.trace_path, self._race_trace_backup),
                         (self.image_path, self._race_image_backup)):
            if os.path.isfile(src):
                shutil.copy2(src, dst)

        # Clear every variant's leftover trace so its content reflects THIS run: runMCJobs runs
        # with delete_old_output=false, so a race loser that is KILLED before nuXmv runs would
        # keep a prior debug_trace_array.txt. After clearing, a present file is from this run --
        # CEX content => winner, blind ("no counterexample ... up to N") => dropout, absent => unchecked.
        self._race_packages = packages
        for pkg in packages:
            clear_package_trace(pkg)
        # A variant's mc_runtimes.txt is bumped only after its nuXmv exits (and the trace is
        # already fully written), so a bump tells us that variant finished -> report no-CEX ones.
        self._race_pre_runtime_mtime = {
            p: (os.path.getmtime(os.path.join(p, "mc_runtimes.txt"))
                if os.path.isfile(os.path.join(p, "mc_runtimes.txt")) else -1.0)
            for p in packages
        }
        self._race_reported_dropouts = set()

        self._set_buttons_enabled(False)
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        print(f"⚡ Racing {len(packages)} model-checker variant(s)...")
        try:
            self._race_proc = subprocess.Popen(
                [sys.executable, _MC_WORKER, MC_SCRIPT],
                cwd=REPO_ROOT, start_new_session=True,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as e:
            QApplication.restoreOverrideCursor()
            self._set_buttons_enabled(True)
            QMessageBox.critical(self, "Launch failed", f"Could not start model checker: {e}")
            return

        self.terminate_btn.setEnabled(True)
        self._show_race_progress()
        self.race_timer.start()

    def _show_race_progress(self):
        """Red ('negative') bar: advances by one per variant that dropped out without a CEX."""
        self.progress_bar.setStyleSheet(
            "QProgressBar { text-align: center; }"
            "QProgressBar::chunk { background-color: #c62828; }")
        self.progress_bar.setRange(0, len(self._race_packages))
        self._update_race_progress()
        self.progress_bar.setVisible(True)

    def _update_race_progress(self):
        dropped, total = len(self._race_reported_dropouts), len(self._race_packages)
        self.progress_bar.setValue(dropped)
        self.progress_bar.setFormat(f"{dropped}/{total} variants dropped out (no counterexample)")

    def _hide_race_progress(self):
        self.progress_bar.setVisible(False)
        self.progress_bar.setStyleSheet("")

    def _race_winner(self):
        """First variant (in folder order) with a fresh counterexample from THIS run.

        Leftover traces were cleared at race start, so any CEX trace present now was produced by
        the current check; the mtime gate in package_has_fresh_counterexample is a second guard.
        """
        for pkg in self._race_packages:
            if package_has_fresh_counterexample(pkg):
                return pkg
        return None

    def _poll_race(self):
        # Report variants that finished without a counterexample so a long race can be judged.
        self._report_dropouts()

        # The first variant to produce a fresh counterexample ends the race immediately.
        winner = self._race_winner()
        if winner is not None:
            self._finish_race(winner, error=False)
            return

        rc = self._race_proc.poll()
        if rc is not None:
            # Worker finished: no variant produced a counterexample this run.
            winner = self._race_winner()
            self._finish_race(winner, error=(winner is None and rc != 0))

    def _report_dropouts(self):
        """Print each variant whose MODEL CHECK finished without a counterexample, once.

        Each variant appends to mc_runtimes.txt twice: after building (kratos) and again after
        the nuXmv check. Only the nuXmv line means the check actually ran, so a bare mtime bump
        (build done, check still pending or killed) must not be mistaken for a no-CEX result.
        """
        for pkg in self._race_packages:
            if pkg in self._race_reported_dropouts:
                continue
            runtime_file = os.path.join(pkg, "mc_runtimes.txt")
            if not os.path.isfile(runtime_file):
                continue
            if os.path.getmtime(runtime_file) <= self._race_pre_runtime_mtime.get(pkg, -1.0):
                continue
            try:
                with open(runtime_file, "r", errors="replace") as f:
                    lines = [ln for ln in f.read().splitlines() if ln.strip()]
            except OSError:
                continue
            if not lines or "nuXmv" not in lines[-1]:
                continue  # built but the check has not (yet) finished for this variant
            if package_has_fresh_counterexample(pkg):
                continue
            self._race_reported_dropouts.add(pkg)
            self._update_race_progress()
            print(f"\u26aa {os.path.basename(pkg)} finished with no counterexample (dropped out).")

    def _finish_race(self, winner, error, aborted_by_user=False):
        self.race_timer.stop()
        _kill_process_group(self._race_proc)
        self._race_proc = None
        self._hide_race_progress()
        QApplication.restoreOverrideCursor()
        self.terminate_btn.setEnabled(False)
        self._set_buttons_enabled(True)

        if winner is not None:
            for backup in (self._race_trace_backup, self._race_image_backup):
                if backup and os.path.isfile(backup):
                    os.remove(backup)
            # Point the view at the winning variant's artifacts and redraw.
            self.trace_path = package_trace_path(winner)
            self.image_path = package_image_path(winner)
            self.smv_path = os.path.join(winner, "EnvModel.smv")
            self.reload_from_trace()
            print(f"✅ Winner: {os.path.basename(winner)}")
            return

        # No variant produced a counterexample (or the worker crashed): keep the current view as
        # the user left it (e.g. a rotated section) with the previous solution still drawn, rather
        # than snapping edits back. Only restore the trace/image FILES that were cleared at race
        # start so self.trace_path stays valid -- deliberately do NOT reload/redraw.
        for backup, dst in ((self._race_trace_backup, self.trace_path),
                            (self._race_image_backup, self.image_path)):
            if backup and os.path.isfile(backup):
                shutil.move(backup, dst)

        if aborted_by_user:
            QMessageBox.information(
                self, "Model checker terminated",
                "Terminated all model-checker instances.\n"
                "Kept the current view.")
        elif error:
            QMessageBox.warning(
                self, "Model checker aborted",
                "The model checker run was aborted or failed.\n"
                "Kept the current view.")
        else:
            QMessageBox.information(
                self, "No counterexample",
                "No configured variant produced a counterexample for the new obstacle layout.\n"
                "Kept the current view (the previous solution is still shown).")

    def terminate_race(self):
        """Kill every nuXmv instance of the in-flight MC race (leaves other actions untouched)."""
        if self._synth_proc is not None:
            print("\U0001f6d1 Terminating the obstacle synthesis...")
            self._finish_synth("Terminated by user.")
            return
        if self._race_proc is None:
            return
        print("\U0001f6d1 Terminating all model-checker instances...")
        self._finish_race(winner=None, error=False, aborted_by_user=True)

    # --- Obstacle synthesis (parking/synth_obstacles.py) ---------------------------------------

    def start_synth(self):
        """Run the CEGIS obstacle synthesis detached; _poll_synth shows each step as it arrives."""
        if self._race_proc is not None or self._synth_proc is not None:
            return
        # Always sync the drawn obstacles: plain drags don't mark the config dirty (the normal race
        # patches them into EnvModel.smv instead), but the synthesis regenerates from the tpl.json.
        try:
            self._write_config_from_scene(obstacles_only=not self._config_dirty())
        except (RuntimeError, OSError, ValueError) as e:
            QMessageBox.critical(self, "Config update failed", str(e))
            return
        os.makedirs(SYNTH_HISTORY_DIR, exist_ok=True)
        events = os.path.join(SYNTH_HISTORY_DIR, "events.jsonl")
        open(events, "w").close()
        self._synth_events_pos = 0
        self._synth_done = False
        self._synth_log = []
        self._synth_refuted = []
        self._synth_log_file = open(os.path.join(SYNTH_HISTORY_DIR, "synth.log"), "w")
        try:
            self._synth_proc = subprocess.Popen(
                [sys.executable, _SYNTH_SCRIPT, "--history", SYNTH_HISTORY_DIR],
                cwd=REPO_ROOT, start_new_session=True, stdin=subprocess.DEVNULL,
                stdout=self._synth_log_file, stderr=subprocess.STDOUT)
        except OSError as e:
            self._synth_log_file.close()
            QMessageBox.critical(self, "Launch failed", f"Could not start the synthesis: {e}")
            return
        self._set_buttons_enabled(False)
        self.play_btn.setEnabled(True)
        self.terminate_btn.setEnabled(True)
        self.progress_bar.setRange(0, 0)
        self.progress_bar.setVisible(True)
        self.synth_status.setVisible(True)
        self._synth_say("Starting obstacle synthesis (log: examples/synth_history/synth.log)...")
        self.synth_timer.start()

    def _synth_say(self, line):
        print(f"\U0001f9e9 {line}")
        self._synth_log.append(line)
        self.synth_status.setText("\n".join(self._synth_log[-6:]))

    def _show_snapshot(self, folder):
        self.trace_path = os.path.join(folder, "debug_trace_array.txt")
        self.smv_path = os.path.join(folder, "EnvModel.smv")
        self.image_path = os.path.join(folder, "preview.png")
        self.reload_from_trace()

    def _draw_synth_refuted(self, transform):
        """Dashed outlines of free-obstacle placements already refuted by a shorter route."""
        if transform is None:
            return
        pen = QPen(QColor(200, 0, 0), 2, Qt.PenStyle.DashLine)
        for tl_x, tl_y, br_x, br_y in self._synth_refuted:
            x1, y1 = world_to_pixel(tl_x, tl_y, transform, self.image_w, self.image_h)
            x2, y2 = world_to_pixel(br_x, br_y, transform, self.image_w, self.image_h)
            item = self.scene.addRect(QRectF(min(x1, x2), min(y1, y2), abs(x2 - x1), abs(y2 - y1)),
                                      pen, QBrush(Qt.BrushStyle.NoBrush))
            item.setZValue(1.5)

    def _poll_synth(self):
        try:
            with open(os.path.join(SYNTH_HISTORY_DIR, "events.jsonl"), "r") as f:
                f.seek(self._synth_events_pos)
                chunk = f.read()
        except OSError:
            chunk = ""
        complete = chunk[:chunk.rfind("\n") + 1]
        self._synth_events_pos += len(complete.encode())
        for line in complete.splitlines():
            try:
                self._handle_synth_event(json.loads(line))
            except (ValueError, KeyError) as e:
                print(f"Bad synthesis event {line!r}: {e}")
            if self._synth_proc is None:
                return
        if self._synth_proc.poll() is not None and not complete:
            self._finish_synth(None if self._synth_done else
                               "The synthesis process ended unexpectedly; see examples/synth_history/synth.log.")

    def _handle_synth_event(self, ev):
        kind, it = ev["type"], ev.get("iter")
        if kind == "phase":
            if ev["phase"] == "regen":
                self._synth_say("Generating the EnvModel packages...")
            elif ev["phase"] == "synth":
                self._synth_say(f"Iteration {it}: searching a placement of the free obstacles that "
                                f"still allows parking within {self._synth_moves} move(s)...")
            else:
                self._synth_say(f"Iteration {it}: checking every package for a route with FEWER "
                                f"than {self._synth_moves} move(s) around this placement...")
        elif kind == "start":
            self._synth_moves = ev["moves"]
            self._on_regen_success()
            self._synth_say(f"{len(ev['packages'])} package(s) {ev['packages']}; free obstacle(s) "
                            f"{ev['free']}; goal: exactly {ev['moves']} move(s).")
        elif kind == "candidate":
            self._show_snapshot(ev["snapshot"])
            self._synth_say(f"Iteration {it}: candidate from {ev['package']} ({ev['seconds']:.0f} s), "
                            f"shown with its {ev['moves']}-move route.")
        elif kind == "refuted":
            self._synth_refuted += [tuple(ev["obstacles"][o]) for o in ev["free"]]
            self._show_snapshot(ev["snapshot"])
            self._synth_say(f"Iteration {it}: REFUTED by the shown {ev['moves']}-move route "
                            f"({ev['package']}, {ev['seconds']:.0f} s). Next placements must block it "
                            f"({ev['lemmas']} route(s) to block; dashed red = refuted placements).")
        elif kind == "certified":
            self._synth_done = True
            pkg = ev["package"]
            self.trace_path, self.image_path = package_trace_path(pkg), package_image_path(pkg)
            self.smv_path = os.path.join(pkg, "EnvModel.smv")
            self.reload_from_trace()
            self._finish_synth(f"CERTIFIED after {it} iteration(s): this placement forces exactly "
                               f"{ev['moves']} move(s); no package has a shorter route.", success=True)
        elif kind == "impossible":
            self._synth_done = True
            if ev["lemmas"] == 0:
                why = (f"no placement of the free obstacles allows parking within {ev['moves']} "
                       "move(s) at all (for any configured section count)")
            else:
                why = (f"every placement within the shift ranges admits a route with fewer than "
                       f"{ev['moves']} move(s): {ev['lemmas']} refuting route(s) together cover "
                       "all placements (dashed red = the refuted ones)")
            self._finish_synth(f"No solution: {why}.")
        elif kind == "gave_up":
            self._synth_done = True
            self._finish_synth(f"Gave up after {it} iterations ({ev['lemmas']} refuting routes).")
        elif kind == "error":
            self._synth_done = True
            self._finish_synth(ev["message"])

    def _finish_synth(self, message, success=False):
        self.synth_timer.stop()
        _kill_process_group(self._synth_proc)
        self._synth_proc = None
        if self._synth_log_file is not None:
            self._synth_log_file.close()
            self._synth_log_file = None
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setVisible(False)
        self.terminate_btn.setEnabled(False)
        self._set_buttons_enabled(True)
        if message:
            self._synth_say(message)
            (QMessageBox.information if success else QMessageBox.warning)(
                self, "Obstacle synthesis", message)

    def closeEvent(self, event):
        """Never leave an orphaned worker (and its nuXmv children) running after the window closes."""
        if self._race_proc is not None:
            self.race_timer.stop()
            _kill_process_group(self._race_proc)
            self._race_proc = None
        if self._synth_proc is not None:
            self.synth_timer.stop()
            _kill_process_group(self._synth_proc)
            self._synth_proc = None
        super().closeEvent(event)


if __name__ == "__main__":
    # `--race` runs the multi-config race first (detached, auto-killing losers) and opens the
    # winner; otherwise it just opens whatever variant finished first (see select_startup_package).
    raced_winner = race_first_solution() if "--race" in sys.argv else None

    app = QApplication(sys.argv)
    startup_pkg = raced_winner or select_startup_package()
    print(f"Opening package: {os.path.basename(startup_pkg)}")
    window = MainWindow(package_image_path(startup_pkg),
                        package_trace_path(startup_pkg),
                        os.path.join(startup_pkg, "EnvModel.smv"))
    window.resize(800, 600)
    window.show()
    sys.exit(app.exec())
