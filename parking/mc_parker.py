import sys
import re
import math
import os
import platform
import shutil
import signal
import subprocess
import time
import ctypes
import ctypes.util
from contextlib import contextmanager
from ctypes import create_string_buffer, sizeof
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QPushButton, QGraphicsView, QGraphicsScene, QGraphicsRectItem,
                             QGraphicsEllipseItem, QGraphicsSimpleTextItem, QMessageBox)
from PyQt6.QtCore import Qt, QRectF, QTimer, QPointF
from PyQt6.QtGui import QPixmap, QBrush, QPen, QColor, QPainterPath, QPolygonF, QFont


# Repo root = parent of this parking/ folder; anchors all paths independent of the caller's cwd.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# --- Fixed locations for the parking config (see EnvModel generation into examples/gp_config). ---
TRACE_PATH = os.path.join(REPO_ROOT, "examples/gp_config/debug_trace_array.txt")
SMV_PATH = os.path.join(REPO_ROOT, "examples/gp_config/EnvModel.smv")
IMAGE_PATH = os.path.join(REPO_ROOT, "examples/gp_config/0/preview2/preview2_0.png")
# The config template; section-1 pose and obstacle positions are written here before regeneration.
TPL_PATH = os.path.join(REPO_ROOT, "src/templates/envmodel_config.tpl.json")
# Re-runs the model checker on the already-generated EnvModel.smv (no tpl.json regeneration).
# Path is relative to bin/, the working directory the MC runs in (see run_model_checker).
MC_SCRIPT = "@{../src/templates/envmodel_config.tpl.json}@.runMCJobs[16]"
# Regenerates EnvModel.smv/main.smv from the config; required after any config change.
ENVGEN_SCRIPT = "@{../src/templates/envmodel_config.tpl.json}@.generateEnvmodels"
# Renders only the smooth birdseye counterexample visualization (images/video).
TESTCASE_SCRIPT = "@{../src/templates/envmodel_config.tpl.json}@.generateTestCases[cex-smooth-birdseye]"

# runMCJobs model-checks every examples/gp* package (except the bare 'gp' prefix folder). With a
# >1 range on a '#'-variable in the tpl.json, generateEnvmodels emits one such package per value
# (cross-product for several ranges), so a single MC run races multiple configs at once.
EXAMPLES_DIR = os.path.join(REPO_ROOT, "examples")
_PACKAGE_PREFIX = "gp"
_MC_WORKER = os.path.join(REPO_ROOT, "parking", "_mc_worker.py")


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


def race_first_solution(poll_interval=1.0):
    """Model-check all configured variants and stop at the FIRST complete counterexample.

    Runs the worker detached from the terminal (stdin=/dev/null, own session) so the nuXmv
    instances can never grab the tty and drop you into a shared interactive prompt, polls the
    variant folders, then kills the whole job group the moment one variant has a fresh
    counterexample. Returns the winning package dir, or None if none solved.
    """
    packages = discover_mc_packages()
    if not packages:
        print("No 'gp' packages found. Run EnvModel generation first.")
        return None

    # Clear leftover traces so a package can only win by producing a CEX in THIS run.
    for p in packages:
        clear_package_trace(p)

    print(f"Racing {len(packages)} config(s); first counterexample wins...")
    proc = subprocess.Popen(
        [sys.executable, _MC_WORKER, MC_SCRIPT],
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


def _read_trace_values(trace_path):
    """Parse a debug_trace_array.txt dump into a {name: float} map.

    Names keep whatever prefix they have in the file (e.g. 'env.section_0.source.x').
    """
    values = {}
    line_re = re.compile(r'^\s*([A-Za-z_][\w.]*)\s*=\s*(-?\d+(?:\.\d+)?)\s*$')
    if not os.path.isfile(trace_path):
        return values
    with open(trace_path, "r") as f:
        for line in f:
            m = line_re.match(line)
            if m:
                values[m.group(1)] = float(m.group(2))
    return values


def _trace_getter(trace_path):
    """Return a getter accepting both 'env.'-prefixed (dump) and bare (MC trace) names."""
    v = _read_trace_values(trace_path)

    def get(name):
        if name in v:
            return v[name]
        return v.get("env." + name)

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
    """Snap a scene/pixel point onto the world raster (integer metres) the tpl.json quantises to."""
    wx, wy = pixel_to_world(px, py, transform, image_width, image_height)
    return world_to_pixel(round(wx / step) * step, round(wy / step) * step,
                          transform, image_width, image_height)


# Set by MainWindow once the world<->pixel transform for the current scene is known, so the
# draggable items can snap their pose onto the same 1 m world raster as the target-section angle
# snaps to its granularity. None (identity) until a transform is available.
_scene_raster_snap = None  # callable(scene_x, scene_y) -> (scene_x, scene_y)


def snap_scene_point(px, py):
    """Snap a scene point onto the active 1 m world raster (identity if no transform is set)."""
    if _scene_raster_snap is None:
        return px, py
    return _scene_raster_snap(px, py)


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


def connection_splines_pixel(trace_path, image_width, image_height, samples=24):
    """Cubic-Bezier connection arcs between sections as pixel polylines [[(x, y), ...], ...].

    Mirrors RoadGraph::Way::getNodesXML: each directed connection src->tgt is a cubic Bezier
    from src's drain to tgt's origin, control points offset by dist/3 along each section's
    tangent (dist = |drain_src - origin_tgt|). Only connections internal to the ego component
    are drawn, matching road_segments_pixel.
    """
    transform = compute_fit_transform(trace_path, image_width, image_height)
    if transform is None:
        return []

    get = _trace_getter(trace_path)
    component = _ego_connected_sections(get)

    def geom(sec):
        sx = get(f"section_{sec}.source.x")
        sy = get(f"section_{sec}.source.y")
        if sx is None or sy is None:
            return None
        angle = 2.0 * math.pi * (get(f"section_{sec}.angle") or 0.0) / 360.0
        length = get(f"section_{sec}_end") or 0.0
        return (sx, sy), (sx + length * math.cos(angle), sy + length * math.sin(angle))

    splines = []
    for src in sorted(component):
        src_geom = geom(src)
        if src_geom is None:
            continue
        (osx, osy), odrain = src_geom
        c = 0
        while True:
            tgt = get(f"outgoing_connection_{c}_of_section_{src}")
            c += 1
            if tgt is None:
                break
            tgt = int(tgt)
            if tgt < 0 or tgt not in component:
                continue
            tgt_geom = geom(tgt)
            if tgt_geom is None:
                continue
            (tsx, tsy), tdrain = tgt_geom

            p0, p3 = odrain, (tsx, tsy)
            dist = math.hypot(p3[0] - p0[0], p3[1] - p0[1])
            if dist < 1e-6:
                continue
            # dir_mine = drain - origin (of src); dir_succ = origin - drain (of tgt); both len dist/3.
            dm = (odrain[0] - osx, odrain[1] - osy)
            dm_len = math.hypot(*dm) or 1.0
            dm = (dm[0] / dm_len * dist / 3.0, dm[1] / dm_len * dist / 3.0)
            ds = (tsx - tdrain[0], tsy - tdrain[1])
            ds_len = math.hypot(*ds) or 1.0
            ds = (ds[0] / ds_len * dist / 3.0, ds[1] / ds_len * dist / 3.0)
            p1 = (p0[0] + dm[0], p0[1] + dm[1])
            p2 = (p3[0] + ds[0], p3[1] + ds[1])

            pts = [world_to_pixel(*_cubic_bezier(i / samples, p0, p1, p2, p3),
                                  transform=transform, image_width=image_width,
                                  image_height=image_height)
                   for i in range(samples + 1)]
            splines.append(pts)
    return splines


def patch_smv_obstacles(smv_path, obstacles_world):
    """Overwrite the 'rect_obstacles_{tl,br}_{x,y}_N := <int>;' DEFINEs in EnvModel.smv.

    'obstacles_world' is a list of (tl_x, tl_y, br_x, br_y) tuples (world coords) by index.
    Only the numeric literals change, so the model stays otherwise identical.
    """
    with open(smv_path, "r") as f:
        content = f.read()

    for idx, (tl_x, tl_y, br_x, br_y) in enumerate(obstacles_world):
        for field, value in (("tl_x", tl_x), ("tl_y", tl_y), ("br_x", br_x), ("br_y", br_y)):
            pattern = re.compile(r"(rect_obstacles_" + field + "_" + str(idx) + r"\s*:=\s*)-?\d+(\s*;)")
            content, n = pattern.subn(r"\g<1>" + str(int(round(value))) + r"\g<2>", content)
            if n == 0:
                raise RuntimeError(f"Could not find 'rect_obstacles_{field}_{idx}' in {smv_path}.")

    with open(smv_path, "w") as f:
        f.write(content)


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
            vals[target_slot] = str(value)
            content = _replace_tpl_string_value(content, key, _format_paren_list(vals))

    packed = {
        "RECT_OBSTACLES_TL_Xs": [str(o[0]) for o in obstacles],
        "RECT_OBSTACLES_TL_Ys": [str(o[1]) for o in obstacles],
        "RECT_OBSTACLES_BR_Xs": [str(o[2]) for o in obstacles],
        "RECT_OBSTACLES_BR_Ys": [str(o[3]) for o in obstacles],
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
    def __init__(self, x, y, w, h, on_delete=None):
        super().__init__(0, 0, w, h)
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
        self.setPos(self.section.length_px, 0)

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
    """Fixed-length target section: movable and rotatable (about its source end) but not
    resizable. Rotation is constrained to multiples of the angle granularity."""
    def __init__(self, source_px, length_px, angle_deg, half_width_px, angle_granularity, on_changed):
        super().__init__(0.0, -half_width_px, length_px, 2.0 * half_width_px)
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
        
        layout.addWidget(self.view)

        # Drops a new draggable obstacle; like moving section 1, this needs an EnvModel regen
        # (the obstacle COUNT changes, which patch_smv_obstacles cannot do on the fly).
        self.add_obstacle_btn = QPushButton("Add Obstacle")
        self.add_obstacle_btn.clicked.connect(self.add_obstacle)
        layout.addWidget(self.add_obstacle_btn)

        # Model checks every configured variant in parallel and adopts the first counterexample
        # (a single-variant config is just the degenerate one-runner case).
        self.refresh_btn = QPushButton("Re-run Model Checker (first counterexample wins)")
        self.refresh_btn.clicked.connect(self.trigger_external_process)
        layout.addWidget(self.refresh_btn)

        # Stays clickable while the MC race runs; kills every nuXmv instance at once.
        self.terminate_btn = QPushButton("Terminate Model Checker (kill all nuXmv)")
        self.terminate_btn.clicked.connect(self.terminate_race)
        self.terminate_btn.setEnabled(False)
        layout.addWidget(self.terminate_btn)

        self.envgen_btn = QPushButton("Re-run EnvModel Generation")
        self.envgen_btn.clicked.connect(self.run_envmodel_generation)
        layout.addWidget(self.envgen_btn)

        self.testcase_btn = QPushButton("Generate Test Case (Smooth Birdseye)")
        self.testcase_btn.clicked.connect(self.run_testcase_generation)
        layout.addWidget(self.testcase_btn)

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

        # Target-section (section 1) editing state. Moving/rotating it invalidates the current
        # packages, so the MC re-run is blocked until the EnvModels are regenerated.
        self._section_dirty = False
        # Adding/removing obstacles also invalidates the packages (the obstacle count changes).
        self._obstacles_dirty = False
        self.section_item = None
        self._transform = None
        self._target_slot = None
        self._target_section = None
        self._ppm = None
        self.angle_granularity = parse_angle_granularity(TPL_PATH)
        # Scene items re-baselined/removed once a moved layout is committed by regeneration.
        self._trajectory_items = []
        self._obstacle_ghost_items = []
        self._section_ghost_item = None

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
        splines = connection_splines_pixel(self.trace_path, self.image_w, self.image_h)
        rects_data = obstacles_to_pixel_rects(self.trace_path, self.image_w, self.image_h)
        target_section = target_section_from_smv(self.smv_path)
        transform = compute_fit_transform(self.trace_path, self.image_w, self.image_h)
        ppm = transform[2] if transform else 50.0
        self._transform = transform
        self._ppm = ppm

        # Activate 1 m world-raster snapping for the draggable items against this scene's transform.
        global _scene_raster_snap
        if transform is not None:
            _scene_raster_snap = (lambda px, py, t=transform, iw=self.image_w, ih=self.image_h:
                                  _snap_scene_to_world_raster(px, py, t, iw, ih))
        else:
            _scene_raster_snap = None

        # The target (section 1) is user-adjustable unless it is the pinned origin section 0.
        self._target_section = target_section
        self._target_slot = target_slot_in_fixed_sections(TPL_PATH, target_section)
        draggable_target = target_section != 0 and self._target_slot is not None
        self.section_item = None

        self.scene.clear()
        self.rect_items = []
        self._trajectory_items = []
        self._obstacle_ghost_items = []
        self._section_ghost_item = None

        if segments:
            self._draw_roads(segments, splines, target_section, ppm, draggable_target)
            # Static ghosts of the ORIGINAL obstacle positions, so the starting layout stays
            # visible once the draggable (red) obstacles are moved during a session.
            self._draw_obstacle_ghosts(rects_data)
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

        self._apply_visu_mode()
        self.scale_to_fit()

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
        for pts in (splines or []):
            if len(pts) < 2:
                continue
            path = QPainterPath(QPointF(pts[0][0], pts[0][1]))
            for x, y in pts[1:]:
                path.lineTo(x, y)
            self._trajectory_items.append(self.scene.addPath(path, arc_pen))
        for x1, y1, x2, y2, sec, is_ego in segments:
            self._trajectory_items.append(
                self.scene.addLine(x1, y1, x2, y2, ego_pen if is_ego else road_pen))
        for x1, y1, x2, y2, sec, is_ego in segments:
            if is_ego:
                self._draw_parking_marking(x1, y1, x2, y2, "START", ppm)
            elif target_section is not None and sec == target_section and not draggable_target:
                self._draw_parking_marking(x1, y1, x2, y2, "TARGET", ppm)

    def _draw_parking_marking(self, x1, y1, x2, y2, label, ppm):
        """White parking-bay outline hugging a section, with a label (start/target indicator)."""
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy) or 1.0
        ux, uy = dx / length, dy / length
        nx, ny = -uy, ux                    # unit normal
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
        mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0
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
        
        rect_item = ResizableRectItem(x, y, w, h, on_delete=self._remove_obstacle)
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

    def _remove_obstacle(self, item):
        """Delete an obstacle from the scene; needs a regen before the MC can run again."""
        if item in self.rect_items:
            self.rect_items.remove(item)
        self.scene.removeItem(item)
        self._mark_obstacles_dirty()

    def _mark_obstacles_dirty(self):
        """The obstacle set changed: block the MC re-run until the EnvModels are regenerated."""
        self._obstacles_dirty = True
        self._block_mc_until_regen(
            "Obstacles added/removed - regenerate the EnvModels before re-running the MC.")

    def collect_obstacles_world(self):
        """Inverse-transform the current on-screen rectangles back to integer world coords."""
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
            obstacles.append((int(round(tl[0])), int(round(tl[1])),
                              int(round(br[0])), int(round(br[1]))))
        return obstacles

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
            self.angle_granularity, self._on_section_changed)
        self.scene.addItem(self.section_item)

    def _draw_section_ghost(self, x1, y1, length_px, angle_deg, half_w):
        """Faint outline of the target section's original pose (like the obstacle ghosts)."""
        a = math.radians(angle_deg)
        ux, uy = math.cos(a), math.sin(a)
        nx, ny = -uy, ux
        x2, y2 = x1 + length_px * ux, y1 + length_px * uy
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
            self._block_mc_until_regen(
                "Target section moved - regenerate the EnvModels before re-running the MC.")

    def _block_mc_until_regen(self, reason):
        """Grey out the MC re-run and explain that an EnvModel regeneration is required first."""
        self.refresh_btn.setEnabled(False)
        self.refresh_btn.setToolTip(reason)

    def _config_dirty(self):
        """True while the scene has un-regenerated edits (section pose or obstacle set)."""
        return self._section_dirty or self._obstacles_dirty

    def _write_config_from_scene(self):
        """Hardcode the target section pose (when adjustable) and obstacles into the tpl.json."""
        if self._transform is None:
            raise RuntimeError("No transform available to write the scene into the config.")
        obstacles = self.collect_obstacles_world()
        if self.section_item is not None and self._target_slot is not None:
            src = self.section_item.pos()
            sx_w, sy_w = pixel_to_world(src.x(), src.y(), self._transform, self.image_w, self.image_h)
            angle = int(round(self.section_item.rotation())) % 360
            patch_tpl_config(TPL_PATH, self._target_slot,
                             int(round(sx_w)), int(round(sy_w)), angle, obstacles)
        else:
            patch_tpl_config(TPL_PATH, None, None, None, None, obstacles)

    def _on_regen_success(self):
        """After a successful regeneration the drawn layout matches the config again."""
        committed = self._config_dirty()
        self._section_dirty = False
        self._obstacles_dirty = False
        if committed:
            self.refresh_btn.setEnabled(True)
            self.refresh_btn.setToolTip("")
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

    def _set_buttons_enabled(self, enabled):
        self.envgen_btn.setEnabled(enabled)
        self.testcase_btn.setEnabled(enabled)
        self.add_obstacle_btn.setEnabled(enabled)
        # The MC re-run stays disabled while section 1 or the obstacle set has un-regenerated
        # edits: the current packages are stale until EnvModel regeneration writes them in.
        self.refresh_btn.setEnabled(enabled and not self._config_dirty())

    def _run_script_with_ui(self, script, description, reload_after, on_success=None):
        """Run a vfm script with wait-cursor/disabled buttons and a completion dialog."""
        self._set_buttons_enabled(False)
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        error = None
        try:
            print(f"\u26a1 Running {description}...")
            print(_run_vfm_script(script))
        except (Exception, KeyboardInterrupt) as e:
            error = e
            print(f"{description} aborted or failed: {e}")
        finally:
            QApplication.restoreOverrideCursor()
            self._set_buttons_enabled(True)

        if error is None and on_success is not None:
            on_success()

        if error is None and reload_after:
            self.reload_from_trace()

        if error is not None:
            QMessageBox.warning(self, f"{description} failed",
                                f"{description} was aborted or failed.")
        else:
            QMessageBox.information(self, f"{description} finished",
                                    f"{description} completed successfully.")

    def run_envmodel_generation(self):
        # A moved target section or a changed obstacle set must be written into the config
        # BEFORE regenerating, so the new EnvModels reflect the drawn layout.
        if self._config_dirty():
            try:
                self._write_config_from_scene()
            except (RuntimeError, OSError, ValueError) as e:
                QMessageBox.critical(self, "Config update failed", str(e))
                return
        # Wipe existing gp* variant folders first so an old parameter range can't leave obsolete
        # packages around (regeneration recreates the current set; the vfm cache keeps this fast).
        remove_mc_packages()
        self._run_script_with_ui(ENVGEN_SCRIPT, "EnvModel generation", reload_after=False,
                                 on_success=self._on_regen_success)

    def run_testcase_generation(self):
        self._run_script_with_ui(TESTCASE_SCRIPT, "Test case generation (smooth birdseye)",
                                 reload_after=True)

    def trigger_external_process(self):
        """Race all configured variants; adopt the first counterexample and kill the rest.

        Cheap variants (few sections / coarse granularity) finish first; if one already yields a
        counterexample the race ends immediately and the more expensive configs are killed off.
        Only escalation to a harder config (or all variants finishing UNSAT) resolves otherwise.
        """
        if self._race_proc is not None:
            return  # A race is already in flight.

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
                patch_smv_obstacles(os.path.join(pkg, "EnvModel.smv"), obstacles_world)
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
        self.race_timer.start()

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
            print(f"\u26aa {os.path.basename(pkg)} finished with no counterexample (dropped out).")

    def _finish_race(self, winner, error, aborted_by_user=False):
        self.race_timer.stop()
        _kill_process_group(self._race_proc)
        self._race_proc = None
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
        if self._race_proc is None:
            return
        print("\U0001f6d1 Terminating all model-checker instances...")
        self._finish_race(winner=None, error=False, aborted_by_user=True)

    def closeEvent(self, event):
        """Never leave an orphaned worker (and its nuXmv children) running after the window closes."""
        if self._race_proc is not None:
            self.race_timer.stop()
            _kill_process_group(self._race_proc)
            self._race_proc = None
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
