import os
import re
import glob
import json
import math
import hashlib
import shutil
from pathlib import Path
from typing import List, Dict, Optional
import numpy as np
import platform
import ctypes
import ctypes.util
from contextlib import contextmanager
from ctypes import CDLL
from .morty_debug_plots import plot_cex_lengths_cumulative, plot_mc_runtimes_cumulative


def ensure_empty_file(path: str) -> None:
    p = Path(path)

    if p.exists() and not p.is_file():
        return

    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open('w'):
        pass

def min_max_curr(successful_so_far, done_so_far, max_to_expect):
    percent = 100 * successful_so_far / done_so_far
    min_good = 100 * successful_so_far / max_to_expect
    max_good = 100 * (max_to_expect - done_so_far + successful_so_far) / max_to_expect
        
    return str(round(min_good, 1)) + "% <= " + str(round(percent, 1)) + "% <= " + str(round(max_good, 1)) + "%"

# Pure pursuit.
def dpoint_following_angle(dpoint_y, ego_y, heading, ddist, bwd):
    return heading - math.atan((dpoint_y - ego_y) * bwd / ddist)

def maxDifferenceArray(A):
    maxDiff = -1
    for i in range(len(A)):
        for j in range(len(A)):
            maxDiff = max(maxDiff, abs(A[j] - A[i]))
    return maxDiff

def inverseSortingArray(egos_x: List[float]):
    b = True
    for i in range(len(egos_x) - 1):
        b = b and (egos_x[i + 1] < egos_x[i])
    return b


def _latest_nuxmv_runtime_seconds(config_name: str, generated_path_prefix: str):
    """Return latest nuXmv runtime in seconds from mc_runtimes.txt for a given config.

    Parameters:
    - config_name: name of the config directory
    - generated_path_prefix: prefix path where config dirs live
    """
    runtime_file = os.path.join(generated_path_prefix + config_name, 'mc_runtimes.txt')
    if not os.path.exists(runtime_file):
        return np.nan

    try:
        with open(runtime_file, "r", encoding="utf-8", errors="ignore") as f:
            lines = f.readlines()
    except OSError:
        return np.nan

    for line in reversed(lines):
        if "nuXmv" not in line:
            continue
        m = re.search(r"(\d+):(\d+):(\d+(?:\.\d+)?)\s+time elapsed", line)
        if m:
            hours = int(m.group(1))
            minutes = int(m.group(2))
            seconds = float(m.group(3))
            return hours * 3600 + minutes * 60 + seconds

    return np.nan
 
def _hash_file(path: str) -> str:
    h = hashlib.md5()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(8192), b''):
            h.update(chunk)
    return h.hexdigest()

def _snapshot_configs(ucd_config_prios_str: List[str], generated_path_prefix: str, restrict_to: Optional[Dict[str, List[str]]] = None):
    """Return {config_name: {rel_path: md5_hex}} for all config dirs.
    If restrict_to is given, only include rel_paths present in that dict."""
    snap = {}
    for config_name in ucd_config_prios_str:
        config_dir = generated_path_prefix + config_name
        allowed = restrict_to.get(config_name) if restrict_to else None
        file_hashes = {}
        for dirpath, _, filenames in os.walk(config_dir):
            for fname in filenames:
                full_path = os.path.join(dirpath, fname)
                rel_path = os.path.relpath(full_path, config_dir)
                if allowed is not None and rel_path not in allowed:
                    continue
                file_hashes[rel_path] = _hash_file(full_path)
        snap[config_name] = file_hashes
    return snap

def _save_configs_to_archive(seedo: int, subfolder: str, generated_path_prefix: str, ucd_config_prios_str: List[str], restrict_to: Optional[Dict[str, List[str]]] = None):
    """Copy config files into the detailed archive under the given subfolder.
    If restrict_to is given ({config_name: set_of_rel_paths}), only copy those files."""
    archive_path = f'{generated_path_prefix}/detailed_archive/run_{seedo}/{subfolder}/'
    for config_name in ucd_config_prios_str:
        config_dir = generated_path_prefix + config_name
        allowed = restrict_to.get(config_name) if restrict_to else None
        for dirpath, _, filenames in os.walk(config_dir):
            for fname in filenames:
                full_path = os.path.join(dirpath, fname)
                rel_path = os.path.relpath(full_path, config_dir)
                if allowed is not None and rel_path not in allowed:
                    continue
                dest = os.path.join(archive_path + config_name, rel_path)
                os.makedirs(os.path.dirname(dest), exist_ok=True)
                shutil.copy2(full_path, dest)

def archive(seedo: int, global_counter: int, detailed_archive_flag: bool, generated_path_prefix: str, ucd_config_prios_str: List[str], snapshot_hashes: Dict[str, Dict[str, str]], selected_config: Optional[str] = None):
    if detailed_archive_flag:
        archive_path = f'{generated_path_prefix}/detailed_archive/run_{seedo}/iteration_{global_counter}/'
        if not os.path.exists(archive_path):
            os.makedirs(archive_path)
        for config_name in ucd_config_prios_str:
            config_dir = generated_path_prefix + config_name
            baseline = snapshot_hashes.get(config_name, {})
            for dirpath, _, filenames in os.walk(config_dir):
                for fname in filenames:
                    full_path = os.path.join(dirpath, fname)
                    rel_path = os.path.relpath(full_path, config_dir)
                    current_hash = _hash_file(full_path)
                    if rel_path not in baseline or baseline[rel_path] != current_hash:
                        dest = os.path.join(archive_path + config_name, rel_path)
                        os.makedirs(os.path.dirname(dest), exist_ok=True)
                        shutil.copy2(full_path, dest)
        # Record which config was selected this iteration. This is the single source
        # of truth for downstream tooling: it cannot be reliably reconstructed from
        # the raw archived files.
        if selected_config:
            with open(os.path.join(archive_path, 'selected_config.txt'), 'w') as f:
                f.write(selected_config + '\n')

def _run_seed_map_path(generated_path_prefix: str) -> str:
    return generated_path_prefix + "/run_seeds.json"

def dump_run_seed_map(run_seed_map: Dict[int, int], generated_path_prefix: str) -> None:
    """Persist the run_id -> seed mapping so a later --dryrun can reproduce the exact
    car initialization for each run ID. Keys are stored as strings (JSON requirement)."""
    path = _run_seed_map_path(generated_path_prefix)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump({str(run_id): seed for run_id, seed in run_seed_map.items()}, f, indent=2, sort_keys=True)

def load_run_seed_pairs(generated_path_prefix: str):
    """Load the persisted run_id -> seed mapping as a list of (run_id, seed) pairs sorted by
    run_id. Returns an empty list if no mapping file exists (e.g. pre-split archives)."""
    path = _run_seed_map_path(generated_path_prefix)
    if not os.path.exists(path):
        return []
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return []
    return [(int(run_id), int(seed)) for run_id, seed in sorted(data.items(), key=lambda kv: int(kv[0]))]

def discard_run_files(run_id: int, generated_path_prefix: str, ucd_config_prios_str: List[str], mc_results_offsets: Dict[str, int]) -> None:
    """Remove every on-disk artifact of a run that was excluded for failing before the
    minimum-iterations threshold, so excluded runs leave no data/statistics/archives behind."""
    # Per-run debug plots.
    for name in (f"mc_runtime_debug_{run_id}.pdf", f"cex_length_debug_{run_id}.pdf"):
        plot_path = f"{generated_path_prefix}/{name}"
        if os.path.exists(plot_path):
            os.remove(plot_path)

    # Recorded videos.
    for video_file in glob.glob(f"{generated_path_prefix}/videos/vid_{run_id}*"):
        try:
            os.remove(video_file)
        except OSError:
            pass

    # Detailed archive folder and any (partial) zip for this run.
    archive_folder = f"{generated_path_prefix}/detailed_archive/run_{run_id}"
    if os.path.isdir(archive_folder):
        shutil.rmtree(archive_folder, ignore_errors=True)
    for zip_file in glob.glob(f"{generated_path_prefix}/detailed_archive/run_{run_id}.zip*"):
        try:
            os.remove(zip_file)
        except OSError:
            pass

    # Truncate the per-config MC results log back to its pre-run length, dropping this run's lines.
    for config_name, offset in mc_results_offsets.items():
        results_path = generated_path_prefix + config_name + "/morty_mc_results.txt"
        if os.path.exists(results_path):
            with open(results_path, "r+b") as f:
                f.truncate(offset)


class RunScheduler:
    """Decouples simulation seeds from run IDs.

    In a normal run it keeps issuing fresh seeds and only advances the run ID once a run is
    committed (i.e. it reached the min-iterations threshold), so early, unsolvable failures
    neither consume a run ID nor leave artifacts behind. In a dry run it instead replays the
    recorded (run_id, seed) pairs so the exact same car initialization is reproduced.

    Usage:
        scheduler = RunScheduler(generated_path_prefix, max_exps, args.dryrun)
        for run_id, seed in scheduler:
            ... run ...
            if valid:
                scheduler.commit()   # advances run_id and persists the seed
            # otherwise just continue; the next seed reuses the same run_id
    """

    def __init__(self, generated_path_prefix: str, max_exps: int, dryrun: bool):
        self.generated_path_prefix = generated_path_prefix
        self.max_exps = max_exps
        self.dryrun = bool(dryrun)
        self.run_id = 0
        self.seed = -1
        self.run_seed_map: Dict[int, int] = {}
        if self.dryrun:
            # Backward compatibility with archives created before the seed/run_id split.
            self.dryrun_pairs = load_run_seed_pairs(generated_path_prefix) or [(i, i) for i in range(max_exps)]
        else:
            self.dryrun_pairs = None
            dump_run_seed_map(self.run_seed_map, generated_path_prefix)  # Reset the mapping for a fresh session.
        self.dryrun_index = 0

    def __iter__(self):
        return self

    def __next__(self):
        if self.dryrun:
            if self.dryrun_index >= len(self.dryrun_pairs):
                raise StopIteration
            self.run_id, self.seed = self.dryrun_pairs[self.dryrun_index]
            self.dryrun_index += 1
        else:
            if self.run_id >= self.max_exps:
                raise StopIteration
            self.seed += 1  # run_id advances only once a run turns out to be valid (see commit()).
        return self.run_id, self.seed

    def commit(self):
        """Persist the seed for the current run_id and advance to the next run ID.

        No-op for dry runs, which iterate a fixed (run_id, seed) list."""
        if not self.dryrun:
            self.run_seed_map[self.run_id] = self.seed
            dump_run_seed_map(self.run_seed_map, self.generated_path_prefix)
            self.run_id += 1


def snapshot_mc_results_offsets(ucd_config_prios_str: List[str], generated_path_prefix: str) -> Dict[str, int]:
    """Record the current byte length of each config's morty_mc_results.txt so that an excluded
    run's appended lines can later be truncated away again (see discard_run)."""
    offsets = {}
    for config_name in ucd_config_prios_str:
        results_path = generated_path_prefix + config_name + "/morty_mc_results.txt"
        offsets[config_name] = os.path.getsize(results_path) if os.path.exists(results_path) else 0
    return offsets


def run_reached_threshold(success: bool, global_counter: int, min_iterations: int, dryrun: bool) -> bool:
    """Whether a run counts as valid. A dry run and any (quick) success always count; a genuine
    run counts once it reached min_iterations iterations. Only early FAILURES return False."""
    return bool(dryrun) or success or global_counter >= min_iterations


def discard_run(run_id: int, generated_path_prefix: str, ucd_config_prios_str: List[str],
                mc_results_offsets: Dict[str, int], all_cex_length_histories: dict,
                all_selected_runtime_histories: dict) -> None:
    """Undo everything an excluded (early-failing) run produced: drop its in-memory statistics,
    regenerate the cumulative plots without it, and delete its on-disk artifacts."""
    all_cex_length_histories.pop(run_id, None)
    all_selected_runtime_histories.pop(run_id, None)
    plot_cex_lengths_cumulative(all_cex_length_histories, f"{generated_path_prefix}/cex_length_debug_all.pdf")
    plot_mc_runtimes_cumulative(all_selected_runtime_histories, f"{generated_path_prefix}/mc_runtime_debug_all.pdf")
    plot_mc_runtimes_cumulative(all_selected_runtime_histories, f"{generated_path_prefix}/mc_runtime_debug_all_log.pdf", log_scale=True)
    discard_run_files(run_id, generated_path_prefix, ucd_config_prios_str, mc_results_offsets)

if platform.system() == 'Windows':
    kernel32 = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel32.FreeLibrary.argtypes = [ctypes.c_void_p]
    kernel32.FreeLibrary.restype = ctypes.c_int
elif platform.system() == 'Linux':
    libc = ctypes.CDLL(ctypes.util.find_library('c'))
    libc.dlclose.argtypes = [ctypes.c_void_p]
    libc.dlclose.restype = ctypes.c_int
    
@contextmanager
def clean_library_context(lib_path):
    try:
        lib = ctypes.CDLL(lib_path)
        yield lib
    finally:
        if 'lib' in locals():
            handle = lib._handle
            
            if platform.system() == 'Windows':
                kernel32.FreeLibrary(handle)
            elif platform.system() == 'Linux':
                libc.dlclose(handle)
        else:
            print("Warning: Library object not found for cleanup.")
               
@contextmanager
def morty_script_context():
    # Assum Linux until...
    dll_name = 'libvfm.so'
    dll_dir = './lib'
    
    if platform.system() == 'Windows': # ...proven otherwise.
        dll_name = 'VFM_MAIN_LIB.dll'
        dll_dir = os.path.join(os.getcwd(), 'bin')
        # Ensure the directory with native DLLs is discoverable on Windows so CDLL can load dependencies.
        # Add bin to PATH if not already present (works on Python 3.7+).
        if dll_dir not in os.environ.get('PATH', ''):
            os.environ['PATH'] = dll_dir + os.pathsep + os.environ.get('PATH', '')
   
    with clean_library_context(dll_dir + '/' + dll_name) as morty_lib:
        morty_lib.expandScript.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_size_t]
        morty_lib.expandScript.restype = ctypes.c_char_p
        
        yield morty_lib

# --- How to use
# for item in [...]:
#     with morty_script_context() as morty_lib:
#         # morty_lib is already loaded, configured, and will be cleaned up
#         res1 = morty_lib.expandScript(b"call_one", b"data", 1024)
#         res2 = morty_lib.expandScript(b"call_two", b"data", 1024)


# ============================================================================
# Visualization / painting helpers.
# These keep highway-env rendering monkey-patches and geometry math out of the
# pure model-checking loop in morty.py. Nothing here is needed for the MC logic
# itself; it only affects how frames are drawn.
# ============================================================================

# --- Fixed geometry of the generated plain_road PNG -------------------------
# Measured directly from a generated plain_road_0.png (2400x1820, see repo
# memory plain-road-image-geometry.md). The generator (Plain2DTranslator) is
# Euclidean in meters: the lane-based lateral layout is already resolved into
# pixels when painting, so the SAME pixels-per-meter applies to BOTH axes (no
# stretch, no per-axis scale). We therefore use one uniform scale:
#   image_px = world_m * BG_PIXELS_PER_METER + BG_ZERO_PIXEL   (per axis)
# The PNG is regenerated for the current scene each step and already contains
# the full, correct road graph (merges/branches/curves) at the right places, so
# we blit it EXACTLY ONCE with a single uniform scale (no tiling, no stretch).
# A fixed image pixel is pinned to world x=0 (ego-independent). If the road is
# shorter than the visible region, the fix is to generate a longer road, NOT to
# repeat pixels (which would corrupt non-straight graphs).
# T&E knobs:
#   * BG_PIXELS_PER_METER = 12: CONFIRMED. Lane 48 px / 4.0 m; scaled road width
#     matches HE exactly. Same value governs longitudinal spacing (dashes).
#   * BG_ZERO_PIXEL_X = 500 (Plain2DTranslator x-offset; world x 0 -> image x 500).
#   * BG_ZERO_PIXEL_Y = 1019: CONFIRMED. Center of the road band.
BG_PIXELS_PER_METER = 12.0
BG_ZERO_PIXEL_X = 0.0
BG_ZERO_PIXEL_Y = 961.0


class VizState:
    """Shared mutable state for the visualization monkey-patches.

    Held in a single object so the patched render callbacks (installed once) and
    the main loop in morty.py refer to the same containers by reference. This is
    what lets the MC loop stay free of loose module-level globals.
    """

    def __init__(self):
        self.trajectories = {}    # vehicle id -> list of [x, y, priority]
        self.pp_targets = {}      # vehicle id -> [x, y] pure-pursuit target
        self.selected_cnt = None  # currently active CEX priority (for coloring)
        self.pos_to_draw = []     # list of [coord, color] planned MC positions


def get_scene_bounding_box(env, car_ids):
    """
    Finds the tightest axis-aligned (unrotated) bounding box that contains 
    every point of the specified cars in the highway-env scene.
    
    Args:
        env: The highway-env / gymnasium environment instance.
        car_ids (list): List of vehicle IDs to include.
        
    Returns:
        dict: A dictionary containing the bounding box boundaries:
              xmin, ymin, xmax, ymax and the box dimensions.
    """
    all_corners = []
    
    # Access all active vehicles in the current road scene
    vehicles = env.unwrapped.road.vehicles
    
    # Filter for the requested cars based on ID
    # Note: Depending on your wrapper/setup, you can match by ID, or index.
    # Here, we assume vehicles can be filtered or matched. If you have unique custom IDs:
    target_vehicles = [v for v in vehicles if getattr(v, 'id', None) in car_ids]
    
    # Fallback if your vehicles do not have an 'id' attribute (match by list index):
    if not target_vehicles:
        for idx, v in enumerate(vehicles):
            if idx in car_ids:
                target_vehicles.append(v)
                
    if not target_vehicles:
        raise ValueError(
            f"No IDs found in scene. Specified: {car_ids}, Available: {vehicles}")

    for vehicle in target_vehicles:
        # Correctly unpack position coordinates
        x, y = vehicle.position[0], vehicle.position[1]
        heading = vehicle.heading
        length = vehicle.LENGTH
        width = vehicle.WIDTH
        
        # Define corners relative to the vehicle's center (local coordinates)
        local_corners = np.array([
            [length / 2,  width / 2],
            [length / 2, -width / 2],
            [-length / 2, -width / 2],
            [-length / 2,  width / 2]
        ])
        
        # 2. Construct 2D rotation matrix for the heading angle
        cos_h = np.cos(heading)
        sin_h = np.sin(heading)
        rotation_matrix = np.array([
            [cos_h, -sin_h],
            [sin_h,  cos_h]
        ])
        
        # 3. Rotate and translate corners to global coordinates
        global_corners = (local_corners @ rotation_matrix.T) + np.array([x, y])
        all_corners.extend(global_corners)
        
    # 4. Find the global minimum and maximum across all gathered corners
    all_corners = np.array(all_corners)
    x_min, y_min = np.min(all_corners, axis=0)
    x_max, y_max = np.max(all_corners, axis=0)
    
    return {
        "xmin": x_min,
        "ymin": y_min,
        "xmax": x_max,
        "ymax": y_max,
        "width": x_max - x_min,
        "height": y_max - y_min
    }


def blit_background_rigid(surface, image, world_ref_x_m, world_ref_y_m, num_lanes,
                          pixels_per_meter=BG_PIXELS_PER_METER,
                          ref_pixel_x=BG_ZERO_PIXEL_X,
                          ref_pixel_y=BG_ZERO_PIXEL_Y):
    """
    Blit the generated road PNG so image pixel (ref_pixel_x, ref_pixel_y) lands
    exactly on world coordinate (world_ref_x_m, world_ref_y_m) under the current
    camera. A SINGLE uniform scale is used for both axes (the source PNG is
    Euclidean in meters, so it must not be stretched) and the image is blitted
    exactly once: it already encodes the full road graph for the current scene,
    so there is nothing to tile. The reference pixel stays pinned to the given
    world coordinate, keeping the road fixed regardless of the ego.
    """
    import pygame

    # screen px per image px = (screen px / world m) / (image px / world m)
    scale = surface.scaling / pixels_per_meter
    src_w, src_h = image.get_size()
    scaled_w = max(1, int(round(src_w * scale)))
    scaled_h = max(1, int(round(src_h * scale)))
    # Nearest-neighbour scale (not smoothscale) so the (0,0,0) colorkey stays
    # exact and no anti-aliased near-black fringe leaks through.
    scaled_bg = pygame.transform.scale(image, (scaled_w, scaled_h))
    scaled_bg.set_colorkey((0, 0, 0))

    anchor_screen_x, anchor_screen_y = surface.vec2pix((world_ref_x_m, world_ref_y_m))
    blit_x = anchor_screen_x - ref_pixel_x * scale
    # TODO: replace 4 with actual lane width; TODO: find out why odd num_lanes behave differently than even.
    blit_y = anchor_screen_y - (ref_pixel_y + (num_lanes - 1.0) * pixels_per_meter * 4.0 / 2.0 - 1.0 * (num_lanes - 1) % 2) * scale

    surface.blit(scaled_bg, (int(round(blit_x)), int(round(blit_y))))


def get_road_world_rect(env):
    """
    Compute the world-space rectangle (in meters) covered by the HE road network,
    including lateral lane width. Returns (x_min, y_min, width, height).
    """
    road = env.unwrapped.road
    xs = []
    ys = []
    for lane in road.network.lanes_list():
        for longitudinal in (0.0, lane.length):
            half_width = lane.width_at(longitudinal) / 2.0
            for lateral in (-half_width, half_width):
                pos = lane.position(longitudinal, lateral)
                xs.append(float(pos[0]))
                ys.append(float(pos[1]))
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    return x_min, y_min, x_max - x_min, y_max - y_min


def get_background_world_rect(image, world_ref_x_m, world_ref_y_m, num_lanes,
                              pixels_per_meter=BG_PIXELS_PER_METER,
                              ref_pixel_x=BG_ZERO_PIXEL_X,
                              ref_pixel_y=BG_ZERO_PIXEL_Y):
    """Return the world-space rectangle covered by the generated background image."""
    src_w, src_h = image.get_size()
    lane_offset_px = (num_lanes - 1.0) * pixels_per_meter * 4.0 / 2.0 - 1.0 * (num_lanes - 1) % 2
    anchor_pixel_y = ref_pixel_y + lane_offset_px

    x_min = world_ref_x_m - ref_pixel_x / pixels_per_meter
    x_max = world_ref_x_m + (src_w - ref_pixel_x) / pixels_per_meter
    y_min = world_ref_y_m - anchor_pixel_y / pixels_per_meter
    y_max = world_ref_y_m + (src_h - anchor_pixel_y) / pixels_per_meter
    return x_min, y_min, x_max - x_min, y_max - y_min


def get_background_content_pixel_bbox(image, black_tol=10):
    """Return the tight pixel bbox (px_min_x, px_min_y, px_max_x, px_max_y) of the
    non-black (i.e. actual road) content in the generated background PNG.

    The PNG is padded with (0,0,0) black that is rendered transparent via the
    colorkey, so its raw rectangle is much larger than the drawn road. Framing on
    this content bbox avoids centering on that empty padding. Returns None if the
    image is entirely black.
    """
    import pygame

    arr = pygame.surfarray.array3d(image)  # shape (width, height, 3)
    mask = np.any(arr > black_tol, axis=2)  # True where a pixel is non-black
    cols = np.any(mask, axis=1)  # any content per x-column
    rows = np.any(mask, axis=0)  # any content per y-row
    if not cols.any() or not rows.any():
        return None
    xs = np.where(cols)[0]
    ys = np.where(rows)[0]
    return int(xs[0]), int(ys[0]), int(xs[-1]), int(ys[-1])


def get_background_content_world_rect(image, world_ref_x_m, world_ref_y_m, num_lanes,
                                      content_px_bbox=None,
                                      pixels_per_meter=BG_PIXELS_PER_METER,
                                      ref_pixel_x=BG_ZERO_PIXEL_X,
                                      ref_pixel_y=BG_ZERO_PIXEL_Y):
    """World-space rectangle covered by the actual (non-black) road content of the
    generated background image, using the same pixel<->world anchoring as
    blit_background_rigid. Falls back to the full-image rect if there is no
    content. Returns (x_min, y_min, width, height)."""
    if content_px_bbox is None:
        content_px_bbox = get_background_content_pixel_bbox(image)
    if content_px_bbox is None:
        return get_background_world_rect(
            image, world_ref_x_m, world_ref_y_m, num_lanes,
            pixels_per_meter, ref_pixel_x, ref_pixel_y)

    px_min_x, px_min_y, px_max_x, px_max_y = content_px_bbox
    lane_offset_px = (num_lanes - 1.0) * pixels_per_meter * 4.0 / 2.0 - 1.0 * (num_lanes - 1) % 2
    anchor_pixel_y = ref_pixel_y + lane_offset_px

    x_min = world_ref_x_m + (px_min_x - ref_pixel_x) / pixels_per_meter
    x_max = world_ref_x_m + (px_max_x - ref_pixel_x) / pixels_per_meter
    y_min = world_ref_y_m + (px_min_y - anchor_pixel_y) / pixels_per_meter
    y_max = world_ref_y_m + (px_max_y - anchor_pixel_y) / pixels_per_meter
    return x_min, y_min, x_max - x_min, y_max - y_min


def install_vehicle_graphics_patches(args, viz_state):
    """Patch highway-env's VehicleGraphics for morty's rendering.

    - get_color: make crash red take priority over morty's custom colors.
    - display: overlay per-vehicle trajectories, pure-pursuit targets, planned
      MC positions and vehicle-index labels.

    All mutable drawing state lives on `viz_state`; `args` toggles overlays.
    """
    from highway_env.vehicle.graphics import VehicleGraphics

    # COP: Patch VehicleGraphics.get_color so that crashed always takes priority over custom color.
    # Without this, highway-env checks vehicle.color BEFORE vehicle.crashed, so any custom
    # color (blue for blind, green for CEX) would suppress the native red crash rendering —
    # including during the intermediate simulation sub-steps within env.step().
    _original_get_color = VehicleGraphics.get_color

    @classmethod
    def _patched_get_color(cls, vehicle, transparent=False):
        if vehicle.crashed:
            color = cls.RED
            if transparent:
                color = (color[0], color[1], color[2], 30)
            return color
        return _original_get_color.__func__(cls, vehicle, transparent)
    VehicleGraphics.get_color = _patched_get_color

    # COP: Patch VehicleGraphics.display to draw trajectories and car IDs.
    # Important: Draw trajectories from inside VehicleGraphics.display so lines are rendered
    # before the frame blit/flip done by EnvViewer.display.
    _original_display = VehicleGraphics.display.__func__

    @classmethod
    def _patched_display(cls, vehicle, surface, transparent=False, offscreen=False, label=False, draw_roof=False):
        try:
            import pygame

            # Persist trajectory in world coordinates and draw it in the current camera view.
            vehicle_id = id(vehicle)
            if vehicle_id not in viz_state.trajectories:
                viz_state.trajectories[vehicle_id] = []

            trajectory = viz_state.trajectories[vehicle_id]
            current_position = [float(vehicle.position[0]), float(vehicle.position[1])]
            stationary = abs(getattr(vehicle, 'speed', 0.0)) < 0.1

            if not trajectory:
                trajectory.append([*current_position, viz_state.selected_cnt])
            elif not stationary:
                last_position = trajectory[-1]
                dx = current_position[0] - last_position[0]
                dy = current_position[1] - last_position[1]
                moved = abs(dx) > 0.05 or abs(dy) > 0.05
                # Ignore one-time teleport/snap after initialization to avoid fake long tails.
                if moved:
                    jump2 = dx * dx + dy * dy
                    if len(trajectory) == 1 and jump2 > 100.0:  # >10 m in one render step is likely a teleport.
                        trajectory[0] = [*current_position, viz_state.selected_cnt]
                    else:
                        trajectory.append([*current_position, viz_state.selected_cnt])
                        if len(trajectory) > 2000:
                            viz_state.trajectories[vehicle_id] = trajectory[-2000:]

            if not args.hide_trajectories and len(trajectory) > 1:
                # Draw trajectory segments colored by priority
                priority_color_map = {
                    None: (100, 150, 200),
                    0: (31, 119, 180),    # tab:blue
                    1: (255, 127, 14),    # tab:orange
                    2: (44, 160, 44),     # tab:green
                    3: (214, 39, 40),     # tab:red
                    4: (148, 103, 189),   # tab:purple
                    5: (140, 86, 75),     # tab:brown
                }
                font = pygame.font.Font(None, 14)
                for i in range(len(trajectory) - 1):
                    p1_pix = surface.pos2pix(trajectory[i][0], trajectory[i][1])
                    p2_pix = surface.pos2pix(trajectory[i + 1][0], trajectory[i + 1][1])
                    priority = trajectory[i][2] if len(trajectory[i]) > 2 else None
                    color = priority_color_map.get(priority, (100, 150, 200))
                    pygame.draw.line(surface, color, p1_pix, p2_pix, width=2)

                    # Label each priority block once, near its first segment.
                    prev_priority = trajectory[i - 1][2] if i > 0 and len(trajectory[i - 1]) > 2 else None
                    if i == 0 or priority != prev_priority:
                        label = "X" if priority is None else str(priority)
                        midx = (p1_pix[0] + p2_pix[0]) // 2
                        midy = (p1_pix[1] + p2_pix[1]) // 2
                        text = font.render(label, True, (20, 20, 20), (255, 255, 255))
                        if args.show_prio_numbers:
                            surface.blit(text, (midx + 2, midy - 10))

            if not args.hide_pure_pursuit and vehicle_id in viz_state.pp_targets:
                target = viz_state.pp_targets[vehicle_id]
                # Offset line start to front axle instead of vehicle center
                front_offset = vehicle.LENGTH / 2.0
                p_vehicle_x = current_position[0] + front_offset * np.cos(vehicle.heading)
                p_vehicle_y = current_position[1] + front_offset * np.sin(vehicle.heading)
                p_vehicle = surface.pos2pix(p_vehicle_x, p_vehicle_y)
                p_target = surface.pos2pix(target[0], target[1])
                pygame.draw.line(surface, (255, 140, 0), p_vehicle, p_target, width=2)
                pygame.draw.circle(surface, (255, 180, 0), p_target, 5)

        except Exception as e:
            print(f"Warning: Error drawing trajectories: {e}")
            input("Press Enter to continue..." + str(e))

        _original_display(cls, vehicle, surface, transparent=transparent, offscreen=offscreen, label=False, draw_roof=draw_roof)

        try:
            import pygame

            for pos in viz_state.pos_to_draw:
                pixx = surface.pos2pix(pos[0][0], pos[0][1])
                pygame.draw.circle(surface, pos[1], pixx, 3)

        except Exception as e:
            raise RuntimeError(f"Error drawing global positions: {e}")

        if not surface.is_visible(vehicle.position):
            return
        try:
            import pygame
            idx = vehicle.road.vehicles.index(vehicle)
            font = pygame.font.Font(None, 18)
            text = font.render(str(idx), True, (0, 0, 0), (255, 255, 255))
            position = [*surface.pos2pix(vehicle.position[0], vehicle.position[1])]
            surface.blit(text, (position[0] - 5, position[1] - 15))
        except (ValueError, AttributeError):
            pass
    VehicleGraphics.display = _patched_display


def install_world_surface_patches(bg_image_state, num_lanes, fit_background=False):
    """Patch highway-env's WorldSurface/RoadGraphics for morty's camera + road.

    - WorldSurface.move_display_window_to: frame the camera on ALL vehicles
      (instead of a single ego) using the scene bounding box.
    - RoadGraphics.display: after the native HE road (grey fill + lane markings),
      blit the generated "background" road on top of it, so the HE road sits
      UNDERNEATH the generated road while vehicles (drawn later) stay on top.

    `bg_image_state` is the mutable dict holding the loaded/converted PNG.
    """
    from highway_env.road.graphics import WorldSurface, RoadGraphics
    import highway_env as _he
    import pygame  # Required to draw/scale images

    # Publish the CURRENT render state on the highway_env module so the
    # install-once RoadGraphics.display patch below always reads the latest
    # bg_image_state, not a dict captured on the first call. The caller recreates
    # bg_image_state for every run/seed; with the seed scheduler the first seed(s)
    # may be discarded before ever loading a background, which previously pinned
    # the patch to an empty dict and suppressed the background for all later runs.
    _he._morty_bg_state = bg_image_state
    _he._morty_bg_num_lanes = num_lanes
    _he._morty_bg_fit = fit_background

    _orig_move_display_window_to = WorldSurface.move_display_window_to

    def _move_display_window_to_all(self, position):
        try:
            env_ref = getattr(_he, '_display_env', None)
            if env_ref is None:
                return _orig_move_display_window_to(self, position)

            vehicles = getattr(env_ref.unwrapped, 'road').vehicles
            if not vehicles:
                return _orig_move_display_window_to(self, position)

            bg_state = getattr(_he, '_morty_bg_state', None)
            nlanes = getattr(_he, '_morty_bg_num_lanes', num_lanes)
            fit = getattr(_he, '_morty_bg_fit', False)

            if fit and bg_state is not None and bg_state["image"] is not None:
                image = bg_state["image"]
                road_x, road_y, road_w, road_h = get_road_world_rect(env_ref)
                # Cache the (expensive) non-black content scan per image object.
                if bg_state.get("content_bbox_for") != id(image):
                    bg_state["content_bbox"] = get_background_content_pixel_bbox(image)
                    bg_state["content_bbox_for"] = id(image)
                bbox_x, bbox_y, bbox_w, bbox_h = get_background_content_world_rect(
                    image,
                    world_ref_x_m=0.0,
                    world_ref_y_m=road_y + road_h / 2.0,
                    num_lanes=nlanes,
                    content_px_bbox=bg_state["content_bbox"],
                )
                bbox = {
                    "xmin": bbox_x,
                    "ymin": bbox_y,
                    "xmax": bbox_x + bbox_w,
                    "ymax": bbox_y + bbox_h,
                }
            else:
                bbox = get_scene_bounding_box(env_ref, [i for i in range(1000)])

            # World-space margin (in meters) kept around the framed content on
            # every side, so nothing sits flush against the screen edge.
            margin_m = 10
            target_width_m = (bbox["xmax"] - bbox["xmin"]) + (margin_m * 2)
            target_height_m = (bbox["ymax"] - bbox["ymin"]) + (margin_m * 2)
            screen_width, screen_height = self.get_width(), self.get_height()
            scale_x = screen_width / max(1.0, target_width_m)
            scale_y = screen_height / max(1.0, target_height_m)
            self.scaling = min(scale_x, scale_y)

            # Center the bounding box exactly in the middle of the screen,
            # independent of `self.centering_position` (which highway-env may set
            # to an off-center value and would otherwise shift content off-screen).
            center = np.array([
                (bbox["xmin"] + bbox["xmax"]) / 2.0,
                (bbox["ymin"] + bbox["ymax"]) / 2.0,
            ])
            self.origin = center - np.array([
                self.get_width() / (2.0 * self.scaling),
                self.get_height() / (2.0 * self.scaling),
            ])
        except Exception as e:
            _orig_move_display_window_to(self, position)
            raise e

    WorldSurface.move_display_window_to = _move_display_window_to_all
    # Provide a hook so the patched function can find the current env.
    _he._display_env = None

    # Patch RoadGraphics.display so the generated "background" road is blitted
    # AFTER the highway-env road (grey fill + HE lane markings). This puts the
    # HE road UNDERNEATH the generated road; vehicles are drawn afterwards by
    # display_traffic, so they end up on top of both. Patch only once.
    if not getattr(RoadGraphics, "_morty_bg_patched", False):
        _orig_road_display = RoadGraphics.display

        def _road_display_with_bg(road, surface):
            _orig_road_display(road, surface)
            try:
                env_ref = getattr(_he, '_display_env', None)
                # Read the CURRENT state (updated on every install call) rather than
                # a dict captured once, so a background loaded in a later run is used.
                bg_state = getattr(_he, '_morty_bg_state', None)
                nlanes = getattr(_he, '_morty_bg_num_lanes', num_lanes)
                if (env_ref is None or bg_state is None
                        or bg_state["image"] is None):
                    return
                # convert_alpha requires an initialized display/surface.
                if (not bg_state["converted"] and pygame.display.get_init()
                        and pygame.display.get_surface() is not None):
                    bg_state["image"] = bg_state["image"].convert_alpha()
                    bg_state["image"].set_colorkey((0, 0, 0))
                    bg_state["converted"] = True
                # Derive background world extent/anchor from HE road geometry so the
                # painted road matches highway-env's road under camera scale/translation.
                road_x, road_y, road_w, road_h = get_road_world_rect(env_ref)
                blit_background_rigid(
                    surface,
                    bg_state["image"],
                    world_ref_x_m=0.0,
                    world_ref_y_m=road_y + road_h / 2.0,
                    num_lanes = nlanes
                )
            except Exception as e:
                raise e

        RoadGraphics.display = staticmethod(_road_display_with_bg)
        RoadGraphics._morty_bg_patched = True
