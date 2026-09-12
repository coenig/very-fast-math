import sys
import re
import math
import os
import platform
import shutil
import ctypes
import ctypes.util
from contextlib import contextmanager
from ctypes import create_string_buffer, sizeof
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QPushButton, QGraphicsView, QGraphicsScene, QGraphicsRectItem,
                             QMessageBox)
from PyQt6.QtCore import Qt, QRectF
from PyQt6.QtGui import QPixmap, QBrush, QPen, QColor


# Repo root = parent of this parking/ folder; anchors all paths independent of the caller's cwd.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# --- Fixed locations for the parking config (see EnvModel generation into examples/gp_config). ---
TRACE_PATH = os.path.join(REPO_ROOT, "examples/gp_config/debug_trace_array.txt")
SMV_PATH = os.path.join(REPO_ROOT, "examples/gp_config/EnvModel.smv")
IMAGE_PATH = os.path.join(REPO_ROOT, "examples/gp_config/0/preview2/preview2_0.png")
# Re-runs the model checker on the already-generated EnvModel.smv (no tpl.json regeneration).
# Path is relative to bin/, the working directory the MC runs in (see run_model_checker).
MC_SCRIPT = "@{../src/templates/envmodel_config.tpl.json}@.runMCJobs[16]"


def _read_trace_values(trace_path):
    """Parse a debug_trace_array.txt dump into a {name: float} map.

    Names keep whatever prefix they have in the file (e.g. 'env.section_0.source.x').
    """
    values = {}
    line_re = re.compile(r'^\s*([A-Za-z_][\w.]*)\s*=\s*(-?\d+(?:\.\d+)?)\s*$')
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


def run_model_checker():
    """Invoke the vfm library to re-run the MC on the current EnvModel.smv. Returns stdout text.

    The config's relative paths (../examples, ../external, ...) resolve from bin/, so the call
    must run with bin/ as the working directory.
    """
    result = create_string_buffer(1000000)
    prev_cwd = os.getcwd()
    os.chdir(os.path.join(REPO_ROOT, 'bin'))
    try:
        with _vfm_lib_context() as lib:
            res = lib.expandScript(MC_SCRIPT.encode('utf-8'), result, sizeof(result))
    finally:
        os.chdir(prev_cwd)
    return res.decode(errors="replace") if res else ""


def trace_has_counterexample(trace_path):
    """A CEX result file contains the counterexample marker; a blind run does not."""
    if not os.path.isfile(trace_path):
        return False
    with open(trace_path, "r", errors="replace") as f:
        content = f.read()
    return ("Trace Type: Counterexample" in content
            or "as demonstrated by the following" in content)

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
        # so it works correctly regardless of the view's zoom/fit scale.
        parent = self.parent_rect
        local = parent.mapFromScene(event.scenePos())
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


# Custom rectangle item with handles
class ResizableRectItem(QGraphicsRectItem):
    def __init__(self, x, y, w, h):
        super().__init__(0, 0, w, h)
        self.setPos(x, y)
        self._updating_handles = False
        
        self.setPen(QPen(QColor("red"), 2))
        self.setBrush(QBrush(Qt.GlobalColor.transparent))
        
        self.setFlags(
            QGraphicsRectItem.GraphicsItemFlag.ItemIsMovable |
            QGraphicsRectItem.GraphicsItemFlag.ItemIsSelectable |
            QGraphicsRectItem.GraphicsItemFlag.ItemSendsGeometryChanges
        )

        self.tl_handle = None
        self.br_handle = None

        self.tl_handle = ResizeHandle(self, is_bottom_right=False)
        self.br_handle = ResizeHandle(self, is_bottom_right=True)
        
        self.update_handles()

    def update_handles(self, exclude=None):
        self._updating_handles = True
        if self.tl_handle and self.tl_handle != exclude:
            self.tl_handle.update_position()
        if self.br_handle and self.br_handle != exclude:
            self.br_handle.update_position()
        self._updating_handles = False

    def itemChange(self, change, value):
        if change == QGraphicsRectItem.GraphicsItemChange.ItemPositionChange:
            self.update_handles()
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

        # Add Button for External Process
        self.refresh_btn = QPushButton("Re-run Model Checker")
        self.refresh_btn.clicked.connect(self.trigger_external_process)
        layout.addWidget(self.refresh_btn)

        # Load image and obstacle rectangles from the current trace.
        self.reload_from_trace()

    def reload_from_trace(self):
        """(Re)draw the preview image and overlay obstacle rectangles read from the trace."""
        pixmap = QPixmap(self.image_path)
        if pixmap.isNull():
            print(f"Warning: Could not load image from '{self.image_path}'")
            self.scene.clear()
            self.rect_items = []
            self.scene.setSceneRect(0, 0, 800, 600)
            self.scale_to_fit()
            return

        self.image_w = pixmap.width()
        self.image_h = pixmap.height()
        rects_data = obstacles_to_pixel_rects(self.trace_path, self.image_w, self.image_h)

        self.scene.clear()
        self.rect_items = []
        self.scene.addPixmap(pixmap)
        self.scene.setSceneRect(QRectF(pixmap.rect()))
        for rect in rects_data:
            self.add_rectangle(*rect)

        self.scale_to_fit()

    def scale_to_fit(self):
        """💡 Scale the scene layout to fit perfectly inside the viewport boundaries."""
        if not self.scene.sceneRect().isEmpty():
            self.view.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def resizeEvent(self, event):
        """💡 Hook into window resize events to recalculate scale instantly."""
        super().resizeEvent(event)
        self.scale_to_fit()

    def add_rectangle(self, x1, y1, x2, y2):
        x = min(x1, x2)
        y = min(y1, y2)
        w = abs(x2 - x1)
        h = abs(y2 - y1)
        
        rect_item = ResizableRectItem(x, y, w, h)
        self.scene.addItem(rect_item)
        self.rect_items.append(rect_item)

    def collect_obstacles_world(self):
        """Inverse-transform the current on-screen rectangles back to integer world coords."""
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

    def trigger_external_process(self):
        obstacles_world = self.collect_obstacles_world()
        if not obstacles_world:
            QMessageBox.warning(self, "No obstacles", "No obstacle rectangles to write.")
            return

        try:
            patch_smv_obstacles(self.smv_path, obstacles_world)
        except RuntimeError as e:
            QMessageBox.critical(self, "SMV update failed", str(e))
            return

        # The MC run overwrites the trace and preview image; keep the previous ones so we can
        # restore the last valid view if the new obstacle layout yields no counterexample.
        trace_backup = self.trace_path + ".prev"
        image_backup = self.image_path + ".prev"
        for src, dst in ((self.trace_path, trace_backup), (self.image_path, image_backup)):
            if os.path.isfile(src):
                shutil.copy2(src, dst)

        self.refresh_btn.setEnabled(False)
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        mc_error = None
        try:
            print("⚡ Re-running model checker...")
            output = run_model_checker()
            print(output)
        except (Exception, KeyboardInterrupt) as e:
            # A user abort (Ctrl+C killing nuXmv) or any library failure must not crash the GUI.
            mc_error = e
            print(f"Model checker aborted or failed: {e}")
        finally:
            QApplication.restoreOverrideCursor()
            self.refresh_btn.setEnabled(True)

        if mc_error is None and trace_has_counterexample(self.trace_path):
            for backup in (trace_backup, image_backup):
                if os.path.isfile(backup):
                    os.remove(backup)
            self.reload_from_trace()
            return

        # Aborted, failed, or no counterexample: revert to the preserved trace and image.
        for backup, dst in ((trace_backup, self.trace_path), (image_backup, self.image_path)):
            if os.path.isfile(backup):
                shutil.move(backup, dst)
        self.reload_from_trace()

        if mc_error is not None:
            QMessageBox.warning(
                self, "Model checker aborted",
                "The model checker run was aborted or failed.\n"
                "Restored the previous trace and image.")
        else:
            QMessageBox.information(
                self, "No counterexample",
                "The model checker produced no counterexample for the new obstacle layout.\n"
                "Restored the previous trace and image.")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow(IMAGE_PATH, TRACE_PATH, SMV_PATH)
    window.resize(800, 600)
    window.show()
    sys.exit(app.exec())
