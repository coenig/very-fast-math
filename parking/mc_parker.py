import sys
import re
import math
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QPushButton, QGraphicsView, QGraphicsScene, QGraphicsRectItem)
from PyQt6.QtCore import Qt, QRectF
from PyQt6.QtGui import QPixmap, QBrush, QPen, QColor


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


def obstacles_to_pixel_rects(trace_path, image_width, image_height):
    """Read rectangular obstacles from an MC trace and map them into preview2 pixels.

    Replicates the C++ 'fit_to_roads' birdseye painter (env2d_simple.h / highway_image.cpp):
    the road-graph bounding box (section source+drain centerlines, ghosts excluded) is
    centered in the canvas at a uniform scale 'ppm' (pixels per meter). Obstacles share the
    same world frame as the section origins, so the same mapping applies to their corners.

    Returns a list of (x1, y1, x2, y2) pixel tuples, one per obstacle.
    """
    v = _read_trace_values(trace_path)

    def get(name):
        # Accept both 'env.'-prefixed (dump) and bare (MC trace) variable names.
        if name in v:
            return v[name]
        return v.get("env." + name)

    # --- Collect the road sections (source point, heading, length). ---
    points = []
    sec = 0
    while get(f"section_{sec}.source.x") is not None:
        sx = get(f"section_{sec}.source.x")
        sy = get(f"section_{sec}.source.y")
        angle_deg = get(f"section_{sec}.angle") or 0.0
        length = get(f"section_{sec}_end") or 0.0
        angle = 2.0 * math.pi * angle_deg / 360.0
        # Origin and drain point, exactly as RoadGraph::getDrainPoint() computes it.
        points.append((sx, sy))
        points.append((sx + length * math.cos(angle), sy + length * math.sin(angle)))
        sec += 1

    if not points:
        return []

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0
    bb_w = max_x - min_x
    bb_h = max_y - min_y

    # Road-graph lane width is stored in cm in the trace (see mc_trajectory_to_gif.cpp).
    lane_width = (get("lane_width") or 400.0) / 100.0
    num_lanes = get("num_lanes") or 1.0

    # Fit-to-roads canvas math (env2d_simple.h getBirdseyeView).
    lateral_margin = num_lanes * lane_width / 2.0 + lane_width
    content_w = (max(1.0, bb_w) + 2.0 * lateral_margin) * 1.10
    content_h = (max(1.0, bb_h) + 2.0 * lateral_margin) * 1.10
    ppm = min(5000.0 / content_w, 40.0, 12000.0 / content_h)

    def to_pixel(wx, wy):
        # Bounding-box center sits at the image center; uniform scale, no axis flip.
        return ((wx - center_x) * ppm + image_width / 2.0,
                (wy - center_y) * ppm + image_height / 2.0)

    rects = []
    obs = 0
    while get(f"rect_obstacles_tl_x_{obs}") is not None:
        tl = to_pixel(get(f"rect_obstacles_tl_x_{obs}"), get(f"rect_obstacles_tl_y_{obs}"))
        br = to_pixel(get(f"rect_obstacles_br_x_{obs}"), get(f"rect_obstacles_br_y_{obs}"))
        rects.append((tl[0], tl[1], br[0], br[1]))
        obs += 1

    return rects

# Handle item for resizing
class ResizeHandle(QGraphicsRectItem):
    def __init__(self, parent, is_bottom_right=True):
        super().__init__(-5, -5, 10, 10, parent)
        self.parent_rect = parent
        self.is_bottom_right = is_bottom_right
        
        self.setBrush(QBrush(QColor("dodgerblue")))
        self.setPen(QPen(QColor("white"), 1))
        
        self.setFlags(
            QGraphicsRectItem.GraphicsItemFlag.ItemIsMovable |
            QGraphicsRectItem.GraphicsItemFlag.ItemSendsGeometryChanges
        )
        self.setCursor(Qt.CursorShape.SizeAllCursor)
        self.update_position()

    def update_position(self):
        rect = self.parent_rect.rect()
        if self.is_bottom_right:
            self.setPos(rect.right(), rect.bottom())
        else:
            self.setPos(rect.left(), rect.top())

    def mousePressEvent(self, event):
        event.accept()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        event.accept()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        event.accept()
        super().mouseReleaseEvent(event)

    def itemChange(self, change, value):
        if change == QGraphicsRectItem.GraphicsItemChange.ItemPositionChange and self.parent_rect:
            if getattr(self.parent_rect, '_updating_handles', False):
                return super().itemChange(change, value)
                
            new_pos = value
            rect = self.parent_rect.rect()
            
            if self.is_bottom_right:
                new_w = max(10, new_pos.x() - rect.left())
                new_h = max(10, new_pos.y() - rect.top())
                self.parent_rect.setRect(rect.left(), rect.top(), new_w, new_h)
            else:
                dx = new_pos.x() - rect.left()
                dy = new_pos.y() - rect.top()
                
                new_w = max(10, rect.width() - dx)
                new_h = max(10, rect.height() - dy)
                
                self.parent_rect.setRect(rect.left() + dx, rect.top() + dy, new_w, new_h)
                
            self.parent_rect.update_handles(exclude=self)
            
        return super().itemChange(change, value)


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
    def __init__(self, image_path, rects_data):
        super().__init__()
        self.setWindowTitle("Simple Image Annotation Tool")
        self.image_path = image_path

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
        self.refresh_btn = QPushButton("Trigger Refresh Process")
        self.refresh_btn.clicked.connect(self.trigger_external_process)
        layout.addWidget(self.refresh_btn)

        # Load image and rectangles
        self.load_image()
        for rect in rects_data:
            self.add_rectangle(*rect)

    def load_image(self):
        existing_rects = [item for item in self.scene.items() if isinstance(item, ResizableRectItem)]
        
        self.scene.clear()
        pixmap = QPixmap(self.image_path)
        if not pixmap.isNull():
            self.scene.addPixmap(pixmap)
            self.scene.setSceneRect(QRectF(pixmap.rect()))
        else:
            print(f"Warning: Could not load image from '{self.image_path}'")
            self.scene.setSceneRect(0, 0, 800, 600)

        for rect in existing_rects:
            self.scene.addItem(rect)
            
        # Trigger an initial scale calculation
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

    def trigger_external_process(self):
        print("⚡ Triggering external process...")
        # TODO: Run your subprocess here
        self.load_image()


if __name__ == "__main__":
    example_image = "examples/gp_config/0/preview2/preview2_0.png" 
    example_rectangle_source = "examples/gp_config/debug_trace_array.txt"

    # Obstacles come straight from the MC trace, transformed into preview2 pixel coordinates.
    app = QApplication(sys.argv)
    pixmap = QPixmap(example_image)
    image_w = pixmap.width() if not pixmap.isNull() else 2112
    image_h = pixmap.height() if not pixmap.isNull() else 1936
    example_rects = obstacles_to_pixel_rects(example_rectangle_source, image_w, image_h)

    window = MainWindow(example_image, example_rects)
    window.resize(800, 600)
    window.show()
    sys.exit(app.exec())
