import sys
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QPushButton, QGraphicsView, QGraphicsScene, QGraphicsRectItem)
from PyQt6.QtCore import Qt, QRectF
from PyQt6.QtGui import QPixmap, QBrush, QPen, QColor

# Handle item for resizing
class ResizeHandle(QGraphicsRectItem):
    def __init__(self, parent, is_bottom_right=True):
        # Create a small 10x10 pixel handle (slightly larger for easier clicking)
        super().__init__(-5, -5, 10, 10, parent)
        self.parent_rect = parent
        self.is_bottom_right = is_bottom_right
        
        # Style the handle (Blue squares)
        self.setBrush(QBrush(QColor("dodgerblue")))
        self.setPen(QPen(QColor("white"), 1))
        
        # Enable dragging on the handle itself
        self.setFlags(
            QGraphicsRectItem.GraphicsItemFlag.ItemIsMovable |
            QGraphicsRectItem.GraphicsItemFlag.ItemSendsGeometryChanges
        )
        self.setCursor(Qt.CursorShape.SizeAllCursor)
        self.update_position()

    def update_position(self):
        """Place the handle at the correct corner of the parent rectangle."""
        rect = self.parent_rect.rect()
        if self.is_bottom_right:
            self.setPos(rect.right(), rect.bottom())
        else:
            self.setPos(rect.left(), rect.top())

    # Intercept mouse events so the parent doesn't capture and move the whole rect!
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
            # If the parent is updating handles, ignore to prevent recursive loops
            if getattr(self.parent_rect, '_updating_handles', False):
                return super().itemChange(change, value)
                
            new_pos = value
            rect = self.parent_rect.rect()
            
            if self.is_bottom_right:
                # Bottom-Right resizing
                new_w = max(10, new_pos.x() - rect.left())
                new_h = max(10, new_pos.y() - rect.top())
                self.parent_rect.setRect(rect.left(), rect.top(), new_w, new_h)
            else:
                # Top-Left resizing:
                # Calculate how much the mouse moved from the previous top-left corner
                dx = new_pos.x() - rect.left()
                dy = new_pos.y() - rect.top()
                
                new_w = max(10, rect.width() - dx)
                new_h = max(10, rect.height() - dy)
                
                # Adjust rect origin (internally)
                self.parent_rect.setRect(rect.left() + dx, rect.top() + dy, new_w, new_h)
                
            # Sync other handles if any
            self.parent_rect.update_handles(exclude=self)
            
        return super().itemChange(change, value)


# Custom rectangle item with handles
class ResizableRectItem(QGraphicsRectItem):
    def __init__(self, x, y, w, h):
        super().__init__(0, 0, w, h)
        self.setPos(x, y)
        self._updating_handles = False
        
        # Styling the main box (Red border, transparent fill)
        self.setPen(QPen(QColor("red"), 2))
        self.setBrush(QBrush(Qt.GlobalColor.transparent))
        
        # Enable moving and selection
        self.setFlags(
            QGraphicsRectItem.GraphicsItemFlag.ItemIsMovable |
            QGraphicsRectItem.GraphicsItemFlag.ItemIsSelectable |
            QGraphicsRectItem.GraphicsItemFlag.ItemSendsGeometryChanges
        )

        # 1. Initialize handle attributes to None first so they exist during setup
        self.tl_handle = None
        self.br_handle = None

        # 2. Instantiate drag handles for BOTH corners
        self.tl_handle = ResizeHandle(self, is_bottom_right=False)
        self.br_handle = ResizeHandle(self, is_bottom_right=True)
        
        # 3. Synchronize positions now that both are ready
        self.update_handles()

    def update_handles(self, exclude=None):
        """Reposition handles when the rectangle's geometry changes."""
        self._updating_handles = True
        
        if self.tl_handle and self.tl_handle != exclude:
            self.tl_handle.update_position()
            
        if self.br_handle and self.br_handle != exclude:
            self.br_handle.update_position()
            
        self._updating_handles = False

    def itemChange(self, change, value):
        # If the rectangle itself is dragged, ensure handles stick along
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
        # We preserve existing Rectangles when reloading image
        existing_rects = [item for item in self.scene.items() if isinstance(item, ResizableRectItem)]
        
        self.scene.clear()
        pixmap = QPixmap(self.image_path)
        if not pixmap.isNull():
            self.scene.addPixmap(pixmap)
            self.scene.setSceneRect(QRectF(pixmap.rect()))
        else:
            print(f"Warning: Could not load image from '{self.image_path}'")
            self.scene.setSceneRect(0, 0, 800, 600)

        # Restore the rect items back onto the new background
        for rect in existing_rects:
            self.scene.addItem(rect)

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
    example_image = "parking.png" 
    example_rects = [
        (50, 50, 150, 150),
        (200, 100, 320, 220)
    ]

    app = QApplication(sys.argv)
    window = MainWindow(example_image, example_rects)
    window.resize(800, 600)
    window.show()
    sys.exit(app.exec())
