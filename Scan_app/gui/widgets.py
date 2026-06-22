from __future__ import annotations

import os
from typing import Any, Callable, Dict, Optional, Tuple

import cv2
import numpy as np
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QCursor, QImage, QPainter, QPixmap
from PyQt5.QtWidgets import (
    QAbstractSpinBox,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFrame,
    QGraphicsPixmapItem,
    QGraphicsScene,
    QGraphicsView,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QDoubleSpinBox,
    QVBoxLayout,
    QWidget,
)

try:
    import open3d as o3d
except Exception:  # pragma: no cover
    o3d = None

try:
    import pyqtgraph as pg
    import pyqtgraph.opengl as gl
    HAS_GL = True
except Exception:  # pragma: no cover
    pg = None
    gl = None
    HAS_GL = False


def cv_to_pixmap(img_bgr: Optional[np.ndarray]) -> QPixmap:
    if img_bgr is None:
        return QPixmap()
    if img_bgr.ndim == 2:
        rgb = cv2.cvtColor(img_bgr, cv2.COLOR_GRAY2RGB)
    else:
        rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    rgb = np.ascontiguousarray(rgb)
    h, w = rgb.shape[:2]
    return QPixmap.fromImage(QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy())


class ImageView(QLabel):
    def __init__(self, title: str = "Image"):
        super().__init__(title)
        self._pix = QPixmap()
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(360, 260)
        self.setStyleSheet("background:#020617; color:#e2e8f0; border:1px solid #334155; border-radius:6px;")

    def set_image(self, img_bgr: Optional[np.ndarray], title: str = "") -> None:
        self._pix = cv_to_pixmap(img_bgr)
        if self._pix.isNull():
            self.setText(title or "No image")
        else:
            self.setText("")
            self._update_scaled()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._update_scaled()

    def _update_scaled(self) -> None:
        if not self._pix.isNull():
            self.setPixmap(self._pix.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))


class ParameterRow(QWidget):
    """One editable setting row with the same Edit -> OK behavior as the original UI."""

    changed = pyqtSignal(str, object)

    def __init__(
        self,
        name: str,
        label: str,
        value: Any,
        value_type: type,
        setter: Callable[[str, Any], None],
        parent=None,
        minimum: Optional[float] = None,
        maximum: Optional[float] = None,
        decimals: int = 3,
        is_path: bool = False,
        path_mode: str = "file",
        options: Optional[list[str]] = None,
    ):
        super().__init__(parent)
        self.name = name
        self.value_type = value_type
        self.setter = setter
        self.is_path = is_path
        self.path_mode = path_mode
        self.options = list(options or [])
        self.equal_value_box_width = 132

        self.setMinimumHeight(42 if is_path else 36)
        self.setMaximumHeight(58 if is_path else 44)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setStyleSheet(
            "ParameterRow { background: transparent; }"
            "QLabel { color:#0f172a; font-size:8.3pt; font-weight:850; }"
            "QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox { background:#ffffff; color:#0f172a; "
            "border:1px solid #64748b; border-radius:5px; padding:3px 6px; font-size:8.3pt; min-height:26px; }"
            "QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled { "
            "background:#f8fafc; color:#0f172a; border:1px solid #cbd5e1; }"
            "QCheckBox { color:#0f172a; font-size:8.3pt; font-weight:850; spacing:6px; }"
            "QCheckBox::indicator { width:15px; height:15px; }"
            "QPushButton { background:#2563eb; color:#ffffff; border:1px solid #1d4ed8; border-radius:5px; "
            "padding:3px 6px; font-size:8pt; font-weight:850; }"
            "QPushButton:hover { background:#1d4ed8; }"
            "QPushButton:disabled { background:#e2e8f0; color:#64748b; border-color:#cbd5e1; }"
        )
        layout = QHBoxLayout(self)
        layout.setContentsMargins(2, 1, 2, 1)
        layout.setSpacing(4)

        self.label = QLabel(label)
        self.label.setMinimumWidth(105)
        self.label.setMaximumWidth(150)
        self.label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        layout.addWidget(self.label)

        if self.options:
            self.editor = QComboBox()
            self.editor.addItems(self.options)
            idx = self.editor.findText(str(value))
            self.editor.setCurrentIndex(max(0, idx))
        elif value_type is bool:
            self.editor = QCheckBox("Enabled")
            self.editor.setChecked(bool(value))
        elif value_type is int:
            self.editor = QSpinBox()
            self.editor.setRange(int(minimum if minimum is not None else -1000000), int(maximum if maximum is not None else 1000000))
            self.editor.setValue(int(value if value is not None else 0))
            self.editor.setAlignment(Qt.AlignCenter)
            self.editor.setButtonSymbols(QAbstractSpinBox.UpDownArrows)
        elif value_type is float:
            self.editor = QDoubleSpinBox()
            self.editor.setDecimals(int(decimals))
            self.editor.setRange(float(minimum if minimum is not None else -1e12), float(maximum if maximum is not None else 1e12))
            self.editor.setValue(float(value if value is not None else 0.0))
            self.editor.setAlignment(Qt.AlignCenter)
            self.editor.setButtonSymbols(QAbstractSpinBox.UpDownArrows)
        else:
            self.editor = QLineEdit(str(value if value is not None else ""))

        self.editor.setEnabled(False)
        self.editor.setMinimumHeight(30 if is_path else 28)
        if is_path:
            self.editor.setMinimumWidth(180)
            self.editor.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        else:
            self.editor.setFixedWidth(self.equal_value_box_width)
            self.editor.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        layout.addWidget(self.editor, 1)

        self.browse_btn = None
        if is_path:
            self.browse_btn = QPushButton("...")
            self.browse_btn.setToolTip("Browse")
            self.browse_btn.setFixedWidth(34)
            self.browse_btn.setMinimumHeight(28)
            self.browse_btn.setEnabled(False)
            self.browse_btn.clicked.connect(self.browse_path)
            layout.addWidget(self.browse_btn)

        self.modify_btn = QPushButton("Edit")
        self.modify_btn.setFixedWidth(44)
        self.modify_btn.setMinimumHeight(28)
        self.modify_btn.clicked.connect(self.toggle_modify)
        layout.addWidget(self.modify_btn)

    def browse_path(self) -> None:
        if not isinstance(self.editor, QLineEdit):
            return
        current = self.editor.text().strip()
        start = current or "."
        if self.path_mode == "dir":
            selected = QFileDialog.getExistingDirectory(self, "Select folder", start)
        elif self.path_mode == "image":
            selected, _ = QFileDialog.getOpenFileName(
                self,
                "Select image",
                start,
                "Images (*.png *.jpg *.jpeg *.bmp *.tif *.tiff *.webp);;All files (*.*)",
            )
        elif self.path_mode == "open_file":
            selected, _ = QFileDialog.getOpenFileName(self, "Select file", start, "YAML/JSON (*.yml *.yaml *.json);;All files (*.*)")
        else:
            selected, _ = QFileDialog.getSaveFileName(self, "Select file", start, "YAML/JSON (*.yml *.yaml *.json);;All files (*.*)")
        if selected:
            self.editor.setText(selected)

    def get_value(self) -> Any:
        if isinstance(self.editor, QComboBox):
            return str(self.editor.currentText()).strip()
        if isinstance(self.editor, QCheckBox):
            return bool(self.editor.isChecked())
        if isinstance(self.editor, QSpinBox):
            return int(self.editor.value())
        if isinstance(self.editor, QDoubleSpinBox):
            return float(self.editor.value())
        text = self.editor.text().strip()
        if self.value_type is str:
            return text
        return self.value_type(text)

    def set_value(self, value: Any) -> None:
        if isinstance(self.editor, QComboBox):
            idx = self.editor.findText(str(value))
            if idx >= 0:
                self.editor.setCurrentIndex(idx)
        elif isinstance(self.editor, QCheckBox):
            self.editor.setChecked(bool(value))
        elif isinstance(self.editor, QSpinBox):
            self.editor.setValue(int(value))
        elif isinstance(self.editor, QDoubleSpinBox):
            self.editor.setValue(float(value))
        elif isinstance(self.editor, QLineEdit):
            self.editor.setText(str(value))

    def toggle_modify(self) -> None:
        if not self.editor.isEnabled():
            self.editor.setEnabled(True)
            if self.browse_btn is not None:
                self.browse_btn.setEnabled(True)
            self.modify_btn.setText("OK")
            return
        try:
            value = self.get_value()
            self.setter(self.name, value)
            self.editor.setEnabled(False)
            if self.browse_btn is not None:
                self.browse_btn.setEnabled(False)
            self.modify_btn.setText("Edit")
            self.changed.emit(self.name, value)
        except Exception as exc:
            QMessageBox.critical(self, "Invalid parameter", f"Cannot apply {self.name}:\n{exc}")


class PylonLikeGraphicsView(QGraphicsView):
    def __init__(self, owner: "ZoomableImageViewer", parent=None):
        super().__init__(parent)
        self.owner = owner
        self.setFrameShape(QFrame.NoFrame)
        self.setAlignment(Qt.AlignCenter)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.NoAnchor)
        self.setResizeAnchor(QGraphicsView.NoAnchor)
        self.setMouseTracking(True)
        self.setRenderHint(QPainter.SmoothPixmapTransform, False)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.setStyleSheet("QGraphicsView{background:#020617;border:1px solid #334155;border-radius:6px;}")

    def wheelEvent(self, event) -> None:
        if not (event.modifiers() & Qt.ControlModifier):
            super().wheelEvent(event)
            return
        delta = event.angleDelta().y()
        if delta == 0:
            event.ignore()
            return
        try:
            view_pos = event.position().toPoint()
        except Exception:
            view_pos = event.pos()
        self.owner.zoom_by_at_view_pos(1.25 if delta > 0 else 1 / 1.25, view_pos)
        event.accept()

    def mouseMoveEvent(self, event) -> None:
        self.owner.remember_mouse_view_pos(event.pos())
        super().mouseMoveEvent(event)

    def mouseDoubleClickEvent(self, event) -> None:
        self.owner.fit_image()
        event.accept()


class ZoomableImageViewer(QWidget):
    """pylon-Viewer-like image panel: Ctrl+wheel zoom, drag pan, double-click fit."""

    def __init__(self, title: str = "Image", parent=None):
        super().__init__(parent)
        self._original_pixmap = QPixmap()
        self._scale_factor = 1.0
        self._fit_to_window = True
        self._min_scale = 0.02
        self._max_scale = 80.0
        self._last_mouse_view_pos = None

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(4)
        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        header.setSpacing(4)
        self.title_label = QLabel(title)
        self.title_label.setStyleSheet("font-weight:900;color:#0f172a;")
        header.addWidget(self.title_label, 1)
        self.zoom_out_btn = QPushButton("-")
        self.fit_btn = QPushButton("Fit")
        self.actual_btn = QPushButton("1:1")
        self.zoom_in_btn = QPushButton("+")
        for b in [self.zoom_out_btn, self.fit_btn, self.actual_btn, self.zoom_in_btn]:
            b.setFixedHeight(24)
            b.setMinimumWidth(36)
            b.setStyleSheet("QPushButton{background:#e2e8f0;color:#0f172a;border:1px solid #94a3b8;border-radius:4px;padding:2px 6px;font-size:8pt;font-weight:800;} QPushButton:hover{background:#cbd5e1;}")
            header.addWidget(b)
        root.addLayout(header)
        self.scene = QGraphicsScene(self)
        self.pixmap_item = QGraphicsPixmapItem()
        self.scene.addItem(self.pixmap_item)
        self.view = PylonLikeGraphicsView(self, self)
        self.view.setScene(self.scene)
        root.addWidget(self.view, 1)
        self.zoom_label = QLabel("Zoom: --   Ctrl + wheel: zoom   Wheel: scroll   Drag: pan   Double-click: fit")
        self.zoom_label.setStyleSheet("color:#64748b;font-size:8pt;")
        root.addWidget(self.zoom_label)
        self.zoom_out_btn.clicked.connect(lambda: self.zoom_by(1 / 1.25))
        self.zoom_in_btn.clicked.connect(lambda: self.zoom_by(1.25))
        self.fit_btn.clicked.connect(self.fit_image)
        self.actual_btn.clicked.connect(self.actual_size)

    def set_title(self, title: str) -> None:
        self.title_label.setText(str(title))

    def clear_image(self, message: str = "No image") -> None:
        self._original_pixmap = QPixmap()
        self._scale_factor = 1.0
        self._fit_to_window = True
        self.pixmap_item.setPixmap(QPixmap())
        self.scene.setSceneRect(0, 0, 1, 1)
        self.view.resetTransform()
        self.zoom_label.setText(f"{message}   Ctrl + wheel: zoom   Wheel: scroll   Drag: pan")

    def set_image(self, img_bgr: Optional[np.ndarray], title: Optional[str] = None, fit: Optional[bool] = None) -> None:
        if title is not None:
            self.set_title(title)
        if img_bgr is None:
            self.clear_image("No image")
            return
        new_pixmap = cv_to_pixmap(img_bgr)
        if new_pixmap.isNull():
            self.clear_image("Invalid image")
            return
        had_image = not self._original_pixmap.isNull()
        old_size = self._original_pixmap.size() if had_image else None
        anchor_view_pos = self._current_anchor_view_pos() if had_image and not self._fit_to_window else None
        anchor_scene_pos = self.view.mapToScene(anchor_view_pos) if anchor_view_pos is not None else None
        self._original_pixmap = new_pixmap
        self.pixmap_item.setPixmap(new_pixmap)
        self.scene.setSceneRect(0, 0, new_pixmap.width(), new_pixmap.height())
        if fit is True or (fit is None and not had_image) or (fit is None and old_size is not None and old_size != new_pixmap.size()):
            self.fit_image()
            return
        if anchor_view_pos is not None and anchor_scene_pos is not None:
            self._restore_scene_pos_at_view_pos(anchor_scene_pos, anchor_view_pos)
        self._update_zoom_label()

    def fit_image(self) -> None:
        if self._original_pixmap.isNull():
            return
        self.view.resetTransform()
        self.view.fitInView(self.pixmap_item, Qt.KeepAspectRatio)
        self._scale_factor = float(self.view.transform().m11())
        self._fit_to_window = True
        self._update_zoom_label()

    def actual_size(self) -> None:
        if self._original_pixmap.isNull():
            return
        self._scale_factor = 1.0
        self._fit_to_window = False
        self.view.resetTransform()
        self.view.scale(1.0, 1.0)
        self._update_zoom_label()

    def zoom_by(self, factor: float) -> None:
        if self._original_pixmap.isNull():
            return
        self.zoom_by_at_view_pos(factor, self.view.viewport().rect().center())

    def zoom_by_at_view_pos(self, factor: float, view_pos) -> None:
        if self._original_pixmap.isNull():
            return
        target = max(self._min_scale, min(self._scale_factor * float(factor), self._max_scale))
        if abs(target - self._scale_factor) < 1e-9:
            return
        scene_pos_before = self.view.mapToScene(view_pos)
        self._fit_to_window = False
        step = target / max(self._scale_factor, 1e-12)
        self._scale_factor = target
        self.view.scale(step, step)
        self._restore_scene_pos_at_view_pos(scene_pos_before, view_pos)
        self._update_zoom_label()

    def remember_mouse_view_pos(self, view_pos) -> None:
        self._last_mouse_view_pos = view_pos

    def _current_anchor_view_pos(self):
        viewport = self.view.viewport()
        view_pos = viewport.mapFromGlobal(QCursor.pos())
        if viewport.rect().contains(view_pos):
            self._last_mouse_view_pos = view_pos
            return view_pos
        if self._last_mouse_view_pos is not None and viewport.rect().contains(self._last_mouse_view_pos):
            return self._last_mouse_view_pos
        return None

    def _restore_scene_pos_at_view_pos(self, scene_pos, view_pos) -> None:
        view_pos_after = self.view.mapFromScene(scene_pos)
        dx = view_pos_after.x() - view_pos.x()
        dy = view_pos_after.y() - view_pos.y()
        self.view.horizontalScrollBar().setValue(int(self.view.horizontalScrollBar().value() + dx))
        self.view.verticalScrollBar().setValue(int(self.view.verticalScrollBar().value() + dy))

    def _update_zoom_label(self) -> None:
        mode = "Fit" if self._fit_to_window else "Manual"
        self.zoom_label.setText(f"Zoom: {self._scale_factor * 100:.0f}% ({mode})   Ctrl + wheel: zoom   Wheel: scroll   Drag: pan   Double-click: fit")

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        if self._fit_to_window and not self._original_pixmap.isNull():
            self.fit_image()


class PointCloudView(QWidget):
    """Interactive embedded 3D viewer for scanner point clouds."""

    def __init__(
        self,
        max_points: int = 250000,
        point_size: float = 2.0,
        cfg: Optional[Dict[str, Any]] = None,
        parent=None,
    ):
        super().__init__(parent)
        self.cfg = cfg or {}
        self.max_points = int(max_points)
        self.point_size = float(point_size)
        self.scatter = None
        self.grid_item = None
        self.axis_item = None
        self.camera_items = []
        self.display_points: Optional[np.ndarray] = None
        viewer_cfg = self.cfg.get("viewer", {}) if isinstance(self.cfg, dict) else {}
        self.reference_plane_y_m = float(viewer_cfg.get("reference_plane_y_m", 0.250))

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        if HAS_GL:
            self.view = gl.GLViewWidget()
            self.view.setBackgroundColor((12, 16, 24))
            self.view.opts["distance"] = 1.2
            self.view.opts["fov"] = 55
            self.view.opts["elevation"] = 18
            self.view.opts["azimuth"] = -55
            self.view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            layout.addWidget(self.view, 1)

            self.grid_item = gl.GLGridItem()
            self.grid_item.setSize(x=1.0, y=1.0, z=1.0)
            self.grid_item.setSpacing(x=0.05, y=0.05, z=0.05)
            # Stereo reconstruction uses X-right, Y-down, Z-depth. The object
            # support/turntable surface is therefore an X-Z plane positioned
            # 250 mm below the camera origin along +Y.
            self.grid_item.rotate(90, 1, 0, 0)
            self.grid_item.translate(0.0, self.reference_plane_y_m, 0.0)
            self.view.addItem(self.grid_item)

            self.axis_item = gl.GLAxisItem()
            self.axis_item.setSize(0.15, 0.15, 0.15)
            self.view.addItem(self.axis_item)
        else:
            self.view = QLabel("Embedded 3D viewer is unavailable. Install pyqtgraph and PyOpenGL.")
            self.view.setAlignment(Qt.AlignCenter)
            self.view.setStyleSheet("background:#0b1020; color:#e5e7eb; border:1px solid #334155;")
            layout.addWidget(self.view, 1)

        nav_panel = QFrame()
        nav_panel.setStyleSheet(
            "QFrame { background:#0f172a; border-bottom:1px solid #334155; } "
            "QPushButton { background:#1f2937; color:#e5e7eb; border:1px solid #475569; "
            "border-radius:4px; padding:4px 7px; font-size:8.5pt; font-weight:700; } "
            "QPushButton:hover { background:#334155; } "
            "QDoubleSpinBox { background:#111827; color:#e5e7eb; border:1px solid #475569; "
            "border-radius:4px; padding:2px 4px; font-size:8.5pt; } "
            "QLabel { color:#e5e7eb; font-size:8.5pt; }"
        )
        nav_layout = QHBoxLayout(nav_panel)
        nav_layout.setContentsMargins(6, 4, 6, 4)
        nav_layout.setSpacing(5)
        self.view_fit_btn = QPushButton("Fit")
        self.view_top_btn = QPushButton("Top")
        self.view_front_btn = QPushButton("Front")
        self.view_side_btn = QPushButton("Side")
        self.view_zoom_in_btn = QPushButton("+")
        self.view_zoom_out_btn = QPushButton("-")
        self.point_size_spin = QDoubleSpinBox()
        self.point_size_spin.setRange(0.5, 12.0)
        self.point_size_spin.setSingleStep(0.5)
        self.point_size_spin.setDecimals(1)
        self.point_size_spin.setValue(self.point_size)
        self.point_size_spin.setMaximumWidth(70)
        for btn in (
            self.view_fit_btn,
            self.view_top_btn,
            self.view_front_btn,
            self.view_side_btn,
            self.view_zoom_in_btn,
            self.view_zoom_out_btn,
        ):
            btn.setMinimumHeight(24)
        self.view_fit_btn.clicked.connect(self.fit_cloud_view)
        self.view_top_btn.clicked.connect(lambda: self.set_view_preset("top"))
        self.view_front_btn.clicked.connect(lambda: self.set_view_preset("front"))
        self.view_side_btn.clicked.connect(lambda: self.set_view_preset("side"))
        self.view_zoom_in_btn.clicked.connect(lambda: self.zoom_view(0.75))
        self.view_zoom_out_btn.clicked.connect(lambda: self.zoom_view(1.35))
        self.point_size_spin.valueChanged.connect(self.set_point_size)
        nav_layout.addWidget(self.view_fit_btn)
        nav_layout.addWidget(self.view_top_btn)
        nav_layout.addWidget(self.view_front_btn)
        nav_layout.addWidget(self.view_side_btn)
        nav_layout.addWidget(self.view_zoom_in_btn)
        nav_layout.addWidget(self.view_zoom_out_btn)
        nav_layout.addStretch(1)
        nav_layout.addWidget(QLabel("Point"))
        nav_layout.addWidget(self.point_size_spin)
        layout.insertWidget(0, nav_panel)

        self.info_label = QLabel(
            f"3D point cloud: waiting for scan completion | plane Y={self.reference_plane_y_m * 1000:.0f} mm"
        )
        self.info_label.setStyleSheet("background:#0f172a;color:#cbd5e1;padding:6px;font-weight:700;")
        layout.addWidget(self.info_label)

    def clear_cloud(self) -> None:
        if HAS_GL and self.scatter is not None:
            try:
                self.view.removeItem(self.scatter)
            except Exception:
                pass
            self.scatter = None
        if HAS_GL:
            for item in self.camera_items:
                try:
                    self.view.removeItem(item)
                except Exception:
                    pass
        self.camera_items = []
        self.display_points = None
        self.info_label.setText(
            f"3D point cloud: waiting for scan completion | plane Y={self.reference_plane_y_m * 1000:.0f} mm"
        )

    def _cloud_center_extent(self) -> Tuple[np.ndarray, float]:
        if self.display_points is None or len(self.display_points) == 0:
            return np.zeros(3, dtype=np.float32), 0.5
        finite = np.isfinite(self.display_points).all(axis=1)
        points = self.display_points[finite]
        if len(points) == 0:
            return np.zeros(3, dtype=np.float32), 0.5
        center = points.mean(axis=0).astype(np.float32)
        extent = float(np.linalg.norm(points.max(axis=0) - points.min(axis=0)))
        return center, max(0.05, extent)

    def fit_cloud_view(self) -> None:
        if not HAS_GL:
            return
        center, extent = self._cloud_center_extent()
        self.view.opts["center"] = pg.Vector(float(center[0]), float(center[1]), float(center[2]))
        self.view.opts["distance"] = max(0.20, extent * 1.65)
        self.view.update()

    def zoom_view(self, factor: float) -> None:
        if not HAS_GL:
            return
        distance = float(self.view.opts.get("distance", 1.0))
        self.view.opts["distance"] = max(0.02, distance * float(factor))
        self.view.update()

    def set_view_preset(self, preset: str) -> None:
        if not HAS_GL:
            return
        self.fit_cloud_view()
        if preset == "top":
            self.view.opts["elevation"] = -90
            self.view.opts["azimuth"] = -90
        elif preset == "front":
            self.view.opts["elevation"] = 0
            self.view.opts["azimuth"] = -90
        elif preset == "side":
            self.view.opts["elevation"] = 0
            self.view.opts["azimuth"] = 0
        self.view.update()

    def set_point_size(self, value: float) -> None:
        self.point_size = float(value)
        if isinstance(self.cfg, dict):
            self.cfg.setdefault("viewer", {})["display_3d_point_size"] = self.point_size
        if HAS_GL and self.scatter is not None:
            try:
                self.scatter.setData(size=self.point_size)
            except Exception:
                pass
            self.view.update()

    def _add_line_item(
        self,
        points: np.ndarray,
        color: Tuple[float, float, float, float],
        width: float = 2.0,
    ) -> None:
        if not HAS_GL:
            return
        item = gl.GLLinePlotItem(
            pos=points.astype(np.float32),
            color=color,
            width=width,
            antialias=True,
        )
        self.view.addItem(item)
        self.camera_items.append(item)

    def show_stereo_camera_positions(self, calibration: Optional[Dict[str, Any]] = None) -> None:
        if not HAS_GL:
            return
        for item in self.camera_items:
            try:
                self.view.removeItem(item)
            except Exception:
                pass
        self.camera_items = []

        left_center = np.zeros(3, dtype=np.float32)
        right_center = np.array([0.08, 0.0, 0.0], dtype=np.float32)
        if calibration:
            rotation = calibration.get("R")
            translation = calibration.get("T")
            if isinstance(rotation, np.ndarray) and isinstance(translation, np.ndarray):
                try:
                    right_center = (-rotation.T @ translation.reshape(3, 1)).reshape(3).astype(np.float32)
                    scale_to_meter = bool(
                        self.cfg.get("reconstruction", {}).get("scale_to_meter", True)
                    )
                    if scale_to_meter:
                        right_center /= 1000.0
                except Exception:
                    pass

        baseline = float(np.linalg.norm(right_center - left_center))
        axis_len = max(0.025, min(0.15, baseline * 0.35 if baseline > 1e-6 else 0.05))

        def draw_camera(center: np.ndarray, outline_color: Tuple[float, float, float, float]) -> None:
            axes = (
                (np.array([axis_len, 0, 0], dtype=np.float32), (1.0, 0.1, 0.1, 1.0)),
                (np.array([0, axis_len, 0], dtype=np.float32), (0.1, 1.0, 0.1, 1.0)),
                (np.array([0, 0, axis_len], dtype=np.float32), (0.1, 0.35, 1.0, 1.0)),
            )
            for offset, color in axes:
                self._add_line_item(np.vstack([center, center + offset]), color, 3.0)
            size = axis_len * 0.35
            box = np.array(
                [
                    [center[0] - size, center[1] - size, center[2]],
                    [center[0] + size, center[1] - size, center[2]],
                    [center[0] + size, center[1] + size, center[2]],
                    [center[0] - size, center[1] + size, center[2]],
                    [center[0] - size, center[1] - size, center[2]],
                ],
                dtype=np.float32,
            )
            self._add_line_item(box, outline_color, 2.0)

        draw_camera(left_center, (1.0, 0.9, 0.1, 1.0))
        draw_camera(right_center, (0.1, 1.0, 1.0, 1.0))
        self._add_line_item(np.vstack([left_center, right_center]), (1, 1, 1, 1), 2.0)

    def _show_arrays(
        self,
        points: np.ndarray,
        colors: np.ndarray,
        title: str,
        calibration: Optional[Dict[str, Any]] = None,
    ) -> None:
        points = np.asarray(points, dtype=np.float32)
        colors = np.asarray(colors, dtype=np.float32)
        if colors.shape != points.shape:
            colors = np.full_like(points, 0.85, dtype=np.float32)

        total_points = len(points)
        if total_points > self.max_points:
            indices = np.linspace(0, total_points - 1, self.max_points).astype(np.int64)
            points = points[indices]
            colors = colors[indices]
            sampled_note = f"displayed {len(points):,}/{total_points:,} points"
        else:
            sampled_note = f"displayed {len(points):,} points"

        if HAS_GL:
            self.clear_cloud()
            self.display_points = points
            rgba = np.ones((len(colors), 4), dtype=np.float32)
            rgba[:, :3] = np.clip(colors, 0, 1)
            self.scatter = gl.GLScatterPlotItem(
                pos=points,
                color=rgba,
                size=self.point_size,
                pxMode=True,
            )
            self.view.addItem(self.scatter)
            self.show_stereo_camera_positions(calibration)
            self.fit_cloud_view()
            self.view.update()
        else:
            self.view.setText(f"{title}\nPoints: {len(points):,}")
        self.info_label.setText(f"{title}: {sampled_note}")

    def show_point_cloud_object(
        self,
        pcd,
        title: str = "Point cloud",
        calibration: Optional[Dict[str, Any]] = None,
    ) -> None:
        if o3d is None:
            self.info_label.setText("Open3D is required to show point clouds.")
            return
        if pcd is None:
            self.info_label.setText("No point cloud available.")
            return
        points = np.asarray(pcd.points, dtype=np.float32)
        if points.size == 0:
            self.clear_cloud()
            self.info_label.setText("Generated point cloud is empty.")
            return
        colors = np.asarray(pcd.colors, dtype=np.float32)
        self._show_arrays(points, colors, title, calibration)

    def load_ply(self, path: str) -> None:
        if o3d is None:
            self.info_label.setText("Open3D is required to load point clouds.")
            return
        if not path or not os.path.exists(path):
            self.info_label.setText(f"Point cloud file not found: {path}")
            return
        pcd = o3d.io.read_point_cloud(path)
        points = np.asarray(pcd.points, dtype=np.float32)
        if points.size == 0:
            self.info_label.setText(f"Empty point cloud: {path}")
            return
        colors = np.asarray(pcd.colors, dtype=np.float32)
        self._show_arrays(points, colors, f"Loaded fused point cloud: {os.path.basename(path)}")

    def load_point_cloud(self, path: str) -> None:
        """Compatibility alias matching the reference viewer API."""
        self.load_ply(path)
