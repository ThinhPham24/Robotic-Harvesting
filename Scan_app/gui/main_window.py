from __future__ import annotations

import csv
import copy
import os
import shutil
import time
import traceback
from pathlib import Path
from typing import Any, Dict, List, Tuple

import cv2
import numpy as np
from PyQt5.QtCore import QEvent, QObject, QThread, QTimer, Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QApplication,
    QAbstractSpinBox,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QShortcut,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

try:
    import open3d as o3d
except Exception:  # pragma: no cover
    o3d = None

from core.calibration import StereoCalibrationManager, TurntableCalibrationManager
from core.camera_manager import StereoCameraManager
from core.postprocessing import PointCloudPostProcessor
from core.project_io import (
    append_csv_row,
    create_scan_folder,
    ensure_dir,
    get_nested,
    load_config,
    read_json,
    save_yaml,
    set_nested,
    timestamp,
    timestamp_for_scan_folder,
    write_json,
)
from core.segmentation import ObjectSegmenter
from core.stereo_reconstruction import StereoReconstructor
from core.stitching import PointCloudStitcher
from core.turntable_controller import TurntableController
from gui.widgets import ImageView, ParameterRow, PointCloudView, ZoomableImageViewer


def make_turntable_placement_guide(width: int = 1100, height: int = 720) -> np.ndarray:
    """Create an in-app guide image for ChArUco board placement."""
    img = np.full((height, width, 3), (245, 248, 252), np.uint8)
    # title
    cv2.putText(img, "Turntable Calibration Setup", (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.25, (15, 23, 42), 3, cv2.LINE_AA)
    cv2.putText(img, "Mount the 13 x 8 ChArUco board vertically and rigidly on the turntable", (40, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (51, 65, 85), 2, cv2.LINE_AA)

    # camera
    cam_x, cam_y = 100, 360
    cv2.rectangle(img, (cam_x - 60, cam_y - 45), (cam_x + 60, cam_y + 45), (30, 64, 175), -1)
    cv2.circle(img, (cam_x, cam_y), 28, (226, 232, 240), -1)
    cv2.circle(img, (cam_x, cam_y), 16, (15, 23, 42), -1)
    cv2.putText(img, "Stereo camera", (40, cam_y + 85), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (15, 23, 42), 2, cv2.LINE_AA)

    # turntable
    center = (735, 375)
    cv2.circle(img, center, 210, (203, 213, 225), -1)
    cv2.circle(img, center, 210, (71, 85, 105), 4)
    cv2.circle(img, center, 6, (220, 38, 38), -1)
    cv2.putText(img, "Turntable center", (center[0] - 95, center[1] + 250), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (15, 23, 42), 2, cv2.LINE_AA)

    # board, perspective rectangle
    board = np.array([[620, 175], [860, 205], [860, 500], [620, 470]], np.int32)
    cv2.fillConvexPoly(img, board, (255, 255, 255))
    cv2.polylines(img, [board], True, (15, 23, 42), 3)
    # draw approximate chess squares
    for i in range(13):
        t = i / 12.0
        p1 = (board[0] * (1 - t) + board[1] * t).astype(int)
        p2 = (board[3] * (1 - t) + board[2] * t).astype(int)
        cv2.line(img, tuple(p1), tuple(p2), (100, 116, 139), 1)
    for j in range(8):
        t = j / 7.0
        p1 = (board[0] * (1 - t) + board[3] * t).astype(int)
        p2 = (board[1] * (1 - t) + board[2] * t).astype(int)
        cv2.line(img, tuple(p1), tuple(p2), (100, 116, 139), 1)
    cv2.putText(img, "Rigid 13 x 8 ChArUco board", (600, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (15, 23, 42), 2, cv2.LINE_AA)

    # arrows and notes
    cv2.arrowedLine(img, (170, cam_y), (510, 365), (37, 99, 235), 4, tipLength=0.04)
    cv2.putText(img, "Board must be visible to BOTH cameras", (230, 315), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (30, 64, 175), 2, cv2.LINE_AA)
    cv2.arrowedLine(img, (735, 535), (735, 460), (220, 38, 38), 4, tipLength=0.05)
    cv2.putText(img, "Offset from axis; clamp at two rigid points", (520, 550), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (185, 28, 28), 2, cv2.LINE_AA)

    notes = [
        "1. Connect stereo cameras and load/create stereoMap.yml first.",
        "2. Use a stiff backing plate and two rigid clamps; do not use flexible wire support.",
        "3. Start with the board near one edge of visibility; the default sweep is 90 degrees.",
        "4. The app waits for post-stop motion to settle before accepting each pose.",
        "5. The JSON is saved only when circle, plane, coverage, and pose checks pass.",
    ]
    y = 570
    for n in notes:
        cv2.putText(img, n, (40, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (51, 65, 85), 2, cv2.LINE_AA)
        y += 28
    return img


def draw_horizontal_guides(img_bgr: np.ndarray, step: int = 80) -> np.ndarray:
    if img_bgr is None:
        return img_bgr
    out = img_bgr.copy()
    h, w = out.shape[:2]
    for y in range(0, h, max(20, int(step))):
        cv2.line(out, (0, y), (w - 1, y), (0, 255, 255), 1)
    return out


def make_status_image(title: str, lines: List[str] | None = None, width: int = 1280, height: int = 720) -> np.ndarray:
    lines = lines or []
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = (18, 25, 37)
    cv2.rectangle(img, (25, 25), (width - 25, min(height - 25, 90 + 35 * len(lines))), (70, 130, 230), 2)
    cv2.putText(img, title, (45, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (255, 255, 255), 2, cv2.LINE_AA)
    y = 105
    for line in lines:
        cv2.putText(img, str(line), (45, y), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (220, 235, 255), 2, cv2.LINE_AA)
        y += 35
    return img


def camera_roi_summary_text(cfg: dict) -> str:
    enabled = bool(get_nested(cfg, "camera.enable_center_crop", False))
    cw = int(get_nested(cfg, "camera.crop_width", 0) or 0)
    ch = int(get_nested(cfg, "camera.crop_height", 0) or 0)
    cx = int(get_nested(cfg, "camera.crop_center_x", -1))
    cy = int(get_nested(cfg, "camera.crop_center_y", -1))
    if not enabled:
        return "Camera ROI: full image from Basler camera"
    return f"Camera ROI crop: {cw} x {ch} centered at ({cx}, {cy})"



class TurntableCalibrationWorker(QObject):
    log_signal = pyqtSignal(str)
    finished_signal = pyqtSignal(dict)
    stopped_signal = pyqtSignal()
    error_signal = pyqtSignal(str)

    def __init__(self, cfg: dict, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self._stop = False

    def log(self, msg: str) -> None:
        self.log_signal.emit(str(msg))

    def stop(self) -> None:
        self._stop = True
        self.log("[TURNTABLE CALIB] Stop requested; waiting for the current hardware command to finish.")

    @pyqtSlot()
    def run(self) -> None:
        camera = StereoCameraManager(self.cfg, log_fn=self.log)
        turntable = TurntableController(self.cfg, log_fn=self.log)
        stereo = StereoCalibrationManager(self.cfg, log_fn=self.log)
        try:
            stereo.load()
            camera.connect()
            turntable.connect()
            save_dir = Path(get_nested(self.cfg, "paths.output_root")) / "turntable_calibration" / timestamp()
            calib = TurntableCalibrationManager(self.cfg, stereo, log_fn=self.log)
            data = calib.calibrate_turntable_axis(
                camera,
                turntable,
                save_dir,
                stop_requested=lambda: self._stop,
            )
            self.finished_signal.emit(data)
        except InterruptedError:
            self.log("[TURNTABLE CALIB STOPPED] Calibration stopped by operator.")
            self.stopped_signal.emit()
        except Exception:
            self.error_signal.emit(traceback.format_exc())
        finally:
            camera.close()
            turntable.close()


class StereoCalibrationWorker(QObject):
    log_signal = pyqtSignal(str)
    status_signal = pyqtSignal(str)
    image_signal = pyqtSignal(object, object, str, str)
    finished_signal = pyqtSignal(dict)
    error_signal = pyqtSignal(str)

    def __init__(self, cfg: dict, capture_dir: str, save_path: str, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self.capture_dir = capture_dir
        self.save_path = save_path
        self._stop = False

    def log(self, msg: str) -> None:
        self.log_signal.emit(str(msg))

    @pyqtSlot()
    def stop(self) -> None:
        self._stop = True

    @pyqtSlot()
    def run(self) -> None:
        """Run calibration from saved images and emit 2D previews.

        This keeps the original stereo-vision calibration behavior: while the
        Calibration button is running, every saved stereo pair is shown in the
        2D display with chessboard corners and accepted/rejected status.
        """
        try:
            calib = StereoCalibrationManager(self.cfg, log_fn=self.log)
            calib.clear_points()
            pairs = StereoCalibrationManager.saved_calibration_pairs(self.capture_dir)
            if not pairs:
                raise FileNotFoundError(
                    f"No matched calibration pairs were found in {self.capture_dir}. "
                    "Expected calib_SL_*.png and calib_SR_*.png."
                )

            resolution_percent = float(get_nested(self.cfg, "stereo_calibration.resolution_percent", 100))
            counts = StereoCalibrationManager.calibration_folder_image_counts(self.capture_dir)
            self.log(
                "[CALIB] Folder image count: "
                f"total={counts['total_calibration_images']}, left={counts['left_images']}, "
                f"right={counts['right_images']}, matched={counts['matched_pairs']}"
            )

            accepted = 0
            rejected = 0
            last_good_left = None
            last_good_right = None
            for pair_index, (left_path, right_path) in enumerate(pairs, start=1):
                if self._stop:
                    self.log("[CALIB] Stereo calibration stopped by operator.")
                    return
                left = cv2.imread(left_path, cv2.IMREAD_COLOR)
                right = cv2.imread(right_path, cv2.IMREAD_COLOR)
                if left is None or right is None:
                    rejected += 1
                    self.log(f"[CALIB WARNING] Cannot read pair: {left_path}, {right_path}")
                    continue

                left = calib.resizeImage(left, resolution_percent)
                right = calib.resizeImage(right, resolution_percent)
                ok, _preview, msg = calib.add_pair(left, right)

                left_preview = getattr(calib, "last_preview_left", None)
                right_preview = getattr(calib, "last_preview_right", None)
                if left_preview is None:
                    left_preview = left
                if right_preview is None:
                    right_preview = right

                status_text = "accepted" if ok else "rejected"
                left_title = (
                    f"Left chessboard {pair_index}/{len(pairs)}: corners found, {status_text}"
                    if getattr(calib, "last_found_left", False)
                    else f"Left chessboard {pair_index}/{len(pairs)}: corners not found, {status_text}"
                )
                right_title = (
                    f"Right chessboard {pair_index}/{len(pairs)}: corners found, {status_text}"
                    if getattr(calib, "last_found_right", False)
                    else f"Right chessboard {pair_index}/{len(pairs)}: corners not found, {status_text}"
                )
                self.image_signal.emit(left_preview, right_preview, left_title, right_title)
                self.status_signal.emit(
                    f"Loading calibration folder. pair {pair_index} / {len(pairs)} | "
                    f"accepted={accepted + (1 if ok else 0)}, rejected={rejected + (0 if ok else 1)}"
                )

                if ok:
                    accepted += 1
                    last_good_left = left
                    last_good_right = right
                else:
                    rejected += 1
                    self.log(f"[CALIB WARNING] Rejected saved pair: {os.path.basename(left_path)}, {os.path.basename(right_path)} - {msg}")
                self.log(f"[CALIB PAIR {pair_index:03d}] {os.path.basename(left_path)} / {os.path.basename(right_path)} | {msg}")

            if self._stop:
                self.log("[CALIB] Stereo calibration stopped before final calibration.")
                return
            if accepted < 3:
                raise RuntimeError(
                    f"At least 3 valid stereo chessboard pairs are required for calibration. "
                    f"Accepted={accepted}, rejected={rejected}."
                )

            self.status_signal.emit("S2 calibration running from accepted saved images...")
            result = calib.calibrate()
            calib.save(self.save_path)

            # After saving, show each rectified camera image in its own panel
            # so the operator can compare corresponding horizontal features.
            if last_good_left is None or last_good_right is None:
                left_path, right_path = pairs[-1]
                last_good_left = cv2.imread(left_path, cv2.IMREAD_COLOR)
                last_good_right = cv2.imread(right_path, cv2.IMREAD_COLOR)
                if last_good_left is not None and last_good_right is not None:
                    last_good_left = calib.resizeImage(last_good_left, resolution_percent)
                    last_good_right = calib.resizeImage(last_good_right, resolution_percent)
            if last_good_left is not None and last_good_right is not None:
                validation = calib.validate_rectification(last_good_left, last_good_right, save_dir=Path(self.save_path).parent)
                rect_l = validation.get("rect_left")
                rect_r = validation.get("rect_right")
                if rect_l is None or rect_r is None:
                    rect_l, rect_r = calib.rectify(last_good_left, last_good_right)
                self.image_signal.emit(
                    draw_horizontal_guides(rect_l),
                    draw_horizontal_guides(rect_r),
                    "Rectified left - calibration check",
                    "Rectified right - calibration check",
                )

            summary = {k: v for k, v in result.items() if not isinstance(v, np.ndarray)}
            summary.update({"accepted": accepted, "rejected": rejected, **counts, "resolution_percent": resolution_percent})
            self.finished_signal.emit(summary)
        except Exception:
            self.error_signal.emit(traceback.format_exc())



class ObjectInfoDialog(QDialog):
    """Dialog shown before each scan to collect only object weight.

    The object name/ID is fixed automatically from the scan timestamp.
    This avoids customer typing mistakes and makes every object folder unique.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Object weight before scan")
        self.setModal(True)
        self.setMinimumWidth(430)

        root = QVBoxLayout(self)
        title = QLabel("Enter the weight for this object")
        title.setStyleSheet("font-size:11pt;font-weight:900;color:#0f172a;")
        root.addWidget(title)

        note = QLabel(
            "The object name is generated automatically from the scan timestamp:\n"
            "YYYY_MM_DD_HHMMSS\n\n"
            "A new scan folder will be created using:\n"
            "YYYY_MM_DD_HHMMSS_weight_<weight>_<unit>\n\n"
            "All stereo image pairs for this object will be saved inside that folder."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color:#475569;font-weight:700;")
        root.addWidget(note)

        form = QFormLayout()
        self.object_name_label = QLabel("Generated automatically at scan start")
        self.object_name_label.setStyleSheet("background:#f8fafc;border:1px solid #cbd5e1;border-radius:6px;padding:6px;color:#0f172a;font-weight:700;")
        self.weight_spin = QDoubleSpinBox()
        self.weight_spin.setDecimals(3)
        self.weight_spin.setRange(0.0, 100000000.0)
        self.weight_spin.setValue(0.0)
        self.weight_spin.setSuffix(" ")
        self.weight_spin.installEventFilter(self)
        self.weight_spin.lineEdit().installEventFilter(self)
        self.weight_unit_combo = QComboBox()
        self.weight_unit_combo.addItems(["g", "kg"])

        weight_row = QWidget()
        weight_layout = QHBoxLayout(weight_row)
        weight_layout.setContentsMargins(0, 0, 0, 0)
        weight_layout.addWidget(self.weight_spin, 1)
        weight_layout.addWidget(self.weight_unit_combo)

        form.addRow("Object name:", self.object_name_label)
        form.addRow("Weight:", weight_row)
        root.addLayout(form)

        self.preview_label = QLabel("")
        self.preview_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.preview_label.setStyleSheet("background:#f8fafc;border:1px solid #cbd5e1;border-radius:6px;padding:8px;color:#0f172a;")
        root.addWidget(self.preview_label)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        root.addWidget(buttons)

        self.weight_spin.valueChanged.connect(self._update_preview)
        self.weight_unit_combo.currentTextChanged.connect(self._update_preview)
        self._update_preview()
        QTimer.singleShot(0, self._focus_and_select_weight)

    def _focus_and_select_weight(self) -> None:
        self.weight_spin.setFocus(Qt.OtherFocusReason)
        self.weight_spin.lineEdit().selectAll()

    def eventFilter(self, watched, event) -> bool:
        if (
            watched in (self.weight_spin, self.weight_spin.lineEdit())
            and event.type() in (QEvent.FocusIn, QEvent.MouseButtonPress)
        ):
            # Run after QDoubleSpinBox handles the event so its default
            # cursor placement cannot undo the full selection.
            QTimer.singleShot(0, self.weight_spin.lineEdit().selectAll)
        return super().eventFilter(watched, event)

    def _weight_text(self) -> str:
        return f"{float(self.weight_spin.value()):.3f}".rstrip("0").rstrip(".") or "0"

    def _update_preview(self) -> None:
        # The exact timestamp is generated at scan start. This preview shows the naming rule.
        weight = self._weight_text()
        unit = self.weight_unit_combo.currentText().strip()
        self.preview_label.setText(
            "Object name preview:\n"
            "YYYY_MM_DD_HHMMSS\n\n"
            "Folder name preview:\n"
            f"YYYY_MM_DD_HHMMSS_weight_{weight}_{unit}"
        )

    def object_info(self) -> Dict[str, Any]:
        weight_value = float(self.weight_spin.value())
        weight_text = self._weight_text()
        return {
            "object_name": "",
            "object_name_rule": "YYYY_MM_DD_HHMMSS generated at scan start",
            "weight": weight_value,
            "weight_text": weight_text,
            "weight_unit": self.weight_unit_combo.currentText().strip(),
        }


class NextObjectDialog(QDialog):
    """Scan-complete notice dismissed by Enter, Escape, or the window X."""

    def __init__(
        self,
        parent=None,
        capture_only: bool = False,
        elapsed_text: str = "",
    ):
        super().__init__(parent)
        self.setWindowTitle("Scan complete - prepare next object")
        self.setModal(True)
        self.setMinimumWidth(520)

        root = QVBoxLayout(self)
        title = QLabel("Current object finished")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(
            "font-size:16pt;font-weight:900;color:#166534;padding:10px;"
        )
        root.addWidget(title)

        detail = (
            "Stereo images were saved successfully.\n"
            "Point-cloud generation was deferred to the Point Cloud Processing tab."
            if capture_only
            else "Stereo capture and point-cloud processing finished successfully."
        )
        if elapsed_text:
            detail += f"\n\nTotal scan time: {elapsed_text}"
        message = QLabel(
            detail
            + "\n\nRemove the current object and place the next object on the turntable."
            + "\n\nPress Enter to continue, Escape to continue, or click X."
        )
        message.setAlignment(Qt.AlignCenter)
        message.setWordWrap(True)
        message.setStyleSheet(
            "background:#f0fdf4;border:1px solid #86efac;border-radius:8px;"
            "padding:18px;color:#14532d;font-size:11pt;font-weight:700;"
        )
        root.addWidget(message)

        continue_button = QPushButton("Continue to Next Object [Enter]")
        continue_button.setObjectName("Primary")
        continue_button.clicked.connect(self.accept)
        continue_button.setDefault(True)
        continue_button.setAutoDefault(True)
        root.addWidget(continue_button)
        continue_button.setFocus()

    def keyPressEvent(self, event) -> None:
        if event.key() in (Qt.Key_Return, Qt.Key_Enter, Qt.Key_Escape):
            self.accept()
            return
        super().keyPressEvent(event)


class WorkflowDoneDialog(QDialog):
    """Reusable completion notice closed by Enter, Escape, button, or X."""

    def __init__(self, title: str, message: str, parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setModal(True)
        self.setMinimumWidth(500)
        root = QVBoxLayout(self)
        heading = QLabel("DONE")
        heading.setAlignment(Qt.AlignCenter)
        heading.setStyleSheet(
            "font-size:18pt;font-weight:900;color:#166534;padding:8px;"
        )
        root.addWidget(heading)
        body = QLabel(message)
        body.setWordWrap(True)
        body.setAlignment(Qt.AlignCenter)
        body.setStyleSheet(
            "background:#f0fdf4;border:1px solid #86efac;border-radius:8px;"
            "padding:16px;color:#14532d;font-size:10.5pt;font-weight:700;"
        )
        root.addWidget(body)
        button = QPushButton("OK [Enter]")
        button.setObjectName("Primary")
        button.clicked.connect(self.accept)
        button.setDefault(True)
        button.setAutoDefault(True)
        root.addWidget(button)
        button.setFocus()

    def keyPressEvent(self, event) -> None:
        if event.key() in (Qt.Key_Return, Qt.Key_Enter, Qt.Key_Escape):
            self.accept()
            return
        super().keyPressEvent(event)


class ScanWorker(QObject):
    log_signal = pyqtSignal(str)
    status_signal = pyqtSignal(str)
    image_signal = pyqtSignal(object, object)
    fused_signal = pyqtSignal(str)
    completed_signal = pyqtSignal(bool)
    finished_signal = pyqtSignal()
    error_signal = pyqtSignal(str)

    def __init__(
        self,
        cfg: dict,
        camera: StereoCameraManager,
        turntable: TurntableController,
        object_info: Dict[str, Any] | None = None,
        parent=None,
    ):
        super().__init__(parent)
        self.cfg = cfg
        self.camera = camera
        self.turntable = turntable
        self.object_info = dict(object_info or {})
        # Object name is fixed to the scan timestamp at run time.
        self.object_name = ""
        self.weight = self.object_info.get("weight", "")
        self.weight_text = str(self.object_info.get("weight_text") or self.weight)
        self.weight_unit = str(self.object_info.get("weight_unit") or "")
        self._stop = False

    @pyqtSlot()
    def stop(self) -> None:
        self._stop = True

    def log(self, msg: str) -> None:
        self.log_signal.emit(str(msg))

    def _save_pair(self, scan_dir: Path, view_idx: int, angle_deg: float, pair) -> Tuple[str, str]:
        raw_dir = ensure_dir(scan_dir / "raw_stereo")
        left_path = raw_dir / f"view_{view_idx:03d}_angle_{angle_deg:07.3f}_SL_{pair.timestamp}.png"
        right_path = raw_dir / f"view_{view_idx:03d}_angle_{angle_deg:07.3f}_SR_{pair.timestamp}.png"
        if not cv2.imwrite(str(left_path), pair.left):
            raise IOError(f"Failed to save left image: {left_path}")
        if not cv2.imwrite(str(right_path), pair.right):
            raise IOError(f"Failed to save right image: {right_path}")
        return str(left_path), str(right_path)

    @pyqtSlot()
    def run(self) -> None:
        camera = self.camera
        turntable = self.turntable
        camera.log = self.log
        turntable.log = self.log
        stereo = StereoCalibrationManager(self.cfg, log_fn=self.log)
        try:
            calib_file = str(get_nested(self.cfg, "paths.calibration_file"))
            turntable_calib_file = str(get_nested(self.cfg, "paths.turntable_calibration_file"))
            if not os.path.exists(calib_file):
                raise FileNotFoundError(f"Stereo calibration YAML is missing: {calib_file}")
            stereo.load(calib_file)
            generate_pointcloud_now = bool(get_nested(
                self.cfg,
                "reconstruction.generate_pointcloud_after_capture",
                True,
            ))
            # Capture-only mode does not load CUDA, YOLO, or Open3D processing.
            segmenter = (
                ObjectSegmenter(self.cfg, log_fn=self.log)
                if generate_pointcloud_now else None
            )

            require_tt = bool(get_nested(self.cfg, "charuco_turntable_calibration.require_turntable_calibration", True))
            use_axis = bool(get_nested(self.cfg, "charuco_turntable_calibration.use_calibrated_turntable_axis", True))
            tt_calib = TurntableCalibrationManager(self.cfg, stereo, log_fn=self.log)
            if use_axis:
                if not os.path.exists(turntable_calib_file):
                    if require_tt:
                        raise FileNotFoundError("Turntable calibration is missing. Please run Calibrate Turntable first.")
                    self.log("[WARNING] Turntable calibration JSON is missing. Calibrated-axis stitching is disabled.")
                    use_axis = False
                else:
                    data = tt_calib.load_turntable_calibration(turntable_calib_file)
                    self.log(
                        "[OK] Turntable calibration loaded: "
                        f"center={data['turntable_center_m']}, axis={data['turntable_axis_unit']}, "
                        f"distance={data.get('camera_to_turntable_distance_m'):.4f} m"
                    )

            created_at_iso = time.strftime("%Y-%m-%dT%H:%M:%S")
            created_at_folder = timestamp_for_scan_folder()
            self.object_name = created_at_folder
            self.object_info["object_name"] = self.object_name
            scan_dir = create_scan_folder(
                self.cfg,
                self.object_name,
                weight=self.weight_text,
                weight_unit=self.weight_unit,
                created_at_folder=created_at_folder,
            )
            self.log(f"[SCAN] Output folder: {scan_dir}")
            object_index = len([p for p in Path(get_nested(self.cfg, "paths.output_root")).iterdir() if p.is_dir()])
            object_metadata = dict(self.object_info)
            object_metadata.update({
                "created_at": created_at_iso,
                "object_index": object_index,
                "object_name": self.object_name,
                "weight": self.weight,
                "weight_text": self.weight_text,
                "weight_unit": self.weight_unit,
                "object_folder": str(scan_dir),
                "folder_naming_rule": "YYYY_MM_DD_HHMMSS_weight_<weight>_<unit>",
            })
            write_json(scan_dir / "object_info.json", object_metadata)
            write_json(scan_dir / "scan_config_snapshot.json", self.cfg)
            shutil.copy2(calib_file, scan_dir / "stereo_calibration_used.yml")
            if use_axis and os.path.exists(turntable_calib_file):
                shutil.copy2(
                    turntable_calib_file,
                    scan_dir / "turntable_axis_calibration_used.json",
                )
            self.log(f"[SAVE] Object metadata: {scan_dir / 'object_info.json'}")

            viewpoints = max(1, int(get_nested(self.cfg, "turntable.viewpoint_number", 24)))
            total_angle = float(get_nested(self.cfg, "turntable.total_scan_angle_deg", 360.0))
            auto_angle = bool(get_nested(self.cfg, "turntable.auto_compute_rotation_angle", True))
            step_angle = total_angle / viewpoints if auto_angle else float(get_nested(self.cfg, "turntable.rotate_angle_per_step", 15.0))
            speed = int(get_nested(self.cfg, "turntable.rotate_speed", 20))
            wait_s = float(get_nested(self.cfg, "turntable.scan_stabilization_before_capture_sec", 1.0))
            acquire_during_rotation = bool(get_nested(self.cfg, "turntable.acquire_during_rotation", False))
            if acquire_during_rotation:
                self.log("[WARNING] acquire_during_rotation=True can blur images and reduce stitching accuracy.")

            if camera.cameras is None:
                raise RuntimeError(
                    "Stereo cameras are disconnected. Press 'Connect All' on the Scan tab."
                )
            if turntable.io is None and turntable.ser is None:
                raise RuntimeError(
                    "Turntable/Arduino is disconnected. Press 'Connect All' on the Scan tab."
                )
            self.status_signal.emit("Homing turntable with base ArUco IDs 1-10")
            home_result = tt_calib.move_turntable_to_aruco_home(camera, turntable)
            self.log(
                f"[SCAN HOME] Turntable home verification: "
                f"{'OK' if home_result.get('verified') else 'disabled/not verified'}"
            )
            object_metadata["aruco_home"] = home_result
            write_json(scan_dir / "object_info.json", object_metadata)

            captures: List[Dict[str, Any]] = []
            self.status_signal.emit("Capturing stereo views")
            for view_idx in range(viewpoints):
                if self._stop:
                    self.log("[STOP] Scan stopped before capture finished.")
                    return
                target_angle = float(view_idx * step_angle)
                rotation_command = 0.0 if view_idx == 0 else float(step_angle)
                if view_idx > 0:
                    self.log(f"[TURNTABLE] Rotate to view {view_idx:03d}: +{rotation_command:.3f} deg")
                    turntable.rotate_relative(rotation_command, speed=speed, wait_after=True)
                self.log(f"[STABILIZE] Waiting {wait_s:.3f} s before capture to avoid vibration blur.")
                time.sleep(max(0.0, wait_s))
                pair = camera.grab_synchronized_pair()
                self.image_signal.emit(pair.left, pair.right)
                left_path, right_path = self._save_pair(scan_dir, view_idx, target_angle, pair)
                row = {
                    "object_name": self.object_name,
                    "weight": self.weight_text,
                    "weight_unit": self.weight_unit,
                    "object_folder": str(scan_dir),
                    "view_index": view_idx,
                    "target_angle_deg": target_angle,
                    "actual_rotation_command_deg": rotation_command,
                    "stabilization_time_s": wait_s,
                    "timestamp": pair.timestamp,
                    "left_image_path": left_path,
                    "right_image_path": right_path,
                    "point_cloud_path": "",
                    "transform_method": "calibrated_turntable_axis" if use_axis else "none",
                    "turntable_calibration_file_used": turntable_calib_file if use_axis else "",
                }
                captures.append({"row": row, "left": pair.left, "right": pair.right})
                append_csv_row(
                    scan_dir / "scan_log.csv",
                    row,
                    [
                        "object_name", "weight", "weight_unit", "object_folder",
                        "view_index", "target_angle_deg", "actual_rotation_command_deg",
                        "stabilization_time_s", "timestamp", "left_image_path", "right_image_path",
                        "point_cloud_path", "transform_method", "turntable_calibration_file_used",
                    ],
                )
                self.log(f"[CAPTURE] View {view_idx:03d}: angle={target_angle:.3f} deg")

            turntable.release_backlash_or_shaking()

            if self._stop:
                self.log("[STOP] Scan stopped before reconstruction.")
                return
            if not generate_pointcloud_now:
                write_json(scan_dir / "capture_complete.json", {
                    "captured_views": len(captures),
                    "pointcloud_generation": "deferred",
                    "scan_folder": str(scan_dir),
                })
                self.status_signal.emit("Capture complete; point-cloud generation deferred")
                self.log(
                    "[DONE] Stereo capture finished. Point-cloud generation is "
                    "disabled; use the Point Cloud Processing tab later."
                )
                self.completed_signal.emit(True)
                return

            self.status_signal.emit("Reconstructing point clouds")
            recon = StereoReconstructor(self.cfg, stereo, log_fn=self.log)
            post = PointCloudPostProcessor(self.cfg, log_fn=self.log)
            pcd_paths: List[str] = []
            transforms: List[np.ndarray] = []
            transform_infos: List[Dict[str, Any]] = []
            pc_dir = ensure_dir(scan_dir / "pointclouds")
            for cap in captures:
                row = cap["row"]
                view_idx = int(row["view_index"])
                pcd, disparity, rect_l, rect_r = recon.create_point_cloud_from_pair(
                    cap["left"],
                    cap["right"],
                    segmenter=segmenter if segmenter and segmenter.enabled else None,
                )
                if use_axis:
                    pcd = post.crop_to_turntable_volume(pcd, tt_calib.calib_data)
                recon.save_debug_outputs(view_idx, disparity, rect_l, rect_r, scan_dir)
                pcd_path = pc_dir / f"view_{view_idx:03d}.ply"
                if o3d is None or not o3d.io.write_point_cloud(str(pcd_path), pcd):
                    raise IOError(f"Failed to write PLY: {pcd_path}")
                row["point_cloud_path"] = str(pcd_path)
                pcd_paths.append(str(pcd_path))
                angle = float(row["target_angle_deg"])
                if use_axis:
                    T = tt_calib.transform_view_to_home(angle)
                    method = "calibrated_turntable_axis"
                else:
                    T = np.eye(4, dtype=np.float64)
                    method = "identity_no_turntable_calibration"
                transforms.append(T)
                transform_infos.append({
                    "view_index": view_idx,
                    "target_angle_deg": angle,
                    "transform_method": method,
                    "turntable_calibration_file_used": turntable_calib_file if use_axis else "",
                })
                self.log(f"[POINTCLOUD] View {view_idx:03d}: {pcd_path} ({len(pcd.points)} points)")

            write_json(scan_dir / "viewpoint_transforms.json", {"views": transform_infos})
            if len(pcd_paths) >= 2 and bool(get_nested(self.cfg, "stitching.build_fused_pointcloud_after_scan", True)):
                self.status_signal.emit("Stitching point clouds")
                stitcher = PointCloudStitcher(self.cfg, log_fn=self.log)
                fused_path, _ = stitcher.fuse_pointcloud_paths_with_known_transforms(
                    pcd_paths,
                    transforms,
                    scan_dir / "registration",
                    refine_icp=bool(get_nested(self.cfg, "stitching.icp_refine_after_axis_alignment", True)),
                    transform_infos=transform_infos,
                )
                self.fused_signal.emit(fused_path)
            self.log("[DONE] Scan, reconstruction, stitching, and PLY saving finished.")
            self.completed_signal.emit(False)
        except Exception:
            self.error_signal.emit(traceback.format_exc())
        finally:
            self.finished_signal.emit()


class DeferredPointCloudWorker(QObject):
    log_signal = pyqtSignal(str)
    status_signal = pyqtSignal(str)
    fused_signal = pyqtSignal(str)
    finished_signal = pyqtSignal()
    error_signal = pyqtSignal(str)

    def __init__(self, cfg: dict, scan_folder: str, parent=None):
        super().__init__(parent)
        self.cfg = copy.deepcopy(cfg)
        self.scan_folder = Path(scan_folder)

    def log(self, message: str) -> None:
        self.log_signal.emit(str(message))

    @pyqtSlot()
    def run(self) -> None:
        try:
            scan_dir = self.scan_folder
            if not scan_dir.is_dir():
                raise FileNotFoundError(f"Scan folder not found: {scan_dir}")
            log_path = scan_dir / "scan_log.csv"
            if not log_path.is_file():
                raise FileNotFoundError(f"scan_log.csv not found in: {scan_dir}")
            with open(log_path, "r", newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            if not rows:
                raise RuntimeError(f"No captured views found in {log_path}")

            stereo = StereoCalibrationManager(self.cfg, log_fn=self.log)
            stereo_copy = scan_dir / "stereo_calibration_used.yml"
            stereo.load(str(stereo_copy) if stereo_copy.is_file() else None)
            turntable_calib = TurntableCalibrationManager(
                self.cfg,
                stereo,
                log_fn=self.log,
            )
            calibration_copy = scan_dir / "turntable_axis_calibration_used.json"
            if calibration_copy.is_file():
                turntable_calib.load_turntable_calibration(str(calibration_copy))
            else:
                turntable_calib.load_turntable_calibration()

            segmenter = ObjectSegmenter(self.cfg, log_fn=self.log)
            recon = StereoReconstructor(self.cfg, stereo, log_fn=self.log)
            post = PointCloudPostProcessor(self.cfg, log_fn=self.log)
            pc_dir = ensure_dir(scan_dir / "pointclouds")
            pcd_paths: List[str] = []
            transforms: List[np.ndarray] = []
            transform_infos: List[Dict[str, Any]] = []

            for position, row in enumerate(rows):
                view_idx = int(row.get("view_index", position))
                left_path = Path(row.get("left_image_path", ""))
                right_path = Path(row.get("right_image_path", ""))
                if not left_path.is_file():
                    matches = sorted((scan_dir / "raw_stereo").glob(
                        f"view_{view_idx:03d}_*_SL_*.png"
                    ))
                    left_path = matches[0] if matches else left_path
                if not right_path.is_file():
                    matches = sorted((scan_dir / "raw_stereo").glob(
                        f"view_{view_idx:03d}_*_SR_*.png"
                    ))
                    right_path = matches[0] if matches else right_path
                pair = StereoCameraManager.load_pair_from_files(left_path, right_path)
                self.status_signal.emit(
                    f"Generating point cloud {position + 1}/{len(rows)}"
                )
                pcd, disparity, rect_l, rect_r = recon.create_point_cloud_from_pair(
                    pair.left,
                    pair.right,
                    segmenter=segmenter if segmenter.enabled else None,
                )
                pcd = post.crop_to_turntable_volume(
                    pcd,
                    turntable_calib.calib_data,
                )
                recon.save_debug_outputs(view_idx, disparity, rect_l, rect_r, scan_dir)
                pcd_path = pc_dir / f"view_{view_idx:03d}.ply"
                if o3d is None or not o3d.io.write_point_cloud(str(pcd_path), pcd):
                    raise IOError(f"Failed to write PLY: {pcd_path}")
                angle = float(row.get("target_angle_deg", 0.0))
                pcd_paths.append(str(pcd_path))
                transforms.append(turntable_calib.transform_view_to_home(angle))
                transform_infos.append({
                    "view_index": view_idx,
                    "target_angle_deg": angle,
                    "transform_method": "deferred_calibrated_turntable_axis",
                    "source_left": str(left_path),
                    "source_right": str(right_path),
                })
                self.log(
                    f"[DEFERRED POINTCLOUD] View {view_idx:03d}: "
                    f"{len(pcd.points):,} points."
                )

            if len(pcd_paths) < 2:
                raise RuntimeError("At least two point clouds are required for fusion.")
            self.status_signal.emit("Fusing deferred point clouds")
            stitcher = PointCloudStitcher(self.cfg, log_fn=self.log)
            fused_path, report = stitcher.fuse_pointcloud_paths_with_known_transforms(
                pcd_paths,
                transforms,
                scan_dir / "registration",
                refine_icp=bool(get_nested(
                    self.cfg,
                    "stitching.icp_refine_after_axis_alignment",
                    True,
                )),
                transform_infos=transform_infos,
            )
            write_json(scan_dir / "deferred_processing_report.json", report)
            self.fused_signal.emit(fused_path)
            self.status_signal.emit("Deferred point-cloud generation complete")
            self.log(f"[DONE] Deferred fused PLY: {fused_path}")
        except Exception:
            self.error_signal.emit(traceback.format_exc())
        finally:
            self.finished_signal.emit()


class ScannerMainWindow(QMainWindow):
    def __init__(self, cfg: dict):
        super().__init__()
        self.cfg = cfg
        self.scan_thread = None
        self.scan_worker = None
        self.scan_started_at = None
        self.scan_camera = None
        self.scan_turntable = None
        self.deferred_thread = None
        self.deferred_worker = None
        self.calib_thread = None
        self.calib_worker = None
        # A saved JSON from an earlier launch is not sufficient. The physical
        # camera/turntable scene must be recalibrated once after each app start.
        self.turntable_calibrated_this_session = False
        self.stereo_calib_thread = None
        self.stereo_calib_worker = None
        self.calib_camera = None
        self.calib_last_pair = None
        self.preview_timer = QTimer(self)
        self.preview_timer.timeout.connect(self.grab_calibration_preview)
        self.setWindowTitle("Stereo Turntable Scanner - Modular")
        self.resize(1500, 900)
        self._build_ui()

    def _build_ui(self) -> None:
        self.setStyleSheet(
            "QMainWindow{background:#f1f5f9;} QWidget{font-family:Segoe UI,Arial;font-size:9pt;} "
            "QPushButton{background:#334155;color:white;border-radius:5px;padding:6px 10px;font-weight:700;} "
            "QPushButton:hover{background:#475569;} QPushButton#Primary{background:#2563eb;} QPushButton#Danger{background:#dc2626;} "
            "QGroupBox{background:white;border:1px solid #cbd5e1;border-radius:8px;margin-top:12px;padding:10px;} "
            "QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 4px;font-weight:800;} "
            "QTextEdit,QLineEdit,QSpinBox,QDoubleSpinBox{background:white;border:1px solid #94a3b8;border-radius:5px;padding:4px;}"
        )
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.scan_tab_index = self.tabs.addTab(self._build_scan_tab(), "Scan")
        self.stereo_calib_tab_index = self.tabs.addTab(self._build_stereo_calib_tab(), "Stereo Vision Calibration")
        self.turntable_calib_tab_index = self.tabs.addTab(self._build_turntable_calib_tab(), "Turntable Calibration")
        self.pointcloud_tab_index = self.tabs.addTab(
            self._build_pointcloud_processing_tab(),
            "Point Cloud Generation",
        )

    def _make_param_group(self, title_text: str, parent_layout: QVBoxLayout) -> QGridLayout:
        group = QGroupBox(title_text)
        group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        group.setStyleSheet(
            "QGroupBox { background:#ffffff; border:1px solid #cbd5e1; border-radius:8px; "
            "margin-top:10px; padding:5px; font-size:8.5pt; font-weight:900; color:#0f172a; }"
            "QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 6px; background:#ffffff; color:#0f172a; }"
        )
        grid = QGridLayout(group)
        grid.setContentsMargins(5, 10, 5, 5)
        grid.setHorizontalSpacing(3)
        grid.setVerticalSpacing(2)
        parent_layout.addWidget(group)
        return grid

    def _set_config_value_from_row(self, path: str, value: Any) -> None:
        set_nested(self.cfg, path, value)
        self._sync_same_path_rows(path, value)
        save_yaml(get_nested(self.cfg, "_config_file"), {k: v for k, v in self.cfg.items() if not k.startswith("_")})
        self.log(f"[PARAM] {path} = {value}")

    def _sync_same_path_rows(self, path: str, value: Any) -> None:
        for mapping_name in ("setting_rows", "stereo_setting_rows", "tt_setting_rows"):
            mapping = getattr(self, mapping_name, {})
            row = mapping.get(path)
            if row is not None:
                row.set_value(value)

    def _set_row_value(self, mapping_name: str, path: str, value: Any) -> None:
        mapping = getattr(self, mapping_name, {})
        row = mapping.get(path)
        if row is not None:
            row.set_value(value)
        editor_map = getattr(self, mapping_name.replace("_rows", "_widgets"), {})
        editor = editor_map.get(path)
        if isinstance(editor, QLineEdit):
            editor.setText(str(value))
        elif isinstance(editor, QSpinBox):
            editor.setValue(int(value))
        elif isinstance(editor, QDoubleSpinBox):
            editor.setValue(float(value))
        elif isinstance(editor, QCheckBox):
            editor.setChecked(bool(value))
        elif isinstance(editor, QComboBox):
            idx = editor.findText(str(value))
            if idx >= 0:
                editor.setCurrentIndex(idx)

    def _add_param_row(
        self,
        grid: QGridLayout,
        row_index: int,
        registry_name: str,
        path: str,
        label: str,
        value_type: type,
        **kwargs,
    ) -> int:
        value = get_nested(self.cfg, path)
        row = ParameterRow(path, label, value, value_type, self._set_config_value_from_row, **kwargs)
        row.changed.connect(lambda p, v: self.on_parameter_changed(p, v))
        rows = getattr(self, registry_name)
        widgets = getattr(self, registry_name.replace("_rows", "_widgets"))
        rows[path] = row
        widgets[path] = row.editor
        grid.addWidget(row, row_index, 0, 1, 2)
        return row_index + 1

    def on_parameter_changed(self, path: str, value: Any) -> None:
        camera_paths = {
            "camera.exposure_time_us", "camera.balance_white_auto", "camera.timeout_ms",
            "camera.left_serial", "camera.right_serial", "camera.num_cameras_to_open",
        }
        if path in camera_paths and self.calib_camera is not None:
            self.log("[INFO] Camera parameter changed. Reconnect calibration cameras to apply the new setting.")
        if path.startswith("camera.crop") or path == "camera.enable_center_crop":
            if hasattr(self, "calib_camera_summary_label"):
                self.calib_camera_summary_label.setText(camera_roi_summary_text(self.cfg))
        if path == "paths.turntable_placement_guide_image":
            self.load_turntable_placement_guide()

    def _build_scan_tab(self) -> QWidget:
        tab = QWidget()
        root = QVBoxLayout(tab)
        header = QFrame()
        header_layout = QHBoxLayout(header)
        self.object_name_label = QLabel("Object name: auto timestamp")
        self.object_name_label.setStyleSheet("font-weight:800;color:#0f172a;background:#e2e8f0;border-radius:5px;padding:6px 10px;")
        self.btn_start_scan = QPushButton("Start Scan")
        self.btn_start_scan.setObjectName("Primary")
        self.btn_stop_scan = QPushButton("Stop")
        self.btn_stop_scan.setObjectName("Danger")
        self.btn_connect_scan_hardware = QPushButton("Connect All")
        self.scan_connection_label = QLabel(
            "Camera + Turntable + Arduino: DISCONNECTED"
        )
        self.scan_connection_label.setStyleSheet(
            "background:#fee2e2;color:#991b1b;border-radius:5px;"
            "padding:6px 10px;font-weight:900;"
        )
        self.scan_reconstruction_checkbox = QCheckBox("Stereo reconstruction")
        self.scan_reconstruction_checkbox.setChecked(bool(get_nested(
            self.cfg,
            "reconstruction.generate_pointcloud_after_capture",
            True,
        )))
        self.scan_reconstruction_checkbox.setStyleSheet(
            "QCheckBox { background:#e2e8f0;color:#0f172a;border-radius:5px;"
            "padding:7px 10px;font-weight:800; }"
        )
        for w in [
            self.object_name_label,
            self.btn_connect_scan_hardware,
            self.scan_connection_label,
            self.btn_start_scan,
            self.btn_stop_scan,
            self.scan_reconstruction_checkbox,
        ]:
            header_layout.addWidget(w)
        header_layout.addStretch(1)
        root.addWidget(header)

        splitter = QSplitter()
        root.addWidget(splitter, 1)
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        self.left_view = ImageView("Left image")
        self.right_view = ImageView("Right image")
        img_splitter = QSplitter()
        img_splitter.addWidget(self.left_view)
        img_splitter.addWidget(self.right_view)
        left_layout.addWidget(img_splitter, 2)
        self.cloud_view = PointCloudView(
            int(get_nested(self.cfg, "viewer.display_3d_max_points", 250000)),
            float(get_nested(self.cfg, "viewer.display_3d_point_size", 2.0)),
            cfg=self.cfg,
        )
        left_layout.addWidget(QLabel("Fused 3D Point Cloud"))
        left_layout.addWidget(self.cloud_view, 3)
        self.fused_path_label = QLabel("No fused PLY yet")
        self.fused_path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        left_layout.addWidget(self.fused_path_label)
        splitter.addWidget(left_panel)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.addWidget(self._settings_group())
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        right_layout.addWidget(QLabel("Log"))
        right_layout.addWidget(self.log_box, 1)
        splitter.addWidget(right_panel)
        splitter.setSizes([1050, 450])

        self.btn_start_scan.clicked.connect(self.start_scan)
        self.btn_stop_scan.clicked.connect(self.stop_scan)
        self.btn_connect_scan_hardware.clicked.connect(
            self.toggle_scan_hardware_connection
        )
        self.scan_enter_shortcut = QShortcut(QKeySequence(Qt.Key_Return), tab)
        self.scan_enter_shortcut.setContext(Qt.WidgetWithChildrenShortcut)
        self.scan_enter_shortcut.activated.connect(
            lambda: self.btn_start_scan.click()
            if self.btn_start_scan.isEnabled()
            else None
        )
        self.scan_keypad_enter_shortcut = QShortcut(QKeySequence(Qt.Key_Enter), tab)
        self.scan_keypad_enter_shortcut.setContext(Qt.WidgetWithChildrenShortcut)
        self.scan_keypad_enter_shortcut.activated.connect(
            lambda: self.btn_start_scan.click()
            if self.btn_start_scan.isEnabled()
            else None
        )
        self.scan_reconstruction_checkbox.toggled.connect(
            lambda enabled: self._set_config_value_from_row(
                "reconstruction.generate_pointcloud_after_capture",
                bool(enabled),
            )
        )
        return tab

    def _settings_group(self) -> QGroupBox:
        group = QGroupBox("Scan settings")
        root = QVBoxLayout(group)
        root.setContentsMargins(5, 8, 5, 5)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        self.setting_rows: Dict[str, ParameterRow] = {}
        self.setting_widgets: Dict[str, Any] = {}

        g = self._make_param_group("Project / calibration paths", layout)
        r = 0
        r = self._add_param_row(g, r, "setting_rows", "paths.calibration_file", "Stereo YAML", str, is_path=True, path_mode="open_file")
        r = self._add_param_row(g, r, "setting_rows", "paths.turntable_calibration_file", "Turntable JSON", str, is_path=True, path_mode="open_file")
        r = self._add_param_row(g, r, "setting_rows", "paths.output_root", "Output root", str, is_path=True, path_mode="dir")

        g = self._make_param_group("Scan motion", layout)
        r = 0
        r = self._add_param_row(g, r, "setting_rows", "turntable.viewpoint_number", "Viewpoints", int, minimum=1, maximum=720)
        r = self._add_param_row(g, r, "setting_rows", "turntable.total_scan_angle_deg", "Total angle", float, minimum=1.0, maximum=720.0, decimals=3)
        r = self._add_param_row(g, r, "setting_rows", "turntable.auto_compute_rotation_angle", "Auto step", bool)
        r = self._add_param_row(g, r, "setting_rows", "turntable.rotate_angle_per_step", "Step angle", float, minimum=-720.0, maximum=720.0, decimals=3)
        r = self._add_param_row(g, r, "setting_rows", "turntable.rotate_speed", "Speed", int, minimum=1, maximum=1000)
        r = self._add_param_row(g, r, "setting_rows", "turntable.rotate_acceleration", "Acceleration", int, minimum=1, maximum=100)
        r = self._add_param_row(g, r, "setting_rows", "turntable.scan_stabilization_before_capture_sec", "Wait before cap", float, minimum=0.0, maximum=60.0, decimals=3)
        r = self._add_param_row(g, r, "setting_rows", "turntable.final_extra_rotation_deg", "Final extra deg", float, minimum=0.0, maximum=60.0, decimals=3)
        r = self._add_param_row(g, r, "setting_rows", "turntable.acquire_during_rotation", "Capture moving", bool)

        g = self._make_param_group("Stereo reconstruction", layout)
        r = 0
        r = self._add_param_row(g, r, "setting_rows", "reconstruction.min_depth_mm", "Min Z mm", float, minimum=-100000.0, maximum=100000.0, decimals=1)
        r = self._add_param_row(g, r, "setting_rows", "reconstruction.max_depth_mm", "Max Z mm", float, minimum=-100000.0, maximum=1000000.0, decimals=1)
        r = self._add_param_row(g, r, "setting_rows", "reconstruction.sgbm_min_disparity", "Min disp", int, minimum=-512, maximum=2048)
        r = self._add_param_row(g, r, "setting_rows", "reconstruction.sgbm_num_disparities", "Num disp", int, minimum=16, maximum=4096)
        r = self._add_param_row(g, r, "setting_rows", "reconstruction.sgbm_block_size", "Window", int, minimum=3, maximum=51)
        r = self._add_param_row(g, r, "setting_rows", "reconstruction.point_stride", "Point stride", int, minimum=1, maximum=50)

        g = self._make_param_group("Turntable noise crop", layout)
        r = 0
        r = self._add_param_row(g, r, "setting_rows", "postprocessing.use_turntable_volume_crop", "Enable crop", bool)
        r = self._add_param_row(g, r, "setting_rows", "postprocessing.turntable_radius_m", "Radius m", float, minimum=0.001, maximum=2.0, decimals=4)
        r = self._add_param_row(g, r, "setting_rows", "postprocessing.turntable_radius_margin_m", "Radius margin m", float, minimum=0.0, maximum=1.0, decimals=4)
        r = self._add_param_row(g, r, "setting_rows", "postprocessing.use_turntable_height_crop", "Height crop", bool)
        r = self._add_param_row(g, r, "setting_rows", "postprocessing.maximum_object_height_m", "Max height m", float, minimum=0.001, maximum=5.0, decimals=3)

        g = self._make_param_group("Object segmentation", layout)
        r = 0
        r = self._add_param_row(g, r, "setting_rows", "segmentation.enabled", "Enable model", bool)
        r = self._add_param_row(g, r, "setting_rows", "segmentation.model_path", "YOLO .pt model", str, is_path=True, path_mode="open_file")
        r = self._add_param_row(g, r, "setting_rows", "segmentation.device", "Device", str, options=["auto", "cuda:0", "cpu"])
        r = self._add_param_row(g, r, "setting_rows", "segmentation.half_precision", "CUDA FP16", bool)
        r = self._add_param_row(g, r, "setting_rows", "segmentation.object_class_id", "Object class", int, minimum=-1, maximum=10000)
        r = self._add_param_row(g, r, "setting_rows", "segmentation.confidence_threshold", "Confidence", float, minimum=0.0, maximum=1.0, decimals=3)
        r = self._add_param_row(g, r, "setting_rows", "segmentation.iou_threshold", "NMS IoU", float, minimum=0.0, maximum=1.0, decimals=3)
        r = self._add_param_row(g, r, "setting_rows", "segmentation.mask_threshold", "Mask threshold", float, minimum=0.0, maximum=1.0, decimals=3)
        r = self._add_param_row(g, r, "setting_rows", "segmentation.image_size", "Image size", int, minimum=32, maximum=8192)
        r = self._add_param_row(g, r, "setting_rows", "segmentation.max_detections", "Max instances", int, minimum=1, maximum=1000)
        r = self._add_param_row(g, r, "setting_rows", "segmentation.keep_largest_instance_only", "Largest only", bool)

        g = self._make_param_group("Stitching", layout)
        r = 0
        r = self._add_param_row(g, r, "setting_rows", "stitching.icp_refine_after_axis_alignment", "ICP refine", bool)
        r = self._add_param_row(g, r, "setting_rows", "stitching.registration_icp_distance_m", "ICP dist m", float, minimum=0.0001, maximum=1.0, decimals=5)
        r = self._add_param_row(g, r, "setting_rows", "stitching.fused_voxel_size_m", "Fused voxel m", float, minimum=0.0, maximum=1.0, decimals=5)

        layout.addStretch(1)
        scroll.setWidget(content)
        root.addWidget(scroll, 1)
        return group

    def _build_pointcloud_processing_tab(self) -> QWidget:
        tab = QWidget()
        root = QVBoxLayout(tab)
        header = QFrame()
        header_layout = QHBoxLayout(header)
        self.deferred_folder_edit = QLineEdit()
        self.deferred_folder_edit.setPlaceholderText(
            "Select a completed scan folder containing scan_log.csv and raw_stereo"
        )
        self.btn_deferred_browse = QPushButton("Select Scan Folder")
        self.btn_deferred_run = QPushButton("Generate Point Cloud")
        self.btn_deferred_run.setObjectName("Primary")
        header_layout.addWidget(QLabel("Captured folder"))
        header_layout.addWidget(self.deferred_folder_edit, 1)
        header_layout.addWidget(self.btn_deferred_browse)
        header_layout.addWidget(self.btn_deferred_run)
        root.addWidget(header)

        self.deferred_status_label = QLabel(
            "Waiting for a captured scan folder. Current Scan-tab reconstruction, "
            "YOLO segmentation, crop, and stitching settings will be used."
        )
        self.deferred_status_label.setWordWrap(True)
        self.deferred_status_label.setStyleSheet(
            "background:#e2e8f0;color:#0f172a;padding:8px;border-radius:5px;font-weight:700;"
        )
        root.addWidget(self.deferred_status_label)

        splitter = QSplitter(Qt.Horizontal)
        self.deferred_cloud_view = PointCloudView(
            int(get_nested(self.cfg, "viewer.display_3d_max_points", 250000)),
            float(get_nested(self.cfg, "viewer.display_3d_point_size", 2.0)),
            cfg=self.cfg,
        )
        splitter.addWidget(self.deferred_cloud_view)
        self.deferred_log_box = QTextEdit()
        self.deferred_log_box.setReadOnly(True)
        splitter.addWidget(self.deferred_log_box)
        splitter.setSizes([1050, 450])
        root.addWidget(splitter, 1)

        self.btn_deferred_browse.clicked.connect(self.select_deferred_scan_folder)
        self.btn_deferred_run.clicked.connect(self.run_deferred_pointcloud)
        return tab

    def _build_stereo_calib_tab(self) -> QWidget:
        """Build the Stereo Vision Calibration tab using the original UI structure."""
        tab = QWidget()
        root = QHBoxLayout(tab)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        splitter.setHandleWidth(10)
        splitter.setStyleSheet("QSplitter::handle { background:#cbd5e1; border:1px solid #94a3b8; }")
        root.addWidget(splitter, 1)

        # Left control/parameter/log panel: same structure as the original calibration tab.
        left_panel = QWidget()
        left_panel.setMinimumWidth(360)
        left_panel.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(5)

        title = QLabel("Stereo Vision Calibration")
        title.setStyleSheet("font-size:12px; font-weight:900; color:#0f172a;")
        left_layout.addWidget(title)

        actions = QFrame()
        actions.setObjectName("ActionBar")
        action_layout = QGridLayout(actions)
        action_layout.setContentsMargins(4, 4, 4, 4)
        action_layout.setHorizontalSpacing(4)
        action_layout.setVerticalSpacing(4)

        self.btn_calib_connect = QPushButton("Connect")
        self.btn_calib_save_pair = QPushButton("Save Pair [Space]")
        self.btn_calib_select_folder = QPushButton("Image Folder")
        self.btn_calib_clear = QPushButton("Clear Images")
        self.btn_run_stereo_calib = QPushButton("Calibration")
        self.btn_calib_sgbm_3d = QPushButton("SGBM 3D")
        self.btn_calib_save_pair.setObjectName("Primary")
        self.btn_run_stereo_calib.setObjectName("Primary")

        self.btn_calib_connect.setToolTip("Connect/disconnect stereo cameras for live preview and image capture.")
        self.btn_calib_save_pair.setToolTip("Save one fresh pair as calib_SL_*.png and calib_SR_*.png. Press Space as a shortcut.")
        self.btn_calib_select_folder.setToolTip("Create/select a folder for calibration images.")
        self.btn_calib_clear.setToolTip("Delete saved calib_SL_*.png and calib_SR_*.png images from the current image folder.")
        self.btn_run_stereo_calib.setToolTip("Run stereo calibration from the saved image folder and save stereoMap.yml.")
        self.btn_calib_sgbm_3d.setToolTip("Load stereoMap.yml and generate one SGBM 3D point cloud from a stereo pair.")

        buttons = [
            self.btn_calib_connect,
            self.btn_calib_save_pair,
            self.btn_calib_select_folder,
            self.btn_calib_clear,
            self.btn_run_stereo_calib,
            self.btn_calib_sgbm_3d,
        ]
        for i, btn in enumerate(buttons):
            btn.setMinimumHeight(24)
            btn.setMaximumHeight(30)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            action_layout.addWidget(btn, i // 2, i % 2)
        left_layout.addWidget(actions)

        self.calib_status_label = QLabel("Saved calibration pairs: 0")
        self.calib_status_label.setWordWrap(False)
        self.calib_status_label.setStyleSheet("color:#0f172a;font-weight:700;")
        left_layout.addWidget(self.calib_status_label)

        left_layout.addWidget(self._stereo_calib_settings_group(), 1)

        self.calib_log_box = QTextEdit()
        self.calib_log_box.setReadOnly(True)
        self.calib_log_box.setMinimumHeight(90)
        left_layout.addWidget(self.calib_log_box, 0)

        # Right preview panel: 2D images + 3D point cloud tabs, matching original intent.
        preview_panel = QWidget()
        preview_layout = QVBoxLayout(preview_panel)
        preview_layout.setContentsMargins(0, 0, 0, 0)
        preview_layout.setSpacing(5)

        preview_title = QLabel("Stereo calibration display")
        preview_title.setStyleSheet("font-size:10pt;font-weight:900;color:#0f172a;")
        preview_layout.addWidget(preview_title)

        self.calib_display_tabs = QTabWidget()
        images_tab = QWidget()
        images_layout = QVBoxLayout(images_tab)
        images_layout.setContentsMargins(0, 0, 0, 0)
        image_splitter = QSplitter(Qt.Horizontal)
        image_splitter.setChildrenCollapsible(False)
        self.calib_left_view = ZoomableImageViewer("Left camera", self)
        self.calib_right_view = ZoomableImageViewer("Right camera", self)
        image_splitter.addWidget(self.calib_left_view)
        image_splitter.addWidget(self.calib_right_view)
        image_splitter.setSizes([700, 700])
        images_layout.addWidget(image_splitter, 1)

        cloud_tab = QWidget()
        cloud_layout = QVBoxLayout(cloud_tab)
        cloud_layout.setContentsMargins(0, 0, 0, 0)
        self.calib_cloud_view = PointCloudView(
            int(get_nested(self.cfg, "viewer.display_3d_max_points", 250000)),
            float(get_nested(self.cfg, "viewer.display_3d_point_size", 2.0)),
            cfg=self.cfg,
        )
        self.calib_cloud_view.info_label.setText("3D point cloud: press SGBM 3D")
        cloud_layout.addWidget(self.calib_cloud_view, 1)

        self.calib_display_tabs.addTab(images_tab, "2D Images")
        self.calib_display_tabs.addTab(cloud_tab, "3D Point Cloud")
        preview_layout.addWidget(self.calib_display_tabs, 1)

        splitter.addWidget(left_panel)
        splitter.addWidget(preview_panel)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([430, 1250])

        self.btn_calib_connect.clicked.connect(self.toggle_calibration_camera_connection)
        self.btn_calib_save_pair.clicked.connect(self.save_current_calibration_pair)
        self.btn_calib_select_folder.clicked.connect(self.select_calibration_capture_folder)
        self.btn_calib_clear.clicked.connect(self.clear_calibration_images)
        self.btn_run_stereo_calib.clicked.connect(self.run_stereo_calibration)
        self.btn_calib_sgbm_3d.clicked.connect(self.generate_calibration_sgbm_3d)

        self.space_shortcut = QShortcut(QKeySequence(Qt.Key_Space), tab)
        self.space_shortcut.setContext(Qt.WidgetWithChildrenShortcut)
        self.space_shortcut.activated.connect(self.save_current_calibration_pair)

        self.calib_left_view.set_image(make_status_image("Stereo vision calibration", ["Connect cameras, save chessboard pairs, then press Calibration."]), "Stereo vision calibration")
        self.calib_right_view.clear_image("No right image")
        self._update_calib_status_label()
        return tab

    def _stereo_calib_settings_group(self) -> QScrollArea:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.stereo_setting_rows: Dict[str, ParameterRow] = {}
        self.stereo_setting_widgets: Dict[str, Any] = {}

        g = self._make_param_group("Camera setup", layout)
        self.calib_camera_summary_label = QLabel(camera_roi_summary_text(self.cfg))
        self.calib_camera_summary_label.setStyleSheet("color:#475569; font-size:8.2pt; font-weight:700;")
        g.addWidget(self.calib_camera_summary_label, 0, 0, 1, 2)
        r = 1
        r = self._add_param_row(g, r, "stereo_setting_rows", "camera.left_serial", "Left serial", str)
        r = self._add_param_row(g, r, "stereo_setting_rows", "camera.right_serial", "Right serial", str)
        r = self._add_param_row(g, r, "stereo_setting_rows", "camera.exposure_time_us", "Exposure us", float, minimum=1, maximum=10000000, decimals=1)
        r = self._add_param_row(g, r, "stereo_setting_rows", "camera.balance_white_auto", "White balance", str, options=["Off", "Once", "Continuous"])
        r = self._add_param_row(g, r, "stereo_setting_rows", "camera.timeout_ms", "Timeout ms", int, minimum=100, maximum=60000)
        r = self._add_param_row(g, r, "stereo_setting_rows", "camera.enable_center_crop", "Enable crop", bool)
        r = self._add_param_row(g, r, "stereo_setting_rows", "camera.crop_width", "Width", int, minimum=0, maximum=10000)
        r = self._add_param_row(g, r, "stereo_setting_rows", "camera.crop_height", "Height", int, minimum=0, maximum=10000)
        r = self._add_param_row(g, r, "stereo_setting_rows", "camera.crop_center_x", "Center X", int, minimum=-1, maximum=10000)
        r = self._add_param_row(g, r, "stereo_setting_rows", "camera.crop_center_y", "Center Y", int, minimum=-1, maximum=10000)

        g = self._make_param_group("Calibration paths", layout)
        r = 0
        r = self._add_param_row(g, r, "stereo_setting_rows", "paths.calibration_capture_dir", "Image folder", str, is_path=True, path_mode="dir")
        r = self._add_param_row(g, r, "stereo_setting_rows", "paths.calibration_file", "Calib file", str, is_path=True, path_mode="file")

        g = self._make_param_group("Chessboard", layout)
        r = 0
        r = self._add_param_row(g, r, "stereo_setting_rows", "stereo_calibration.board_columns", "Columns", int, minimum=2, maximum=50)
        r = self._add_param_row(g, r, "stereo_setting_rows", "stereo_calibration.board_rows", "Rows", int, minimum=2, maximum=50)
        r = self._add_param_row(g, r, "stereo_setting_rows", "stereo_calibration.square_size_mm", "Square mm", float, minimum=0.1, maximum=1000, decimals=3)
        r = self._add_param_row(g, r, "stereo_setting_rows", "stereo_calibration.epipolar_error_threshold", "Max epi err", float, minimum=0.0, maximum=100.0, decimals=3)
        r = self._add_param_row(g, r, "stereo_setting_rows", "stereo_calibration.resolution_percent", "Resolution %", int, minimum=10, maximum=100)
        r = self._add_param_row(g, r, "stereo_setting_rows", "stereo_calibration.reject_high_rms", "Reject RMS", bool)

        g = self._make_param_group("SGBM point cloud", layout)
        r = 0
        for path, label, typ, mn, mx, dec in [
            ("reconstruction.sgbm_min_disparity", "Min disp", int, -512, 2048, 0),
            ("reconstruction.sgbm_num_disparities", "Num disp", int, 16, 4096, 0),
            ("reconstruction.sgbm_block_size", "Window", int, 3, 51, 0),
            ("reconstruction.sgbm_disp12_max_diff", "Disp12 diff", int, -1, 256, 0),
            ("reconstruction.sgbm_prefilter_cap", "Prefilter", int, 1, 255, 0),
            ("reconstruction.sgbm_uniqueness_ratio", "Unique", int, 0, 100, 0),
            ("reconstruction.sgbm_speckle_window_size", "Speckle win", int, 0, 1000, 0),
            ("reconstruction.sgbm_speckle_range", "Speckle rng", int, 0, 100, 0),
            ("reconstruction.min_depth_mm", "Min Z mm", float, -100000, 100000, 1),
            ("reconstruction.max_depth_mm", "Max Z mm", float, -100000, 1000000, 1),
            ("reconstruction.point_stride", "Point stride", int, 1, 50, 0),
            ("reconstruction.voxel_size_m", "Voxel m", float, 0.0, 1.0, 5),
            ("reconstruction.outlier_nb_neighbors", "Outlier nb", int, 0, 500, 0),
            ("reconstruction.outlier_std_ratio", "Outlier std", float, 0.1, 20.0, 2),
        ]:
            if typ is int:
                r = self._add_param_row(g, r, "stereo_setting_rows", path, label, int, minimum=mn, maximum=mx)
            else:
                r = self._add_param_row(g, r, "stereo_setting_rows", path, label, float, minimum=mn, maximum=mx, decimals=dec)

        layout.addStretch(1)
        scroll.setWidget(content)
        return scroll

    def _build_turntable_calib_tab(self) -> QWidget:
        tab = QWidget()
        root = QHBoxLayout(tab)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        splitter.setHandleWidth(10)
        splitter.setStyleSheet("QSplitter::handle { background:#cbd5e1; border:1px solid #94a3b8; }")
        root.addWidget(splitter, 1)

        left_panel = QWidget()
        left_panel.setMinimumWidth(360)
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(5)
        title = QLabel("Turntable Calibration")
        title.setStyleSheet("font-size:12px; font-weight:900; color:#0f172a;")
        left_layout.addWidget(title)

        actions = QFrame()
        action_layout = QGridLayout(actions)
        action_layout.setContentsMargins(4, 4, 4, 4)
        action_layout.setHorizontalSpacing(4)
        action_layout.setVerticalSpacing(4)
        self.btn_run_turntable_calib = QPushButton("Run Calibration")
        self.btn_run_turntable_calib.setObjectName("Primary")
        self.btn_reload_tt_calib = QPushButton("Check")
        for i, btn in enumerate([self.btn_run_turntable_calib, self.btn_reload_tt_calib]):
            btn.setMinimumHeight(26)
            action_layout.addWidget(btn, 0, i)
        self.tt_run_state_label = QLabel("Status: STOPPED")
        self.tt_run_state_label.setAlignment(Qt.AlignCenter)
        self.tt_run_state_label.setStyleSheet(
            "background:#475569;color:white;border-radius:4px;padding:5px;font-weight:900;"
        )
        action_layout.addWidget(self.tt_run_state_label, 1, 0, 1, 2)
        left_layout.addWidget(actions)
        self.tt_status_label = QLabel("Place the ChArUco board as shown, then run calibration.")
        self.tt_status_label.setWordWrap(True)
        self.tt_status_label.setStyleSheet("color:#0f172a;font-weight:700;")
        self.tt_status_label.setText(
            "Calibration required after every application start. Mount the "
            "ChArUco board as shown, then press Run Calibration."
        )
        left_layout.addWidget(self.tt_status_label)
        left_layout.addWidget(self._turntable_calib_settings_group(), 1)

        self.tt_log_box = QTextEdit()
        self.tt_log_box.setReadOnly(True)
        self.tt_log_box.setMinimumHeight(90)
        left_layout.addWidget(self.tt_log_box, 0)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        self.turntable_guide_view = ZoomableImageViewer("How to place the ChArUco board", self)
        self.load_turntable_placement_guide()
        right_layout.addWidget(self.turntable_guide_view, 2)
        self.turntable_live_left = ZoomableImageViewer("Turntable calibration debug left", self)
        right_layout.addWidget(self.turntable_live_left, 1)

        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([430, 1250])

        self.btn_run_turntable_calib.clicked.connect(self.calibrate_turntable_now)
        self.btn_reload_tt_calib.clicked.connect(self.check_turntable_calibration_file)
        return tab

    def _turntable_calib_settings_group(self) -> QScrollArea:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        self.tt_setting_rows: Dict[str, ParameterRow] = {}
        self.tt_setting_widgets: Dict[str, Any] = {}

        g = self._make_param_group("Turntable calibration file", layout)
        r = 0
        r = self._add_param_row(g, r, "tt_setting_rows", "paths.turntable_calibration_file", "Calib JSON", str, is_path=True, path_mode="file")
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.require_turntable_calibration", "Required", bool)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.require_calibration_each_app_start", "Every app start", bool)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.use_calibrated_turntable_axis", "Use axis", bool)

        g = self._make_param_group("ChArUco board", layout)
        r = 0
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.charuco_squares_x", "Squares X", int, minimum=2, maximum=50)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.charuco_squares_y", "Squares Y", int, minimum=2, maximum=50)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.charuco_square_length_m", "Square m", float, minimum=0.001, maximum=1.0, decimals=5)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.charuco_marker_length_m", "Marker m", float, minimum=0.001, maximum=1.0, decimals=5)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.charuco_dictionary", "Dictionary", str, options=["DICT_4X4_50", "DICT_4X4_100", "DICT_5X5_100", "DICT_6X6_250"])
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.charuco_legacy_pattern", "Legacy pattern", bool)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.expected_camera_to_turntable_distance_m", "Expected dist m", float, minimum=0.0, maximum=10.0, decimals=4)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.distance_sanity_tolerance_m", "Dist tolerance m", float, minimum=0.001, maximum=2.0, decimals=4)

        g = self._make_param_group("Calibration motion", layout)
        r = 0
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.calibration_viewpoints", "Viewpoints", int, minimum=4, maximum=100)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.calibration_total_angle_deg", "Total angle", float, minimum=1.0, maximum=720.0, decimals=3)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.calibration_stabilization_sec", "Wait s", float, minimum=0.0, maximum=30.0, decimals=3)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.coarse_search_step_deg", "Forward search deg", float, minimum=1.0, maximum=180.0, decimals=2)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.coarse_search_speed", "Search speed", int, minimum=1, maximum=100)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.coarse_search_max_rotations", "Forward tries", int, minimum=0, maximum=72)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.minimum_charuco_corners", "Min corners", int, minimum=4, maximum=200)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.maximum_reprojection_error_px", "Max reproj px", float, minimum=0.1, maximum=20.0, decimals=3)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.stability_timeout_sec", "Settle timeout", float, minimum=0.5, maximum=60.0, decimals=2)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.stability_translation_tolerance_m", "Stable move m", float, minimum=0.00001, maximum=0.05, decimals=5)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.maximum_circle_rms_m", "Max circle RMS", float, minimum=0.0001, maximum=0.05, decimals=5)
        r = self._add_param_row(g, r, "tt_setting_rows", "charuco_turntable_calibration.minimum_angular_coverage_deg", "Min coverage", float, minimum=30.0, maximum=360.0, decimals=1)
        r = self._add_param_row(g, r, "tt_setting_rows", "turntable.rotate_speed", "Speed", int, minimum=1, maximum=1000)
        r = self._add_param_row(g, r, "tt_setting_rows", "turntable.rotate_acceleration", "Acceleration", int, minimum=1, maximum=100)
        r = self._add_param_row(g, r, "tt_setting_rows", "turntable.serial_port", "Serial port", str)
        r = self._add_param_row(g, r, "tt_setting_rows", "turntable.baudrate", "Baudrate", int, minimum=1200, maximum=2000000)

        g = self._make_param_group("Base ArUco home (IDs 1-10)", layout)
        r = 0
        r = self._add_param_row(g, r, "tt_setting_rows", "aruco_fallback.use_aruco_home_return", "Use home", bool)
        r = self._add_param_row(g, r, "tt_setting_rows", "aruco_fallback.aruco_dictionary", "Base dictionary", str, options=["DICT_7X7_50", "DICT_7X7_100", "DICT_4X4_50", "DICT_4X4_100", "DICT_5X5_100"])
        r = self._add_param_row(g, r, "tt_setting_rows", "aruco_fallback.aruco_home_id", "Home ID", int, minimum=0, maximum=999)
        r = self._add_param_row(g, r, "tt_setting_rows", "aruco_fallback.aruco_marker_count", "Marker count", int, minimum=1, maximum=360)
        r = self._add_param_row(g, r, "tt_setting_rows", "aruco_fallback.aruco_first_id", "First ID", int, minimum=0, maximum=999)
        r = self._add_param_row(g, r, "tt_setting_rows", "aruco_fallback.aruco_positive_id_direction", "ID direction", int, minimum=-1, maximum=1)
        r = self._add_param_row(g, r, "tt_setting_rows", "aruco_fallback.aruco_home_search_step_deg", "Home search deg", float, minimum=-180.0, maximum=180.0, decimals=2)
        r = self._add_param_row(g, r, "tt_setting_rows", "aruco_fallback.aruco_home_search_speed", "Home search speed", int, minimum=1, maximum=100)
        r = self._add_param_row(g, r, "tt_setting_rows", "aruco_fallback.aruco_home_correction_speed", "Home fine speed", int, minimum=1, maximum=100)

        layout.addStretch(1)
        scroll.setWidget(content)
        return scroll

    def apply_settings(self) -> None:
        for widgets in [getattr(self, "setting_widgets", {}), getattr(self, "stereo_setting_widgets", {}), getattr(self, "tt_setting_widgets", {})]:
            for path, widget in widgets.items():
                if isinstance(widget, QSpinBox):
                    value = int(widget.value())
                elif isinstance(widget, QDoubleSpinBox):
                    value = float(widget.value())
                elif isinstance(widget, QLineEdit):
                    value = widget.text().strip()
                elif isinstance(widget, QCheckBox):
                    value = bool(widget.isChecked())
                elif isinstance(widget, QComboBox):
                    value = widget.currentText().strip()
                else:
                    continue
                set_nested(self.cfg, path, value)
        save_yaml(get_nested(self.cfg, "_config_file"), {k: v for k, v in self.cfg.items() if not k.startswith("_")})
        self.log("[PARAM] Settings applied and saved to YAML.")

    def browse_stereo_calibration(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Select stereoMap.yml",
            str(Path(get_nested(self.cfg, "paths.calibration_file")).parent),
            "YAML (*.yml *.yaml);;All files (*)",
        )
        if path:
            self.setting_widgets["paths.calibration_file"].setText(path)
            if "paths.calibration_file" in self.stereo_setting_widgets:
                self.stereo_setting_widgets["paths.calibration_file"].setText(path)
            set_nested(self.cfg, "paths.calibration_file", path)
            self.log(f"[PARAM] Stereo calibration file: {path}")

    def log(self, msg: str) -> None:
        text = str(msg)
        for box_name in (
            "log_box",
            "calib_log_box",
            "tt_log_box",
            "deferred_log_box",
        ):
            box = getattr(self, box_name, None)
            if box is not None:
                try:
                    box.append(text)
                except Exception:
                    pass

    def set_images(self, left, right) -> None:
        self.left_view.set_image(left, "Left image")
        self.right_view.set_image(right, "Right image")

    def _update_calib_status_label(self) -> None:
        try:
            capture_dir = str(get_nested(self.cfg, "paths.calibration_capture_dir"))
            counts = StereoCalibrationManager.calibration_folder_image_counts(capture_dir)
            self.calib_status_label.setText(
                f"Saved pairs: {counts['matched_pairs']} | "
                f"images={counts['total_calibration_images']} "
                f"(L={counts['left_images']}, R={counts['right_images']})"
            )
        except Exception:
            pass

    def toggle_calibration_camera_connection(self) -> None:
        if self.calib_camera is None:
            self.connect_calibration_cameras()
            self.start_calibration_preview()
        else:
            self.stop_calibration_preview()
            try:
                self.calib_camera.close()
            except Exception:
                pass
            self.calib_camera = None
            self.btn_calib_connect.setText("Connect")
            self.calib_status_label.setText("Stereo calibration cameras disconnected.")
            self.log("[CALIB] Cameras disconnected.")

    def generate_calibration_sgbm_3d(self) -> None:
        try:
            self.apply_settings()
            stereo = StereoCalibrationManager(self.cfg, log_fn=self.log)
            stereo.load()
            source_note = "current calibration pair"
            if self.calib_last_pair is None:
                pairs = StereoCalibrationManager.saved_calibration_pairs(get_nested(self.cfg, "paths.calibration_capture_dir"))
                if pairs:
                    self.calib_last_pair = StereoCameraManager.load_pair_from_files(pairs[-1][0], pairs[-1][1])
                    source_note = f"latest saved pair: {Path(pairs[-1][0]).name}"
                else:
                    if self.calib_camera is None:
                        self.connect_calibration_cameras()
                    self.calib_last_pair = self.calib_camera.grab_synchronized_pair()
                    source_note = "fresh camera pair"
            recon = StereoReconstructor(self.cfg, stereo, log_fn=self.log)
            pcd, disparity, rect_l, rect_r = recon.create_point_cloud_from_pair(self.calib_last_pair.left, self.calib_last_pair.right)
            if o3d is None or len(pcd.points) <= 0:
                raise RuntimeError("SGBM generated an empty point cloud. Check disparity/depth settings.")
            self.calib_left_view.set_image(draw_horizontal_guides(rect_l), "SGBM rectified left")
            self.calib_right_view.set_image(draw_horizontal_guides(rect_r), "SGBM rectified right")
            self.calib_display_tabs.setCurrentIndex(1)
            self.calib_cloud_view.show_point_cloud_object(
                pcd,
                title="SGBM calibration point cloud",
                calibration=stereo.calibration,
            )
            self.calib_status_label.setText(f"SGBM point cloud displayed: {len(pcd.points)} points")
            self.log(f"[SGBM] Source: {source_note}")
            self.log(f"[SGBM] Point cloud displayed ({len(pcd.points)} points).")
        except Exception:
            QMessageBox.critical(self, "SGBM point-cloud error", traceback.format_exc())

    # ------------------------- Stereo calibration tab -------------------------
    def connect_calibration_cameras(self) -> None:
        try:
            self.apply_settings()
            if self.scan_hardware_is_connected():
                self.disconnect_scan_hardware()
                self.log(
                    "[CALIB] Released the shared Scan connection because "
                    "calibration requires exclusive camera access."
                )
            if self.calib_camera is None:
                self.calib_camera = StereoCameraManager(self.cfg, log_fn=self.log)
                self.calib_camera.connect()
            if hasattr(self, "btn_calib_connect"):
                self.btn_calib_connect.setText("Disconnect")
            self.calib_status_label.setText("Stereo cameras connected for calibration.")
        except Exception:
            QMessageBox.critical(self, "Camera connection error", traceback.format_exc())

    def start_calibration_preview(self) -> None:
        try:
            if self.calib_camera is None:
                self.connect_calibration_cameras()
            self.preview_timer.start(120)
            self.calib_status_label.setText("Live calibration preview started.")
        except Exception:
            QMessageBox.critical(self, "Preview error", traceback.format_exc())

    def stop_calibration_preview(self) -> None:
        self.preview_timer.stop()
        self.calib_status_label.setText("Live calibration preview stopped.")

    def grab_calibration_preview(self) -> None:
        try:
            if self.calib_camera is None:
                return
            pair = self.calib_camera.grab_synchronized_pair()
            self.calib_last_pair = pair
            self.calib_left_view.set_image(pair.left, "Calibration left image")
            self.calib_right_view.set_image(pair.right, "Calibration right image")
        except Exception as exc:
            self.preview_timer.stop()
            self.log(f"[CALIB PREVIEW ERROR] {exc}")

    def save_current_calibration_pair(self) -> None:
        try:
            self.apply_settings()
            if self.calib_camera is None:
                self.connect_calibration_cameras()
            was_previewing = self.preview_timer.isActive()
            if was_previewing:
                self.preview_timer.stop()
            pair = self.calib_camera.grab_synchronized_pair()
            self.calib_last_pair = pair
            capture_dir = ensure_dir(get_nested(self.cfg, "paths.calibration_capture_dir"))
            left_path = Path(capture_dir) / f"calib_SL_{pair.timestamp}.png"
            right_path = Path(capture_dir) / f"calib_SR_{pair.timestamp}.png"
            if not cv2.imwrite(str(left_path), pair.left):
                raise IOError(f"Failed to save {left_path}")
            if not cv2.imwrite(str(right_path), pair.right):
                raise IOError(f"Failed to save {right_path}")
            self.calib_left_view.set_image(pair.left, "Saved left image")
            self.calib_right_view.set_image(pair.right, "Saved right image")
            counts = StereoCalibrationManager.calibration_folder_image_counts(capture_dir)
            self.calib_status_label.setText(
                f"Saved pair. Folder: {capture_dir}\n"
                f"Left={counts['left_images']} Right={counts['right_images']} Matched={counts['matched_pairs']}"
            )
            self.log(f"[SAVE] {left_path}")
            self.log(f"[SAVE] {right_path}")
            if was_previewing:
                self.preview_timer.start(300)
        except Exception:
            QMessageBox.critical(self, "Save calibration pair error", traceback.format_exc())

    def select_calibration_capture_folder(self) -> None:
        current = str(get_nested(self.cfg, "paths.calibration_capture_dir"))
        selected = QFileDialog.getExistingDirectory(self, "Select calibration image folder", current)
        if selected:
            set_nested(self.cfg, "paths.calibration_capture_dir", selected)
            self.stereo_setting_widgets["paths.calibration_capture_dir"].setText(selected)
            counts = StereoCalibrationManager.calibration_folder_image_counts(selected)
            self.calib_status_label.setText(f"Selected folder: {selected}\nMatched pairs: {counts['matched_pairs']}")
            self.log(f"[CALIB] Selected folder: {selected}")

    def clear_calibration_images(self) -> None:
        try:
            capture_dir = Path(get_nested(self.cfg, "paths.calibration_capture_dir"))
            files = list(capture_dir.glob("calib_SL_*.png")) + list(capture_dir.glob("calib_SR_*.png"))
            if not files:
                self.calib_status_label.setText("No saved calibration pair images found.")
                return
            reply = QMessageBox.question(
                self,
                "Clear calibration images",
                f"Delete {len(files)} saved calibration images from:\n{capture_dir}?\n\nThe stereoMap.yml result will not be deleted.",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                return
            for f in files:
                try:
                    f.unlink()
                except Exception as exc:
                    self.log(f"[CLEAR WARNING] Cannot delete {f}: {exc}")
            self.calib_status_label.setText("Saved calibration pair images cleared.")
            self.log(f"[CLEAR] Deleted {len(files)} calibration images.")
        except Exception:
            QMessageBox.critical(self, "Clear calibration images error", traceback.format_exc())

    def run_stereo_calibration(self) -> None:
        if self.stereo_calib_thread is not None:
            QMessageBox.warning(self, "Busy", "Stereo calibration is already running.")
            return
        self.apply_settings()
        capture_dir = str(get_nested(self.cfg, "paths.calibration_capture_dir"))
        save_path = str(get_nested(self.cfg, "paths.calibration_file"))
        counts = StereoCalibrationManager.calibration_folder_image_counts(capture_dir)
        if counts["matched_pairs"] < 3:
            QMessageBox.warning(
                self,
                "Not enough calibration pairs",
                f"At least 3 matched stereo pairs are required. Current matched pairs: {counts['matched_pairs']}.\n\n"
                "Use Save Pair to capture more chessboard images first.",
            )
            return
        if self.preview_timer.isActive():
            self.preview_timer.stop()
            self.log("[CALIB] Live preview stopped before running calibration from saved images.")
        if self.calib_camera is not None:
            try:
                self.calib_camera.close()
                self.log("[CALIB] Stereo cameras disconnected before Calibration.")
            except Exception as exc:
                self.log(f"[CALIB WARNING] Could not disconnect stereo cameras before calibration: {exc}")
            finally:
                self.calib_camera = None
                self.btn_calib_connect.setText("Connect")
        self.calib_display_tabs.setCurrentIndex(0)
        self.calib_left_view.set_image(
            make_status_image(
                "Calibration running",
                [
                    "Saved stereo image pairs will be loaded now.",
                    "Chessboard detection previews will appear here.",
                    "Do not close the app until calibration finishes.",
                ],
            ),
            "Calibration running",
        )
        self.calib_right_view.clear_image("Waiting for calibration preview")
        QApplication.processEvents()
        self.stereo_calib_thread = QThread()
        self.stereo_calib_worker = StereoCalibrationWorker(self.cfg, capture_dir, save_path)
        self.stereo_calib_worker.moveToThread(self.stereo_calib_thread)
        self.stereo_calib_thread.started.connect(self.stereo_calib_worker.run)
        self.stereo_calib_worker.log_signal.connect(self.log)
        self.stereo_calib_worker.status_signal.connect(self.calib_status_label.setText)
        self.stereo_calib_worker.image_signal.connect(self.on_stereo_calibration_preview_image)
        self.stereo_calib_worker.error_signal.connect(self.on_worker_error)
        self.stereo_calib_worker.error_signal.connect(self.stereo_calib_thread.quit)
        self.stereo_calib_worker.finished_signal.connect(self.on_stereo_calibration_finished)
        self.stereo_calib_worker.finished_signal.connect(self.stereo_calib_thread.quit)
        self.stereo_calib_worker.finished_signal.connect(self.stereo_calib_worker.deleteLater)
        self.stereo_calib_thread.finished.connect(self.stereo_calib_thread.deleteLater)
        self.stereo_calib_thread.finished.connect(lambda: setattr(self, "stereo_calib_thread", None))
        self.stereo_calib_thread.start()
        self.calib_status_label.setText("Stereo calibration is running...")
        self.log("[START] Stereo calibration started.")

    def on_stereo_calibration_preview_image(self, left, right, left_title: str, right_title: str) -> None:
        """Display calibration progress images in the 2D Images tab."""
        self.calib_display_tabs.setCurrentIndex(0)
        if left is not None:
            self.calib_left_view.set_image(left, left_title or "Left calibration image")
        else:
            self.calib_left_view.clear_image(left_title or "No left image")
        if right is not None:
            self.calib_right_view.set_image(right, right_title or "Right calibration image")
        else:
            self.calib_right_view.clear_image(right_title or "No right image")
        QApplication.processEvents()

    def on_stereo_calibration_finished(self, data: dict) -> None:
        msg = (
            f"Stereo calibration finished.\n"
            f"Left RMS={data.get('left_rms')} | Right RMS={data.get('right_rms')} | Stereo RMS={data.get('stereo_rms')}\n"
            f"Used pairs={data.get('num_pairs_used')} / {data.get('num_pairs')}\n"
            f"Saved: {get_nested(self.cfg, 'paths.calibration_file')}"
        )
        self.calib_status_label.setText(msg)
        self.log("[STEREO CALIB DONE] " + msg.replace("\n", " | "))
        WorkflowDoneDialog(
            "Stereo calibration complete",
            "Stereo Vision Calibration finished successfully.\n\n"
            f"Stereo RMS: {data.get('stereo_rms')}\n"
            f"Saved: {get_nested(self.cfg, 'paths.calibration_file')}\n\n"
            "Press Enter or click X to close.",
            self,
        ).exec_()

    def test_rectification(self) -> None:
        try:
            self.apply_settings()
            if self.calib_last_pair is None:
                if self.calib_camera is None:
                    self.connect_calibration_cameras()
                self.calib_last_pair = self.calib_camera.grab_synchronized_pair()
            stereo = StereoCalibrationManager(self.cfg, log_fn=self.log)
            stereo.load()
            result = stereo.validate_rectification(self.calib_last_pair.left, self.calib_last_pair.right, save_dir=Path(get_nested(self.cfg, "paths.calibration_file")).parent)
            rect_l = result.get("rect_left")
            rect_r = result.get("rect_right")
            if rect_l is None or rect_r is None:
                rect_l, rect_r = stereo.rectify(self.calib_last_pair.left, self.calib_last_pair.right)
            self.calib_left_view.set_image(draw_horizontal_guides(rect_l), "Rectified left - calibration check")
            self.calib_right_view.set_image(draw_horizontal_guides(rect_r), "Rectified right - calibration check")
            self.calib_display_tabs.setCurrentIndex(0)
            self.calib_status_label.setText(f"Rectification test complete. Valid chessboard check={result.get('valid')}")
        except Exception:
            QMessageBox.critical(self, "Rectification test error", traceback.format_exc())

    # ------------------------- Turntable calibration tab -------------------------
    def load_turntable_placement_guide(self) -> None:
        """Show the configured placement image immediately, with a safe fallback."""
        if not hasattr(self, "turntable_guide_view"):
            return
        image_path = str(get_nested(
            self.cfg,
            "paths.turntable_placement_guide_image",
            "",
        ) or "").strip()
        image = None
        if image_path:
            try:
                # imdecode supports Windows paths containing non-ASCII characters.
                encoded = np.fromfile(image_path, dtype=np.uint8)
                image = cv2.imdecode(encoded, cv2.IMREAD_COLOR) if encoded.size else None
            except Exception as exc:
                self.log(f"[GUIDE IMAGE WARNING] Cannot read {image_path}: {exc}")
        if image is not None:
            self.turntable_guide_view.set_image(
                image,
                f"Turntable placement guide: {Path(image_path).name}",
                fit=True,
            )
            self.log(f"[GUIDE IMAGE] Loaded turntable placement guide: {image_path}")
            return
        self.turntable_guide_view.set_image(
            make_turntable_placement_guide(),
            "Generated turntable placement guide",
            fit=True,
        )
        if image_path:
            self.log(
                f"[GUIDE IMAGE WARNING] Image is missing or unreadable: {image_path}. "
                "Showing the generated fallback guide."
            )

    def show_turntable_calibration_required(self) -> None:
        self.tabs.setCurrentIndex(self.turntable_calib_tab_index)
        self.load_turntable_placement_guide()
        QMessageBox.warning(
            self,
            "Turntable calibration required",
            "Turntable calibration is missing.\n\n"
            "Please mount the 13 x 8 ChArUco board vertically, rigidly, and offset from the rotation axis as shown, "
            "then press 'Run Turntable Calibration'.",
        )

    def calibrate_turntable_now(self) -> None:
        if self.calib_thread is not None:
            if self.calib_worker is not None:
                self.calib_worker.stop()
                self.btn_run_turntable_calib.setEnabled(False)
                self.btn_run_turntable_calib.setText("Stopping...")
                self.tt_run_state_label.setText("Status: STOPPING")
                self.tt_status_label.setText(
                    "STOPPING — Waiting for the current turntable or camera command to finish."
                )
            return
        self.apply_settings()
        if self.scan_hardware_is_connected():
            self.disconnect_scan_hardware()
            self.log(
                "[TURNTABLE CALIB] Released the shared Scan connection because "
                "turntable calibration requires exclusive hardware access."
            )
        calib_file = str(get_nested(self.cfg, "paths.calibration_file"))
        if not os.path.exists(calib_file):
            QMessageBox.warning(
                self,
                "Stereo calibration required",
                "Stereo calibration YAML is missing. Please run Stereo Vision Calibration first.",
            )
            self.tabs.setCurrentIndex(self.stereo_calib_tab_index)
            return

        # The turntable worker creates its own StereoCameraManager. Release any
        # camera objects held by the stereo-calibration preview first; Basler
        # devices cannot be attached to two InstantCamera instances at once.
        if self.preview_timer.isActive():
            self.preview_timer.stop()
        if self.calib_camera is not None:
            try:
                self.calib_camera.close()
                self.log("[TURNTABLE CALIB] Released stereo-preview cameras.")
            except Exception as exc:
                self.log(f"[TURNTABLE CALIB WARNING] Could not release preview cameras: {exc}")
            finally:
                self.calib_camera = None
                self.btn_calib_connect.setText("Connect")

        self.tabs.setCurrentIndex(self.turntable_calib_tab_index)
        self.calib_thread = QThread()
        self.calib_worker = TurntableCalibrationWorker(self.cfg)
        self.calib_worker.moveToThread(self.calib_thread)
        self.calib_thread.started.connect(self.calib_worker.run)
        self.calib_worker.log_signal.connect(self.log)
        self.calib_worker.error_signal.connect(self.on_turntable_calibration_error)
        self.calib_worker.error_signal.connect(self.calib_thread.quit)
        self.calib_worker.stopped_signal.connect(self.on_turntable_calibration_stopped)
        self.calib_worker.stopped_signal.connect(self.calib_thread.quit)
        self.calib_worker.finished_signal.connect(self.on_turntable_calibration_finished)
        self.calib_worker.finished_signal.connect(self.calib_thread.quit)
        self.calib_worker.finished_signal.connect(self.calib_worker.deleteLater)
        self.calib_thread.finished.connect(self.calib_thread.deleteLater)
        self.calib_thread.finished.connect(self.on_turntable_calibration_thread_stopped)
        self.calib_thread.finished.connect(lambda: setattr(self, "calib_thread", None))
        self.set_turntable_calibration_running(True)
        self.calib_thread.start()
        self.tt_status_label.setText(
            "RUNNING — Turntable calibration is active. Do not touch the board or turntable."
        )
        self.log("[START] Turntable calibration started.")

    def set_turntable_calibration_running(self, running: bool) -> None:
        self.btn_run_turntable_calib.setEnabled(True)
        self.btn_run_turntable_calib.setText("Stop Calibration" if running else "Run Calibration")
        self.btn_run_turntable_calib.setStyleSheet(
            "background:#dc2626;color:white;font-weight:800;"
            if running else
            "background:#2563eb;color:white;font-weight:800;"
        )
        self.btn_reload_tt_calib.setEnabled(not running)
        if running:
            self.tt_run_state_label.setText("Status: RUNNING")
            self.tt_run_state_label.setStyleSheet(
                "background:#16a34a;color:white;border-radius:4px;padding:5px;font-weight:900;"
            )
        else:
            self.tt_run_state_label.setText("Status: STOPPED")
            self.tt_run_state_label.setStyleSheet(
                "background:#475569;color:white;border-radius:4px;padding:5px;font-weight:900;"
            )

    def on_turntable_calibration_thread_stopped(self) -> None:
        self.set_turntable_calibration_running(False)
        self.calib_worker = None

    def on_turntable_calibration_stopped(self) -> None:
        self.tt_status_label.setText("STOPPED — Turntable calibration was stopped by the operator.")
        self.log("[TURNTABLE CALIB] Calibration stopped safely; no new calibration was saved.")

    def on_turntable_calibration_error(self, err: str) -> None:
        self.tt_status_label.setText("STOPPED — Turntable calibration failed. See the error log.")
        self.log("[TURNTABLE CALIB STOPPED WITH ERROR]\n" + err)
        QMessageBox.critical(self, "Turntable calibration error", err)

    def on_turntable_calibration_finished(self, data: dict) -> None:
        self.turntable_calibrated_this_session = True
        msg = (
            f"STOPPED — Turntable calibration finished successfully.\n"
            f"center={data.get('turntable_center_m')}\n"
            f"axis={data.get('turntable_axis_unit')}\n"
            f"distance={data.get('camera_to_turntable_distance_m'):.4f} m | "
            f"circle RMS={data.get('rms_circle_fit_error_m', 0.0) * 1000.0:.3f} mm | "
            f"plane RMS={data.get('rms_plane_fit_error_m', 0.0) * 1000.0:.3f} mm\n"
            f"coverage={data.get('angular_coverage_deg', 0.0):.1f} deg | "
            f"inliers={data.get('num_inlier_views', 0)}/{data.get('num_calibration_views', 0)} | "
            f"angle scale={data.get('angle_scale', 1.0):.5f}\n"
            f"Saved: {get_nested(self.cfg, 'paths.turntable_calibration_file')}"
        )
        self.tt_status_label.setText(msg)
        self.log("[TURNTABLE CALIB DONE] " + msg.replace("\n", " | "))
        WorkflowDoneDialog(
            "Turntable calibration complete",
            "Turntable Calibration finished successfully.\n\n"
            f"Circle RMS: {data.get('rms_circle_fit_error_m', 0.0) * 1000.0:.3f} mm\n"
            f"Coverage: {data.get('angular_coverage_deg', 0.0):.1f} degrees\n"
            f"Saved: {get_nested(self.cfg, 'paths.turntable_calibration_file')}\n\n"
            "Press Enter or click X to close.",
            self,
        ).exec_()
        self.tabs.setCurrentIndex(self.scan_tab_index)
        self.btn_start_scan.setFocus()
        self.log("[WORKFLOW] Turntable calibration acknowledged. Switched to the Scan tab.")

    def check_turntable_calibration_file(self) -> None:
        try:
            self.apply_settings()
            stereo = StereoCalibrationManager(self.cfg, log_fn=self.log)
            manager = TurntableCalibrationManager(self.cfg, stereo, log_fn=self.log)
            data = manager.load_turntable_calibration()
            self.tt_status_label.setText(
                f"Calibration JSON is valid.\ncenter={data.get('turntable_center_m')}\naxis={data.get('turntable_axis_unit')}\n"
                f"distance={data.get('camera_to_turntable_distance_m')}"
            )
        except Exception:
            QMessageBox.critical(self, "Turntable calibration check error", traceback.format_exc())

    # ------------------------- Scan actions -------------------------
    def set_scan_running(self, running: bool) -> None:
        self.btn_start_scan.setEnabled(not running)
        self.btn_connect_scan_hardware.setEnabled(not running)
        if running:
            self.btn_start_scan.setText("SCANNING...")
            self.btn_start_scan.setStyleSheet(
                "QPushButton { background:#f59e0b; color:#111827; "
                "border:2px solid #b45309; border-radius:6px; padding:8px 16px; "
                "font-weight:900; font-size:10pt; }"
            )
            self.object_name_label.setStyleSheet(
                "font-weight:900;color:#7c2d12;background:#ffedd5;"
                "border:2px solid #fb923c;border-radius:5px;padding:6px 10px;"
            )
        else:
            self.btn_start_scan.setText("Start Scan")
            self.btn_start_scan.setStyleSheet("")
            self.object_name_label.setStyleSheet(
                "font-weight:800;color:#0f172a;background:#e2e8f0;"
                "border-radius:5px;padding:6px 10px;"
            )

    def scan_hardware_is_connected(self) -> bool:
        return bool(
            self.scan_camera is not None
            and self.scan_camera.cameras is not None
            and self.scan_turntable is not None
            and (
                self.scan_turntable.io is not None
                or self.scan_turntable.ser is not None
            )
        )

    def update_scan_connection_status(self) -> None:
        connected = self.scan_hardware_is_connected()
        self.btn_connect_scan_hardware.setText(
            "Disconnect All" if connected else "Connect All"
        )
        self.scan_connection_label.setText(
            "Camera + Turntable + Arduino: CONNECTED"
            if connected
            else "Camera + Turntable + Arduino: DISCONNECTED"
        )
        self.scan_connection_label.setStyleSheet(
            (
                "background:#dcfce7;color:#166534;border-radius:5px;"
                "padding:6px 10px;font-weight:900;"
            )
            if connected
            else (
                "background:#fee2e2;color:#991b1b;border-radius:5px;"
                "padding:6px 10px;font-weight:900;"
            )
        )

    def connect_scan_hardware(self) -> bool:
        if self.scan_hardware_is_connected():
            return True
        self.apply_settings()
        self.disconnect_scan_hardware(log_message=False)
        camera = StereoCameraManager(self.cfg, log_fn=self.log)
        turntable = TurntableController(self.cfg, log_fn=self.log)
        try:
            if self.preview_timer.isActive():
                self.preview_timer.stop()
            if self.calib_camera is not None:
                self.calib_camera.close()
                self.calib_camera = None
                self.btn_calib_connect.setText("Connect")
            camera.connect()
            turntable.connect()
        except Exception:
            camera.close()
            turntable.close()
            self.scan_camera = None
            self.scan_turntable = None
            self.update_scan_connection_status()
            error = traceback.format_exc()
            self.log("[CONNECTION ERROR]\n" + error)
            QMessageBox.critical(self, "Machine connection error", error)
            return False
        self.scan_camera = camera
        self.scan_turntable = turntable
        self.update_scan_connection_status()
        self.log(
            "[CONNECTION] Stereo cameras, turntable, and Arduino connected. "
            "The same connection will be reused for every scan."
        )
        return True

    def disconnect_scan_hardware(self, log_message: bool = True) -> None:
        if self.scan_camera is not None:
            self.scan_camera.close()
        if self.scan_turntable is not None:
            self.scan_turntable.close()
        self.scan_camera = None
        self.scan_turntable = None
        if hasattr(self, "btn_connect_scan_hardware"):
            self.update_scan_connection_status()
        if log_message:
            self.log("[CONNECTION] Camera, turntable, and Arduino disconnected.")

    def toggle_scan_hardware_connection(self) -> None:
        if self.scan_hardware_is_connected():
            self.disconnect_scan_hardware()
        else:
            self.connect_scan_hardware()

    def select_deferred_scan_folder(self) -> None:
        selected = QFileDialog.getExistingDirectory(
            self,
            "Select captured scan folder",
            str(get_nested(self.cfg, "paths.output_root")),
        )
        if selected:
            self.deferred_folder_edit.setText(selected)
            self.deferred_status_label.setText(
                f"Selected: {selected}\nPress Generate Point Cloud to process saved stereo pairs."
            )

    def run_deferred_pointcloud(self) -> None:
        if self.deferred_thread is not None:
            QMessageBox.warning(
                self,
                "Busy",
                "Deferred point-cloud generation is already running.",
            )
            return
        self.apply_settings()
        scan_folder = self.deferred_folder_edit.text().strip()
        if not scan_folder or not Path(scan_folder).is_dir():
            QMessageBox.warning(
                self,
                "Scan folder required",
                "Select a valid captured scan folder first.",
            )
            return
        if not (Path(scan_folder) / "scan_log.csv").is_file():
            QMessageBox.warning(
                self,
                "Invalid scan folder",
                "The selected folder does not contain scan_log.csv.",
            )
            return

        self.deferred_thread = QThread()
        self.deferred_worker = DeferredPointCloudWorker(self.cfg, scan_folder)
        self.deferred_worker.moveToThread(self.deferred_thread)
        self.deferred_thread.started.connect(self.deferred_worker.run)
        self.deferred_worker.log_signal.connect(self.log)
        self.deferred_worker.status_signal.connect(
            self.deferred_status_label.setText
        )
        self.deferred_worker.fused_signal.connect(
            self.on_deferred_fused_ready
        )
        self.deferred_worker.error_signal.connect(self.on_worker_error)
        self.deferred_worker.error_signal.connect(self.deferred_thread.quit)
        self.deferred_worker.finished_signal.connect(self.deferred_thread.quit)
        self.deferred_worker.finished_signal.connect(
            self.deferred_worker.deleteLater
        )
        self.deferred_thread.finished.connect(self.deferred_thread.deleteLater)
        self.deferred_thread.finished.connect(
            lambda: setattr(self, "deferred_thread", None)
        )
        self.btn_deferred_run.setEnabled(False)
        self.deferred_thread.finished.connect(
            lambda: self.btn_deferred_run.setEnabled(True)
        )
        self.deferred_status_label.setText("Deferred processing is running...")
        self.deferred_thread.start()

    def on_deferred_fused_ready(self, fused_path: str) -> None:
        self.deferred_status_label.setText(
            f"Point-cloud generation complete:\n{fused_path}"
        )
        self.deferred_cloud_view.load_ply(fused_path)
        WorkflowDoneDialog(
            "Point-cloud processing complete",
            "Deferred point-cloud generation and registration finished successfully.\n\n"
            f"Saved PLY:\n{fused_path}\n\n"
            "Press Enter or click X to close.",
            self,
        ).exec_()

    def show_cuda_diagnostics(self) -> None:
        info = ObjectSegmenter.cuda_diagnostics()
        if info["cuda_available"]:
            status = (
                "CUDA is ready.\n\n"
                f"Python: {info['python']}\n"
                f"GPU: {info['cuda_device_name']}\n"
                f"PyTorch: {info['torch_version']}\n"
                f"PyTorch CUDA build: {info['torch_cuda_build']}\n"
                f"Ultralytics: {info['ultralytics_version'] or 'not installed'}"
            )
            QMessageBox.information(self, "CUDA check", status)
            return
        status = (
            "Windows and the NVIDIA driver can see the GPU, but Scan_app's Python "
            "cannot use CUDA yet.\n\n"
            f"Python: {info['python']}\n"
            f"NVIDIA GPU/driver: {info['gpu'] or 'not detected by nvidia-smi'}\n"
            f"PyTorch installed: {info['torch_installed']}\n"
            f"PyTorch CUDA build: {info['torch_cuda_build'] or 'none'}\n"
            f"Ultralytics installed: {info['ultralytics_installed']}\n\n"
            "Install into this exact Python environment:\n"
            f'"{info["python"]}" -m pip install torch torchvision '
            "--index-url https://download.pytorch.org/whl/cu126\n"
            f'"{info["python"]}" -m pip install ultralytics\n\n'
            "Restart the app, then press Check CUDA again."
        )
        QMessageBox.warning(self, "CUDA is not ready", status)

    def start_scan(self) -> None:
        if self.scan_thread is not None:
            QMessageBox.warning(self, "Busy", "Scan is already running.")
            return
        if not self.scan_hardware_is_connected():
            QMessageBox.warning(
                self,
                "Machine is disconnected",
                "Connect the stereo cameras, turntable, and Arduino first.\n\n"
                "Press 'Connect All' on the Scan tab. The connection will then "
                "remain open and be reused for all following scans.",
            )
            self.tabs.setCurrentIndex(self.scan_tab_index)
            self.btn_connect_scan_hardware.setFocus()
            return
        self.apply_settings()
        stereo_calib_file = str(get_nested(self.cfg, "paths.calibration_file"))
        if not os.path.exists(stereo_calib_file):
            QMessageBox.warning(
                self,
                "Stereo calibration required",
                "Stereo calibration YAML is missing.\n\nPlease open the Stereo Vision Calibration tab, capture chessboard pairs, and run calibration before scanning.",
            )
            self.tabs.setCurrentIndex(self.stereo_calib_tab_index)
            return
        turntable_file = str(get_nested(self.cfg, "paths.turntable_calibration_file"))
        require_tt = bool(get_nested(self.cfg, "charuco_turntable_calibration.require_turntable_calibration", True))
        use_axis = bool(get_nested(self.cfg, "charuco_turntable_calibration.use_calibrated_turntable_axis", True))
        require_session_calibration = bool(get_nested(
            self.cfg,
            "charuco_turntable_calibration.require_calibration_each_app_start",
            True,
        ))
        if (
            require_tt
            and use_axis
            and require_session_calibration
            and not self.turntable_calibrated_this_session
        ):
            self.tabs.setCurrentIndex(self.turntable_calib_tab_index)
            self.tt_status_label.setText(
                "Turntable calibration is required for this application session "
                "before scanning."
            )
            QMessageBox.warning(
                self,
                "Turntable recalibration required",
                "The camera, turntable, or surrounding scene may have changed since "
                "the application was started.\n\n"
                "Run Turntable Calibration now. After it finishes successfully, "
                "return to the Scan tab and press Start Scan again.",
            )
            self.log(
                "[SCAN BLOCKED] Turntable calibration has not been completed "
                "during this application session."
            )
            return
        if require_tt and use_axis and not os.path.exists(turntable_file):
            self.show_turntable_calibration_required()
            return

        object_dialog = ObjectInfoDialog(self)
        if object_dialog.exec_() != QDialog.Accepted:
            self.log("[CANCEL] Scan was cancelled before object weight was confirmed.")
            return
        object_info = object_dialog.object_info()
        self.object_name_label.setText("Object name: auto timestamp")

        self.scan_thread = QThread()
        self.scan_worker = ScanWorker(
            self.cfg,
            self.scan_camera,
            self.scan_turntable,
            object_info,
        )
        self.scan_worker.moveToThread(self.scan_thread)
        self.scan_thread.started.connect(self.scan_worker.run)
        self.scan_worker.log_signal.connect(self.log)
        self.scan_worker.status_signal.connect(lambda s: self.log(f"[STATUS] {s}"))
        self.scan_worker.image_signal.connect(self.set_images)
        self.scan_worker.fused_signal.connect(self.on_fused_ready)
        self.scan_worker.completed_signal.connect(self.on_scan_completed)
        self.scan_worker.error_signal.connect(self.on_worker_error)
        self.scan_worker.finished_signal.connect(self.scan_thread.quit)
        self.scan_worker.finished_signal.connect(self.scan_worker.deleteLater)
        self.scan_thread.finished.connect(self.scan_thread.deleteLater)
        self.scan_thread.finished.connect(lambda: setattr(self, "scan_thread", None))
        self.scan_thread.finished.connect(lambda: self.set_scan_running(False))
        self.set_scan_running(True)
        self.scan_started_at = time.perf_counter()
        self.scan_thread.start()
        self.log("[START] Scan started.")

    def stop_scan(self) -> None:
        if self.scan_worker is not None:
            self.scan_worker.stop()
            self.log("[STOP] Stop requested.")

    def on_fused_ready(self, fused_path: str) -> None:
        self.fused_path_label.setText(fused_path)
        self.cloud_view.load_ply(fused_path)

    def on_scan_completed(self, capture_only: bool) -> None:
        elapsed_seconds = (
            max(0.0, time.perf_counter() - self.scan_started_at)
            if self.scan_started_at is not None
            else 0.0
        )
        self.scan_started_at = None
        minutes, seconds = divmod(elapsed_seconds, 60.0)
        hours, minutes = divmod(int(minutes), 60)
        if hours:
            elapsed_text = f"{hours:d} h {minutes:d} min {seconds:.1f} sec"
        elif minutes:
            elapsed_text = f"{minutes:d} min {seconds:.1f} sec"
        else:
            elapsed_text = f"{seconds:.1f} sec"
        self.log(
            f"[SCAN TIME] Scan completed in {elapsed_text} "
            f"({elapsed_seconds:.3f} seconds total)."
        )
        self.tabs.setCurrentIndex(self.scan_tab_index)
        self.object_name_label.setText("Object finished - prepare next object")
        dialog = NextObjectDialog(
            self,
            capture_only=capture_only,
            elapsed_text=elapsed_text,
        )
        dialog.exec_()
        self.object_name_label.setText("Object name: auto timestamp")
        self.left_view.clear_image("Waiting for next object")
        self.right_view.clear_image("Waiting for next object")
        self.log("[NEXT OBJECT] Operator acknowledged completion.")
        # Queue the next object workflow after the completion dialog has closed
        # and the scan worker thread has returned to the event loop.
        QTimer.singleShot(150, self.start_scan)

    def on_worker_error(self, err: str) -> None:
        self.log("[ERROR]\n" + err)
        QMessageBox.critical(self, "Scanner error", err)

    def closeEvent(self, event) -> None:
        try:
            self.preview_timer.stop()
            if self.calib_camera is not None:
                self.calib_camera.close()
            self.disconnect_scan_hardware(log_message=False)
        except Exception:
            pass
        super().closeEvent(event)
