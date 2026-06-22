from __future__ import annotations

import glob
import json
import math
import os
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np

try:
    from scipy.optimize import least_squares
except Exception:  # pragma: no cover
    least_squares = None

from .camera_manager import StereoCameraManager
from .project_io import ensure_dir, get_nested, read_json, timestamp, write_json
from .turntable_controller import TurntableController


class StereoCalibrationManager:
    """Stereo calibration manager using the original standalone behavior.

    IMPORTANT:
    The calibration calculation below intentionally follows the user's original
    standalone script behavior:
        - sorted(glob(...calib_SL_*.png)) and sorted(glob(...calib_SR_*.png))
        - zip(imagesLeft, imagesRight) pairing
        - chessboardSize=(8, 6), squareSize=16 when standard mode is enabled
        - cv2.findChessboardCorners classic detector
        - cv2.calibrateCamera(..., imageShape[::-1], None, None)
        - cv2.stereoCalibrate(..., imageShape[::-1], criteria, flags) positional call
        - epipolar filtering with threshold=1.0
        - stereoRectify alpha=1 and CV_16SC2 maps

    Scanner/point-cloud methods are kept compatible with the rest of the UI.
    """

    def __init__(self, cfg: AppConfig, log_fn: Callable[[str], None] = print):
        self.cfg = cfg
        self.log = log_fn
        self.objpoints: List[np.ndarray] = []
        self.imgpoints_l: List[np.ndarray] = []
        self.imgpoints_r: List[np.ndarray] = []
        # image_shape follows the original script: gray.shape = (height, width)
        self.image_shape: Optional[Tuple[int, int]] = None
        # image_size is kept for UI/scanner compatibility: (width, height)
        self.image_size: Optional[Tuple[int, int]] = None
        self.calibration: Dict[str, Any] = {}
        self.stereoMapL: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self.stereoMapR: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self.Q: Optional[np.ndarray] = None
        self.last_preview_left: Optional[np.ndarray] = None
        self.last_preview_right: Optional[np.ndarray] = None
        self.last_found_left: bool = False
        self.last_found_right: bool = False

    # ============================================================
    # Original standalone helper behavior
    # ============================================================
    def use_standard_calibration_method(self) -> bool:
        return bool(getattr(self.cfg, "USE_STANDARD_CALIBRATION_METHOD", True))

    def _calib_value(self, name: str, standard_value: Any) -> Any:
        if self.use_standard_calibration_method():
            return standard_value
        return getattr(self.cfg, name, standard_value)

    def calibration_file_path(self) -> str:
        """Return the calibration YAML path from current or legacy config objects."""
        path = get_nested(self.cfg, "paths.calibration_file")
        if not path:
            path = getattr(self.cfg, "CALIB_FILE", None)
        if not path:
            raise ValueError("Stereo calibration file path is not configured.")
        return str(path)

    def apply_standard_calibration_settings_to_cfg(self) -> None:
        standard_settings = {
            "CHESSBOARD_COLUMNS": 8,
            "CHESSBOARD_ROWS": 6,
            "CHESSBOARD_SQUARE_SIZE_MM": 16,
            "CHESSBOARD_DETECTION_MODE": "CLASSIC",
            "CALIB_EPIPOLAR_ERROR_THRESHOLD": 1.0,
            "CALIB_RESOLUTION_PERCENT": 100,
            "CALIB_REJECT_HIGH_RMS": False,
        }
        for key, value in standard_settings.items():
            try:
                setattr(self.cfg, key, value)
            except Exception:
                pass
        self.log(
            "[S2 CALIB] Standard calibration settings: chessboard=(8,6), "
            "square=16 mm, detection=CLASSIC, threshold=1.0, resolution=100%."
        )

    def setupDirectories(
        self,
        folderName: str = "stereo_camera_calibration_data",
        imageFolder: str = "images_top",
        calibratedFolder: str = "Calibrated",
    ) -> Tuple[str, str]:
        # Same logic as the original standalone code.
        currentDir = os.getcwd()
        folderPath = os.path.join(currentDir, folderName)
        calibSavePath = os.path.join(folderPath, calibratedFolder)
        inputImagePath = os.path.join(folderPath, imageFolder)

        if not os.path.isdir(os.path.abspath(calibSavePath)):
            os.mkdir(calibSavePath)
        if not os.path.isdir(os.path.abspath(os.path.join(calibSavePath, imageFolder))):
            os.mkdir(os.path.join(calibSavePath, imageFolder))

        return inputImagePath, os.path.join(calibSavePath, imageFolder)

    def clear_points(self) -> None:
        self.objpoints.clear()
        self.imgpoints_l.clear()
        self.imgpoints_r.clear()
        self.image_shape = None
        self.image_size = None
        self.calibration.clear()
        self.stereoMapL = None
        self.stereoMapR = None
        self.Q = None
        self.last_preview_left = None
        self.last_preview_right = None
        self.last_found_left = False
        self.last_found_right = False
        self.log("[OK] Calibration point buffers cleared.")

    def board_pattern(self) -> Tuple[int, int]:
        # Original standalone code: chessboardSize = (8, 6), columns, rows.
        return int(self._calib_value("CHESSBOARD_COLUMNS", 8)), int(self._calib_value("CHESSBOARD_ROWS", 6))

    def board_square_size_mm(self) -> float:
        # Original standalone code: squareSize = 16.
        return float(self._calib_value("CHESSBOARD_SQUARE_SIZE_MM", 16))

    def board_object_points(self) -> np.ndarray:
        chessboardSize = self.board_pattern()
        objP = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
        objP[:, :2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)
        objP *= self.board_square_size_mm()
        return objP

    def calculateMean(self, data):
        return sum(data) / len(data)

    def calculateStd(self, data):
        squaredData = [x * x for x in data]
        return (self.calculateMean(squaredData) - self.calculateMean(data) ** 2) ** 0.5

    def undistortRectify(self, frameL, frameR, stereoMapL, stereoMapR):
        undistortedL = cv2.remap(frameL, stereoMapL[0], stereoMapL[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
        undistortedR = cv2.remap(frameR, stereoMapR[0], stereoMapR[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
        return undistortedL, undistortedR

    def resizeImage(self, img, percent):
        width = int(img.shape[1] * percent / 100)
        height = int(img.shape[0] * percent / 100)
        return cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)

    def find_corners(self, image_bgr: np.ndarray) -> Tuple[bool, Optional[np.ndarray], np.ndarray]:
        """Classic chessboard detector, same as the original code.

        This method is used by add_pair() so the UI can show corner previews.
        It does not call cv2.imshow(), because the PyQt UI already displays
        the preview images. This display omission does not change calibration
        data or results.
        """
        chessboardSize = self.board_pattern()
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        gray = cv2.cvtColor(image_bgr.copy(), cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(
            gray,
            chessboardSize,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK | cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if ret:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        preview = image_bgr.copy()
        if ret and corners is not None:
            cv2.drawChessboardCorners(preview, chessboardSize, corners, ret)
        return bool(ret), corners, preview

    def add_pair(self, left_bgr: np.ndarray, right_bgr: np.ndarray) -> Tuple[bool, np.ndarray, str]:
        """Add one stereo chessboard pair using original corner-detection behavior."""
        found_l, corners_l, prev_l = self.find_corners(left_bgr)
        found_r, corners_r, prev_r = self.find_corners(right_bgr)
        self.last_preview_left = prev_l
        self.last_preview_right = prev_r
        self.last_found_left = bool(found_l)
        self.last_found_right = bool(found_r)

        # Build side-by-side preview, same interface as the existing UI manager.
        preview_l = cv2.resize(
            prev_l,
            (min(700, prev_l.shape[1]), int(prev_l.shape[0] * min(700, prev_l.shape[1]) / prev_l.shape[1])),
        )
        preview_r = cv2.resize(
            prev_r,
            (min(700, prev_r.shape[1]), int(prev_r.shape[0] * min(700, prev_r.shape[1]) / prev_r.shape[1])),
        )
        if preview_l.shape[0] != preview_r.shape[0]:
            target_h = min(preview_l.shape[0], preview_r.shape[0])
            preview_l = cv2.resize(preview_l, (int(preview_l.shape[1] * target_h / preview_l.shape[0]), target_h))
            preview_r = cv2.resize(preview_r, (int(preview_r.shape[1] * target_h / preview_r.shape[0]), target_h))
        preview = np.hstack([preview_l, preview_r])

        if not (found_l and found_r and corners_l is not None and corners_r is not None):
            msg = f"Corners not found. left={found_l}, right={found_r}"
            self.log("[WARNING] " + msg)
            return False, preview, msg

        if left_bgr.shape[:2] != right_bgr.shape[:2]:
            msg = f"Left/right image sizes do not match. left={left_bgr.shape[:2]}, right={right_bgr.shape[:2]}"
            self.log("[WARNING] " + msg)
            return False, preview, msg

        current_shape = left_bgr.shape[:2]  # (height, width), same style as gray.shape in original code
        if self.image_shape is None:
            self.image_shape = current_shape
            self.image_size = current_shape[::-1]  # UI/scanner compatibility: (width, height)
        elif current_shape != self.image_shape:
            msg = f"Calibration pair image size changed. expected={self.image_shape}, got={current_shape}"
            self.log("[WARNING] " + msg)
            return False, preview, msg

        self.objpoints.append(self.board_object_points())
        self.imgpoints_l.append(corners_l)
        self.imgpoints_r.append(corners_r)
        msg = f"Accepted calibration pair {len(self.objpoints)}"
        self.log("[OK] " + msg)
        return True, preview, msg

    # ============================================================
    # Folder-pair behavior: keep original sorted zip()
    # ============================================================
    @staticmethod
    def saved_calibration_pairs(capture_dir: str) -> List[Tuple[str, str]]:
        """Return stereo pairs using original behavior: sorted left/right lists + zip()."""
        if not capture_dir or not os.path.isdir(capture_dir):
            return []
        imagesLeft = sorted(glob.glob(os.path.join(capture_dir, 'calib_SL_*.png')))
        imagesRight = sorted(glob.glob(os.path.join(capture_dir, 'calib_SR_*.png')))
        return list(zip(imagesLeft, imagesRight))

    @staticmethod
    def calibration_folder_image_counts(capture_dir: str) -> Dict[str, int]:
        if not capture_dir or not os.path.isdir(capture_dir):
            return {
                "left_images": 0,
                "right_images": 0,
                "total_calibration_images": 0,
                "matched_pairs": 0,
                "unmatched_left": 0,
                "unmatched_right": 0,
            }
        imagesLeft = sorted(glob.glob(os.path.join(capture_dir, 'calib_SL_*.png')))
        imagesRight = sorted(glob.glob(os.path.join(capture_dir, 'calib_SR_*.png')))
        matched = min(len(imagesLeft), len(imagesRight))
        return {
            "left_images": len(imagesLeft),
            "right_images": len(imagesRight),
            "total_calibration_images": len(imagesLeft) + len(imagesRight),
            "matched_pairs": matched,
            "unmatched_left": max(0, len(imagesLeft) - matched),
            "unmatched_right": max(0, len(imagesRight) - matched),
        }

    def resize_percent(self, img: np.ndarray, percent: float) -> np.ndarray:
        return self.resizeImage(img, percent)

    def load_pairs_from_folder(
        self,
        input_image_path: str,
        left_pattern: str = "calib_SL_*.png",
        right_pattern: str = "calib_SR_*.png",
        resolution_percent: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Load pairs from folder using the original sorted zip() behavior."""
        resolution_percent = float(resolution_percent if resolution_percent is not None else getattr(self.cfg, "CALIB_RESOLUTION_PERCENT", 100))
        self.clear_points()
        imagesLeft = sorted(glob.glob(os.path.join(input_image_path, left_pattern)))
        imagesRight = sorted(glob.glob(os.path.join(input_image_path, right_pattern)))
        pairs = list(zip(imagesLeft, imagesRight))
        if not imagesLeft or not imagesRight:
            raise FileNotFoundError("No images found. Check your input folder and filename patterns.")
        counts = self.calibration_folder_image_counts(input_image_path)
        self.log(
            "[CALIB] Folder image count: "
            f"total={counts['total_calibration_images']}, left={counts['left_images']}, "
            f"right={counts['right_images']}, matched stereo pairs={counts['matched_pairs']}"
        )
        accepted = 0
        failed = 0
        for pair_index, (left_path, right_path) in enumerate(pairs, start=1):
            left = cv2.imread(left_path, 1)
            right = cv2.imread(right_path, 1)
            if left is None or right is None:
                failed += 1
                self.log(f"[CALIB WARNING] Cannot read pair: {left_path}, {right_path}")
                continue
            left = self.resizeImage(left, resolution_percent)
            right = self.resizeImage(right, resolution_percent)
            ok, _preview, msg = self.add_pair(left, right)
            accepted += int(ok)
            failed += int(not ok)
            self.log(f"[CALIB PAIR {pair_index:03d}] {os.path.basename(left_path)} / {os.path.basename(right_path)} | {msg}")
        return {
            "accepted": accepted,
            "failed": failed,
            "total_left": len(imagesLeft),
            "total_right": len(imagesRight),
            "matched_pairs": len(pairs),
            "unmatched_left": max(0, len(imagesLeft) - len(pairs)),
            "unmatched_right": max(0, len(imagesRight) - len(pairs)),
            "resolution_percent": resolution_percent,
        }

    # ============================================================
    # Original calibration math behavior
    # ============================================================
    def calibrateCamera(self, objPoints, imgPoints, imageShape):
        ret, mtx, dist, rVecs, tVecs = cv2.calibrateCamera(objPoints, imgPoints, imageShape[::-1], None, None)
        h, w = imageShape[:2]
        optimalMtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        return ret, mtx, dist, rVecs, tVecs, optimalMtx, roi

    def stereoCalibrate(self, objPoints, imgPointsL, imgPointsR, mtxL, distL, mtxR, distR, imageShape, criteria_stereo):
        flags = (cv2.CALIB_FIX_INTRINSIC | cv2.CALIB_FIX_PRINCIPAL_POINT |
                 cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_ZERO_TANGENT_DIST |
                 cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_SAME_FOCAL_LENGTH)
        retS, MLS, dLS, MRS, dRS, R, T, E, F = cv2.stereoCalibrate(
            objPoints, imgPointsL, imgPointsR, mtxL, distL, mtxR, distR,
            imageShape[::-1], criteria_stereo, flags
        )
        return retS, MLS, dLS, MRS, dRS, R, T, E, F

    def stereoRectify(self, MLS, dLS, MRS, dRS, imageShape, R, T):
        rectifyScale = 1
        RL, RR, PL, PR, Q, roiL, roiR = cv2.stereoRectify(MLS, dLS, MRS, dRS, imageShape[::-1], R, T, rectifyScale, (0, 0))
        stereoMapL = cv2.initUndistortRectifyMap(MLS, dLS, RL, PL, imageShape[::-1], cv2.CV_16SC2)
        stereoMapR = cv2.initUndistortRectifyMap(MRS, dRS, RR, PR, imageShape[::-1], cv2.CV_16SC2)
        return stereoMapL, stereoMapR, Q, RL, RR, PL, PR, roiL, roiR

    def stereo_calibration_errors(self, imgPointsL, imgPointsR, F) -> Tuple[List[Tuple[int, float]], float]:
        """Compute stereo calibration error using the original fundamental-matrix formula."""
        errors = []
        totalError = 0
        totalPoints = 0

        for i in range(len(imgPointsL)):
            ptsL = cv2.convertPointsToHomogeneous(imgPointsL[i])[:, 0, :]
            ptsR = cv2.convertPointsToHomogeneous(imgPointsR[i])[:, 0, :]

            err = []
            for pL, pR in zip(ptsL, ptsR):
                val = abs(np.dot(pR, np.dot(F, pL.T)))
                err.append(val)

            meanErr = np.mean(err)
            errors.append((i, float(meanErr)))
            totalError += np.sum(err)
            totalPoints += len(err)

        avgError = totalError / totalPoints
        self.log(f"Stereo calibration mean epipolar error: {avgError:.4f}")
        return errors, float(avgError)

    # Alias with original function-style name.
    def stereoCalibrationError(self, imgPointsL, imgPointsR, F):
        errors, _avg = self.stereo_calibration_errors(imgPointsL, imgPointsR, F)
        return errors

    def filter_good_pairs_by_epipolar_error(
        self,
        errors: List[Tuple[int, float]],
        threshold: float,
    ) -> Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray], List[int]]:
        goodObj, goodL, goodR, kept_indices = [], [], [], []
        for i, err in errors:
            if err <= threshold:
                goodObj.append(self.objpoints[i])
                goodL.append(self.imgpoints_l[i])
                goodR.append(self.imgpoints_r[i])
                kept_indices.append(i)
            else:
                self.log(f"Removed pair {i} due to high error: {err:.3f}")
        return goodObj, goodL, goodR, kept_indices

    def filterGoodPairs(self, objPoints, imgPointsL, imgPointsR, errors, threshold=1.0):
        goodObj, goodL, goodR = [], [], []
        for i, err in errors:
            if err <= threshold:
                goodObj.append(objPoints[i])
                goodL.append(imgPointsL[i])
                goodR.append(imgPointsR[i])
            else:
                self.log(f"Removed pair {i} due to high error: {err:.3f}")
        return goodObj, goodL, goodR

    def calibrate(self) -> Dict[str, Any]:
        """Run calibration with the same behavior/order as the original standalone script."""
        if len(self.objpoints) < 3:
            raise RuntimeError("At least 3 valid stereo chessboard pairs are required for calibration.")
        if self.image_shape is None:
            raise RuntimeError("Image size is unknown.")

        imageShape = self.image_shape  # original style: grayR.shape = (height, width)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        self.log("[S2 CALIB] Calibrating left camera...")
        ret_l, mtxL, distL, rVecsL, tVecsL, optimal_mtxL, roiL = self.calibrateCamera(
            self.objpoints, self.imgpoints_l, imageShape
        )

        self.log("[S2 CALIB] Calibrating right camera...")
        ret_r, mtxR, distR, rVecsR, tVecsR, optimal_mtxR, roiR = self.calibrateCamera(
            self.objpoints, self.imgpoints_r, imageShape
        )

        self.log("[S2 CALIB] Running initial stereo calibration...")
        rms_initial, MLS, dLS, MRS, dRS, R, T, E, F = self.stereoCalibrate(
            self.objpoints,
            self.imgpoints_l,
            self.imgpoints_r,
            optimal_mtxL,
            distL,
            optimal_mtxR,
            distR,
            imageShape,
            criteria,
        )

        stereoErrors, epipolar_error_initial = self.stereo_calibration_errors(self.imgpoints_l, self.imgpoints_r, F)
        threshold = float(getattr(self.cfg, "CALIB_EPIPOLAR_ERROR_THRESHOLD", 1.0))
        objPoints_good, imgPointsL_good, imgPointsR_good = self.filterGoodPairs(
            self.objpoints, self.imgpoints_l, self.imgpoints_r, stereoErrors, threshold=threshold
        )
        kept_indices = [i for i, err in stereoErrors if err <= threshold]

        if len(objPoints_good) < 3:
            raise RuntimeError(
                "Fewer than 3 calibration pairs remained after epipolar-error filtering. "
                f"Accepted={len(self.objpoints)}, kept={len(objPoints_good)}, threshold={threshold}."
            )

        self.log("[S2 CALIB] Running final stereo calibration with filtered pairs...")
        rms, MLS, dLS, MRS, dRS, R, T, E, F = self.stereoCalibrate(
            objPoints_good,
            imgPointsL_good,
            imgPointsR_good,
            optimal_mtxL,
            distL,
            optimal_mtxR,
            distR,
            imageShape,
            criteria,
        )
        _stereoErrors_final, epipolar_error_final = self.stereo_calibration_errors(imgPointsL_good, imgPointsR_good, F)

        stereoMapL, stereoMapR, Q, R1, R2, P1, P2, roi1, roi2 = self.stereoRectify(MLS, dLS, MRS, dRS, imageShape, R, T)

        self.calibration = {
            "left_rms": float(ret_l),
            "right_rms": float(ret_r),
            "stereo_rms_initial": float(rms_initial),
            "stereo_rms": float(rms),
            "epipolar_error_initial": float(epipolar_error_initial),
            "epipolar_error": float(epipolar_error_final),
            "epipolar_error_threshold": float(threshold),
            "num_pairs_removed": int(len(self.objpoints) - len(objPoints_good)),
            "kept_pair_indices": np.asarray(kept_indices, dtype=np.int32),
            "K1": MLS,
            "D1": dLS,
            "K2": MRS,
            "D2": dRS,
            "K1_single": mtxL,
            "D1_single": distL,
            "K2_single": mtxR,
            "D2_single": distR,
            "optimal_K1": optimal_mtxL,
            "optimal_K2": optimal_mtxR,
            "roi_left": np.asarray(roiL, dtype=np.int32),
            "roi_right": np.asarray(roiR, dtype=np.int32),
            "R": R,
            "T": T,
            "E": E,
            "F": F,
            "R1": R1,
            "R2": R2,
            "P1": P1,
            "P2": P2,
            "Q": Q,
            "stereoMapL_x": stereoMapL[0],
            "stereoMapL_y": stereoMapL[1],
            "stereoMapR_x": stereoMapR[0],
            "stereoMapR_y": stereoMapR[1],
            "camera_R_mtx_original_save": mtxR,
            "camera_L_mtx_original_save": mtxL,
            "image_width": int(imageShape[::-1][0]),
            "image_height": int(imageShape[::-1][1]),
            "board_columns": int(self.board_pattern()[0]),
            "board_rows": int(self.board_pattern()[1]),
            "square_size_mm": float(self.board_square_size_mm()),
            "num_pairs": int(len(self.objpoints)),
            "num_pairs_used": int(len(objPoints_good)),
        }
        self.stereoMapL = stereoMapL
        self.stereoMapR = stereoMapR
        self.Q = Q
        self.image_size = imageShape[::-1]

        baseline_mm = float(np.linalg.norm(T))
        self.log(
            f"[OK] S2 calibration finished. Left RMS={ret_l:.6f}, Right RMS={ret_r:.6f}, "
            f"Stereo RMS={rms:.6f}, baseline={baseline_mm:.3f} mm"
        )
        if float(rms) > 2.0:
            self.log("[WARNING] Stereo RMS is high. Use more varied board poses and remove bad pairs.")
            if bool(getattr(self.cfg, "CALIB_REJECT_HIGH_RMS", False)):
                raise RuntimeError(
                    f"Stereo calibration RMS is too high ({float(rms):.3f} px). "
                    "Calibration was not saved. Remove bad image pairs or capture more varied board poses."
                )
        return self.calibration

    def calibrate_from_saved_folder(
        self,
        input_image_path: str,
        save_file: Optional[str] = None,
        left_pattern: str = "calib_SL_*.png",
        right_pattern: str = "calib_SR_*.png",
    ) -> Dict[str, Any]:
        self.load_pairs_from_folder(input_image_path, left_pattern, right_pattern, resolution_percent=getattr(self.cfg, "CALIB_RESOLUTION_PERCENT", 100))
        calib = self.calibrate()
        self.save(save_file or getattr(self.cfg, "CALIB_FILE", None))
        imagesLeft = sorted(glob.glob(os.path.join(input_image_path, left_pattern)))
        imagesRight = sorted(glob.glob(os.path.join(input_image_path, right_pattern)))
        if imagesLeft and imagesRight:
            testImgL = self.resizeImage(cv2.imread(imagesLeft[-1], 1), int(getattr(self.cfg, "CALIB_RESOLUTION_PERCENT", 100)))
            testImgR = self.resizeImage(cv2.imread(imagesRight[-1], 1), int(getattr(self.cfg, "CALIB_RESOLUTION_PERCENT", 100)))
            self.validate_rectification(testImgL, testImgR, save_dir=os.path.dirname(save_file or getattr(self.cfg, "CALIB_FILE", "")))
        return calib

    def validate_rectification(
        self,
        test_left_bgr: np.ndarray,
        test_right_bgr: np.ndarray,
        save_dir: Optional[str] = None,
    ) -> Dict[str, Any]:
        if self.stereoMapL is None or self.stereoMapR is None:
            self.load()
        IL, IR = self.undistortRectify(test_left_bgr, test_right_bgr, self.stereoMapL, self.stereoMapR)
        grayL = cv2.cvtColor(IL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(IR, cv2.COLOR_BGR2GRAY)

        save_dir = save_dir or os.path.dirname(self.calibration_file_path())
        if save_dir:
            ensure_dir(save_dir)
            cv2.imwrite(os.path.join(save_dir, "rectified_left_debug.png"), IL)
            cv2.imwrite(os.path.join(save_dir, "rectified_right_debug.png"), IR)
        else:
            cv2.imwrite("rectified_left_debug.png", IL)
            cv2.imwrite("rectified_right_debug.png", IR)

        chessboardSize = self.board_pattern()
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        result: Dict[str, Any] = {
            "rect_left": IL,
            "rect_right": IR,
            "valid": False,
            "vertical_disparity_mean": None,
            "vertical_disparity_std": None,
        }

        try:
            # Same as original validation code.
            retL, cornersL = cv2.findChessboardCornersSB(grayL, chessboardSize)
            retR, cornersR = cv2.findChessboardCornersSB(grayR, chessboardSize)
            if retL and retR:
                cornersL = cv2.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1), criteria)
                cornersR = cv2.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1), criteria)
                error = [cornersL[pi][0][1] - cornersR[pi][0][1] for pi in range(len(cornersL))]
                meanErr = self.calculateMean(error)
                stdErr = self.calculateStd(error)
                self.log(f"Rectification vertical disparity mean: {meanErr}")
                self.log(f"Rectification vertical disparity std: {stdErr}")
                result.update({
                    "valid": True,
                    "vertical_disparity_mean": float(meanErr),
                    "vertical_disparity_std": float(stdErr),
                })

                for line in range(0, int(IR.shape[0] / 40)):
                    IL[line * 40, :] = (0, 255, 0)
                    IR[line * 40, :] = (0, 255, 0)

                calibratedImg = np.hstack([IL, IR])
                result["calibrated_image"] = calibratedImg
                if save_dir:
                    cv2.imwrite(os.path.join(save_dir, f"calibratedImg_{int(getattr(self.cfg, 'CALIB_RESOLUTION_PERCENT', 100))}.png"), calibratedImg)
                return result
            else:
                self.log("Cannot detect corners on the rectified images.")
                return result
        except Exception as e:
            self.log(f"[S2 VALIDATION WARNING] Rectification validation failed: {e}")
            return result

    def save(self, file_path: Optional[str] = None) -> None:
        file_path = str(file_path or self.calibration_file_path())
        if not self.calibration:
            raise RuntimeError("No calibration data to save. Run calibration first.")
        ensure_dir(os.path.dirname(file_path))
        fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_WRITE)
        if not fs.isOpened():
            raise IOError(f"Cannot open calibration file for writing: {file_path}")

        # Same keys as original standalone saveParameters().
        fs.write('stereoMapL_x', self.calibration["stereoMapL_x"])
        fs.write('stereoMapL_y', self.calibration["stereoMapL_y"])
        fs.write('stereoMapR_x', self.calibration["stereoMapR_x"])
        fs.write('stereoMapR_y', self.calibration["stereoMapR_y"])
        fs.write('q', self.calibration["Q"])
        fs.write('camera_R_mtx', self.calibration.get("camera_R_mtx_original_save", self.calibration["K2_single"]))
        fs.write('camera_L_mtx', self.calibration.get("camera_L_mtx_original_save", self.calibration["K1_single"]))

        # Extra UI-compatible keys; these do not change calibration behavior.
        for key in ["K1", "D1", "K2", "D2", "R", "T", "E", "F", "R1", "R2", "P1", "P2", "Q"]:
            if key in self.calibration:
                fs.write(key, self.calibration[key])
        fs.release()

        # Keep summary files for UI debugging; no effect on calibration math.
        json_path = os.path.splitext(file_path)[0] + "_summary.json"
        summary = {k: v for k, v in self.calibration.items() if not isinstance(v, np.ndarray)}
        try:
            with open(json_path, "w", encoding="utf-8") as f:
                json.dump(summary, f, indent=2)
            self.log(f"[OK] Summary saved: {json_path}")
        except Exception as e:
            self.log(f"[WARNING] Could not save summary JSON: {e}")
        self.log(f"[OK] Calibration saved: {file_path}")

    def save_txt_summary(self, txt_path: str) -> None:
        if not self.calibration:
            return
        lines = []
        for key in [
            "left_rms", "right_rms", "stereo_rms_initial", "stereo_rms",
            "epipolar_error_initial", "epipolar_error", "epipolar_error_threshold",
            "num_pairs", "num_pairs_used", "num_pairs_removed",
            "image_width", "image_height", "board_columns", "board_rows", "square_size_mm",
        ]:
            if key in self.calibration:
                lines.append(f"{key}: {self.calibration[key]}")
        if "T" in self.calibration:
            T = self.calibration["T"]
            lines.append(f"baseline_mm: {float(np.linalg.norm(T))}")
            lines.append("T:")
            lines.append(str(T))
        if "R" in self.calibration:
            lines.append("R:")
            lines.append(str(self.calibration["R"]))
        ensure_dir(os.path.dirname(txt_path))
        with open(txt_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        self.log(f"[OK] TXT summary saved: {txt_path}")

    def load(self, file_path: Optional[str] = None) -> None:
        file_path = str(file_path or self.calibration_file_path())
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"Calibration file not found: {file_path}")
        fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise IOError(f"Cannot open calibration file: {file_path}")

        data: Dict[str, Any] = {}
        for key in ["stereoMapL_x", "stereoMapL_y", "stereoMapR_x", "stereoMapR_y"]:
            mat = fs.getNode(key).mat()
            if mat is None:
                fs.release()
                raise ValueError(f"Calibration file does not contain {key}.")
            data[key] = mat
        Q = fs.getNode("Q").mat()
        if Q is None:
            Q = fs.getNode("q").mat()
        if Q is None:
            fs.release()
            raise ValueError("Calibration file does not contain Q or q.")
        data["Q"] = Q
        for key in ["K1", "D1", "K2", "D2", "R", "T", "R1", "R2", "P1", "P2", "camera_L_mtx", "camera_R_mtx"]:
            mat = fs.getNode(key).mat()
            if mat is not None:
                data[key] = mat
        fs.release()
        self.calibration = data
        self.stereoMapL = (data["stereoMapL_x"], data["stereoMapL_y"])
        self.stereoMapR = (data["stereoMapR_x"], data["stereoMapR_y"])
        self.Q = data["Q"]
        if isinstance(self.cfg, dict):
            self.cfg.setdefault("paths", {})["calibration_file"] = file_path
        else:
            self.cfg.CALIB_FILE = file_path
        self.log(f"[OK] Calibration loaded: {file_path}")

    def rectify(self, left_bgr: np.ndarray, right_bgr: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if self.stereoMapL is None or self.stereoMapR is None:
            self.load()
        rect_l = cv2.remap(left_bgr, self.stereoMapL[0], self.stereoMapL[1], cv2.INTER_LANCZOS4)
        rect_r = cv2.remap(right_bgr, self.stereoMapR[0], self.stereoMapR[1], cv2.INTER_LANCZOS4)
        return rect_l, rect_r

    def rectified_left_camera_matrix(self) -> np.ndarray:
        """Return the 3x3 intrinsic matrix matching the rectified left image."""
        if not self.calibration:
            self.load()

        projection = self.calibration.get("P1")
        if isinstance(projection, np.ndarray) and projection.shape in ((3, 3), (3, 4)):
            return np.asarray(projection[:, :3], dtype=np.float64).copy()

        # Legacy calibration files may not contain P1. Their left intrinsic
        # matrix is still a usable fallback for ChArUco pose estimation.
        for key in ("K1", "camera_L_mtx"):
            matrix = self.calibration.get(key)
            if isinstance(matrix, np.ndarray) and matrix.shape == (3, 3):
                self.log(
                    f"[CALIB WARNING] Saved calibration has no valid P1; "
                    f"using {key} for the rectified left image."
                )
                return np.asarray(matrix, dtype=np.float64).copy()

        raise ValueError(
            "Saved stereo calibration does not contain a usable left camera "
            "matrix. Expected P1, K1, or camera_L_mtx."
        )

    def raw_left_camera_matrix_and_distortion(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return raw left-camera intrinsics and distortion with legacy fallback."""
        if not self.calibration:
            self.load()

        matrix = self.calibration.get("K1")
        if not isinstance(matrix, np.ndarray) or matrix.shape != (3, 3):
            matrix = self.calibration.get("camera_L_mtx")
        if not isinstance(matrix, np.ndarray) or matrix.shape != (3, 3):
            raise ValueError(
                "Saved stereo calibration does not contain a usable raw left "
                "camera matrix. Expected K1 or camera_L_mtx."
            )

        distortion = self.calibration.get("D1")
        if not isinstance(distortion, np.ndarray) or distortion.size < 4:
            self.log(
                "[CALIB WARNING] Saved calibration has no valid D1; "
                "using zero distortion coefficients."
            )
            distortion = np.zeros((1, 5), dtype=np.float64)

        return (
            np.asarray(matrix, dtype=np.float64).copy(),
            np.asarray(distortion, dtype=np.float64).reshape(1, -1).copy(),
        )

    # ============================================================
    # Point-cloud methods kept from the UI manager
    # ============================================================
    def compute_disparity(self, rect_l: np.ndarray, rect_r: np.ndarray) -> np.ndarray:
        gray_l = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(rect_r, cv2.COLOR_BGR2GRAY)
        block_size = int(self.cfg.SGBM_BLOCK_SIZE)
        block_size = block_size if block_size % 2 == 1 else block_size + 1
        num_disp = int(self.cfg.SGBM_NUM_DISPARITIES)
        num_disp = max(16, int(np.ceil(num_disp / 16.0) * 16))
        stereo = cv2.StereoSGBM_create(
            minDisparity=int(self.cfg.SGBM_MIN_DISPARITY),
            numDisparities=num_disp,
            blockSize=block_size,
            P1=8 * 3 * block_size ** 2,
            P2=32 * 3 * block_size ** 2,
            disp12MaxDiff=int(self.cfg.SGBM_DISP12_MAX_DIFF),
            preFilterCap=int(self.cfg.SGBM_PREFILTER_CAP),
            uniquenessRatio=int(self.cfg.SGBM_UNIQUENESS_RATIO),
            speckleWindowSize=int(self.cfg.SGBM_SPECKLE_WINDOW_SIZE),
            speckleRange=int(self.cfg.SGBM_SPECKLE_RANGE),
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        return stereo.compute(gray_l, gray_r).astype(np.float32) / 16.0

    def disparity_to_point_map(self, disparity: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if self.Q is None:
            self.load()
        point_map_mm = cv2.reprojectImageTo3D(disparity, self.Q)
        X = point_map_mm[:, :, 0]
        Y = point_map_mm[:, :, 1]
        Z = point_map_mm[:, :, 2]
        valid = (
            np.isfinite(X) & np.isfinite(Y) & np.isfinite(Z)
            & (disparity > int(self.cfg.SGBM_MIN_DISPARITY))
            & (np.abs(Z) > float(self.cfg.MIN_DEPTH_MM))
            & (np.abs(Z) < float(self.cfg.MAX_DEPTH_MM))
        )
        return point_map_mm, valid

    def point_cloud_from_point_map(self, point_map_mm: np.ndarray, valid: np.ndarray, color_bgr: np.ndarray):
        if o3d is None:
            raise RuntimeError("open3d is not installed.")
        mask = valid.copy()
        stride = max(1, int(self.cfg.POINT_STRIDE))
        if stride > 1:
            stride_mask = np.zeros_like(mask, dtype=bool)
            stride_mask[::stride, ::stride] = True
            mask &= stride_mask
        points = point_map_mm[mask].reshape(-1, 3)
        colors = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)[mask].reshape(-1, 3)
        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        colors = colors[finite].astype(np.float64) / 255.0
        if bool(self.cfg.SCALE_TO_METER):
            points = points / 1000.0
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        if float(self.cfg.VOXEL_SIZE_M) > 0:
            pcd = pcd.voxel_down_sample(float(self.cfg.VOXEL_SIZE_M))
        if int(self.cfg.OUTLIER_NB_NEIGHBORS) > 0 and len(pcd.points) > int(self.cfg.OUTLIER_NB_NEIGHBORS):
            pcd, _ = pcd.remove_statistical_outlier(
                nb_neighbors=int(self.cfg.OUTLIER_NB_NEIGHBORS),
                std_ratio=float(self.cfg.OUTLIER_STD_RATIO),
            )
        return pcd

    def create_point_cloud_from_pair(self, left_bgr: np.ndarray, right_bgr: np.ndarray):
        rect_l, rect_r = self.rectify(left_bgr, right_bgr)
        disparity = self.compute_disparity(rect_l, rect_r)
        point_map, valid = self.disparity_to_point_map(disparity)
        pcd = self.point_cloud_from_point_map(point_map, valid, rect_l)
        return pcd, disparity, rect_l, rect_r



def _aruco_dictionary(name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco is not available. Install opencv-contrib-python.")
    aruco = cv2.aruco
    if not hasattr(aruco, name):
        raise ValueError(f"Unsupported ArUco dictionary: {name}")
    return aruco.getPredefinedDictionary(getattr(aruco, name))


def _create_charuco_board(squares_x: int, squares_y: int, square_len: float, marker_len: float, dictionary):
    aruco = cv2.aruco
    if hasattr(aruco, "CharucoBoard_create"):
        return aruco.CharucoBoard_create(squares_x, squares_y, square_len, marker_len, dictionary)
    return aruco.CharucoBoard((squares_x, squares_y), square_len, marker_len, dictionary)


def _rotation_matrix_about_axis(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64).reshape(3)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        raise ValueError("Rotation axis has zero length.")
    axis = axis / norm
    R, _ = cv2.Rodrigues(axis * float(angle_rad))
    return R.astype(np.float64)


def _homogeneous_about_axis(center: np.ndarray, axis: np.ndarray, angle_deg: float) -> np.ndarray:
    center = np.asarray(center, dtype=np.float64).reshape(3)
    R = _rotation_matrix_about_axis(axis, math.radians(float(angle_deg)))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = center - R @ center
    return T


def _fit_circle_3d(
    points: np.ndarray,
    outlier_threshold_m: float = 0.003,
) -> Tuple[np.ndarray, np.ndarray, float, float, float, np.ndarray, float]:
    """Robustly fit a 3D circle and return center, axis, radius, errors and inliers."""
    points = np.asarray(points, dtype=np.float64)
    if points.shape[0] < 4:
        raise ValueError("At least 4 ChArUco board poses are required to fit the turntable circle.")

    inliers = np.ones(len(points), dtype=bool)
    for _ in range(3):
        fit_points = points[inliers]
        mean = fit_points.mean(axis=0)
        centered = fit_points - mean
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        u, v, normal = vh

        all_centered = points - mean
        x = all_centered @ u
        y = all_centered @ v
        plane_residuals = all_centered @ normal

        fit_x = x[inliers]
        fit_y = y[inliers]
        A = np.column_stack([2 * fit_x, 2 * fit_y, np.ones_like(fit_x)])
        b = fit_x * fit_x + fit_y * fit_y
        cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
        radius = math.sqrt(max(c + cx * cx + cy * cy, 0.0))

        if least_squares is not None:
            initial = np.array([cx, cy, radius], dtype=np.float64)

            def radial_error(params):
                return np.sqrt(
                    (fit_x - params[0]) ** 2 + (fit_y - params[1]) ** 2
                ) - params[2]

            optimized = least_squares(
                radial_error,
                initial,
                loss="soft_l1",
                f_scale=max(0.0005, float(outlier_threshold_m) / 2.0),
            )
            cx, cy, radius = optimized.x

        radial_residuals = np.sqrt((x - cx) ** 2 + (y - cy) ** 2) - radius
        combined = np.sqrt(radial_residuals ** 2 + plane_residuals ** 2)
        median = float(np.median(combined))
        mad = float(np.median(np.abs(combined - median)))
        adaptive_threshold = median + max(float(outlier_threshold_m), 4.5 * 1.4826 * mad)
        new_inliers = combined <= adaptive_threshold
        if int(new_inliers.sum()) < 4 or np.array_equal(new_inliers, inliers):
            break
        inliers = new_inliers

    center_3d = mean + cx * u + cy * v
    radial_rms = float(np.sqrt(np.mean(radial_residuals[inliers] ** 2)))
    plane_rms = float(np.sqrt(np.mean(plane_residuals[inliers] ** 2)))
    if normal[1] < 0:
        normal = -normal
    normal = normal / max(np.linalg.norm(normal), 1e-12)

    angles = np.mod(np.arctan2(y[inliers] - cy, x[inliers] - cx), 2.0 * np.pi)
    angles = np.sort(angles)
    if len(angles) >= 2:
        gaps = np.diff(np.r_[angles, angles[0] + 2.0 * np.pi])
        angular_coverage_deg = math.degrees(2.0 * np.pi - float(np.max(gaps)))
    else:
        angular_coverage_deg = 0.0
    return (
        center_3d,
        normal,
        float(radius),
        radial_rms,
        plane_rms,
        inliers,
        angular_coverage_deg,
    )


class TurntableCalibrationManager:
    """First-time ChArUco-based turntable center/axis calibration."""

    def __init__(self, cfg: dict, stereo_calib: StereoCalibrationManager, log_fn: Callable[[str], None] = print):
        self.cfg = cfg
        self.stereo_calib = stereo_calib
        self.log = log_fn
        self.calib_data: Optional[Dict[str, Any]] = None

    @property
    def calib_file(self) -> str:
        return str(get_nested(self.cfg, "paths.turntable_calibration_file"))

    def load_turntable_calibration(self, path: Optional[str] = None) -> Dict[str, Any]:
        path = path or self.calib_file
        if not os.path.exists(path):
            raise FileNotFoundError(f"Turntable calibration is missing: {path}")
        data = read_json(path)
        for key in ["turntable_center_m", "turntable_axis_unit"]:
            if key not in data:
                raise ValueError(f"Invalid turntable calibration JSON. Missing key: {key}")
        configured_square = float(get_nested(
            self.cfg,
            "charuco_turntable_calibration.charuco_square_length_m",
            0.0,
        ))
        saved_square = float(data.get("square_length_m", 0.0))
        if configured_square > 0 and saved_square > 0 and not math.isclose(
            configured_square,
            saved_square,
            rel_tol=0.001,
            abs_tol=1e-6,
        ):
            raise ValueError(
                f"Turntable calibration scale is stale: JSON square="
                f"{saved_square * 1000.0:.3f} mm, current configured square="
                f"{configured_square * 1000.0:.3f} mm. Rerun Turntable Calibration "
                f"before scanning."
            )
        expected_distance = float(get_nested(
            self.cfg,
            "charuco_turntable_calibration.expected_camera_to_turntable_distance_m",
            0.0,
        ))
        distance_tolerance = float(get_nested(
            self.cfg,
            "charuco_turntable_calibration.distance_sanity_tolerance_m",
            0.10,
        ))
        saved_distance = float(data.get("camera_to_turntable_distance_m", 0.0))
        if (
            expected_distance > 0
            and saved_distance > 0
            and abs(saved_distance - expected_distance) > distance_tolerance
        ):
            raise ValueError(
                f"Turntable calibration distance is inconsistent: JSON="
                f"{saved_distance * 1000.0:.1f} mm, expected approximately "
                f"{expected_distance * 1000.0:.1f} mm. Rerun calibration with the "
                f"measured ChArUco square size."
            )
        self.calib_data = data
        return data

    def save_turntable_calibration(self, path: str, data: Dict[str, Any]) -> None:
        write_json(path, data)
        self.calib_data = data
        self.log(f"[SAVE] Turntable calibration saved: {path}")

    @staticmethod
    def _shortest_signed_angle(angle_deg: float) -> float:
        return (float(angle_deg) + 180.0) % 360.0 - 180.0

    def detect_turntable_base_marker(self, image_bgr: np.ndarray) -> Optional[Dict[str, Any]]:
        """Detect the most prominent configured ring marker in the lower image."""
        aruco = cv2.aruco
        dictionary = _aruco_dictionary(str(get_nested(
            self.cfg,
            "aruco_fallback.aruco_dictionary",
            "DICT_4X4_50",
        )))
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        if hasattr(aruco, "ArucoDetector"):
            corners, ids, _ = aruco.ArucoDetector(
                dictionary,
                aruco.DetectorParameters(),
            ).detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(gray, dictionary)
        if ids is None:
            return None

        first_id = int(get_nested(self.cfg, "aruco_fallback.aruco_first_id", 1))
        marker_count = max(1, int(get_nested(
            self.cfg,
            "aruco_fallback.aruco_marker_count",
            10,
        )))
        valid_ids = set(range(first_id, first_id + marker_count))
        minimum_y_fraction = float(get_nested(
            self.cfg,
            "aruco_fallback.aruco_base_min_center_y_fraction",
            0.55,
        ))
        minimum_area_px = float(get_nested(
            self.cfg,
            "aruco_fallback.aruco_base_min_area_px",
            500.0,
        ))
        image_height = image_bgr.shape[0]
        candidates = []
        for marker_corners, marker_id_array in zip(corners, ids):
            marker_id = int(np.asarray(marker_id_array).reshape(-1)[0])
            if marker_id not in valid_ids:
                continue
            polygon = np.asarray(marker_corners, dtype=np.float32).reshape(4, 2)
            center_xy = polygon.mean(axis=0)
            area_px = abs(float(cv2.contourArea(polygon)))
            if center_xy[1] < minimum_y_fraction * image_height or area_px < minimum_area_px:
                continue
            candidates.append({
                "id": marker_id,
                "center_xy": center_xy.tolist(),
                "area_px": area_px,
                "corners": polygon,
            })
        if not candidates:
            return None

        # The marker facing the cameras is normally the largest visible marker.
        selected = max(candidates, key=lambda item: item["area_px"])
        selected["candidate_ids"] = [int(item["id"]) for item in candidates]
        return selected

    def base_marker_angle_deg(self, marker_id: int) -> float:
        first_id = int(get_nested(self.cfg, "aruco_fallback.aruco_first_id", 1))
        marker_count = max(1, int(get_nested(
            self.cfg,
            "aruco_fallback.aruco_marker_count",
            10,
        )))
        positive_direction = 1 if int(get_nested(
            self.cfg,
            "aruco_fallback.aruco_positive_id_direction",
            1,
        )) >= 0 else -1
        marker_step = 360.0 / marker_count
        index = (int(marker_id) - first_id) % marker_count
        return (positive_direction * index * marker_step) % 360.0

    def move_turntable_to_aruco_home(
        self,
        camera: StereoCameraManager,
        turntable: TurntableController,
        stop_requested: Optional[Callable[[], bool]] = None,
    ) -> Dict[str, Any]:
        """Find ring IDs 1..10, move to ID 1/0 degrees, and verify home."""
        if not bool(get_nested(self.cfg, "aruco_fallback.use_aruco_home_return", True)):
            self.log("[ARUCO HOME] Disabled; calibration starts at current position.")
            return {"enabled": False, "verified": False}

        home_id = int(get_nested(self.cfg, "aruco_fallback.aruco_home_id", 1))
        max_tries = max(1, int(get_nested(
            self.cfg,
            "aruco_fallback.aruco_home_search_max_tries",
            12,
        )))
        search_step = float(get_nested(
            self.cfg,
            "aruco_fallback.aruco_home_search_step_deg",
            36.0,
        ))
        search_speed = int(get_nested(
            self.cfg,
            "aruco_fallback.aruco_home_search_speed",
            60,
        ))
        correction_speed = int(get_nested(
            self.cfg,
            "aruco_fallback.aruco_home_correction_speed",
            20,
        ))
        settle_s = max(0.0, float(get_nested(
            self.cfg,
            "aruco_fallback.aruco_home_settle_sec",
            0.5,
        )))
        angle_tolerance = max(0.1, float(get_nested(
            self.cfg,
            "aruco_fallback.aruco_home_angle_tolerance_deg",
            3.0,
        )))

        total_rotation = 0.0
        last_marker = None
        for attempt in range(1, max_tries + 1):
            if stop_requested is not None and stop_requested():
                raise InterruptedError("Turntable calibration stopped by operator.")
            time.sleep(settle_s)
            if stop_requested is not None and stop_requested():
                raise InterruptedError("Turntable calibration stopped by operator.")
            pair = camera.grab_synchronized_pair()
            marker = self.detect_turntable_base_marker(pair.left)
            if marker is None:
                self.log(
                    f"[ARUCO HOME] Attempt {attempt}/{max_tries}: no base marker; "
                    f"search rotate {search_step:+.3f} deg."
                )
                turntable.rotate_relative(search_step, speed=search_speed, wait_after=True)
                total_rotation += search_step
                continue

            last_marker = marker
            marker_id = int(marker["id"])
            marker_angle = self.base_marker_angle_deg(marker_id)
            home_angle = self.base_marker_angle_deg(home_id)
            correction = self._shortest_signed_angle(home_angle - marker_angle)
            self.log(
                f"[ARUCO HOME] Detected base ID {marker_id}, mapped angle="
                f"{marker_angle:.3f} deg, correction={correction:+.3f} deg, "
                f"visible IDs={marker['candidate_ids']}."
            )
            if marker_id == home_id and abs(correction) <= angle_tolerance:
                self.log(f"[ARUCO HOME OK] ID {home_id} verified at the home position.")
                return {
                    "enabled": True,
                    "verified": True,
                    "home_id": home_id,
                    "detected_id": marker_id,
                    "total_rotation_deg": float(total_rotation),
                }
            turntable.rotate_relative(
                correction,
                speed=correction_speed,
                wait_after=True,
            )
            total_rotation += correction

        raise RuntimeError(
            f"Cannot verify turntable ArUco home ID {home_id} after {max_tries} "
            f"attempts. Last detected marker: "
            f"{None if last_marker is None else last_marker['id']}. Check that base "
            f"IDs 1–10 are visible, ordered consistently, and not covered."
        )

    def _board(self):
        dictionary = _aruco_dictionary(str(get_nested(self.cfg, "charuco_turntable_calibration.charuco_dictionary", "DICT_4X4_50")))
        board = _create_charuco_board(
            int(get_nested(self.cfg, "charuco_turntable_calibration.charuco_squares_x", 12)),
            int(get_nested(self.cfg, "charuco_turntable_calibration.charuco_squares_y", 7)),
            float(get_nested(self.cfg, "charuco_turntable_calibration.charuco_square_length_m", 0.010)),
            float(get_nested(self.cfg, "charuco_turntable_calibration.charuco_marker_length_m", 0.008)),
            dictionary,
        )
        if hasattr(board, "setLegacyPattern"):
            board.setLegacyPattern(bool(get_nested(
                self.cfg,
                "charuco_turntable_calibration.charuco_legacy_pattern",
                False,
            )))
        return board, dictionary

    def detect_charuco_pose(self, left_bgr: np.ndarray, right_bgr: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray, Dict[str, Any]]:
        if right_bgr is not None:
            rect_l, _ = self.stereo_calib.rectify(left_bgr, right_bgr)
            K = self.stereo_calib.rectified_left_camera_matrix()
            D = np.zeros((1, 5), dtype=np.float64)
        else:
            rect_l = left_bgr
            K, D = self.stereo_calib.raw_left_camera_matrix_and_distortion()
        board, dictionary = self._board()
        aruco = cv2.aruco
        gray = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)

        charuco_corners = None
        charuco_ids = None
        if hasattr(aruco, "CharucoDetector"):
            detector = aruco.CharucoDetector(board)
            charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)
        elif hasattr(aruco, "ArucoDetector"):
            params = aruco.DetectorParameters()
            detector = aruco.ArucoDetector(dictionary, params)
            marker_corners, marker_ids, _rejected = detector.detectMarkers(gray)
        else:
            marker_corners, marker_ids, _rejected = aruco.detectMarkers(gray, dictionary)

        if marker_ids is None or len(marker_ids) < 3:
            raise RuntimeError("ChArUco detection failed: too few ArUco markers detected.")
        if charuco_corners is None or charuco_ids is None:
            retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, gray, board
            )
        else:
            retval = len(charuco_ids)
        minimum_corners = max(4, int(get_nested(
            self.cfg,
            "charuco_turntable_calibration.minimum_charuco_corners",
            20,
        )))
        if charuco_ids is None or charuco_corners is None or int(retval) < minimum_corners:
            raise RuntimeError(
                f"ChArUco detection failed: {int(retval or 0)} corners detected; "
                f"at least {minimum_corners} are required."
            )

        if hasattr(aruco, "estimatePoseCharucoBoard"):
            ok, rvec, tvec = aruco.estimatePoseCharucoBoard(
                charuco_corners, charuco_ids, board, K, D, None, None
            )
            object_points, image_points = board.matchImagePoints(charuco_corners, charuco_ids)
        else:
            object_points, image_points = board.matchImagePoints(charuco_corners, charuco_ids)
            ok, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                K,
                D,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
        if not ok:
            raise RuntimeError("ChArUco pose estimation failed.")

        projected, _ = cv2.projectPoints(object_points, rvec, tvec, K, D)
        image_xy = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
        projected_xy = np.asarray(projected, dtype=np.float64).reshape(-1, 2)
        reprojection_error_px = float(np.sqrt(np.mean(np.sum(
            (projected_xy - image_xy) ** 2,
            axis=1,
        ))))
        maximum_reprojection_error_px = float(get_nested(
            self.cfg,
            "charuco_turntable_calibration.maximum_reprojection_error_px",
            1.5,
        ))
        if reprojection_error_px > maximum_reprojection_error_px:
            raise RuntimeError(
                f"ChArUco pose rejected: reprojection RMS={reprojection_error_px:.3f} px "
                f"exceeds {maximum_reprojection_error_px:.3f} px."
            )

        R, _ = cv2.Rodrigues(rvec)
        sx = int(get_nested(self.cfg, "charuco_turntable_calibration.charuco_squares_x", 12))
        sy = int(get_nested(self.cfg, "charuco_turntable_calibration.charuco_squares_y", 7))
        square_len = float(get_nested(self.cfg, "charuco_turntable_calibration.charuco_square_length_m", 0.010))
        board_center_local = np.array([sx * square_len / 2.0, sy * square_len / 2.0, 0.0], dtype=np.float64)
        board_center_camera = (R @ board_center_local.reshape(3, 1) + np.asarray(tvec, dtype=np.float64).reshape(3, 1)).reshape(3)

        debug = rect_l.copy()
        try:
            aruco.drawDetectedMarkers(debug, marker_corners, marker_ids)
            aruco.drawDetectedCornersCharuco(debug, charuco_corners, charuco_ids)
            cv2.drawFrameAxes(debug, K, D, rvec, tvec, 0.05)
        except Exception:
            pass
        info = {
            "num_detected_markers": int(len(marker_ids)),
            "num_charuco_corners": int(len(charuco_ids)),
            "reprojection_error_px": reprojection_error_px,
            "rvec": np.asarray(rvec, dtype=float).reshape(3).tolist(),
            "tvec": np.asarray(tvec, dtype=float).reshape(3).tolist(),
            "board_center_camera_m": board_center_camera.tolist(),
        }
        return board_center_camera, np.asarray(rvec).reshape(3), np.asarray(tvec).reshape(3), {"debug": debug, "info": info}

    def capture_stable_charuco_pose(
        self,
        camera: StereoCameraManager,
        stop_requested: Optional[Callable[[], bool]] = None,
    ) -> Tuple[Any, np.ndarray, np.ndarray, np.ndarray, Dict[str, Any], Dict[str, Any]]:
        """Capture repeatedly until post-stop board movement has settled."""
        timeout_s = max(0.5, float(get_nested(
            self.cfg,
            "charuco_turntable_calibration.stability_timeout_sec",
            6.0,
        )))
        interval_s = max(0.02, float(get_nested(
            self.cfg,
            "charuco_turntable_calibration.stability_sample_interval_sec",
            0.20,
        )))
        tolerance_m = max(0.00001, float(get_nested(
            self.cfg,
            "charuco_turntable_calibration.stability_translation_tolerance_m",
            0.0005,
        )))
        required_samples = max(2, int(get_nested(
            self.cfg,
            "charuco_turntable_calibration.stability_required_samples",
            3,
        )))

        deadline = time.monotonic() + timeout_s
        previous_center = None
        stable_count = 0
        valid_count = 0
        last_error: Optional[Exception] = None
        while time.monotonic() < deadline:
            if stop_requested is not None and stop_requested():
                raise InterruptedError("Turntable calibration stopped by operator.")
            pair = camera.grab_synchronized_pair()
            try:
                center, rvec, tvec, result = self.detect_charuco_pose(pair.left, pair.right)
                valid_count += 1
                movement_m = None
                if previous_center is not None:
                    movement_m = float(np.linalg.norm(center - previous_center))
                    stable_count = stable_count + 1 if movement_m <= tolerance_m else 1
                else:
                    stable_count = 1
                previous_center = center
                self.log(
                    f"[TURNTABLE SETTLE] valid={valid_count}, stable={stable_count}/"
                    f"{required_samples}, movement="
                    f"{'first sample' if movement_m is None else f'{movement_m * 1000.0:.3f} mm'}"
                )
                if stable_count >= required_samples:
                    stability = {
                        "stable_samples": int(stable_count),
                        "translation_tolerance_m": float(tolerance_m),
                        "last_movement_m": float(movement_m or 0.0),
                    }
                    return pair, center, rvec, tvec, result, stability
            except Exception as exc:
                last_error = exc
                stable_count = 0
            time.sleep(interval_s)

        if previous_center is not None:
            raise RuntimeError(
                f"Board remained visible but did not stop moving within {timeout_s:.1f} s. "
                f"Check clamp rigidity, motor holding torque, and turntable backlash."
            )
        raise RuntimeError(f"ChArUco was not detected during settling: {last_error}")

    def try_single_charuco_pose(
        self,
        camera: StereoCameraManager,
    ) -> Optional[Tuple[Any, np.ndarray, np.ndarray, np.ndarray, Dict[str, Any]]]:
        """Quick one-frame visibility test used during coarse turntable search."""
        pair = camera.grab_synchronized_pair()
        try:
            center, rvec, tvec, result = self.detect_charuco_pose(pair.left, pair.right)
            return pair, center, rvec, tvec, result
        except Exception:
            return None

    def calibrate_turntable_axis(
        self,
        camera: StereoCameraManager,
        turntable: TurntableController,
        save_dir: str | Path,
        stop_requested: Optional[Callable[[], bool]] = None,
    ) -> Dict[str, Any]:
        ensure_dir(save_dir)
        if not self.stereo_calib.calibration:
            self.stereo_calib.load()
        home_result = self.move_turntable_to_aruco_home(
            camera,
            turntable,
            stop_requested=stop_requested,
        )
        n = max(4, int(get_nested(self.cfg, "charuco_turntable_calibration.calibration_viewpoints", 12)))
        total = float(get_nested(self.cfg, "charuco_turntable_calibration.calibration_total_angle_deg", 360.0))
        # View 0 is captured at the current position, so n views contain n-1
        # rotations. This makes the first-to-last target span equal `total`.
        step = total / float(n - 1)
        wait_s = float(get_nested(self.cfg, "charuco_turntable_calibration.calibration_stabilization_sec", 1.0))
        coarse_search_step = abs(float(get_nested(
            self.cfg,
            "charuco_turntable_calibration.coarse_search_step_deg",
            20.0,
        )))
        coarse_search_speed = int(get_nested(
            self.cfg,
            "charuco_turntable_calibration.coarse_search_speed",
            80,
        ))
        coarse_search_max_rotations = max(0, int(get_nested(
            self.cfg,
            "charuco_turntable_calibration.coarse_search_max_rotations",
            18,
        )))
        coarse_search_wait_s = max(0.0, float(get_nested(
            self.cfg,
            "charuco_turntable_calibration.coarse_search_wait_sec",
            0.15,
        )))
        points: List[np.ndarray] = []
        view_poses: List[Dict[str, Any]] = []
        cumulative_angle = 0.0
        self.log(
            f"[TURNTABLE CALIB] Starting ChArUco calibration: {n} views, "
            f"capture step={step:.3f} deg. If the board is lost, search rotates "
            f"forward only: {coarse_search_step:.3f} deg at speed "
            f"{coarse_search_speed}, up to {coarse_search_max_rotations} steps."
        )
        for i in range(n):
            if stop_requested is not None and stop_requested():
                raise InterruptedError("Turntable calibration stopped by operator.")
            if i > 0:
                self.log(
                    f"[TURNTABLE CALIB] Moving to view {i:03d}/{n - 1:03d}: "
                    f"rotate {step:.3f} deg."
                )
                turntable.rotate_relative(step, wait_after=True)
                cumulative_angle += step
            else:
                self.log(
                    "[TURNTABLE CALIB] View 000 uses the current home position; "
                    "the first rotation occurs before view 001."
                )

            detected = None
            last_error: Optional[Exception] = None
            time.sleep(max(0.0, wait_s))
            if stop_requested is not None and stop_requested():
                raise InterruptedError("Turntable calibration stopped by operator.")
            quick_pose = self.try_single_charuco_pose(camera)
            if quick_pose is not None:
                try:
                    pair, center, rvec, tvec, result, stability = self.capture_stable_charuco_pose(
                        camera,
                        stop_requested=stop_requested,
                    )
                    attempt_tag = f"view_{i:03d}_direct"
                    left_path = Path(save_dir) / f"turntable_calib_{attempt_tag}_SL_{pair.timestamp}.png"
                    right_path = Path(save_dir) / f"turntable_calib_{attempt_tag}_SR_{pair.timestamp}.png"
                    cv2.imwrite(str(left_path), pair.left)
                    cv2.imwrite(str(right_path), pair.right)
                    detected = (
                        center,
                        rvec,
                        tvec,
                        result,
                        left_path,
                        right_path,
                        0,
                        stability,
                    )
                except Exception as exc:
                    last_error = exc
                    self.log(
                        f"[TURNTABLE CALIB] Board was visible at view {i:03d}, "
                        f"but the precision capture failed: {exc}"
                    )
            else:
                self.log(
                    f"[TURNTABLE CALIB] Board is not visible at view {i:03d}."
                )

            if detected is None:
                self.log(
                    f"[TURNTABLE CALIB SEARCH] Rotating in one direction only: "
                    f"forward {coarse_search_step:.1f} deg per step at speed "
                    f"{coarse_search_speed} until the board reappears."
                )
                for coarse_index in range(1, coarse_search_max_rotations + 1):
                    if stop_requested is not None and stop_requested():
                        raise InterruptedError("Turntable calibration stopped by operator.")
                    turntable.rotate_relative(
                        coarse_search_step,
                        speed=coarse_search_speed,
                        wait_after=True,
                    )
                    cumulative_angle += coarse_search_step
                    time.sleep(coarse_search_wait_s)
                    quick_pose = self.try_single_charuco_pose(camera)
                    self.log(
                        f"[TURNTABLE CALIB SEARCH] Forward step "
                        f"{coarse_index}/{coarse_search_max_rotations}, angle="
                        f"{cumulative_angle:.3f} deg: "
                        f"{'board detected' if quick_pose is not None else 'not visible'}"
                    )
                    if quick_pose is None:
                        continue
                    try:
                        pair, center, rvec, tvec, result, stability = (
                            self.capture_stable_charuco_pose(
                                camera,
                                stop_requested=stop_requested,
                            )
                        )
                    except Exception as exc:
                        last_error = exc
                        continue
                    attempt_tag = f"view_{i:03d}_coarse_{coarse_index:03d}"
                    left_path = Path(save_dir) / f"turntable_calib_{attempt_tag}_SL_{pair.timestamp}.png"
                    right_path = Path(save_dir) / f"turntable_calib_{attempt_tag}_SR_{pair.timestamp}.png"
                    cv2.imwrite(str(left_path), pair.left)
                    cv2.imwrite(str(right_path), pair.right)
                    detected = (
                        center,
                        rvec,
                        tvec,
                        result,
                        left_path,
                        right_path,
                        coarse_index,
                        stability,
                    )
                    break

            if detected is None:
                self.log(
                    f"[TURNTABLE CALIB WARNING] View {i:03d} rejected after "
                    f"{coarse_search_max_rotations} forward search steps: {last_error}"
                )
                continue

            center, rvec, tvec, result, left_path, right_path, search_index, stability = detected
            debug_path = Path(save_dir) / f"turntable_calib_debug_{i:03d}_try_{search_index:03d}.png"
            cv2.imwrite(str(debug_path), result["debug"])
            points.append(center)
            pose_info = dict(result["info"])
            pose_info.update({
                "view_index": i,
                "commanded_angle_deg": float(cumulative_angle),
                "search_rotations": int(search_index),
                "search_step_deg": float(coarse_search_step),
                "stability": stability,
                "left_image": str(left_path),
                "right_image": str(right_path),
                "debug_image": str(debug_path),
            })
            view_poses.append(pose_info)
            self.log(
                f"[TURNTABLE CALIB] View {i:03d} detected at "
                f"{cumulative_angle:.3f} deg after {search_index} search rotations: "
                f"center={center.tolist()}"
            )
        if len(points) < 4:
            raise RuntimeError(f"Too few valid ChArUco poses for circle fitting: {len(points)} valid / {n} captured.")
        pts = np.vstack(points)
        outlier_threshold_m = float(get_nested(
            self.cfg,
            "charuco_turntable_calibration.circle_outlier_threshold_m",
            0.003,
        ))
        center_3d, axis, radius, rms, plane_rms, inliers, coverage_deg = _fit_circle_3d(
            pts,
            outlier_threshold_m=outlier_threshold_m,
        )
        minimum_inlier_views = max(4, int(get_nested(
            self.cfg,
            "charuco_turntable_calibration.minimum_inlier_views",
            8,
        )))
        maximum_circle_rms_m = float(get_nested(
            self.cfg,
            "charuco_turntable_calibration.maximum_circle_rms_m",
            0.002,
        ))
        maximum_plane_rms_m = float(get_nested(
            self.cfg,
            "charuco_turntable_calibration.maximum_plane_rms_m",
            0.002,
        ))
        minimum_coverage_deg = float(get_nested(
            self.cfg,
            "charuco_turntable_calibration.minimum_angular_coverage_deg",
            120.0,
        ))
        if int(inliers.sum()) < minimum_inlier_views:
            raise RuntimeError(
                f"Turntable calibration rejected: only {int(inliers.sum())} robust inlier "
                f"views; at least {minimum_inlier_views} are required."
            )
        if rms > maximum_circle_rms_m:
            raise RuntimeError(
                f"Turntable calibration rejected: circle RMS={rms * 1000.0:.3f} mm "
                f"exceeds {maximum_circle_rms_m * 1000.0:.3f} mm."
            )
        if plane_rms > maximum_plane_rms_m:
            raise RuntimeError(
                f"Turntable calibration rejected: plane RMS={plane_rms * 1000.0:.3f} mm "
                f"exceeds {maximum_plane_rms_m * 1000.0:.3f} mm."
            )
        if coverage_deg < minimum_coverage_deg:
            raise RuntimeError(
                f"Turntable calibration rejected at angular-coverage check: "
                f"measured={coverage_deg:.1f} deg, required={minimum_coverage_deg:.1f} deg, "
                f"valid views={len(points)}, inliers={int(inliers.sum())}. Place the board "
                f"near one edge of camera visibility before starting, or reduce Min coverage "
                f"only if repeated calibrations produce the same center and axis."
            )
        distance = float(np.linalg.norm(center_3d))
        expected_distance = float(get_nested(
            self.cfg,
            "charuco_turntable_calibration.expected_camera_to_turntable_distance_m",
            0.0,
        ))
        distance_tolerance = float(get_nested(
            self.cfg,
            "charuco_turntable_calibration.distance_sanity_tolerance_m",
            0.10,
        ))
        if expected_distance > 0 and abs(distance - expected_distance) > distance_tolerance:
            configured_square_mm = float(get_nested(
                self.cfg,
                "charuco_turntable_calibration.charuco_square_length_m",
                0.0,
            )) * 1000.0
            suggested_square_mm = configured_square_mm * expected_distance / distance
            raise RuntimeError(
                f"Turntable calibration rejected: fitted camera-to-center distance="
                f"{distance * 1000.0:.1f} mm, expected approximately "
                f"{expected_distance * 1000.0:.1f} mm. Measure one complete ChArUco "
                f"square edge accurately. Current square={configured_square_mm:.3f} mm; "
                f"distance-based estimate={suggested_square_mm:.3f} mm."
            )
        inlier_pose_indices = np.flatnonzero(inliers).astype(int)
        rejected_pose_indices = np.flatnonzero(~inliers).astype(int)
        inlier_indices = [int(view_poses[j]["view_index"]) for j in inlier_pose_indices]
        rejected_indices = [int(view_poses[j]["view_index"]) for j in rejected_pose_indices]

        # Estimate real angular motion from the observed board trajectory. This
        # compensates systematic motor scale error and post-stop forward creep.
        inlier_points = pts[inliers]
        vectors = inlier_points - center_3d
        vectors -= np.outer(vectors @ axis, axis)
        reference = vectors[0] / max(np.linalg.norm(vectors[0]), 1e-12)
        measured_angles_rad = []
        for vector in vectors:
            unit = vector / max(np.linalg.norm(vector), 1e-12)
            measured_angles_rad.append(math.atan2(
                float(np.dot(axis, np.cross(reference, unit))),
                float(np.dot(reference, unit)),
            ))
        measured_angles_deg = np.rad2deg(np.unwrap(np.asarray(measured_angles_rad)))
        commanded_angles_deg = np.asarray(
            [view_poses[j]["commanded_angle_deg"] for j in inlier_pose_indices],
            dtype=np.float64,
        )
        commanded_delta = np.diff(commanded_angles_deg)
        measured_delta = np.diff(measured_angles_deg)
        local_steps = (
            (np.abs(commanded_delta) > 1e-3)
            & (np.abs(commanded_delta) <= 45.0)
            & (np.abs(measured_delta) > 0.25)
        )
        local_ratios = measured_delta[local_steps] / commanded_delta[local_steps]
        plausible_ratios = local_ratios[
            (np.abs(local_ratios) >= 0.5) & (np.abs(local_ratios) <= 1.5)
        ]
        if len(plausible_ratios) >= 3:
            angle_slope = float(np.median(plausible_ratios))
            angle_offset = float(np.median(
                measured_angles_deg - angle_slope * commanded_angles_deg
            ))
            predicted_angles = angle_slope * commanded_angles_deg + angle_offset
            angle_fit_rms_deg = float(np.sqrt(np.mean(
                (predicted_angles - measured_angles_deg) ** 2
            )))
        else:
            configured_sign = 1 if int(get_nested(
                self.cfg,
                "charuco_turntable_calibration.angle_sign",
                1,
            )) >= 0 else -1
            angle_slope = float(configured_sign)
            angle_offset = 0.0
            angle_fit_rms_deg = float("nan")
            self.log(
                "[TURNTABLE CALIB WARNING] Motor angle scale could not be "
                "estimated from at least three local steps. Full-loop search "
                f"moves were ignored; using sign={configured_sign}, scale=1.0."
            )
        measured_angle_sign = 1 if angle_slope >= 0 else -1
        angle_scale = abs(float(angle_slope))
        data = {
            "created_at": timestamp(),
            "aruco_home": home_result,
            "board_type": "charuco",
            "charuco_squares_x": int(get_nested(self.cfg, "charuco_turntable_calibration.charuco_squares_x", 12)),
            "charuco_squares_y": int(get_nested(self.cfg, "charuco_turntable_calibration.charuco_squares_y", 7)),
            "square_length_m": float(get_nested(self.cfg, "charuco_turntable_calibration.charuco_square_length_m", 0.010)),
            "marker_length_m": float(get_nested(self.cfg, "charuco_turntable_calibration.charuco_marker_length_m", 0.008)),
            "num_calibration_views": int(len(points)),
            "num_inlier_views": int(inliers.sum()),
            "inlier_view_indices": inlier_indices,
            "rejected_view_indices": rejected_indices,
            "turntable_center_m": center_3d.tolist(),
            "turntable_axis_unit": axis.tolist(),
            "turntable_radius_m": float(radius),
            "camera_to_turntable_distance_m": distance,
            "expected_camera_to_turntable_distance_m": expected_distance,
            "angle_sign": measured_angle_sign,
            "angle_scale": angle_scale,
            "angle_fit_offset_deg": float(angle_offset),
            "angle_fit_rms_deg": angle_fit_rms_deg,
            "view_poses": view_poses,
            "rms_circle_fit_error_m": float(rms),
            "rms_plane_fit_error_m": float(plane_rms),
            "angular_coverage_deg": float(coverage_deg),
            "quality_limits": {
                "minimum_inlier_views": minimum_inlier_views,
                "maximum_circle_rms_m": maximum_circle_rms_m,
                "maximum_plane_rms_m": maximum_plane_rms_m,
                "minimum_angular_coverage_deg": minimum_coverage_deg,
            },
        }
        self.save_turntable_calibration(self.calib_file, data)
        self.log(
            f"[TURNTABLE CALIB OK] center={center_3d.tolist()}, axis={axis.tolist()}, "
            f"distance={distance:.4f} m, circle RMS={rms * 1000.0:.3f} mm, "
            f"plane RMS={plane_rms * 1000.0:.3f} mm, coverage={coverage_deg:.1f} deg, "
            f"inliers={int(inliers.sum())}/{len(points)}, angle scale={angle_scale:.5f}, "
            f"angle fit RMS={angle_fit_rms_deg:.3f} deg"
        )
        return data

    def rotation_about_calibrated_axis(self, angle_deg: float) -> np.ndarray:
        data = self.calib_data or self.load_turntable_calibration()
        center = np.array(data["turntable_center_m"], dtype=np.float64)
        axis = np.array(data["turntable_axis_unit"], dtype=np.float64)
        sign = int(data.get("angle_sign", 1))
        scale = float(data.get("angle_scale", 1.0))
        return _homogeneous_about_axis(center, axis, sign * scale * float(angle_deg))

    def transform_view_to_home(self, angle_deg: float) -> np.ndarray:
        return self.rotation_about_calibrated_axis(-float(angle_deg))
