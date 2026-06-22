from __future__ import annotations

from pathlib import Path
from typing import Callable, Tuple

import cv2
import numpy as np

try:
    import open3d as o3d
except Exception:  # pragma: no cover
    o3d = None

from .calibration import StereoCalibrationManager
from .project_io import ensure_dir, get_nested


class StereoReconstructor:
    def __init__(self, cfg: dict, stereo_calib: StereoCalibrationManager, log_fn: Callable[[str], None] = print):
        self.cfg = cfg
        self.calib = stereo_calib
        self.log = log_fn

    def compute_disparity(self, rect_l: np.ndarray, rect_r: np.ndarray) -> np.ndarray:
        gray_l = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(rect_r, cv2.COLOR_BGR2GRAY)
        block_size = int(get_nested(self.cfg, "reconstruction.sgbm_block_size", 7))
        if block_size % 2 == 0:
            block_size += 1
        num_disp = int(get_nested(self.cfg, "reconstruction.sgbm_num_disparities", 256))
        num_disp = max(16, int(np.ceil(num_disp / 16.0) * 16))
        stereo = cv2.StereoSGBM_create(
            minDisparity=int(get_nested(self.cfg, "reconstruction.sgbm_min_disparity", 128)),
            numDisparities=num_disp,
            blockSize=block_size,
            P1=8 * 3 * block_size * block_size,
            P2=32 * 3 * block_size * block_size,
            disp12MaxDiff=int(get_nested(self.cfg, "reconstruction.sgbm_disp12_max_diff", 1)),
            preFilterCap=int(get_nested(self.cfg, "reconstruction.sgbm_prefilter_cap", 31)),
            uniquenessRatio=int(get_nested(self.cfg, "reconstruction.sgbm_uniqueness_ratio", 8)),
            speckleWindowSize=int(get_nested(self.cfg, "reconstruction.sgbm_speckle_window_size", 120)),
            speckleRange=int(get_nested(self.cfg, "reconstruction.sgbm_speckle_range", 2)),
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        return stereo.compute(gray_l, gray_r).astype(np.float32) / 16.0

    def disparity_to_point_map(self, disparity: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if self.calib.Q is None:
            self.calib.load()
        point_map_mm = cv2.reprojectImageTo3D(disparity, self.calib.Q)
        X, Y, Z = point_map_mm[:, :, 0], point_map_mm[:, :, 1], point_map_mm[:, :, 2]
        min_disp = float(get_nested(self.cfg, "reconstruction.sgbm_min_disparity", 128))
        min_depth = float(get_nested(self.cfg, "reconstruction.min_depth_mm", 400.0))
        max_depth = float(get_nested(self.cfg, "reconstruction.max_depth_mm", 650.0))
        valid = (
            np.isfinite(X) & np.isfinite(Y) & np.isfinite(Z)
            & (disparity > min_disp)
            & (np.abs(Z) > min_depth)
            & (np.abs(Z) < max_depth)
        )
        return point_map_mm, valid

    def point_cloud_from_point_map(
        self,
        point_map_mm: np.ndarray,
        valid: np.ndarray,
        color_bgr: np.ndarray,
        object_mask: np.ndarray | None = None,
    ):
        if o3d is None:
            raise RuntimeError("open3d is not installed.")
        mask = valid.copy()
        if object_mask is not None:
            if object_mask.shape != mask.shape:
                raise ValueError(
                    f"Object mask shape {object_mask.shape} does not match "
                    f"rectified image shape {mask.shape}."
                )
            mask &= object_mask.astype(bool)
        stride = max(1, int(get_nested(self.cfg, "reconstruction.point_stride", 2)))
        if stride > 1:
            stride_mask = np.zeros_like(mask, dtype=bool)
            stride_mask[::stride, ::stride] = True
            mask &= stride_mask
        points = point_map_mm[mask].reshape(-1, 3)
        colors = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)[mask].reshape(-1, 3)
        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        colors = colors[finite].astype(np.float64) / 255.0
        if bool(get_nested(self.cfg, "reconstruction.scale_to_meter", True)):
            points = points / 1000.0
        if points.shape[0] == 0:
            raise RuntimeError("Empty point cloud after depth/disparity filtering.")
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        voxel = float(get_nested(self.cfg, "reconstruction.voxel_size_m", 0.002))
        if voxel > 0:
            pcd = pcd.voxel_down_sample(voxel)
        nb = int(get_nested(self.cfg, "reconstruction.outlier_nb_neighbors", 20))
        if nb > 0 and len(pcd.points) > nb:
            pcd, _ = pcd.remove_statistical_outlier(
                nb_neighbors=nb,
                std_ratio=float(get_nested(self.cfg, "reconstruction.outlier_std_ratio", 2.0)),
            )
        if len(pcd.points) == 0:
            raise RuntimeError("Empty point cloud after outlier removal.")
        return pcd

    def create_point_cloud_from_pair(
        self,
        left_bgr: np.ndarray,
        right_bgr: np.ndarray,
        segmenter=None,
    ):
        rect_l, rect_r = self.calib.rectify(left_bgr, right_bgr)
        disparity = self.compute_disparity(rect_l, rect_r)
        point_map, valid = self.disparity_to_point_map(disparity)
        object_mask = segmenter.predict_mask(rect_l) if segmenter is not None else None
        pcd = self.point_cloud_from_point_map(
            point_map,
            valid,
            rect_l,
            object_mask=object_mask,
        )
        return pcd, disparity, rect_l, rect_r

    def save_debug_outputs(self, view_idx: int, disparity: np.ndarray, rect_l: np.ndarray, rect_r: np.ndarray, scan_dir: Path) -> None:
        if bool(get_nested(self.cfg, "reconstruction.save_rectified_images", True)):
            ensure_dir(scan_dir / "rectified")
            cv2.imwrite(str(scan_dir / "rectified" / f"rectified_view_{view_idx:03d}_SL.png"), rect_l)
            cv2.imwrite(str(scan_dir / "rectified" / f"rectified_view_{view_idx:03d}_SR.png"), rect_r)
        ensure_dir(scan_dir / "disparity")
        disp_vis = disparity.copy()
        disp_vis[~np.isfinite(disp_vis)] = 0
        disp_vis[disp_vis < 0] = 0
        disp_norm = cv2.normalize(disp_vis, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        disp_color = cv2.applyColorMap(disp_norm, cv2.COLORMAP_JET)
        cv2.imwrite(str(scan_dir / "disparity" / f"disparity_view_{view_idx:03d}.png"), disp_color)
