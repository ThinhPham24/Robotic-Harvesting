from __future__ import annotations

from typing import Callable

import numpy as np

try:
    import open3d as o3d
except Exception:  # pragma: no cover
    o3d = None

from .project_io import get_nested


class PointCloudPostProcessor:
    def __init__(self, cfg: dict, log_fn: Callable[[str], None] = print):
        self.cfg = cfg
        self.log = log_fn

    def crop_to_turntable_volume(self, pcd, calibration: dict):
        """Keep points inside the physical turntable/object cylinder."""
        if o3d is None:
            raise RuntimeError("open3d is not installed.")
        if pcd is None or len(pcd.points) == 0:
            raise RuntimeError("Cannot crop an empty point cloud.")
        if not bool(get_nested(self.cfg, "postprocessing.use_turntable_volume_crop", True)):
            return pcd

        center = np.asarray(calibration["turntable_center_m"], dtype=np.float64).reshape(3)
        axis = np.asarray(calibration["turntable_axis_unit"], dtype=np.float64).reshape(3)
        axis /= max(float(np.linalg.norm(axis)), 1e-12)
        radius_m = float(get_nested(self.cfg, "postprocessing.turntable_radius_m", 0.100))
        radial_margin_m = float(get_nested(
            self.cfg,
            "postprocessing.turntable_radius_margin_m",
            0.005,
        ))
        maximum_height_m = float(get_nested(
            self.cfg,
            "postprocessing.maximum_object_height_m",
            0.350,
        ))
        below_table_margin_m = float(get_nested(
            self.cfg,
            "postprocessing.below_turntable_margin_m",
            0.010,
        ))

        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        relative = points - center
        axial = relative @ axis
        radial_vectors = relative - np.outer(axial, axis)
        radial_distance = np.linalg.norm(radial_vectors, axis=1)
        mask = (
            np.isfinite(points).all(axis=1)
            & (radial_distance <= radius_m + radial_margin_m)
        )
        use_height_crop = bool(get_nested(
            self.cfg,
            "postprocessing.use_turntable_height_crop",
            False,
        ))
        if use_height_crop:
            mask &= (
                (axial >= -maximum_height_m)
                & (axial <= below_table_margin_m)
            )
        kept = int(mask.sum())
        original = int(len(points))
        if kept == 0:
            raise RuntimeError(
                "Turntable-volume crop removed every point. Verify the turntable "
                "calibration scale, center, axis, radius, and object-height settings."
            )

        cropped = o3d.geometry.PointCloud()
        cropped.points = o3d.utility.Vector3dVector(points[mask])
        if colors.shape == points.shape:
            cropped.colors = o3d.utility.Vector3dVector(colors[mask])
        normals = np.asarray(pcd.normals)
        if normals.shape == points.shape:
            cropped.normals = o3d.utility.Vector3dVector(normals[mask])
        self.log(
            f"[TURNTABLE CROP] Kept {kept:,}/{original:,} points inside "
            f"radius={(radius_m + radial_margin_m) * 1000.0:.1f} mm"
            + (
                f", height={maximum_height_m * 1000.0:.1f} mm."
                if use_height_crop else "."
            )
        )
        return cropped

    def clean(self, pcd):
        if o3d is None:
            raise RuntimeError("open3d is not installed.")
        if pcd is None or len(pcd.points) == 0:
            raise RuntimeError("Cannot post-process an empty point cloud.")
        voxel = float(get_nested(self.cfg, "stitching.fused_voxel_size_m", 0.003))
        if voxel > 0:
            pcd = pcd.voxel_down_sample(voxel)
        nb = int(get_nested(self.cfg, "reconstruction.outlier_nb_neighbors", 20))
        if nb > 0 and len(pcd.points) > nb:
            pcd, _ = pcd.remove_statistical_outlier(
                nb_neighbors=nb,
                std_ratio=float(get_nested(self.cfg, "reconstruction.outlier_std_ratio", 2.0)),
            )
        if len(pcd.points) == 0:
            raise RuntimeError("Point cloud became empty after post-processing.")
        return pcd
