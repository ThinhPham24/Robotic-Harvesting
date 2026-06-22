from __future__ import annotations

import copy
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    import open3d as o3d
except Exception:  # pragma: no cover
    o3d = None

from .postprocessing import PointCloudPostProcessor
from .project_io import ensure_dir, get_nested, write_json


class PointCloudStitcher:
    def __init__(self, cfg: dict, log_fn: Callable[[str], None] = print):
        self.cfg = cfg
        self.log = log_fn
        self.post = PointCloudPostProcessor(cfg, log_fn)

    def _icp_refine(self, source, target, init_T: np.ndarray) -> Tuple[np.ndarray, Dict]:
        threshold = float(get_nested(self.cfg, "stitching.registration_icp_distance_m", 0.008))
        max_iter = int(get_nested(self.cfg, "stitching.registration_max_iterations", 80))
        if len(source.points) == 0 or len(target.points) == 0:
            return init_T, {"fitness": 0.0, "rmse": None, "used": False}
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
        reg = o3d.pipelines.registration.registration_icp(source, target, threshold, init_T, estimation, criteria)
        return np.asarray(reg.transformation), {"fitness": float(reg.fitness), "rmse": float(reg.inlier_rmse), "used": True}

    def fuse_pointcloud_paths_with_known_transforms(
        self,
        pcd_paths: Sequence[str | Path],
        transforms_to_home: Sequence[np.ndarray],
        output_dir: str | Path,
        refine_icp: bool = True,
        transform_infos: Optional[List[Dict]] = None,
    ) -> Tuple[str, Dict]:
        if o3d is None:
            raise RuntimeError("open3d is not installed.")
        if len(pcd_paths) != len(transforms_to_home):
            raise ValueError("Number of point clouds and transforms must match.")
        if len(pcd_paths) == 0:
            raise RuntimeError("No point clouds to fuse.")

        output_dir = ensure_dir(output_dir)
        registered_dir = ensure_dir(Path(output_dir) / "registered_views")
        fused = o3d.geometry.PointCloud()
        report = {"views": [], "refine_icp": bool(refine_icp)}

        for idx, (path, T_home) in enumerate(zip(pcd_paths, transforms_to_home), start=1):
            path = Path(path)
            pcd = o3d.io.read_point_cloud(str(path))
            if pcd is None or len(pcd.points) == 0:
                raise RuntimeError(f"Empty point cloud: {path}")
            T_initial = np.asarray(T_home, dtype=np.float64)
            pcd_aligned = copy.deepcopy(pcd)
            pcd_aligned.transform(T_initial)
            icp_info = {"used": False}
            T_final = T_initial
            if refine_icp and idx > 1 and len(fused.points) > 0:
                # ICP is only a refinement after calibrated turntable-axis alignment.
                source_for_icp = copy.deepcopy(pcd_aligned).voxel_down_sample(float(get_nested(self.cfg, "stitching.registration_voxel_size_m", 0.006)))
                target_for_icp = copy.deepcopy(fused).voxel_down_sample(float(get_nested(self.cfg, "stitching.registration_voxel_size_m", 0.006)))
                delta_T, icp_info = self._icp_refine(source_for_icp, target_for_icp, np.eye(4))
                pcd_aligned.transform(delta_T)
                T_final = delta_T @ T_initial
            registered_path = registered_dir / f"registered_view_{idx:03d}.ply"
            if not o3d.io.write_point_cloud(str(registered_path), pcd_aligned):
                raise IOError(f"Failed to write registered point cloud: {registered_path}")
            fused += pcd_aligned
            voxel = float(get_nested(self.cfg, "stitching.fused_voxel_size_m", 0.003))
            if voxel > 0:
                fused = fused.voxel_down_sample(voxel)
            info = dict(transform_infos[idx - 1]) if transform_infos and idx - 1 < len(transform_infos) else {}
            info.update({
                "view_index": idx,
                "source_pointcloud": str(path),
                "registered_pointcloud": str(registered_path),
                "transform_method": info.get("transform_method", "calibrated_turntable_axis"),
                "T_initial": T_initial.tolist(),
                "T_final": T_final.tolist(),
                "icp": icp_info,
                "points_after_registration": int(len(pcd_aligned.points)),
            })
            report["views"].append(info)
            self.log(f"[STITCH] View {idx:03d}: {len(pcd_aligned.points)} points, method={info['transform_method']}")

        fused = self.post.clean(fused)
        # output_dir is <scan folder>/registration; name the final PLY after
        # the created scan folder so each result is immediately identifiable.
        scan_folder_name = Path(output_dir).parent.name
        fused_path = Path(output_dir) / f"{scan_folder_name}.ply"
        if not o3d.io.write_point_cloud(str(fused_path), fused):
            raise IOError(f"Failed to write fused point cloud: {fused_path}")
        report["fused_pointcloud"] = str(fused_path)
        report["fused_points"] = int(len(fused.points))
        report_path = Path(output_dir) / "registration_report.json"
        write_json(report_path, report)
        self.log(f"[SAVE] Fused point cloud: {fused_path} ({len(fused.points)} points)")
        self.log(f"[SAVE] Registration report: {report_path}")
        return str(fused_path), report
