from __future__ import annotations

from pathlib import Path
from typing import Callable

try:
    import open3d as o3d
except Exception:  # pragma: no cover
    o3d = None


class MeshGenerator:
    def __init__(self, log_fn: Callable[[str], None] = print):
        self.log = log_fn

    def poisson_mesh_from_ply(self, ply_path: str | Path, output_path: str | Path, depth: int = 8) -> str:
        if o3d is None:
            raise RuntimeError("open3d is not installed.")
        pcd = o3d.io.read_point_cloud(str(ply_path))
        if len(pcd.points) == 0:
            raise RuntimeError("Cannot generate mesh from an empty point cloud.")
        pcd.estimate_normals()
        mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=int(depth))
        mesh.compute_vertex_normals()
        if not o3d.io.write_triangle_mesh(str(output_path), mesh):
            raise IOError(f"Failed to write mesh: {output_path}")
        self.log(f"[SAVE] Mesh: {output_path}")
        return str(output_path)
