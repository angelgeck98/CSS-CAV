import copy
import os
import open3d as o3d
import numpy as np

from .attacker import Attacker

class LidarSpoofEarlyAttacker(Attacker):
    def __init__(self):
        super().__init__()
        self.name = "live_lidar_spoofer"

        self.mesh = o3d.geometry.TriangleMesh.create_box(width=2.0, height=1.0, depth=4.0)
        self.mesh.compute_vertex_normals()
        self.raycasting = o3d.t.geometry.RaycastingScene()
        self.mesh_id = self.raycasting.add_triangles(
            o3d.t.geometry.TriangleMesh.from_legacy(self.mesh)
        )

    def spoof(self, attacker_vehicle, lidar_points):
        transform = attacker_vehicle.get_transform()
        forward = transform.get_forward_vector()
        spoof_loc = np.array([
            transform.location.x + forward.x * 5,
            transform.location.y + forward.y * 5,
            transform.location.z
        ])

        spoofed_mesh = copy.deepcopy(self.mesh)
        spoofed_mesh.translate(spoof_loc)

        self.raycasting = o3d.t.geometry.RaycastingScene()
        self.mesh_id = self.raycasting.add_triangles(
            o3d.t.geometry.TriangleMesh.from_legacy(spoofed_mesh)
        )

        ray_origins = np.zeros((lidar_points.shape[0], 3), dtype=np.float32)
        directions = lidar_points[:, :3]
        norms = np.linalg.norm(directions, axis=1, keepdims=True)
        ray_directions = directions / (norms + 1e-6)

        rays = np.hstack((ray_origins, ray_directions)).astype(np.float32)

        results = self.raycasting.cast_rays(o3d.core.Tensor(rays))
        hits = results['t_hit'].numpy()

        valid = hits < np.inf
        spoofed_points = ray_directions[valid] * hits[valid].reshape(-1, 1)

        merged = np.vstack([lidar_points[:, :3], spoofed_points])
        return merged
