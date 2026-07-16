"""Colour-based crate detection (framework-independent).

Ported from the validated Webots vision.py: segment red crates by channel
ratio (brightness-invariant), group red pixels into blobs, and back-project
each blob centroid onto the ground plane using the camera world pose derived
from the robot pose + a fixed mount. No ROS/Webots imports -- the ROS 2
vision_node feeds it a numpy image and the robot pose.

Camera convention: optical axis = camera local +X, up = +Z, left = +Y.
"""

from __future__ import annotations

import math

import numpy as np


def red_mask(rgb: np.ndarray, value_min: int = 60, red_ratio: float = 0.33) -> np.ndarray:
    """Boolean mask of near-pure-red pixels. `rgb` is (H, W, 3), channels R,G,B."""
    r = rgb[:, :, 0].astype(np.int16)
    g = rgb[:, :, 1].astype(np.int16)
    b = rgb[:, :, 2].astype(np.int16)
    limit = red_ratio * r
    return (r > value_min) & (r >= g) & (r >= b) & (g < limit) & (b < limit)


def blob_centroids(mask: np.ndarray, min_pixels: int = 4):
    """4-connectivity connected components; return blob centroids as (u, v)."""
    visited = np.zeros_like(mask, dtype=bool)
    h, w = mask.shape
    out = []
    for r0 in range(h):
        row = mask[r0]
        for c0 in range(w):
            if not row[c0] or visited[r0, c0]:
                continue
            stack = [(r0, c0)]
            visited[r0, c0] = True
            pixels = []
            while stack:
                r, c = stack.pop()
                pixels.append((r, c))
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w and mask[nr, nc] and not visited[nr, nc]:
                        visited[nr, nc] = True
                        stack.append((nr, nc))
            if len(pixels) >= min_pixels:
                vs = sum(p[0] for p in pixels) / len(pixels)
                us = sum(p[1] for p in pixels) / len(pixels)
                out.append((us, vs))
    return out


class GroundProjector:
    """Turn image pixels into world (x, y) on the floor plane.

    Camera world pose is computed from the robot pose (x, y, yaw) composed with
    the fixed mount (forward offset, height, downward pitch) -- no supervisor.
    """

    def __init__(self, width, height, fov, ground_z=0.05,
                 mount_forward=0.2, camera_height=0.40, mount_pitch=0.5,
                 min_ray_down=0.12, max_range=3.0, arena_bound=4.6):
        self.fx = (width / 2.0) / math.tan(fov / 2.0)
        self.cx = (width - 1) / 2.0
        self.cy = (height - 1) / 2.0
        self.ground_z = ground_z
        self.mount_forward = mount_forward
        self.camera_height = camera_height
        self.cos_pitch = math.cos(mount_pitch)
        self.sin_pitch = math.sin(mount_pitch)
        self.min_ray_down = min_ray_down
        self.max_range = max_range
        self.arena_bound = arena_bound

    def pixel_to_ground(self, u, v, rx, ry, yaw):
        x_n = (u - self.cx) / self.fx
        y_n = (v - self.cy) / self.fx
        ray_cam = (1.0, -x_n, -y_n)
        cyaw, syaw = math.cos(yaw), math.sin(yaw)
        cb, sb = self.cos_pitch, self.sin_pitch
        pos = (rx + self.mount_forward * cyaw, ry + self.mount_forward * syaw,
               self.camera_height)
        # R = Rz(yaw) . Ry(pitch); its columns are the camera axes in world.
        r = (cyaw * cb, -syaw, cyaw * sb,
             syaw * cb, cyaw, syaw * sb,
             -sb, 0.0, cb)
        xw = (r[0], r[3], r[6])
        yw = (r[1], r[4], r[7])
        zw = (r[2], r[5], r[8])
        rw = tuple(ray_cam[0] * xw[i] + ray_cam[1] * yw[i] + ray_cam[2] * zw[i]
                   for i in range(3))
        if rw[2] > -self.min_ray_down:
            return None
        t = (self.ground_z - pos[2]) / rw[2]
        if t <= 0 or t > self.max_range:
            return None
        wx, wy = pos[0] + t * rw[0], pos[1] + t * rw[1]
        if abs(wx) > self.arena_bound or abs(wy) > self.arena_bound:
            return None
        return (wx, wy)
