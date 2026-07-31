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


def red_mask(rgb: np.ndarray, value_min: int = 55, min_diff: int = 45,
             max_sym: float = 0.20, max_ratio: float = 0.62) -> np.ndarray:
    """Boolean mask of ripe-strawberry pixels. `rgb` is (H, W, 3), R,G,B.

    This used to be a channel RATIO test (g < 0.33*r and b < 0.33*r), and it
    stopped working the day the world got its realistic rendering. Gazebo
    shades a classic SDF material as

        pixel = scene_ambient*ambient + light_diffuse*diffuse
                + light_specular*specular

    and the berries carry <specular>0.8 0.8 0.8</specular>, so a near-WHITE
    term is added EQUALLY to all three channels. On the lit face of a berry
    that lands at (255, 106, 92): still obviously red to the eye, but
    106 > 0.33*255, so the ratio test threw it away. Only the shaded side of
    a fruit still passed -- exactly the "15 berries in view, vision reports 0
    to 3" in the field log.

    The fix is to test DIFFERENCES, which added white cannot change:

      r - g, r - b >= min_diff   how red the surface is, ratio-free and so
                                 immune to highlights and to shadow;
      |g - b| <= max_sym * r     a red surface reflects little green AND
                                 little blue, and a white highlight lifts
                                 both by the same amount. Orange does not
                                 satisfy this -- which is what was raising
                                 false positives on the robot's own KUKA
                                 orange arm when it swung through frame;
      g <= max_ratio * r         reject the washed-out pinks (bright glass,
                                 white gutter under the sun).

    Difference tests on the red channel are the classic agricultural "excess
    red" index, and they carry over to a real camera far better than ratios,
    which swing with exposure.
    """
    r = rgb[:, :, 0].astype(np.int16)
    g = rgb[:, :, 1].astype(np.int16)
    b = rgb[:, :, 2].astype(np.int16)
    return ((r >= value_min) & (r - g >= min_diff) & (r - b >= min_diff)
            & (np.abs(g - b) <= max_sym * r) & (g <= max_ratio * r))


def _dilate(mask: np.ndarray) -> np.ndarray:
    out = mask.copy()
    out[1:, :] |= mask[:-1, :]
    out[:-1, :] |= mask[1:, :]
    out[:, 1:] |= mask[:, :-1]
    out[:, :-1] |= mask[:, 1:]
    return out


def _erode(mask: np.ndarray) -> np.ndarray:
    out = mask.copy()
    out[1:, :] &= mask[:-1, :]
    out[:-1, :] &= mask[1:, :]
    out[:, 1:] &= mask[:, :-1]
    out[:, :-1] &= mask[:, 1:]
    out[0, :] = out[-1, :] = False          # outside the image = background
    out[:, 0] = out[:, -1] = False
    return out


def close_mask(mask: np.ndarray, iterations: int = 1) -> np.ndarray:
    """Morphological closing: dilate then erode, same number of times.

    A leaf crossing a berry splits it into two red regions, and the labeller
    then reports two fruit where there is one -- the FALSE POSITIVES seen in
    the field log. Closing bridges gaps up to 2*iterations pixels wide without
    growing the blob overall, so a partly-veiled berry counts once. It also
    removes isolated single-pixel speckle, which is what a highlight on a wet
    leaf edge looks like."""
    if iterations <= 0:
        return mask
    out = mask
    for _ in range(iterations):
        out = _dilate(out)
    for _ in range(iterations):
        out = _erode(out)
    return out


def blob_centroids(mask: np.ndarray, min_pixels: int = 4,
                   min_fill: float = 0.0, max_aspect: float = 0.0):
    """4-connectivity connected components; return blob centroids as (u, v).

    Union-find over the SET pixels only. The previous version walked all
    640*480 = 307k pixels in Python on every frame; a frame typically holds a
    couple of thousand red ones, so working from np.nonzero is ~two orders of
    magnitude less Python. That headroom matters: the detector shares a CPU
    with Gazebo's renderer, and a detector that falls behind hands the mission
    stale fruit offsets -- the robot then aligns on where a berry was a second
    ago, which looks exactly like "it picks in thin air".

    Two shape gates, because a strawberry is ROUND and the things that fool a
    colour threshold are not. `min_fill` is the fraction of the bounding box
    the blob actually fills -- a disc fills pi/4 = 0.79, an L of two specks
    bridged by closing fills far less. `max_aspect` is the longer side of the
    box over the shorter -- a disc is 1, a red highlight running along the lip
    of a gutter is 10 or more, and fills its box completely, so only the aspect
    gate catches it. Either set to 0 disables that gate."""
    ys, xs = np.nonzero(mask)
    n = ys.size
    if n == 0:
        return []

    index = np.full(mask.shape, -1, dtype=np.int32)
    index[ys, xs] = np.arange(n, dtype=np.int32)
    # Neighbour already visited in raster order: the pixel to the left and the
    # one above. Gathering them vectorised keeps the Python loop to unions.
    left = np.where(xs > 0, index[ys, xs - 1], -1)
    up = np.where(ys > 0, index[ys - 1, xs], -1)

    parent = list(range(n))

    def find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]      # path halving
            a = parent[a]
        return a

    for i in range(n):
        for nb in (int(left[i]), int(up[i])):
            if nb >= 0:
                ra, rb = find(i), find(nb)
                if ra != rb:
                    parent[max(ra, rb)] = min(ra, rb)

    roots = np.fromiter((find(i) for i in range(n)), dtype=np.int32, count=n)
    counts = np.bincount(roots)
    sum_u = np.bincount(roots, weights=xs.astype(np.float64))
    sum_v = np.bincount(roots, weights=ys.astype(np.float64))
    keep = np.nonzero(counts >= min_pixels)[0]

    if (min_fill > 0.0 or max_aspect > 0.0) and keep.size:
        big = counts.size
        u0 = np.full(big, np.iinfo(np.int32).max, dtype=np.int32)
        u1 = np.full(big, -1, dtype=np.int32)
        v0, v1 = u0.copy(), u1.copy()
        np.minimum.at(u0, roots, xs)
        np.maximum.at(u1, roots, xs)
        np.minimum.at(v0, roots, ys)
        np.maximum.at(v1, roots, ys)
        w = (u1[keep] - u0[keep] + 1).astype(np.float64)
        h = (v1[keep] - v0[keep] + 1).astype(np.float64)
        ok = np.ones(keep.size, dtype=bool)
        if min_fill > 0.0:
            ok &= counts[keep] / (w * h) >= min_fill
        if max_aspect > 0.0:
            ok &= np.maximum(w, h) / np.minimum(w, h) <= max_aspect
        keep = keep[ok]

    return [(sum_u[k] / counts[k], sum_v[k] / counts[k]) for k in keep]


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
