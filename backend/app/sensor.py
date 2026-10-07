"""Simulated 2D lidar using ray casting against the occupancy grid.

Ray casting = shoot a ray from the robot in a direction and find the
distance to the first obstacle cell it hits. We use the DDA ("digital
differential analyzer") grid traversal: walk the ray from cell boundary to
cell boundary, so it visits exactly the cells the ray passes through and
returns the exact hit distance. It is vectorized so thousands of rays
(every beam of every particle) are traced at once with NumPy.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .grid_map import GridMap


def cast_rays(grid: GridMap, xs, ys, angles, max_range: float) -> np.ndarray:
    """Distance from each (x, y) along each angle to the first obstacle.

    xs, ys, angles are broadcastable arrays; the result has their shape.
    Rays that hit nothing within max_range (or leave the map) return max_range.
    """
    xs, ys, angles = np.broadcast_arrays(np.asarray(xs, float), np.asarray(ys, float), np.asarray(angles, float))
    shape = xs.shape
    x0, y0, a = xs.ravel(), ys.ravel(), angles.ravel()
    n = x0.size
    dx, dy = np.cos(a), np.sin(a)

    cx = np.floor(x0).astype(int)
    cy = np.floor(y0).astype(int)
    step_x = np.where(dx >= 0, 1, -1)
    step_y = np.where(dy >= 0, 1, -1)
    with np.errstate(divide="ignore"):
        t_delta_x = np.where(dx != 0, np.abs(1.0 / dx), np.inf)  # ray length to cross one cell in x
        t_delta_y = np.where(dy != 0, np.abs(1.0 / dy), np.inf)
    next_bx = np.where(step_x > 0, cx + 1 - x0, x0 - cx)       # distance (in x) to the next vertical line
    next_by = np.where(step_y > 0, cy + 1 - y0, y0 - cy)
    t_max_x = np.where(dx != 0, next_bx * t_delta_x, np.inf)   # ray length to reach that line
    t_max_y = np.where(dy != 0, next_by * t_delta_y, np.inf)

    dist = np.full(n, float(max_range))
    active = np.ones(n, dtype=bool)
    h, w = grid.height, grid.width

    # A ray starting inside an obstacle hits immediately.
    inside = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
    start_hit = np.zeros(n, dtype=bool)
    start_hit[inside] = grid.occ[cy[inside], cx[inside]]
    dist[start_hit] = 0.0
    active &= ~start_hit & inside

    max_steps = int(2 * np.ceil(max_range)) + 4
    for _ in range(max_steps):
        if not active.any():
            break
        idx = np.nonzero(active)[0]
        go_x = t_max_x[idx] < t_max_y[idx]
        t_here = np.where(go_x, t_max_x[idx], t_max_y[idx])  # distance where we enter the next cell
        cx[idx] += np.where(go_x, step_x[idx], 0)
        cy[idx] += np.where(go_x, 0, step_y[idx])
        t_max_x[idx] += np.where(go_x, t_delta_x[idx], 0)
        t_max_y[idx] += np.where(go_x, 0, t_delta_y[idx])

        too_far = t_here >= max_range
        out = (cx[idx] < 0) | (cx[idx] >= w) | (cy[idx] < 0) | (cy[idx] >= h)
        hit = np.zeros(idx.size, dtype=bool)
        ok = ~out & ~too_far
        hit[ok] = grid.occ[cy[idx[ok]], cx[idx[ok]]]
        dist[idx[hit]] = t_here[hit]
        active[idx[hit | out | too_far]] = False

    return dist.reshape(shape)


@dataclass
class Lidar:
    n_beams: int = 16
    max_range: float = 10.0
    noise_std: float = 0.1

    def beam_angles(self) -> np.ndarray:
        """Beam directions relative to the robot heading (a full 360° ring)."""
        return np.linspace(-np.pi, np.pi, self.n_beams, endpoint=False)

    def expected(self, grid: GridMap, poses: np.ndarray) -> np.ndarray:
        """Noise-free ranges for many poses: shape (N, n_beams)."""
        poses = np.atleast_2d(poses)
        angles = poses[:, 2:3] + self.beam_angles()[None, :]
        return cast_rays(grid, poses[:, 0:1], poses[:, 1:2], angles, self.max_range)

    def scan(self, grid: GridMap, pose, rng: np.random.Generator) -> np.ndarray:
        """A noisy measurement from the true pose: shape (n_beams,)."""
        clean = self.expected(grid, np.asarray(pose, float))[0]
        noisy = clean + rng.normal(0.0, self.noise_std, clean.shape)
        return np.clip(noisy, 0.0, self.max_range)
