"""Monte Carlo Localization (particle filter).

Each particle is a guess of the robot pose (x, y, theta). Every step:
  1. Predict:  move every particle with the same noisy motion model.
  2. Update:   compare the real lidar scan with the scan each particle
               *would* see on the map; good matches get higher weight.
  3. Resample: draw a new set of particles in proportion to weight, so
               unlikely guesses die out and likely ones multiply.
The estimate is the weighted average of the particles.
"""

from __future__ import annotations

import numpy as np

from .grid_map import GridMap
from .motion import MotionNoise, apply_motion, wrap_angle
from .sensor import Lidar


class ParticleFilter:
    def __init__(self, grid: GridMap, lidar: Lidar, n_particles: int = 400,
                 motion_noise: MotionNoise | None = None, sensor_std: float = 0.5,
                 outlier_prob: float = 0.05, jitter=(0.05, 0.05, 0.02),
                 rng: np.random.Generator | None = None):
        self.grid = grid
        self.lidar = lidar
        self.n = n_particles
        # Slightly larger than the simulator's noise: being a bit unsure is
        # safer than being overconfident and losing track of the robot.
        self.motion_noise = motion_noise or MotionNoise(0.04, 0.08, 0.03, 0.12)
        # The likelihood is wider than the true lidar noise to tolerate map
        # discretization and particles that are close but not exact.
        self.sensor_std = sensor_std
        # Chance that a single beam is garbage (e.g. hits an obstacle the map
        # doesn't know about yet). Caps how much one bad beam can hurt a particle.
        self.outlier_prob = outlier_prob
        # Small noise added after resampling so copies of the same particle
        # spread out again instead of collapsing into one point.
        self.jitter = np.asarray(jitter, float)
        self.rng = rng if rng is not None else np.random.default_rng(0)
        self.particles = np.zeros((n_particles, 3))
        self.weights = np.full(n_particles, 1.0 / n_particles)

    def init_gaussian(self, mean, std) -> None:
        """Spread particles around a guess, re-drawing any that land in walls."""
        mean, std = np.asarray(mean, float), np.asarray(std, float)
        p = mean + self.rng.normal(0.0, 1.0, (self.n, 3)) * std
        for _ in range(20):
            bad = ~self._in_free_space(p)
            if not bad.any():
                break
            p[bad] = mean + self.rng.normal(0.0, 1.0, (bad.sum(), 3)) * std
        p[~self._in_free_space(p), :2] = mean[:2]
        p[:, 2] = wrap_angle(p[:, 2])
        self.particles = p
        self.weights = np.full(self.n, 1.0 / self.n)

    def _in_free_space(self, p: np.ndarray) -> np.ndarray:
        cols = np.floor(p[:, 0]).astype(int)
        rows = np.floor(p[:, 1]).astype(int)
        ok = (cols >= 0) & (cols < self.grid.width) & (rows >= 0) & (rows < self.grid.height)
        free = np.zeros(len(p), dtype=bool)
        free[ok] = ~self.grid.occ[rows[ok], cols[ok]]
        return free

    # ----- the three steps ---------------------------------------------------
    def predict(self, rotation: float, distance: float) -> None:
        self.particles = apply_motion(self.particles, rotation, distance, self.motion_noise, self.rng)

    def update(self, scan: np.ndarray) -> None:
        expected = self.lidar.expected(self.grid, self.particles)        # (N, beams)
        err = expected - scan[None, :]
        # Per-beam likelihood: a Gaussian around the expected range, mixed with
        # a small flat "outlier" floor. Beams are treated as independent, so
        # we multiply their probabilities (= add log-probabilities).
        gauss = np.exp(-0.5 * (err / self.sensor_std) ** 2)
        beam_p = (1 - self.outlier_prob) * gauss + self.outlier_prob
        log_w = np.sum(np.log(beam_p), axis=1)
        log_w[~self._in_free_space(self.particles)] = -np.inf          # robots can't be inside walls
        log_w += np.log(self.weights + 1e-300)
        if not np.isfinite(log_w).any():
            self.weights = np.full(self.n, 1.0 / self.n)                 # everyone failed: reset weights
            return
        log_w -= log_w.max()                                              # avoid underflow before exp
        w = np.exp(log_w)
        self.weights = w / w.sum()

    def effective_sample_size(self) -> float:
        return 1.0 / np.sum(self.weights ** 2)

    def resample(self) -> None:
        """Low-variance (systematic) resampling: one random offset, N evenly spaced pointers."""
        positions = (self.rng.random() + np.arange(self.n)) / self.n
        cumulative = np.cumsum(self.weights)
        cumulative[-1] = 1.0
        idx = np.searchsorted(cumulative, positions)
        self.particles = self.particles[idx] + self.rng.normal(0.0, 1.0, (self.n, 3)) * self.jitter
        self.particles[:, 2] = wrap_angle(self.particles[:, 2])
        self.weights = np.full(self.n, 1.0 / self.n)

    def step(self, rotation: float, distance: float, scan: np.ndarray) -> np.ndarray:
        self.predict(rotation, distance)
        self.update(scan)
        # Only resample when the weights have become uneven; resampling
        # every step throws away diversity for no reason.
        if self.effective_sample_size() < self.n / 2:
            self.resample()
        return self.estimate()

    def estimate(self) -> np.ndarray:
        w = self.weights
        x = np.sum(w * self.particles[:, 0])
        y = np.sum(w * self.particles[:, 1])
        # Angles must be averaged as unit vectors: the mean of 179° and -179° is 180°, not 0°.
        th = np.arctan2(np.sum(w * np.sin(self.particles[:, 2])), np.sum(w * np.cos(self.particles[:, 2])))
        return np.array([x, y, th])
