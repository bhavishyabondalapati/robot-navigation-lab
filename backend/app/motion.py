"""Robot motion: a simple "turn, then drive forward" model with Gaussian noise.

A pose is (x, y, theta) in cell units and radians. A control is
(rotation, distance): first rotate by `rotation`, then drive `distance`
along the new heading. Real wheels slip, so the actual motion is the
commanded motion plus random noise that grows with how much we move.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def wrap_angle(a):
    """Map any angle (or array of angles) into [-pi, pi)."""
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


@dataclass
class MotionNoise:
    rot_base: float = 0.02     # rad of rotation noise even for tiny moves
    rot_per_rot: float = 0.05  # extra rotation noise per rad turned
    trans_base: float = 0.01   # cells of distance noise even for tiny moves
    trans_per_trans: float = 0.08  # extra distance noise per cell driven


def apply_motion(poses: np.ndarray, rotation: float, distance: float,
                 noise: MotionNoise | None, rng: np.random.Generator) -> np.ndarray:
    """Move one pose (shape (3,)) or many poses (shape (N, 3)) with noise.

    The same function moves the true robot and every particle, so the
    particle filter's motion model matches the simulator's.
    """
    poses = np.atleast_2d(np.asarray(poses, dtype=float)).copy()
    n = len(poses)
    rot = np.full(n, rotation)
    trans = np.full(n, distance)
    if noise is not None:
        rot += rng.normal(0.0, noise.rot_base + noise.rot_per_rot * abs(rotation), n)
        trans += rng.normal(0.0, noise.trans_base + noise.trans_per_trans * abs(distance), n)
    poses[:, 2] = wrap_angle(poses[:, 2] + rot)
    poses[:, 0] += trans * np.cos(poses[:, 2])
    poses[:, 1] += trans * np.sin(poses[:, 2])
    return poses


def control_towards(pose, target, max_step: float = 0.4, max_turn: float = 0.6):
    """Pick a (rotation, distance) command that heads the robot to `target`.

    Big heading errors are fixed by turning in place first; once roughly
    aligned, the robot turns and drives in the same step.
    """
    x, y, th = pose
    dx, dy = target[0] - x, target[1] - y
    dist = float(np.hypot(dx, dy))
    heading_err = float(wrap_angle(np.arctan2(dy, dx) - th))
    if abs(heading_err) > max_turn:
        return float(np.clip(heading_err, -max_turn, max_turn)), 0.0
    return heading_err, min(max_step, dist)
