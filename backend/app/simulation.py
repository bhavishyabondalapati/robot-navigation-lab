"""Ties everything together: plan a path, drive along it, localize, replan.

The robot is driven by its *estimated* pose (what the particle filter
believes), just like a real robot that can't see its true position. The
true pose is only used to simulate the world: the noisy motion and the
lidar scan.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .grid_map import GridMap
from .motion import MotionNoise, apply_motion, control_towards
from .particle_filter import ParticleFilter
from .planners import PlanResult, center, plan
from .sensor import Lidar


@dataclass
class SimConfig:
    planner: str = "astar"
    seed: int = 42
    n_particles: int = 400
    n_beams: int = 16
    max_range: float = 10.0
    lidar_noise: float = 0.1
    max_steps: int = 1500
    waypoint_tolerance: float = 0.5
    init_std: tuple[float, float, float] = (1.0, 1.0, 0.3)


class Simulation:
    def __init__(self, grid: GridMap, start, goal, config: SimConfig | None = None):
        self.cfg = config or SimConfig()
        self.grid = GridMap(grid.occ)          # own copy: obstacles can change mid-run
        self.start, self.goal = tuple(start), tuple(goal)
        # Separate random streams so e.g. changing the particle count does not
        # change the robot's own noise. Same seed -> identical run.
        seeds = np.random.SeedSequence(self.cfg.seed).spawn(3)
        self.world_rng = np.random.default_rng(seeds[0])
        pf_rng = np.random.default_rng(seeds[1])
        self.plan_seed = int(np.random.default_rng(seeds[2]).integers(1 << 31))

        self.lidar = Lidar(self.cfg.n_beams, self.cfg.max_range, self.cfg.lidar_noise)
        self.motion_noise = MotionNoise()
        sx, sy = center(self.start)
        gx, gy = center(self.goal)
        self.true_pose = np.array([sx, sy, float(np.arctan2(gy - sy, gx - sx))])
        self.pf = ParticleFilter(self.grid, self.lidar, self.cfg.n_particles, rng=pf_rng)
        self.pf.init_gaussian(self.true_pose, self.cfg.init_std)
        self.est_pose = self.pf.estimate()

        self.step_count = 0
        self.errors: list[float] = [self._error()]
        self.status = "running"
        self.replans = 0
        self.collisions = 0
        self.stuck_steps = 0
        self.last_scan = self.lidar.scan(self.grid, self.true_pose, self.world_rng)
        self.plan_result: PlanResult = self._plan(self.start)
        self.waypoint = 1

    # ----- planning ------------------------------------------------------------
    def _plan(self, from_cell) -> PlanResult:
        res = plan(self.cfg.planner, self.grid, from_cell, self.goal, seed=self.plan_seed + self.replans)
        if not res.success:
            self.status = "no_path"
        return res

    def _current_cell(self):
        """Cell to plan from: the estimate's cell, or the nearest free cell to it."""
        cell = (int(self.est_pose[0]), int(self.est_pose[1]))
        if self.grid.is_free(*cell):
            return cell
        free = self.grid.free_cells()
        d = np.linalg.norm(free + 0.5 - self.est_pose[:2], axis=1)
        return tuple(int(v) for v in free[np.argmin(d)])

    def path_blocked(self) -> bool:
        path = self.plan_result.path
        pts = [tuple(self.est_pose[:2])] + list(path[self.waypoint:])
        return any(not self.grid.segment_free(a, b) for a, b in zip(pts[1:], pts[2:])) or \
            any(not self.grid.is_free_point(*p) for p in pts[1:])

    def replan(self) -> None:
        self.replans += 1
        self.status = "running"
        self.plan_result = self._plan(self._current_cell())
        self.waypoint = 1

    def set_cell(self, x: int, y: int, occupied: bool) -> bool:
        """Add/remove an obstacle while running. Returns True if a replan happened."""
        if not self.grid.in_bounds(x, y):
            return False
        robot_cell = (int(self.true_pose[0]), int(self.true_pose[1]))
        if occupied and (x, y) in (robot_cell, self.goal):
            return False  # never drop a wall on top of the robot or the goal
        self.grid.occ[y, x] = occupied
        if self.status == "no_path" and not occupied:
            self.replan()
            return True
        if self.status == "running" and occupied and self.path_blocked():
            self.replan()
            return True
        return False

    # ----- one tick ------------------------------------------------------------
    def step(self) -> None:
        if self.status != "running":
            return
        path = self.plan_result.path
        # Skip waypoints the robot (believes it) has already reached.
        while self.waypoint < len(path) - 1 and \
                np.hypot(*(np.array(path[self.waypoint]) - self.est_pose[:2])) < self.cfg.waypoint_tolerance:
            self.waypoint += 1
        target = path[min(self.waypoint, len(path) - 1)]
        rotation, distance = control_towards(self.est_pose, target)

        # Simulate the real (noisy) motion; walls stop the robot.
        new_pose = apply_motion(self.true_pose, rotation, distance, self.motion_noise, self.world_rng)[0]
        odom_distance = distance
        if self.grid.segment_free(self.true_pose[:2], new_pose[:2], step=0.1):
            self.true_pose = new_pose
        else:
            self.true_pose[2] = new_pose[2]
            # The wheel encoders notice the robot stalled against a wall, so the
            # filter is told "turned, but did not drive" for this step.
            odom_distance = 0.0
            self.collisions += 1
            self.stuck_steps += 1
        if distance > 0 and odom_distance > 0:
            self.stuck_steps = 0

        self.last_scan = self.lidar.scan(self.grid, self.true_pose, self.world_rng)
        self.est_pose = self.pf.step(rotation, odom_distance, self.last_scan)
        self.step_count += 1
        self.errors.append(self._error())

        goal_xy = np.array(center(self.goal))
        if np.hypot(*(self.est_pose[:2] - goal_xy)) < self.cfg.waypoint_tolerance:
            self.status = "reached"
        elif self.stuck_steps >= 8:
            # Pushing against a wall for a while: plan a fresh path from here.
            self.stuck_steps = 0
            self.replan()
        elif self.step_count >= self.cfg.max_steps:
            self.status = "timeout"

    def _error(self) -> float:
        return float(np.hypot(*(self.est_pose[:2] - self.true_pose[:2])))

    # ----- serialization -------------------------------------------------------
    def state(self) -> dict:
        r = lambda a: [round(float(v), 3) for v in a]  # noqa: E731
        return {
            "step": self.step_count,
            "status": self.status,
            "true_pose": r(self.true_pose),
            "est_pose": r(self.est_pose),
            "particles": np.round(self.pf.particles, 2).tolist(),
            "scan": r(self.last_scan),
            "beam_angles": r(self.lidar.beam_angles()),
            "error": round(self.errors[-1], 4),
            "replans": self.replans,
            "collisions": self.collisions,
            "waypoint": self.waypoint,
        }
