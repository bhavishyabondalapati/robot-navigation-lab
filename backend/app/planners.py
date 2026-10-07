"""Path planners: A* (graph search on the grid) and RRT (sampling-based).

Both return a PlanResult so the UI and the benchmark can compare them
side by side. Paths are lists of continuous (x, y) points in cell units.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field

import numpy as np

from .grid_map import GridMap

SQRT2 = math.sqrt(2.0)


@dataclass
class PlanResult:
    planner: str
    success: bool
    path: list[tuple[float, float]] = field(default_factory=list)
    path_length: float = 0.0
    nodes_expanded: int = 0
    planning_time_ms: float = 0.0
    explored: list[tuple[int, int]] = field(default_factory=list)          # A*: cells, in expansion order
    tree: list[tuple[float, float, float, float]] = field(default_factory=list)  # RRT: edges (x1, y1, x2, y2)

    def to_dict(self) -> dict:
        return {
            "planner": self.planner,
            "success": self.success,
            "path": [[round(x, 3), round(y, 3)] for x, y in self.path],
            "path_length": round(self.path_length, 3),
            "nodes_expanded": self.nodes_expanded,
            "planning_time_ms": round(self.planning_time_ms, 3),
            "explored": [list(c) for c in self.explored],
            "tree": [[round(v, 3) for v in e] for e in self.tree],
        }


def path_length(path) -> float:
    if len(path) < 2:
        return 0.0
    p = np.asarray(path, float)
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def center(cell) -> tuple[float, float]:
    return (cell[0] + 0.5, cell[1] + 0.5)


# ----- A* --------------------------------------------------------------------

# 8-connected moves: (dx, dy, cost)
MOVES = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
         (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]


def octile(a, b) -> float:
    """Exact cost of the shortest 8-connected path with no obstacles.

    It never overestimates the true cost, so A* stays optimal (admissible).
    """
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)


def astar(grid: GridMap, start, goal) -> PlanResult:
    t0 = time.perf_counter()
    start, goal = tuple(start), tuple(goal)
    result = PlanResult(planner="astar", success=False)
    if not grid.is_free(*start) or not grid.is_free(*goal):
        result.planning_time_ms = (time.perf_counter() - t0) * 1000
        return result

    g = {start: 0.0}
    parent = {start: None}
    closed = set()
    counter = 0  # tie-breaker so heapq never compares tuples of cells
    open_heap = [(octile(start, goal), counter, start)]

    while open_heap:
        _, _, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue  # stale heap entry (we found a cheaper way to it later)
        closed.add(cur)
        result.explored.append(cur)
        if cur == goal:
            break
        cx, cy = cur
        for dx, dy, cost in MOVES:
            nxt = (cx + dx, cy + dy)
            if not grid.is_free(*nxt) or nxt in closed:
                continue
            # No corner cutting: a diagonal move needs both side cells free.
            if dx and dy and not (grid.is_free(cx + dx, cy) and grid.is_free(cx, cy + dy)):
                continue
            new_g = g[cur] + cost
            if new_g < g.get(nxt, math.inf):
                g[nxt] = new_g
                parent[nxt] = cur
                counter += 1
                heapq.heappush(open_heap, (new_g + octile(nxt, goal), counter, nxt))

    result.nodes_expanded = len(result.explored)
    if goal in closed:
        cells = []
        node = goal
        while node is not None:
            cells.append(node)
            node = parent[node]
        cells.reverse()
        result.success = True
        result.path = [center(c) for c in cells]
        result.path_length = g[goal]
    result.planning_time_ms = (time.perf_counter() - t0) * 1000
    return result


# ----- RRT -------------------------------------------------------------------

def rrt(grid: GridMap, start, goal, rng: np.random.Generator | None = None,
        max_iters: int = 20000, step: float = 1.5, goal_bias: float = 0.1,
        goal_tolerance: float = 1.0, clearance: float = 0.3) -> PlanResult:
    """Rapidly-exploring Random Tree.

    Repeat: sample a random point (sometimes the goal itself), find the nearest
    tree node, take a short step toward the sample, and keep the new node if
    the segment is collision-free. Stop once the goal can be connected.
    """
    t0 = time.perf_counter()
    rng = rng if rng is not None else np.random.default_rng(0)
    result = PlanResult(planner="rrt", success=False)
    if not grid.is_free(*start) or not grid.is_free(*goal):
        result.planning_time_ms = (time.perf_counter() - t0) * 1000
        return result

    s, gpt = np.array(center(start)), np.array(center(goal))
    nodes = np.zeros((max_iters + 2, 2))
    parents = np.full(max_iters + 2, -1, dtype=int)
    nodes[0] = s
    n = 1
    goal_idx = -1

    for _ in range(max_iters):
        if rng.random() < goal_bias:
            sample = gpt
        else:
            sample = rng.uniform([0, 0], [grid.width, grid.height])
        d = np.linalg.norm(nodes[:n] - sample, axis=1)
        near = int(np.argmin(d))
        direction = sample - nodes[near]
        dist = d[near]
        if dist < 1e-9:
            continue
        new = nodes[near] + direction * min(1.0, step / dist)
        if not grid.segment_free(nodes[near], new, clearance=clearance):
            continue
        nodes[n], parents[n] = new, near
        result.tree.append((*nodes[near], *new))
        n += 1
        if np.linalg.norm(new - gpt) <= goal_tolerance + step and grid.segment_free(new, gpt, clearance=clearance):
            nodes[n], parents[n] = gpt, n - 1
            result.tree.append((*new, *gpt))
            goal_idx = n
            n += 1
            break

    result.nodes_expanded = n
    if goal_idx >= 0:
        idx, pts = goal_idx, []
        while idx != -1:
            pts.append((float(nodes[idx, 0]), float(nodes[idx, 1])))
            idx = parents[idx]
        pts.reverse()
        result.success = True
        result.path = pts
        result.path_length = path_length(pts)
    result.planning_time_ms = (time.perf_counter() - t0) * 1000
    return result


def plan(name: str, grid: GridMap, start, goal, seed: int = 0) -> PlanResult:
    if name == "astar":
        return astar(grid, start, goal)
    if name == "rrt":
        return rrt(grid, start, goal, rng=np.random.default_rng(seed))
    raise ValueError(f"unknown planner '{name}'")
