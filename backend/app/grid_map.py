"""2D occupancy grid map and preset worlds.

Coordinate convention used everywhere in the backend:
- The grid is a NumPy bool array of shape (height, width). True = obstacle.
- A cell is addressed as (col, row) == (x, y). grid[row, col] reads it.
- Continuous positions are in "cell units": the cell (3, 5) covers
  x in [3, 4) and y in [5, 6), and its center is (3.5, 5.5).
"""

from __future__ import annotations

import numpy as np

DEFAULT_WIDTH = 40
DEFAULT_HEIGHT = 30


class GridMap:
    def __init__(self, occupancy: np.ndarray):
        self.occ = np.asarray(occupancy, dtype=bool).copy()

    # ----- construction helpers -------------------------------------------
    @classmethod
    def empty(cls, width: int = DEFAULT_WIDTH, height: int = DEFAULT_HEIGHT, border: bool = True):
        occ = np.zeros((height, width), dtype=bool)
        if border:
            occ[0, :] = occ[-1, :] = True
            occ[:, 0] = occ[:, -1] = True
        return cls(occ)

    @classmethod
    def from_list(cls, rows: list[list[int]]):
        return cls(np.array(rows, dtype=bool))

    def to_list(self) -> list[list[int]]:
        return self.occ.astype(int).tolist()

    # ----- queries ---------------------------------------------------------
    @property
    def width(self) -> int:
        return self.occ.shape[1]

    @property
    def height(self) -> int:
        return self.occ.shape[0]

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, x: int, y: int) -> bool:
        return self.in_bounds(x, y) and not self.occ[y, x]

    def is_free_point(self, x: float, y: float) -> bool:
        """Is the continuous point (x, y) inside a free cell?"""
        return self.is_free(int(np.floor(x)), int(np.floor(y)))

    def segment_free(self, p: tuple[float, float], q: tuple[float, float], step: float = 0.2,
                     clearance: float = 0.0) -> bool:
        """Check a straight line between two continuous points by sampling it.

        With clearance > 0 the robot is treated as a small square of half-size
        `clearance`, so the line must also stay that far away from walls.
        """
        p, q = np.asarray(p, float), np.asarray(q, float)
        n = max(2, int(np.ceil(np.linalg.norm(q - p) / step)) + 1)
        pts = p + np.linspace(0.0, 1.0, n)[:, None] * (q - p)
        if clearance > 0:
            offsets = np.array([[0, 0], [-1, -1], [1, -1], [-1, 1], [1, 1]]) * clearance
            pts = (pts[:, None, :] + offsets[None, :, :]).reshape(-1, 2)
        cols = np.floor(pts[:, 0]).astype(int)
        rows = np.floor(pts[:, 1]).astype(int)
        if (cols < 0).any() or (rows < 0).any() or (cols >= self.width).any() or (rows >= self.height).any():
            return False
        return not self.occ[rows, cols].any()

    def free_cells(self) -> np.ndarray:
        """All free cells as an (N, 2) array of (x, y)."""
        rows, cols = np.nonzero(~self.occ)
        return np.stack([cols, rows], axis=1)


# ----- preset maps ----------------------------------------------------------
# Each preset returns (GridMap, start_cell, goal_cell).

def open_field(seed: int = 7):
    m = GridMap.empty()
    rng = np.random.default_rng(seed)
    for _ in range(14):
        w, h = rng.integers(1, 4), rng.integers(1, 4)
        x, y = rng.integers(3, m.width - 5), rng.integers(3, m.height - 5)
        m.occ[y:y + h, x:x + w] = True
    start, goal = (2, 2), (m.width - 3, m.height - 3)
    m.occ[start[1], start[0]] = m.occ[goal[1], goal[0]] = False
    return m, start, goal


def warehouse():
    m = GridMap.empty()
    # Long shelves (2 cells thick) with aisles between them and a gap in the middle.
    for x in range(5, m.width - 4, 5):
        m.occ[4:13, x:x + 2] = True
        m.occ[17:m.height - 4, x:x + 2] = True
    # A loading dock wall near the bottom right.
    m.occ[m.height - 8, m.width - 9:m.width - 1] = True
    m.occ[m.height - 8, m.width - 6] = False
    return m, (2, 2), (m.width - 3, m.height - 3)


def maze(seed: int = 3):
    """Perfect maze via randomized depth-first search (recursive backtracker).

    Maze 'rooms' sit on odd coordinates; walls between them are knocked out.
    Corridors are widened to 2 cells so the robot (and RRT) have room to move.
    """
    rng = np.random.default_rng(seed)
    cw, ch = 13, 9                      # rooms horizontally / vertically (fits 40x30)
    scale = 2                           # corridor width in cells
    small = np.ones((2 * ch + 1, 2 * cw + 1), dtype=bool)
    visited = np.zeros((ch, cw), dtype=bool)
    stack = [(0, 0)]
    visited[0, 0] = True
    small[1, 1] = False
    while stack:
        cx, cy = stack[-1]
        nbrs = [(cx + dx, cy + dy) for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
                if 0 <= cx + dx < cw and 0 <= cy + dy < ch and not visited[cy + dy, cx + dx]]
        if not nbrs:
            stack.pop()
            continue
        nx, ny = nbrs[rng.integers(len(nbrs))]
        visited[ny, nx] = True
        small[2 * ny + 1, 2 * nx + 1] = False
        small[cy + ny + 1, cx + nx + 1] = False   # the wall between the two rooms
        stack.append((nx, ny))
    # Upscale: walls stay 1 cell thick, rooms/corridors become `scale` cells wide.
    sizes_x = [1 if i % 2 == 0 else scale for i in range(small.shape[1])]
    sizes_y = [1 if i % 2 == 0 else scale for i in range(small.shape[0])]
    big = np.repeat(np.repeat(small, sizes_y, axis=0), sizes_x, axis=1)
    m = GridMap.empty()
    h, w = min(big.shape[0], m.height), min(big.shape[1], m.width)
    m.occ[:h, :w] = big[:h, :w]
    m.occ[:, w - 1:] = True  # anything to the right of the maze is solid wall
    m.occ[h - 1:, :] = True
    return m, (1, 1), (w - 3, h - 3)


PRESETS = {
    "open_field": open_field,
    "warehouse": warehouse,
    "maze": maze,
}


def load_preset(name: str):
    if name not in PRESETS:
        raise KeyError(f"unknown preset '{name}', choose from {sorted(PRESETS)}")
    return PRESETS[name]()
