import numpy as np
import pytest

from app.grid_map import GridMap
from app.sensor import Lidar, cast_rays


def room():
    # 20 x 10 room with walls on the border (cells 0 and 19 / 0 and 9).
    return GridMap.empty(20, 10)


def test_ray_hits_walls_in_four_directions():
    m = room()
    x, y = 5.5, 4.5
    angles = np.array([0, np.pi / 2, np.pi, -np.pi / 2])
    d = cast_rays(m, x, y, angles, max_range=50)
    # Right wall starts at x=19, bottom at y=9, left ends at x=1, top ends at y=1.
    assert d == pytest.approx([19 - 5.5, 9 - 4.5, 5.5 - 1, 4.5 - 1])


def test_diagonal_ray_exact_distance():
    m = GridMap.empty(10, 10, border=False)
    m.occ[5, 5] = True
    # From (2, 2) at 45 degrees, the ray enters cell (5, 5) at (5, 5).
    d = cast_rays(m, 2.0, 2.0, np.pi / 4, max_range=50)
    assert float(d) == pytest.approx(3 * np.sqrt(2))


def test_ray_respects_max_range():
    m = room()
    d = cast_rays(m, 2.5, 4.5, 0.0, max_range=5.0)
    assert float(d) == pytest.approx(5.0)


def test_ray_starting_inside_wall_is_zero():
    m = room()
    assert float(cast_rays(m, 0.5, 0.5, 0.3, max_range=10)) == 0.0


def test_vectorized_matches_single_rays():
    m = room()
    m.occ[3:6, 8:10] = True
    rng = np.random.default_rng(0)
    xs, ys = rng.uniform(1, 7, 50), rng.uniform(1, 8, 50)
    ang = rng.uniform(-np.pi, np.pi, 50)
    batch = cast_rays(m, xs, ys, ang, 30)
    singles = [float(cast_rays(m, x, y, a, 30)) for x, y, a in zip(xs, ys, ang)]
    assert batch == pytest.approx(singles)


def test_lidar_scan_noise_is_reproducible_and_small():
    m = room()
    lidar = Lidar(n_beams=8, max_range=30, noise_std=0.1)
    pose = np.array([5.5, 4.5, 0.0])
    clean = lidar.expected(m, pose)[0]
    s1 = lidar.scan(m, pose, np.random.default_rng(1))
    s2 = lidar.scan(m, pose, np.random.default_rng(1))
    assert np.array_equal(s1, s2)
    assert np.abs(s1 - clean).max() < 0.5
