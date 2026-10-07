import math

import numpy as np
import pytest

from app.grid_map import GridMap, load_preset, PRESETS
from app.planners import astar, rrt

SQRT2 = math.sqrt(2)


def grid_from(text: str) -> GridMap:
    """Build a map from ASCII art: '#' = wall, '.' = free."""
    rows = [line.strip() for line in text.strip().splitlines()]
    return GridMap.from_list([[c == "#" for c in row] for row in rows])


def blocked_map():
    # A solid wall splits the map in two: no path from left to right.
    return grid_from("""
        .....#.....
        .....#.....
        .....#.....
        .....#.....
    """)


# ----- A* -----------------------------------------------------------------

def test_astar_straight_line_is_optimal():
    m = GridMap.empty(10, 5, border=False)
    res = astar(m, (0, 2), (9, 2))
    assert res.success
    assert res.path_length == pytest.approx(9.0)
    assert len(res.path) == 10


def test_astar_diagonal_is_optimal():
    m = GridMap.empty(6, 6, border=False)
    res = astar(m, (0, 0), (5, 3))
    # 3 diagonal steps + 2 straight steps is the cheapest 8-connected route.
    assert res.success
    assert res.path_length == pytest.approx(3 * SQRT2 + 2)


def test_astar_goes_around_wall_optimally():
    m = grid_from("""
        .....
        .###.
        .....
    """)
    res = astar(m, (0, 1), (4, 1))
    # Diagonals that would clip the wall's corners are forbidden, so the best
    # route is: up 1, right 4, down 1 = 6.
    assert res.success
    assert res.path_length == pytest.approx(6.0)
    for x, y in res.path:
        assert m.is_free_point(x, y)


def test_astar_no_corner_cutting():
    m = grid_from("""
        .#
        #.
    """)
    # The only connection is diagonally between two walls: not allowed.
    assert not astar(m, (0, 0), (1, 1)).success


def test_astar_reports_no_path_when_blocked():
    res = astar(blocked_map(), (0, 0), (10, 3))
    assert not res.success
    assert res.path == []
    assert res.nodes_expanded > 0


def test_astar_start_or_goal_in_wall():
    m = blocked_map()
    assert not astar(m, (5, 0), (0, 0)).success
    assert not astar(m, (0, 0), (5, 1)).success


def brute_force_cost(m: GridMap, start, goal) -> float:
    """Dijkstra without a heuristic: the reference answer for optimality."""
    import heapq
    from app.planners import MOVES
    dist = {start: 0.0}
    heap = [(0.0, start)]
    while heap:
        d, cur = heapq.heappop(heap)
        if cur == goal:
            return d
        if d > dist[cur]:
            continue
        for dx, dy, c in MOVES:
            nxt = (cur[0] + dx, cur[1] + dy)
            if not m.is_free(*nxt):
                continue
            if dx and dy and not (m.is_free(cur[0] + dx, cur[1]) and m.is_free(cur[0], cur[1] + dy)):
                continue
            if d + c < dist.get(nxt, math.inf):
                dist[nxt] = d + c
                heapq.heappush(heap, (d + c, nxt))
    return math.inf


@pytest.mark.parametrize("seed", range(5))
def test_astar_matches_dijkstra_on_random_grids(seed):
    rng = np.random.default_rng(seed)
    m = GridMap(rng.random((12, 12)) < 0.25)
    m.occ[0, 0] = m.occ[11, 11] = False
    res = astar(m, (0, 0), (11, 11))
    expected = brute_force_cost(m, (0, 0), (11, 11))
    if math.isinf(expected):
        assert not res.success
    else:
        assert res.success
        assert res.path_length == pytest.approx(expected)


# ----- RRT ----------------------------------------------------------------

def test_rrt_finds_path_in_open_map():
    m = GridMap.empty(20, 20)
    res = rrt(m, (2, 2), (17, 17), rng=np.random.default_rng(0))
    assert res.success
    assert res.path[0] == pytest.approx((2.5, 2.5))
    assert res.path[-1] == pytest.approx((17.5, 17.5))
    # Every edge of the final path must be collision-free.
    for a, b in zip(res.path, res.path[1:]):
        assert m.segment_free(a, b)


def test_rrt_reports_no_path_when_blocked():
    res = rrt(blocked_map(), (0, 0), (10, 3), rng=np.random.default_rng(0), max_iters=2000, clearance=0)
    assert not res.success
    assert res.path == []


def test_rrt_is_reproducible_with_seed():
    m, s, g = load_preset("warehouse")
    a = rrt(m, s, g, rng=np.random.default_rng(5))
    b = rrt(m, s, g, rng=np.random.default_rng(5))
    assert a.path == b.path


def test_rrt_never_shorter_than_astar():
    # On a grid, A* (8-connected) isn't the true continuous optimum, but RRT's
    # wandering paths should still not beat it by much on open maps.
    m, s, g = load_preset("open_field")
    a = astar(m, s, g)
    r = rrt(m, s, g, rng=np.random.default_rng(1))
    assert r.success and a.success
    assert r.path_length > 0.9 * a.path_length


@pytest.mark.parametrize("name", sorted(PRESETS))
def test_both_planners_solve_every_preset(name):
    m, s, g = load_preset(name)
    assert astar(m, s, g).success
    assert rrt(m, s, g, rng=np.random.default_rng(0)).success
