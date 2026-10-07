"""Compare A* and RRT on every preset map and print a Markdown table.

Run from the project root:
    python scripts/benchmark.py

A* is deterministic, so it runs once per map. RRT is random, so it runs
with several fixed seeds and we report the mean (and the success rate).
Planning time is the median of repeated runs to smooth out timer noise.
"""

from __future__ import annotations

import statistics
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "backend"))

from app.grid_map import PRESETS, load_preset  # noqa: E402
from app.planners import astar, rrt  # noqa: E402

RRT_SEEDS = range(10)
TIMING_REPEATS = 5


def bench_astar(m, s, g):
    times = []
    for _ in range(TIMING_REPEATS):
        res = astar(m, s, g)
        times.append(res.planning_time_ms)
    return {"success": 1.0 if res.success else 0.0, "length": res.path_length,
            "nodes": res.nodes_expanded, "time": statistics.median(times)}


def bench_rrt(m, s, g):
    runs = [rrt(m, s, g, rng=np.random.default_rng(seed)) for seed in RRT_SEEDS]
    ok = [r for r in runs if r.success]
    return {
        "success": len(ok) / len(runs),
        "length": statistics.mean(r.path_length for r in ok) if ok else float("nan"),
        "nodes": statistics.mean(r.nodes_expanded for r in runs),
        "time": statistics.median(r.planning_time_ms for r in runs),
    }


def main():
    print("| Map | Planner | Success | Path length (cells) | Nodes expanded | Planning time (ms) |")
    print("|---|---|---|---|---|---|")
    for name in sorted(PRESETS):
        m, s, g = load_preset(name)
        for label, fn in (("A*", bench_astar), (f"RRT (mean of {len(RRT_SEEDS)} seeds)", bench_rrt)):
            r = fn(m, s, g)
            print(f"| {name} | {label} | {r['success']:.0%} | {r['length']:.1f} | {r['nodes']:.0f} | {r['time']:.1f} |")


if __name__ == "__main__":
    main()
