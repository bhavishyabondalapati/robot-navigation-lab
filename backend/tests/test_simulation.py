import numpy as np

from app.grid_map import load_preset
from app.simulation import SimConfig, Simulation


def run(sim: Simulation, max_steps=2000):
    while sim.status == "running" and sim.step_count < max_steps:
        sim.step()
    return sim


def test_robot_reaches_goal_and_stays_localized():
    m, s, g = load_preset("warehouse")
    sim = run(Simulation(m, s, g, SimConfig(planner="astar", seed=1)))
    assert sim.status == "reached"
    assert max(sim.errors) < 0.5
    gx, gy = g[0] + 0.5, g[1] + 0.5
    assert np.hypot(sim.true_pose[0] - gx, sim.true_pose[1] - gy) < 1.0


def test_same_seed_same_run():
    m, s, g = load_preset("open_field")
    a = run(Simulation(m, s, g, SimConfig(planner="rrt", seed=7)), 50)
    b = run(Simulation(m, s, g, SimConfig(planner="rrt", seed=7)), 50)
    assert a.state() == b.state()


def test_replans_when_obstacle_blocks_path():
    m, s, g = load_preset("open_field")
    sim = Simulation(m, s, g, SimConfig(planner="astar", seed=0))
    for _ in range(5):
        sim.step()
    old_path = list(sim.plan_result.path)
    # Drop a wall on a path cell a few waypoints ahead of the robot.
    x, y = old_path[sim.waypoint + 6]
    replanned = sim.set_cell(int(x), int(y), True)
    assert replanned
    assert sim.replans == 1
    assert sim.plan_result.success
    assert (int(x) + 0.5, int(y) + 0.5) not in sim.plan_result.path
    run(sim)
    assert sim.status == "reached"


def test_no_path_status_when_goal_is_walled_off():
    m, s, g = load_preset("open_field")
    gx, gy = g
    m.occ[gy - 1:gy + 2, gx - 1:gx + 2] = True
    m.occ[gy, gx] = False
    sim = Simulation(m, s, g)
    assert sim.status == "no_path"
