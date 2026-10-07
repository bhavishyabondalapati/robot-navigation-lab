import numpy as np

from app.grid_map import load_preset
from app.motion import MotionNoise, apply_motion
from app.particle_filter import ParticleFilter
from app.sensor import Lidar


def run_filter(seed: int, steps: int = 40):
    """Drive a robot in a loop and return the position error after each step."""
    m, _, _ = load_preset("open_field")
    lidar = Lidar(n_beams=16, max_range=10, noise_std=0.1)
    world = np.random.default_rng(seed)
    pf = ParticleFilter(m, lidar, n_particles=400, rng=np.random.default_rng(seed + 100))

    true = np.array([6.5, 6.5, 0.0])
    # Start the filter with a WRONG guess (2 cells off) and a wide spread.
    pf.init_gaussian(true + [2.0, -1.5, 0.3], [2.0, 2.0, 0.4])
    errors = [np.hypot(*(pf.estimate()[:2] - true[:2]))]
    for k in range(steps):
        rot, dist = (0.15, 0.3) if k % 10 else (0.6, 0.0)
        true = apply_motion(true, rot, dist, MotionNoise(), world)[0]
        scan = lidar.scan(m, true, world)
        est = pf.step(rot, dist, scan)
        errors.append(np.hypot(*(est[:2] - true[:2])))
    return np.array(errors)


def test_error_shrinks_over_steps():
    errors = run_filter(seed=0)
    assert errors[0] > 1.5
    assert errors[-10:].mean() < 0.4
    assert errors[-1] < errors[0] / 4


def test_filter_is_reproducible():
    assert np.array_equal(run_filter(seed=3, steps=15), run_filter(seed=3, steps=15))


def test_resample_keeps_count_and_resets_weights():
    m, _, _ = load_preset("open_field")
    pf = ParticleFilter(m, Lidar(), n_particles=100, rng=np.random.default_rng(0))
    pf.init_gaussian([10, 10, 0], [1, 1, 0.1])
    pf.weights = np.zeros(100)
    pf.weights[7] = 1.0                       # one particle has all the weight
    pf.resample()
    assert pf.particles.shape == (100, 3)
    assert np.allclose(pf.weights, 0.01)
    # Every new particle is a (jittered) copy of particle 7.
    assert np.abs(pf.particles[:, :2] - pf.particles[:, :2].mean(0)).max() < 0.5


def test_estimate_averages_angles_correctly():
    m, _, _ = load_preset("open_field")
    pf = ParticleFilter(m, Lidar(), n_particles=2)
    pf.particles = np.array([[5, 5, np.pi - 0.1], [5, 5, -np.pi + 0.1]])
    pf.weights = np.array([0.5, 0.5])
    assert abs(abs(pf.estimate()[2]) - np.pi) < 1e-9
