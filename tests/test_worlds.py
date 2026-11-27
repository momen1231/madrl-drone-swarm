"""Tests for YAML world files (rl/configs/worlds/*.yaml).

    python -m pytest tests/test_worlds.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "rl"))

from envs.drone_swarm_env import DroneSwarmEnv, EnvConfig  # noqa: E402
from worlds import load_world  # noqa: E402

WORLDS_DIR = Path(__file__).resolve().parents[1] / "rl" / "configs" / "worlds"


def test_load_four_obstacles_world():
    obstacles = load_world(WORLDS_DIR / "four_obstacles.yaml")

    assert len(obstacles) == 4
    assert all(len(obstacle) == 3 for obstacle in obstacles)


def test_loaded_world_gives_expected_obstacle_count_and_finite_obs():
    env = DroneSwarmEnv(
        EnvConfig(
            n_agents=3,
            seed=0,
            n_obstacles=0,
            obstacles=load_world(WORLDS_DIR / "four_obstacles.yaml"),
        )
    )
    obs, _ = env.reset(seed=0)

    assert env.obstacles.shape == (4, 3)
    assert np.all(np.isfinite(obs))

    for _ in range(50):
        obs, _, _, _, _ = env.step(env.rng.uniform(-1.0, 1.0, (env.n, env.act_dim)))
        assert np.all(np.isfinite(obs))


def test_empty_world_has_no_obstacles():
    assert load_world(WORLDS_DIR / "empty.yaml") == []

    env = DroneSwarmEnv(
        EnvConfig(
            n_agents=3,
            seed=0,
            n_obstacles=0,
            obstacles=load_world(WORLDS_DIR / "empty.yaml"),
        )
    )
    obs, _ = env.reset(seed=0)

    assert env.obstacles.shape == (0, 3)
    assert np.all(np.isfinite(obs))
