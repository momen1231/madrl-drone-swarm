#!/usr/bin/env python3
"""Sanity check: run a random rollout in the swarm environment (numpy only)."""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))

from envs.drone_swarm_env import DroneSwarmEnv, EnvConfig  # noqa: E402
from worlds import load_world  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Sanity check the swarm environment.")
    parser.add_argument(
        "--world",
        default=None,
        help="YAML world file with an 'obstacles' list, e.g. "
        "rl/configs/worlds/rooms.yaml (default: random obstacles)",
    )
    args = parser.parse_args()

    if args.world is None:
        config = EnvConfig(n_agents=3, seed=0)
    else:
        # A named world replaces random sampling. n_obstacles=0 so an
        # explicitly empty world stays empty instead of sampling.
        config = EnvConfig(
            n_agents=3, seed=0, n_obstacles=0, obstacles=load_world(args.world)
        )
    env = DroneSwarmEnv(config)
    obs, info = env.reset(seed=0)
    print(f"obs shape {obs.shape} (expected (3, {env.obs_dim}))")
    assert obs.shape == (3, env.obs_dim), "observation dimension mismatch"

    total = np.zeros(3)
    step = 0
    for step in range(env.cfg.horizon):
        actions = env.rng.uniform(-1.0, 1.0, (3, env.act_dim))
        obs, rewards, terminated, truncated, info = env.step(actions)
        total += rewards
        assert np.all(np.isfinite(obs)), "non-finite observation"
        if terminated.all() or truncated.all():
            break

    print(f"steps {step + 1} | cumulative reward {total.round(2)} | reached {info['reached']}")
    print("env OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
