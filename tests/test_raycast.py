"""Parametrised geometry tests for DroneSwarmEnv._raycast (issue #4).

    python -m pytest tests/test_raycast.py -q

Contract under test — documented here because the environment never spawns
an agent inside an obstacle, so the in-obstacle behaviour is a defensive
contract, not a gameplay one:

* a ray that hits nothing returns ``ray_range`` (the reading is capped);
* an origin *inside* a circle returns the distance to the *far*
  intersection, i.e. the distance to the exit — never a negative value;
* a ray tangent to a circle (``disc == 0``) returns the single touch point.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "rl"))

from envs.drone_swarm_env import DroneSwarmEnv, EnvConfig  # noqa: E402


def make_env(obstacles) -> DroneSwarmEnv:
    config = EnvConfig(seed=0, n_obstacles=0, obstacles=list(obstacles))
    env = DroneSwarmEnv(config)
    env._build_obstacles()  # noqa: SLF001 - geometry check on purpose
    return env


def raycast(obstacles, origin, angle) -> float:
    env = make_env(obstacles)
    return env._raycast(np.array(origin, dtype=float), angle)  # noqa: SLF001


@pytest.mark.parametrize(
    "obstacles, origin, angle",
    [
        # No obstacles at all.
        ([], (0.0, 5.0), 0.0),
        # The surface is at 4.5 m, past ray_range = 3.0 m.
        ([(5.0, 5.0, 0.5)], (0.0, 5.0), 0.0),
        # The only circle is behind the ray (both roots negative).
        ([(2.0, 5.0, 0.5)], (0.0, 5.0), math.pi),
        # The ray misses the circle sideways (disc < 0).
        ([(2.0, 7.0, 0.5)], (0.0, 5.0), 0.0),
        # A near miss: the ray passes 0.1 m outside the circle (disc < 0).
        ([(2.0, 5.6, 0.5)], (0.0, 5.0), 0.0),
    ],
    ids=["no-obstacles", "beyond-range", "behind-ray", "sideways-miss", "near-miss"],
)
def test_raycast_returns_ray_range_when_nothing_is_hit(obstacles, origin, angle):
    assert raycast(obstacles, origin, angle) == pytest.approx(EnvConfig().ray_range)


@pytest.mark.parametrize(
    "obstacles, origin, angle, expected",
    [
        # Origin at the centre: exit is one radius away in any direction.
        ([(5.0, 5.0, 0.5)], (5.0, 5.0), 0.0, 0.5),
        ([(5.0, 5.0, 0.5)], (5.0, 5.0), math.pi, 0.5),
        # Origin 0.2 m off-centre: exit at 5.5 m is 0.3 m away, at 4.5 m is 0.7 m.
        ([(5.0, 5.0, 0.5)], (5.2, 5.0), 0.0, 0.3),
        ([(5.0, 5.0, 0.5)], (5.2, 5.0), math.pi, 0.7),
        ([(5.0, 5.0, 0.5)], (5.3, 5.0), 0.0, 0.2),
    ],
    ids=["centre-east", "centre-west", "offset-east", "offset-west", "offset-east-2"],
)
def test_raycast_from_inside_a_circle_returns_exit_distance(
    obstacles, origin, angle, expected
):
    # Contract: inside an obstacle the raycast reports the distance to the
    # exit (far intersection), never a negative value.
    result = raycast(obstacles, origin, angle)
    assert result == pytest.approx(expected)
    assert result > 0.0


@pytest.mark.parametrize(
    "obstacles, origin, angle, expected",
    [
        # Tangent from above: the ray touches (2.0, 5.0), disc == 0.
        ([(2.0, 5.5, 0.5)], (0.0, 5.0), 0.0, 2.0),
        # Tangent from below.
        ([(2.0, 4.5, 0.5)], (0.0, 5.0), 0.0, 2.0),
        # Tangent point beyond ray_range: the touch at 5.0 m is capped to 3.0 m.
        ([(5.0, 5.5, 0.5)], (0.0, 5.0), 0.0, 3.0),
    ],
    ids=["tangent-above", "tangent-below", "tangent-beyond-range"],
)
def test_raycast_grazing_a_circle(obstacles, origin, angle, expected):
    assert raycast(obstacles, origin, angle) == pytest.approx(expected)


@pytest.mark.parametrize(
    "obstacles, origin, angle, expected",
    [
        # Straight hit: surface at x = 1.5.
        ([(2.0, 5.0, 0.5)], (0.0, 5.0), 0.0, 1.5),
        # Diagonal hit: centre at distance 2*sqrt(2), minus the radius.
        ([(2.0, 2.0, 0.5)], (0.0, 0.0), math.pi / 4, 2.0 * math.sqrt(2.0) - 0.5),
        # Two circles on the ray: the nearer one wins, whatever the list order.
        ([(2.0, 5.0, 0.5), (4.0, 5.0, 0.5)], (0.0, 5.0), 0.0, 1.5),
        ([(4.0, 5.0, 0.5), (2.0, 5.0, 0.5)], (0.0, 5.0), 0.0, 1.5),
    ],
    ids=["straight-hit", "diagonal-hit", "nearest-wins", "nearest-wins-reversed"],
)
def test_raycast_returns_nearest_hit(obstacles, origin, angle, expected):
    assert raycast(obstacles, origin, angle) == pytest.approx(expected)
