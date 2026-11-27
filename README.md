# MADRL Drone Swarm — Autonomous Navigation for an X500 Quadrotor

[![CI](https://github.com/momen1231/madrl-drone-swarm/actions/workflows/ci.yml/badge.svg)](https://github.com/momen1231/madrl-drone-swarm/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Reference implementation and research toolkit for my final-year project (PFA):
**Multi-Agent Deep Reinforcement Learning for autonomous navigation of a drone swarm**,
developed on the **X500** quadrotor model with **ROS 2 Humble**, **Gazebo Sim (Harmonic)**
and **PX4 SITL**.

![Swarm demo](docs/media/demo.gif)

*The GIF is a replay of the kinematic model in `rl/envs/drone_swarm_env.py` drawn by
`scripts/render_demo.mjs` with a scripted goal-seeking controller — it is **not** a Gazebo
capture and **not** a trained policy. It exists so the observation/action contract is
visible without installing ROS.*

The repository has three layers:

| Layer | Folder | What it does |
|---|---|---|
| Simulation | `sim/` | X500 SDF model, Gazebo world, `ros_gz_bridge` configuration |
| Classical control | `ros2_ws/` | ROS 2 nodes: square trajectory without odometry, PX4 offboard demo |
| Learning | `rl/` | Gymnasium environment + PPO / SAC / MAPPO training scripts |

## Why this project

Most open-source drone-learning code is either a pure Python toy environment with no
robotics stack, or a full autopilot stack with no learning baseline. This project keeps
the two sides separable: the same observation/action contract (`rl/envs/drone_swarm_env.py`)
is used by the fast kinematic environment during training and by the ROS 2 / Gazebo
pipeline during evaluation.

## Quick start (no ROS needed)

```bash
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python rl/check_env.py                              # contract + random rollout
python rl/check_env.py --world rl/configs/worlds/rooms.yaml   # same check in a named world
python -m pytest tests -q                           # unit tests
python rl/train_ppo.py --steps 200000 --n-envs 4
```

The training scripts write TensorBoard logs and checkpoints to `runs/`.

## Full stack (ROS 2 Humble + Gazebo + PX4)

See [`docs/installation.md`](docs/installation.md) for the complete setup, the known
pitfalls (SDF `robotNamespace`, Humble bridge syntax, `ros_gz_sim` missing package) and
the exact commands.

```bash
# 1. Gazebo world with the X500
gz sim -r -v 4 sim/worlds/x500_world.sdf

# 2. Bridge Gazebo <-> ROS 2
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=$PWD/sim/bridge/x500_bridge.yaml

# 3. Square trajectory without odometry
ros2 run x500_navigation square_no_odom
```

PX4 SITL path (offboard control):

```bash
MicroXRCEAgent udp4 -p 8888      # terminal 1
ros2 run x500_navigation offboard_demo   # terminal 2
```

## Repository layout

```
sim/models/x500/model.sdf        X500 model with per-plugin <robotNamespace>
sim/worlds/x500_world.sdf        Empty world + model include + pose publisher
sim/bridge/x500_bridge.yaml      IMU, NavSat, 2D LiDAR, air pressure, pose -> ROS 2
ros2_ws/src/x500_navigation/     square_no_odom.py, offboard_demo.py
rl/envs/drone_swarm_env.py       Multi-agent kinematic environment (Gymnasium API)
rl/train_ppo.py                  Single-agent PPO baseline (stable-baselines3)
rl/train_sac.py                  Single-agent SAC baseline (stable-baselines3)
rl/train_mappo.py                Shared-parameter MAPPO (torch)
tests/                           pytest suite for the environment contract
scripts/render_demo.mjs          Offline renderer for docs/media/demo.gif
docs/                            Architecture, installation, experiments
```

## Results

Measured on the lab machine (Ubuntu 22.04, Python 3.10). Success = the agent reaches its
goal inside `goal_tolerance`, greedy policy on the kinematic environment. Checkpoints are
not committed; the numbers below are there to be reproduced, not trusted.

| Run | Algorithm | Success rate | Note |
|---|---|---|---|
| stage 1 | SAC | **92 %** | single-agent wrapper, the best result so far |
| v1 | MAPPO | **42 %** | first MAPPO run that learned something usable |
| v2 | MAPPO | **8 %** | same task, changed training configuration — success collapsed |
| v3 | MAPPO | *pending* | re-run of the v1 setup; the number is being re-measured, not estimated here |

What these numbers actually taught me:

* **A success rate is a cliff, not a slope.** 42 % → 8 % happened without touching the
  environment, only the training configuration. When a metric moves that violently the
  policy is sitting on a knife edge, and the 92 % of SAC and the 8 % of MAPPO are not two
  points on one curve — they are two different regimes.
* **Single-agent SAC beat multi-agent MAPPO at this budget.** Three agents sharing one
  actor is a harder optimisation problem than one agent with its own; at a few hundred
  thousand steps the shared policy has not seen enough per-agent data to catch up. That is
  a sample-budget statement, not a claim that MAPPO is worse in general.
* **The failed run is the informative one.** Keeping v2 in the table makes the load-bearing
  knobs visible; a README that only shows 92 % hides exactly the part someone else will get
  wrong.
* **Classical control still carries the demo.** The square trajectory closes without any
  odometry source (`docs/experiments.md`, E1); the learning layer is evaluated against that
  baseline, not in isolation.

Design decisions behind the environment — reward shaping, the spinning reward hack, the
corner-trapping failure mode — are written up in [`docs/experiments.md`](docs/experiments.md).

## Contributing

Issues labelled [`good first issue`](https://github.com/momen1231/madrl-drone-swarm/labels/good%20first%20issue)
are scoped to be finishable without ROS 2 or Gazebo installed. Read
[`CONTRIBUTING.md`](CONTRIBUTING.md) before opening a pull request.

## Citation

If you use this code in academic work, cite it via [`CITATION.cff`](CITATION.cff) (GitHub
shows a "Cite this repository" button on the sidebar).

## License

MIT — see [`LICENSE`](LICENSE).
