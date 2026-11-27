# Installation

Validated on **Ubuntu 22.04.5 (jammy)**, **ROS 2 Humble**, **Gazebo Sim 8.10.0 (Harmonic)**
and **PX4 SITL**.

## 1. System packages

```bash
sudo apt update
sudo apt install ros-humble-ros-gz \
                 ros-humble-ros-gz-bridge \
                 ros-humble-ros-gz-interfaces \
                 ros-humble-ros-gz-sim
```

`ros_gz_sim` is **not** pulled in by `ros-humble-ros-gz` alone; its absence is the first
error most people hit (`gz sim` integration missing).

## 2. Build the workspace

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 3. Run the simulation

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/sim/plugins/build
gz sim -r -v 4 sim/worlds/x500_world.sdf
```

Second terminal:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=$PWD/sim/bridge/x500_bridge.yaml
```

Checks:

```bash
ros2 topic hz /x500/imu
ros2 topic echo /x500/scan --once
```

## Known pitfalls (all fixed in this repository)

1. **Motor commands land on the wrong model.** With several models loaded, a motor plugin
   without `<robotNamespace>` inherits the namespace Gazebo registered last, e.g.
   `/x500_lidar_2d/command/motor_speed`. The drone then never reacts to `/x500/cmd_vel`.
   Fix: an explicit `<robotNamespace>x500</robotNamespace>` on each of the four motor
   plugins.
2. **The bridge silently ignores topics.** On Humble, `topic_name:` alone does nothing —
   both `ros_topic_name:` and `gz_topic_name:` are required. The short form is Jazzy only.
3. **`MotorFailurePlugin` does not exist** in a standalone Gazebo Harmonic installation;
   remove it from the model.
4. **Version mismatch between `ros_gz_interfaces` and Gazebo** produces
   `Failed to load system plugin`.

## 4. PX4 SITL path

```bash
MicroXRCEAgent udp4 -p 8888                 # terminal 1
ros2 run x500_navigation offboard_demo      # terminal 2
```

Port 8888 is the PX4 default (`uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888`).
PX4 works in NED: climbing means a **negative** z, hence the hover altitude `-2.0` m.

### Troubleshooting the PX4 SITL path

Tested setup: Ubuntu 22.04.5, ROS 2 Humble, Gazebo Harmonic 8.10.0, PX4 SITL
built from the `px4_sitl_default` target with the `uxrce_dds_client` module.
The demo uses the `/fmu/out/vehicle_local_position_v1` and
`/fmu/out/vehicle_status_v2` topic names, which are the PX4 v1.14+ naming —
on an older release the names differ and the demo will wait for telemetry
forever. Confirm the exact PX4 release you run; the steps below assume the
`px4_sitl_default` build.

**MicroXRCEAgent runs, but no `/fmu/` topics appear.**
Symptom: `ros2 topic list | grep /fmu/` is empty, and the demo prints
`offboard_demo started, waiting for PX4 telemetry…` and stays there.
- *Wrong UDP port.* The agent must listen on the port the PX4 client
  connects to. Both sides default to 8888 (`MicroXRCEAgent udp4 -p 8888`;
  PX4: `uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888`). If you changed
  one side, change the other.
- *PX4 built without the client.* `uxrce_dds_client` must be in the
  `px4_sitl_default` build. In the PX4 shell, `uxrce_dds_client status`
  should print a status; "command not found" means the module is missing —
  rebuild with it enabled.
- *Agent on the wrong host.* The client connects to 127.0.0.1, so the agent
  must run on the same machine as PX4 SITL. If PX4 runs in a container or
  VM, point the client at the agent host's address instead.
- *QoS mismatch when checking.* The demo subscribes with BEST_EFFORT +
  TRANSIENT_LOCAL (`px4_qos()` in `offboard_demo.py`), so `ros2 topic echo`
  with its default reliable QoS may show nothing even when the topics
  exist. Check with `ros2 topic list` first, and echo with
  `--qos-reliability best_effort`.

**Offboard mode is refused or never confirmed.**
Symptom: the demo logs `offboard not confirmed, retrying` every 5 s, or PX4
rejects the mode switch.
Cause: PX4 only accepts OFFBOARD after receiving setpoints at 2 Hz or
faster. The demo streams at 20 Hz (`SETPOINT_RATE`) and waits 2 s in WARMUP
before requesting the mode — that wait is load-bearing, not a delay for
show. A node that sends the mode command first, or streams slower than
2 Hz, is refused.
Fix: publish `TrajectorySetpoint` + `OffboardControlMode` for at least 2 s
before sending the mode command (`MAV_CMD_DO_SET_MODE`, param2 = 6), and
keep streaming while offboard is active. Verify the rate with
`ros2 topic hz /fmu/in/trajectory_setpoint`.

**The drone takes off the moment it is armed.**
Symptom: on arming, the drone climbs instead of holding position.
Cause: it armed while still in position mode (OFFBOARD never confirmed)
with a setpoint far from where it sits — e.g. the hover setpoint
`[0, 0, -2.0]` while the drone is on the ground — so PX4 flies to the
setpoint. Arming before any setpoints are streamed does the same.
Fix: follow the demo's order — stream setpoints, request OFFBOARD, wait
until `nav_state == OFFBOARD`, and only then arm. The demo also seeds the
setpoint's z from the first `VehicleLocalPosition` message, so the first
setpoint means "stay where you are", and only switches to z = -2.0 m in
HOVER. Never arm manually while the demo is still logging
`offboard not confirmed, retrying`.

**EKF2 never gets an origin, so local position stays invalid.**
Symptom: the demo never leaves WAIT_FOR_TELEMETRY, or
`VehicleLocalPosition` never becomes valid (`xy_valid` / `z_valid` false).
Cause: EKF2 sets its origin from the GPS fix. The fix comes from the
simulated GPS (`gz-sim-navsat-system` on the model, bridged as
`/x500/navsat`); if `gz sim` is not running, or the world failed to load,
there is no fix and no origin.
Fix: start `gz sim` with the world first and confirm the model is loaded;
in the PX4 shell check `gps status` (a fix) and `ekf2 status` (origin set).
The demo waits for the first local-position message by design, so starting
it early is harmless — it proceeds once the fix exists.

## 5. Learning side only

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python rl/check_env.py
python rl/train_ppo.py --steps 200000 --n-envs 4
```
