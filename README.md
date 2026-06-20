# hexapod_stack

ROS 2 C++ hexapod movement stack for a 6-legged robot with Dynamixel AX-18A servos, controlled via `cmd_vel` (teleop keyboard).

## Architecture

```
cmd_vel (Twist)
    │
    ▼
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│ control_node │────▶│ joint_targets│────▶│  servo_node  │──▶ servos
│  (gait + IK) │     │ (JointState) │     │ (hardware)   │
└──────────────┘     └──────────────┘     └──────────────┘
                            │
                            ▼
                      /joint_states (JointState)
```

| Node | Package | Role |
|------|---------|------|
| `control_node` | hexapod_control | Subscribes `/cmd_vel`, runs wave gait + 6 IK solvers, publishes `/joint_targets` (18 joints) |
| `servo_node` | hexapod_servo | Subscribes `/joint_targets`, writes to servos via Dynamixel SDK (sync write), publishes `/joint_states` |
| `ik_node` | hexapod_ik | Standalone single-leg IK: subscribes `/foot_target` (PointStamped), publishes `/joint_targets` |

## Packages

| Package | Description |
|---------|-------------|
| `hexapod_servo` | Servo driver class + ROS node. Handles Dynamixel AX-18A communication via U2D2. |
| `hexapod_msgs` | Shared message definitions (empty for now — using std JointState). |
| `hexapod_ik` | Analytical IK solver (law of cosines) for a 3-DOF leg. No ROS dependency. |
| `hexapod_gait` | Wave gait generator. 48-step cycle, configurable lift height and step time. |
| `hexapod_control` | Orchestrator node: cmd_vel → gait → IK → /joint_targets for all 6 legs. |
| `hexapod_bringup` | Launch files, YAML configs, per-leg test configs. |

## Hardware

- **Servos:** 18× Dynamixel AX-18A (3 per leg)
- **Controller:** U2D2 USB adapter, 1 Mbps, Protocol 1.0
- **Port:** `/dev/ttyUSB0`

### Servo ID mapping

| Leg | Coxa | Femur | Tibia | Sign (C/F/T) |
|-----|------|-------|-------|---------------|
| Left Front (LF) | 4 | 5 | 6 | +1 / +1 / -1 |
| Left Middle (LM) | 13 | 14 | 15 | +1 / +1 / -1 |
| Left Rear (LR) | 1 | 2 | 3 | +1 / +1 / -1 |
| Right Front (RF) | 7 | 8 | 9 | -1 / -1 / +1 |
| Right Middle (RM) | 16 | 17 | 18 | -1 / -1 / +1 |
| Right Rear (RR) | 10 | 11 | 12 | -1 / -1 / +1 |

## Prerequisites

```bash
# ROS 2 Humble on Ubuntu 22.04
source /opt/ros/humble/setup.bash

# Dynamixel SDK (should be in your workspace)
# ls ros2_ws/src/dynamixel_sdk/

# yaml-cpp
sudo apt install libyaml-cpp-dev

# teleop keyboard
sudo apt install ros-humble-teleop-twist-keyboard
```

## Build

```bash
cd ros2_ws
colcon build --packages-select \
  hexapod_servo hexapod_msgs hexapod_ik \
  hexapod_gait hexapod_control hexapod_bringup

source install/setup.bash
```

## Usage

### Full 6-leg teleop

```bash
ros2 launch hexapod_bringup reignblaze.launch.py
```

Then in the teleop terminal:
- `i` = forward, `k` = backward
- `j` = strafe left, `l` = strafe right
- `u` = rotate left, `o` = rotate right
- `q` = quit

### Single-leg IK test

```bash
# Test individual legs (e.g. Left Rear)
ros2 launch hexapod_bringup test_LR.launch.py
ros2 topic pub --once /foot_target geometry_msgs/PointStamped \
  "{header: {frame_id: 'world'}, point: {x: 0.02, y: 0.08, z: -0.08}}"
```

Available per-leg configs: `test_LR`, `test_LM`, `test_LF`, `test_RR`, `test_RM`, `test_RF`.

## Configuration

All config is in `hexapod_bringup/config/reignblaze.yaml`:

```yaml
hexapod_servo:
  port: "/dev/ttyUSB0"
  baud_rate: 1000000
  protocol_version: 1.0
  servos:
    coxa_joint_LR: { id: 1, type: "AX-18A", ticks: 1024, center: 512, max_radians: 5.236, sign: 1, offset: 0.00614 }
    # ... (see full file for all 18 servos)

hexapod_ik:
  coxa_length: 0.044     # meters
  femur_length: 0.0545   # meters
  tibia_length: 0.1019   # meters

hexapod_gait:
  cycle_length: 48       # steps per full cycle
  lift_height: 0.03      # meters
  step_time: 0.05        # seconds per step

hexapod_control:
  control_loop_rate: 500  # Hz
  joint_names:
    LF: ["coxa_joint_LF", "femur_joint_LF", "tibia_joint_LF"]
    # ... (all 6 legs)
```

## Testing

```bash
colcon test --packages-select hexapod_servo hexapod_ik hexapod_gait
colcon test-result --verbose
```

23 unit tests covering:
- Servo driver (radian/tick conversion, multi-servo)
- IK solver (forward/inverse kinematics, reachability)
- Gait generator (step timing, swing/stance phases)

## Design Decisions

- **Wave gait** (not tripod) for easier single-leg debugging
- **Law of cosines IK** — proven analytical solver, ported from ROS 1 stack
- **yaml-cpp** for config loading — ROS param server can't handle nested YAML maps
- **Sync write** for servo communication — single bus packet for all 18 servos eliminates bus contention
- **No URDF/viz** in v1 — pure movement stack
- **Control node doesn't own hardware** — publishes `/joint_targets`, `servo_node` handles servos

## License

MIT
