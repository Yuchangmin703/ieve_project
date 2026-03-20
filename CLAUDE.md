# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

1/10 scale autonomous racing vehicle built on **ROS 2 Humble**. The system runs on a Jetson with an ESP32 handling low-level motor/servo control via serial.

## Answer

You must think step-by-step and answer in Korean. You should answer detailed.

## Basic HW information

It is based on f1tenth car with ESP32 & MD20a motor driver.

## Build & Run

```bash
# Build all packages
colcon build --symlink-install

# Source workspace (Jetson uses zsh)
source install/setup.zsh
# or for bash:
source install/setup.bash

# Build a single package
colcon build --symlink-install --packages-select control
```

### Running the Full System (6 terminals)

```bash
# T1: Camera perception pipeline
ros2 launch perception perception.launch.py

# T2: LiDAR object tracking
ros2 run perception_lidar perception_node8

# T3: Path planning
ros2 run planning planning_node

# T4: MPC/Pure Pursuit control
ros2 run control control_node2

# T5: Joystick + drive mux
ros2 run joy joy_node &
ros2 run control joy_drive_node &
ros2 run control drive_mux_node

# T6: ESP32 serial bridge
ros2 run serial_bridge serial_node2
```

## Architecture

```
Camera → undistort → bev_warp → lane_mask → centerline_extractor
                                                    ↓
LiDAR → perception_node8 ──────────────→ planning_node
                                                    ↓
                                           /planning/local_path
                                                    ↓
                    joy_drive_node → drive_mux_node ← control_node2
                                         ↓
                                    serial_node2 → ESP32
                                         ↑
                                    /ego_speed (velocity feedback)
```

### Packages

| Package | Language | Purpose |
|---------|----------|---------|
| `perception` | C++ | Camera pipeline: undistortion, BEV warp, lane detection, centerline extraction |
| `perception_lidar` | C++ | LiDAR ROI filtering, object tracking with moving average |
| `planning` | C++ | Trajectory generation, lane-change decisions, obstacle avoidance |
| `control` | C++ | Pure Pursuit with dynamic lookahead, drive mux, joystick control |
| `serial_bridge` | Python | ESP32 UART gateway (`/dev/ttyCH341USB0` at 115200 baud) |

### Key Topics

- `/perception/lanes` (custom `Lanes.msg`) — detected lane centerlines
- `/perception/tracked_objects` (`PoseArray`) — LiDAR-tracked obstacles
- `/planning/local_path` (`nav_msgs/Path`) — planned trajectory with speed profile
- `/auto_drive` (`AckermannDriveStamped`) — autonomous drive commands
- `/joy_cmd` (`AckermannDriveStamped`) — manual joystick commands
- `/ego_speed` (`Float32`) — current vehicle speed from ESP32 encoder

### Custom Messages

Defined in `src/perception/msg/`:
- `Lane.msg` — array of geometry_msgs/Point
- `Lanes.msg` — header + array of Lane

## Key Parameters

- **Lookahead distance**: 0.28m–1.5m (speed-adaptive)
- **Steering limit**: ±30°
- **Max cruise speed**: 4.0 m/s (planning), 0.4 m/s (control output limit)
- **Serial rate**: 50 Hz with watchdog timer
- **LiDAR filter**: 15-frame warm-up, adaptive distance thresholds

## Camera Configuration

Camera calibration files in `src/perception/config/`:
- `hw40_params.yaml` — HW40 camera intrinsics + BEV transform matrix
- `usb2_params.yaml` — USB 2.0 camera alternative

The perception launch file auto-detects which camera is connected.

## Alternative Nodes

Some packages have alternative implementations for experimentation:
- `planning_node` vs `JHplanning_node` (Frenet frame variant)
- `control_node` vs `control_node2` (different MPC tunings)
- `perception_node7` vs `perception_node8` (different tracking approaches)
