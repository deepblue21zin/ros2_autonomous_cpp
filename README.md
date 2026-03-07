# ROS2 Autonomous Vehicle

ROS2 Humble based autonomous driving stack built for a student competition vehicle. The repository integrates camera perception, LiDAR-based parking and obstacle handling, a safety-aware decision layer, and an Arduino vehicle interface into one deployable system.

This README is written as a portfolio document as well as a technical entry point. The vehicle has already been submitted, so this document focuses on verifiable repository evidence rather than unverifiable after-the-fact performance claims.

## Project Snapshot

- Domain: ADAS / mobile robotics / embedded vehicle control
- Runtime: Ubuntu 22.04, ROS2 Humble, C++17, Python 3, OpenCV, Eigen
- Repository scale: 8 ROS2 packages, 27 executable nodes, 15 launch files, 13 config files, 6 perception models
- Sensors and actuators: front camera, rear camera, RPLiDAR, 6 ultrasonic sensors, Arduino-based motor and steering controller
- Development evidence: 69 commits in current history, including ROS1 to ROS2 migration, perception tuning, parking, obstacle avoidance, and embedded control updates

## What This Repository Demonstrates

This codebase is useful for ADAS, robotics, and embedded interviews because it proves more than model inference:

- End-to-end system integration from sensor input to vehicle command output
- Real-time ROS2 node composition with launch orchestration, QoS choices, and runtime parameter tuning
- Control-oriented perception for lane tracking, crosswalk handling, stop line handling, and obstacle-aware steering
- Safety-oriented decision logic with stale-sensor timeouts, speed limiting, steering rate limiting, and explicit stop behavior
- Embedded interface work across ROS2, serial protocol design, Arduino firmware, steering feedback, and PID-based steering actuation
- Mission expansion from track driving to obstacle avoidance and LiDAR-based parking

## Architecture Overview

The stack is organized into four layers:

1. Sensor and driver layer  
   `usb_cam_driver`, `rplidar_driver`, `ultrasonic_driver`, `arduino_driver`

2. Perception layer  
   Lane tracking, stop line and traffic-light related perception, obstacle detection, parking line utilities, YOLO-based object modules

3. Decision and control layer  
   Ackermann command generation, obstacle bias fusion, speed scheduling, lane-loss handling, parking state machine

4. Bringup and operations layer  
   Launch files for track driving, mission driving, parking, bag replay, camera test, and hybrid obstacle test

## Package Map

| Package | Role | Main evidence |
|---|---|---|
| `bringup` | System orchestration and deployment entry points | Track, mission, parking, and hybrid launch composition |
| `common` | Debug and monitoring utilities | LiDAR and ultrasonic inspection tools |
| `decision` | Vehicle command generation and mission logic | 20 Hz decision loop, safety gates, parking state machine |
| `perception_pkg` | Lane and object perception | C++ lane tracking, OpenCV preprocessing, Python model integration |
| `arduino_driver` | ROS2 to MCU interface | Serial bridge, Ackermann to PWM/servo mapping |
| `ultrasonic_driver` | Distance post-processing | Min-range extraction for safety logic |
| `rplidar_driver` | LiDAR wrapping | Parking and obstacle scan input |
| `usb_cam_driver` | Camera wrapping | Image transport and camera-format handling |

## Key Engineering Evidence

The strongest hiring signal in this repository is the combination of source code and the change history. These commits show concrete engineering problems being solved over time.

| Commit | Engineering signal |
|---|---|
| `235250d` | Migrated the stack from ROS1 to ROS2 and reorganized it into ROS2 packages, nodes, launch files, and installable build targets |
| `19dbae6` | Reworked lane tracking from a simpler Canny-Hough style approach toward sliding-window based lane detection to improve dashed-lane handling |
| `a137acc` | Refactored lane tracking and introduced PID steering logic tied to Arduino steering feedback |
| `54250c8` | Added a rear-LiDAR-based parking subsystem with a full state machine and dedicated launch/config path |
| `22adfec` | Added obstacle-avoidance driving mode with its own launch flow, tuning guide, and hybrid controller |
| `9137be8` | Replaced a symmetric trapezoid BEV with explicit four-point calibration to handle tilted camera geometry |
| `561c7c1` | Added curvature-based dynamic lookahead and lane-loss speed fixes for more stable closed-loop driving |
| `94412b1` and follow-up camera commits | Solved camera format and resolution issues around MJPEG and YUYV handling, which is a common deployment problem in robotics systems |

## Technical Highlights

### 1. Camera lane tracking was treated as a control problem, not just an image problem

The lane stack combines image preprocessing, region-of-interest control, polynomial fitting, crosswalk filtering, and steering stabilization.

- `src/perception_pkg/src/lane_tracking_node.cpp`
- Sliding window lane detection with adaptive preprocessing
- Support for YUYV and UYVY camera encodings
- Crosswalk density gating to avoid lane corruption in marked zones
- Single-lane hold and lane-loss recovery logic
- Curvature-based dynamic lookahead and coefficient EMA smoothing
- Debug overlay publishing only when needed to avoid unnecessary runtime cost

This matters in interviews because it shows awareness of closed-loop behavior: detection quality is only useful if it produces stable steering.

### 2. Decision logic includes explicit safety handling instead of naive topic forwarding

The decision layer does more than relay lane steering.

- `src/decision/src/decision_node.cpp`
- 20 Hz control loop with speed scheduling
- Steering soft-zone mapping and steering rate limiting
- Sensor freshness checks with fallback behavior for stale obstacle, ultrasonic, traffic-light, and stop-line inputs
- Lane-loss degradation logic instead of uncontrolled failure
- Obstacle-bias fusion for avoidance-oriented steering changes

This is directly relevant to ADAS and robotics roles because it shows practical thinking about fault containment and degraded operation.

### 3. Embedded work spans protocol design, firmware, and shutdown safety

The embedded side is one of the strongest parts of this repository.

- `src/drivers/arduino_driver/scripts/arduino_bridge_node.py`
- `src/drivers/arduino_driver/ino/motor_control_with_feedback_ros2/motor_control_with_feedback_ros2.ino`
- Ackermann command to PWM/servo conversion
- Continuous serial protocol `V:PWM,S:SERVO`
- Motor stop command on shutdown
- Steering potentiometer feedback
- PID-based steering control in firmware
- Six-channel ultrasonic sensor reporting through the same controller

For embedded companies, this is stronger than a pure ROS project because it proves integration across Linux userspace, serial transport, and MCU firmware behavior.

### 4. The system expanded into multi-mission autonomy

This repository is not a single demo mode.

- Track driving: lane following and speed control
- Mission mode: traffic-light and obstacle-aware decision path
- Parking mode: rear-LiDAR-based vertical parking state machine
- Hybrid mode: lane following combined with obstacle avoidance

That breadth is valuable for robotics hiring because it reflects system growth under changing competition requirements.

## Repository-Verified Design Decisions

Several choices in this repository are especially relevant in professional environments:

- C++ was used for latency-sensitive ROS2 nodes such as lane tracking and decision logic
- Python was retained where iteration speed or dependency integration mattered more than raw performance
- Runtime parameters were externalized into YAML to support tuning without code rebuilds
- Launch files support multiple operating modes instead of hardcoding a single execution path
- Camera encoding issues were handled in code and config, not ignored as environment noise
- Parking and hybrid driving were separated into explicit operational modes, reducing control-path ambiguity

## Hardware and I/O

| Interface | Usage |
|---|---|
| USB camera | Forward lane and object perception |
| Rear camera | Parking-related support path |
| RPLiDAR | Obstacle sensing and parking geometry |
| Ultrasonic x6 | Near-field distance safety input |
| Arduino | Motor drive, steering control, ultrasonic aggregation |
| Ackermann messages | Internal vehicle command representation |

## Interview Framing

If presenting this repository to ADAS, robotics, or embedded teams, the strongest talking points are:

- Built a ROS2 autonomous driving stack that spans perception, decision, embedded control, and mission orchestration
- Solved deployment-grade issues such as camera pixel formats, stale sensor data, shutdown safety, and parameter tuning under real hardware constraints
- Improved the system iteratively through repository-verified changes, including ROS2 migration, BEV calibration, dynamic lookahead, obstacle avoidance, and LiDAR parking
- Worked on code that has clear real-time and safety considerations instead of being limited to offline model experimentation

## Build

```bash
cd /home/deepblue/target_projects/ros2_autonomous_cpp/ros2_autonomous_cpp
colcon build --symlink-install
source install/setup.bash
```

Main dependencies:

- ROS2 Humble
- OpenCV 4.x
- Eigen3
- `ackermann_msgs`
- `cv_bridge`
- `image_transport`
- `pyserial`

## Example Launch Modes

```bash
# Track driving
ros2 launch bringup track_launch.py

# Mission mode
ros2 launch bringup mission_launch.py

# Parking mode
ros2 launch bringup parking_launch.py

# Hybrid lane + obstacle mode
ros2 launch bringup hybrid_drive_launch.py
```

## Useful Entry Points

- `src/perception_pkg/src/lane_tracking_node.cpp`
- `src/decision/src/decision_node.cpp`
- `src/decision/src/parking_node.cpp`
- `src/drivers/arduino_driver/scripts/arduino_bridge_node.py`
- `src/drivers/arduino_driver/ino/motor_control_with_feedback_ros2/motor_control_with_feedback_ros2.ino`
- `src/bringup/launch/track_launch.py`
- `src/bringup/launch/mission_launch.py`

## Documentation

Additional project documents remain in `docs/` and top-level markdown files:

- `docs/PERFORMANCE.md`
- `docs/how_to_run.md`
- `docs/parking_system.md`
- `docs/sensor_calibration.md`
- `HYBRID_DRIVE_GUIDE.md`
- `TUNING_GUIDE.md`

## Limits and Honesty Notes

This repository was developed for a competition vehicle, but the physical platform is no longer available for new benchmark collection. For that reason:

- The README does not invent current performance numbers
- Claims here are tied to code and commit history that can be inspected directly
- If you use this repository in a resume or portfolio, pair it with photos, videos, presentation material, or archived logs if available
- This is a multi-contributor repository, so personal contribution should be explained with touched files and commit history rather than broad ownership claims

## Recommended Resume Framing

Safer individual framing:

> Contributed to a ROS2-based autonomous vehicle stack integrating camera perception, LiDAR parking, obstacle-aware decision logic, and Arduino-based embedded control. Worked on repository-verified improvements including ROS2 migration, dynamic lane tracking, BEV calibration, hybrid obstacle avoidance, and safety-focused command handling.
