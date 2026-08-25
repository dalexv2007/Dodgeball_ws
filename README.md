# Dodgeball — ROS2 Turtlebot Autonomy Project

A ROS2 package that enables a Turtlebot to autonomously detect, approach, and kick a yellow ball using computer vision, LiDAR, and a finite state machine controller.

## Overview

The robot uses its OAK-D RGB camera and 2D LiDAR to locate a yellow ball in the environment, navigate around it to build up a running start, and execute a kick. The system is split into two nodes that communicate over a custom message type:

- **BallFinder** (Python) — Processes camera and LiDAR data to publish ball location estimates
- **DodgeballFSM** (C++) — Consumes ball location and odometry to drive an FSM-based motion controller

## Package Structure

```
dodgeball/
├── msg/
│   └── BallLocation.msg        # Custom message: bearing, distance, found
├── scripts/
│   └── BallFinder.py           # Vision + LiDAR node
├── src/
│   └── Dodgeball_FSM.cpp       # FSM motion controller node
├── include/dodgeball/
│   └── pid_controller.hpp      # Reusable PID controller class
├── CMakeLists.txt
└── package.xml
```

## Custom Message

**`BallLocation.msg`**
| Field | Type | Description |
|-------|------|-------------|
| `bearing` | `int32` | Pixel x-coordinate of ball center in camera frame |
| `distance` | `float32` | Filtered LiDAR distance to ball (meters), or -1.0 if invalid |
| `found` | `bool` | True if a valid ball detection exists |

## Nodes

### BallFinder (Python)

Subscribes to the OAK-D camera and LiDAR scan topics, applies HSV color masking to isolate yellow, and maps the detected pixel column to a LiDAR scan index to get distance. Applies a 5-sample moving average filter on the distance reading before publishing.

**Subscriptions:**
- `/oakd/rgb/preview/image_raw` — Raw camera frames
- `/scan` — LiDAR scan data

**Publications:**
- `/ball_location` — `BallLocation` message
- `/ball_image` — Annotated debug image with mask overlay and detection line

**Key parameters (hardcoded):**
- HSV yellow range: `[20, 150, 150]` – `[30, 255, 255]`
- Minimum yellow pixel count: 50
- Vertical crop: ignores top and bottom 20% of frame
- HFOV mapping: scan index range 147–223 mapped from pixel column 0–250

### DodgeballFSM (C++)

Implements a 6-state Moore FSM that drives the robot through detection, approach, navigation, and kick phases. Uses three independent PID controllers for bearing, distance, and heading control.

**Subscriptions:**
- `/ball_location` — Ball detection data
- `/odom` — Robot pose (position + quaternion → yaw)

**Publications:**
- `/cmd_vel` — Twist velocity commands

**States:**

| State | Behavior | Transition |
|---|---|---|
| `SEARCH` | Rotate in place | → `APPROACH` when ball found |
| `APPROACH` | PID toward ball at target distance | → `NAV_TO_INTER` when aligned and at distance; → `SEARCH` if lost |
| `NAV_TO_INTER` | Navigate to arc waypoint beside ball | → `NAV_TO_KICK_POS` when within 0.5m |
| `NAV_TO_KICK_POS` | Navigate to position behind ball | → `LINE_UP` when within 1.0m |
| `LINE_UP` | Rotate in place to center ball in frame | → `KICK` when bearing error < 10px |
| `KICK` | Drive forward at kick speed for fixed duration | → `SEARCH` after timer expires |

## PID Controller

A simple, header-only PID implementation with integral clamping and output limiting. All three axes (bearing, distance, heading) use separate `PIDController` instances with independently tuned gains.

```cpp
PIDController(double kp, double ki, double kd, double limit);
double compute(double error, double dt);
void reset();
```

**Default gains:**

| Controller | Kp | Ki | Kd | Limit |
|---|---|---|---|---|
| Bearing | 0.01 | 0.0 | 0.0 | 1.0 rad/s |
| Distance | 0.3 | 0.0 | 0.0 | 0.3 m/s |
| Theta | 1.0 | 0.0 | 0.0 | 1.0 rad/s |

## Build and Run

```bash
# From workspace root
colcon build --packages-select dodgeball
source install/setup.bash

# Run BallFinder node
ros2 run dodgeball BallFinder

# Run FSM node
ros2 run dodgeball dodgeball_fsm
```

## Dependencies

- ROS2 (tested on Humble)
- `rclcpp`, `rclpy`
- `sensor_msgs`, `geometry_msgs`, `nav_msgs`
- `cv_bridge`, OpenCV
- `rosidl_default_generators` / `rosidl_default_runtime`

## Tuning

All motion parameters are grouped in the `TunableParams` struct inside `DodgeballFSM.cpp` for easy access:

```cpp
struct TunableParams {
    double search_rotation_speed = 0.5;   // rad/s
    double approach_distance_goal = 1.5;  // meters
    double kick_speed = 0.3;              // m/s
    int kick_duration_ms = 3000;          // milliseconds
    // PID gains...
};
```

## Known Limitations

- HSV thresholds are hardcoded and may require adjustment under different lighting conditions
- LiDAR-to-pixel mapping assumes a fixed camera resolution (250px wide) and a specific scan index range; recalibration needed if hardware changes
- Kick maneuver waypoints are computed once at transition time from odometry and are not updated if the ball moves during navigation
