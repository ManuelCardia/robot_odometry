# first_project — Wheel-Odometry for the AgileX Bunker Pro (ROS 2)

ROS 2 (ament_cmake / C++) package that reconstructs the planar pose of a tracked **AgileX Bunker Pro** from raw motor RPM, publishes it as `nav_msgs/Odometry` + TF, and quantifies the drift of this estimate against the ground-truth TF published in the recorded rosbags.

Developed for the Robotics course at Politecnico di Milano (first project).

## Overview

| Node | Executable | Role |
|------|-----------|------|
| `odometer` | `odometer` | Converts `/bunker_status` motor RPM into a skid-steer odometry estimate (`odom → base_link2`) |
| `tf_error` | `tf_error` | Compares `odom → base_link2` (estimate) to `odom → base_link` (ground truth) at 10 Hz and publishes the error |

```
 rosbag ── /bunker_status ──► odometer ──► /project_odom (nav_msgs/Odometry)
   │                             │
   │                             └──► TF  odom → base_link2
   └── TF  odom → base_link (ground truth)
                                   │
                  tf_error ◄───────┘
                     └──► /tf_error_msg (first_project/TfErrorMsg)
```

## Odometry model

Each track is driven by a motor whose RPM is reported in `BunkerStatus.actuator_states`. The driver emits a spurious duplicate `motor_id = 0` entry with `rpm = 0`; zero readings are therefore discarded when extracting the values.

* `motor_id 0` → right track, `motor_id 1` → left track (sign convention derived empirically from the angular velocity of the recorded data).
* Track speed:  `v_track = RPM · 2π · R / (60 · G)`
* Skid-steer (differential) kinematics:

  ```
  v = (v_R + v_L) / 2
  ω = (v_R − v_L) / L
  ```
* Integration: 2nd-order Runge–Kutta (midpoint) with `θ_mid = θ_k + ω·Δt/2`

  ```
  x_{k+1} = x_k + v·cos(θ_mid)·Δt
  y_{k+1} = y_k + v·sin(θ_mid)·Δt
  θ_{k+1} = wrap(θ_k + ω·Δt)        # wrapped to [-π, π]
  ```
* Samples with `Δt ≤ 0` or `Δt > 1 s` are discarded (bag restarts / dropouts). The first message after start or reset only initialises the timestamp.

### Calibrated parameters

| Parameter | Value | Origin |
|-----------|-------|--------|
| Effective wheel (sprocket) radius `R` | 0.083 m | Fitted from `v` vs. RPM in the bags |
| Gear ratio `G` | 7.5 | Robot documentation |
| Effective track separation `L` | 0.688 m | Fitted from `(v_R − v_L)/ω` |

`L` is deliberately larger than the physical track separation (0.635 m): on a tracked vehicle the tracks slip laterally while turning, so the *effective* kinematic baseline is greater than the geometric one. Using 0.635 m would systematically over-estimate the yaw rate.

These values are currently compile-time constants in `src/odometry.cpp`.

## Interfaces

**`odometer`**

| Kind | Name | Type | Notes |
|------|------|------|-------|
| Sub | `/bunker_status` | `bunker_msgs/msg/BunkerStatus` | RPM source |
| Pub | `/project_odom` | `nav_msgs/msg/Odometry` | frame `odom`, child `base_link2` |
| Pub | `/tf` | `tf2_msgs/msg/TFMessage` | `odom → base_link2` |
| Srv | `reset` | `std_srvs/srv/Empty` | zeroes the pose and restarts time integration |

**`tf_error`** (10 Hz wall timer)

| Kind | Name | Type |
|------|------|------|
| Sub | `/tf`, `/tf_static` | via `tf2_ros::TransformListener` |
| Pub | `/tf_error_msg` | `first_project/msg/TfErrorMsg` |

`msg/TfErrorMsg.msg`

```
std_msgs/Header header
float32 tf_error            # Euclidean (x,y) distance between estimate and ground truth [m]
int32   time_from_start     # seconds since first valid transform pair
float32 travelled_distance  # cumulative path length of the *estimated* trajectory [m]
```

## Requirements

* ROS 2 (Humble or newer recommended — adapt to the distribution you tested on)
* `rclcpp`, `nav_msgs`, `geometry_msgs`, `std_msgs`, `std_srvs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `rviz2`
* **`bunker_msgs`** — message definitions from the AgileX Bunker ROS 2 driver. This package is *not* part of this repository and must be present in the same workspace (or installed) before building.

## Build

```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone <this-repo-url> first_project
# + clone/install the package providing bunker_msgs into the same workspace
cd ~/ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select first_project
source install/setup.bash
```

## Run

All nodes use simulated time, so the bag must be played with `--clock`.

```bash
# Terminal 1 — odometer, tf_error and RViz (config/view.rviz)
ros2 launch first_project first_project.launch.py

# Terminal 2 — replay a recorded bag (provides /bunker_status, ground-truth TF, /clock)
ros2 bag play <path/to/rosbag2_dir> --clock
```

Inspect the error:

```bash
ros2 topic echo /tf_error_msg
```

Reset the estimate to the origin:

```bash
ros2 service call /reset std_srvs/srv/Empty
```

## Limitations

* **Dead reckoning only.** Error grows without bound; there is no exteroceptive correction.
* The error metric is **position only** (Euclidean in the `odom` XY plane). Heading error is not reported.
* `travelled_distance` is accumulated from the *estimated* trajectory, not from the ground truth.
* Kinematic parameters are fitted to the provided bags (this robot, this surface). They are not guaranteed to transfer to other terrain or payloads, since track slip depends on both.
* Parameters are constants, not ROS parameters; changing them requires recompilation.
* `time_from_start` is truncated to whole seconds.

## Repository layout

```
first_project/
├── CMakeLists.txt
├── package.xml
├── config/view.rviz
├── launch/first_project.launch.py
├── msg/TfErrorMsg.msg
├── src/
│   ├── odometry.cpp
│   └── tf_error.cpp
└── docs/screenshots/
```

## License

Set a license before publishing (`package.xml` currently contains `TODO`). MIT is a common choice for coursework.

## Author

Manuel Cardia — Politecnico di Milano
