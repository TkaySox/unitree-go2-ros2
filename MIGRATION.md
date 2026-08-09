# Jazzy / Gazebo Harmonic migration notes

This branch (`jazzy`) ports the repo from **ROS 2 Humble + Gazebo Classic**
(Ubuntu 22.04) to **ROS 2 Jazzy + Gazebo Harmonic** (Ubuntu 24.04 / Noble).

## Why Gazebo Harmonic, not Gazebo Classic?

Gazebo Classic (the `gazebo_ros`/`gazebo_ros2_control`/`gazebo_plugins`
stack) was declared end-of-life in January 2025 and was **never packaged
for Ubuntu 24.04**. There is no `ros-jazzy-gazebo-*` equivalent. The
ROS 2 Jazzy-supported simulator is Gazebo Harmonic (`gz-sim` 8), used
through `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces`, and
`gz_ros2_control`. Keeping Gazebo Classic would mean building it from
source against Noble, which is unsupported and fragile — so this migration
replaces it outright.

## What changed

### Package manifests (`package.xml`)
`gazebo_ros`, `gazebo_ros_pkgs`, `gazebo_plugins`, `gazebo_ros2_control`,
`velodyne_gazebo_plugins` → `ros_gz_sim`, `ros_gz_bridge`,
`ros_gz_interfaces`, `gz_ros2_control` in `go2_description`,
`champ_description`, `champ_gazebo`, `go2_config`.

### URDF/xacro (`gazebo.xacro`, `leg.xacro`, `laser.xacro`, `velodyne.xacro`,
and the equivalent generic-CHAMP files under `champ_description/urdf`)
| Gazebo Classic | Gazebo Harmonic |
|---|---|
| `gazebo_ros2_control/GazeboSystem` hardware plugin | `gz_ros2_control/GazeboSimSystem` |
| `libgazebo_ros2_control.so` world plugin | `gz_ros2_control-system` (`gz_ros2_control::GazeboSimROS2ControlPlugin`) |
| `libgazebo_ros_p3d.so` (ground-truth odom) | `gz-sim-odometry-publisher-system` (`gz::sim::systems::OdometryPublisher`) |
| `libgazebo_ros_imu_sensor.so` | native gz-sim IMU sensor output (no plugin — just a `<topic>`) |
| `<sensor type="ray">` + `libgazebo_ros_ray_sensor.so` (2D laser) | `<sensor type="gpu_lidar">`, native topic |
| `<sensor type="ray">` + `libgazebo_ros_velodyne_laser.so` (Velodyne) | `<sensor type="gpu_lidar">`, native topic |
| `<sensor type="depth">` + `libgazebo_ros_camera.so` (Xtion, generic CHAMP only) | `<sensor type="rgbd_camera">`, native topics |
| *(no equivalent — read via `gazebo::transport` directly)* | `<sensor type="contact">` per foot link |

Gazebo Harmonic sensors publish over **gz transport**, not ROS topics
directly, so every sensor now also needs a `ros_gz_bridge` entry — see
below.

### `ros_gz_bridge` configs (new: `champ/champ_gazebo/config/*.yaml`)
- `gz_bridge.yaml` — sim clock, IMU (`imu` → `imu/data`), ground-truth
  odometry (`odom/ground_truth`).
- `contact_bridge.yaml` — foot contact sensors → `ros_gz_interfaces/msg/Contacts`.
- `laser_bridge.yaml` — 2D laser `scan` topic (used by `robot.xacro`).
- `velodyne_bridge.yaml` — Velodyne `velodyne_points` topic (used by `robot_VLP.xacro`).

### `champ_gazebo/src/contact_sensor.cpp`
Gazebo Classic exposed contacts via the `gazebo::transport` C++ client
library, which does not exist in Gazebo Harmonic. The node was rewritten to
subscribe to a ROS topic (`ros_gz_interfaces/msg/Contacts`) fed by
`ros_gz_bridge` instead of talking to the simulator directly. Build deps on
`GAZEBO_LIBRARIES`/`GAZEBO_INCLUDE_DIRS` were dropped from
`champ_gazebo/CMakeLists.txt`.

### Launch files
`champ_gazebo/launch/gazebo.launch.py`:
- `gzserver`/`gzclient` `ExecuteProcess` calls → `ros_gz_sim`'s
  `gz_sim.launch.py`.
- `gazebo_ros`'s `spawn_entity.py` → `ros_gz_sim`'s `create` executable.
- Controller activation switched from `ros2 control load_controller`
  `ExecuteProcess` calls to `controller_manager/spawner` nodes (more robust
  startup ordering; unrelated to the simulator swap but bundled in since it
  touches the same launch file).
- Added a `ros_gz_bridge` node for the core topics, plus
  `GZ_SIM_RESOURCE_PATH` so meshes resolve when the robot is spawned.

`go2_config`'s `gazebo.launch.py` / `gazebo_velodyne.launch.py` each now
also bring up the matching sensor bridge (laser vs. Velodyne).

### World files (`*.world`)
Bumped to `<sdf version='1.9'>` and stripped the empty `frame=''` pose
attribute that Gazebo Classic used to emit — newer `sdformat` (used by
Harmonic) rejects it. `<physics type='ode'>` was left as-is; Harmonic still
ships the ODE plugin alongside its DART default, but this is worth
confirming on your machine (see checklist below).

### What was **not** touched
- `champ_base` (the C++ control/state-estimation nodes), `champ_msgs`,
  `champ_teleop`, and the `ros2_control` YAML configs are simulator-agnostic
  and needed no changes for Jazzy.
- Nav2/SLAM launch files (`navigate.launch.py`, `slam.launch.py`) — these
  were already marked incomplete (&cross;) before this migration and are out
  of scope here.

## Known caveats / things to verify on real hardware

I do not have a ROS 2 Jazzy + Gazebo Harmonic environment available to
build and run this, so **treat this branch as unverified until you've run
through the checklist below**. Likely trouble spots, roughly in order of
risk:

1. **Contact sensor plumbing.** `contact_bridge.yaml` bridges a single
   `gz/contacts` topic, but gz-sim's default per-sensor contact topic is
   namespaced under `world/<world>/model/<robot>/.../sensor/<name>/contact`
   unless remapped. You'll likely need to either add explicit `<topic>`
   remaps to `gz/contacts` on each `<sensor type="contact">` in
   `gazebo.xacro`, or list all four sensor topics individually in
   `contact_bridge.yaml` and merge them in `contact_sensor.cpp`. Test with
   `gz topic -l` after spawning to see the real topic names first.
2. **`gz_ros2_control-system` plugin filename.** Depending on the exact
   `gz_ros2_control` release packaged for Jazzy, the world-plugin filename
   may be `gz_ros2_control-system` or `libgz_ros2_control-system.so` — check
   `ros2 pkg prefix gz_ros2_control` / the package's own examples if the
   plugin fails to load.
3. **IMU/lidar topic bridging direction and message field mapping.** I used
   the standard `gz.msgs.IMU` / `gz.msgs.LaserScan` / `gz.msgs.PointCloudPacked`
   bridge types; double check frame IDs (`gz_frame_id`) line up with your TF
   tree, especially since `close_loop_odom` and the EKF nodes expect
   specific frames.
4. **`<physics type='ode'>` in the world files** — confirm
   `gz-physics-ode-plugin` is installed and Harmonic accepts `type='ode'`
   directly, or switch to the DART default (drop the `type` attribute) if
   you hit physics-engine load errors.
5. **Material shortcuts** like `Gazebo/FlatBlack` in the generic CHAMP
   description won't resolve the same way in Harmonic's renderer — cosmetic
   only, robot will just render with default coloring.
6. **`rgbd_camera` topic names** on the generic CHAMP Xtion camera — I
   documented the expected `${name}/...` suffixes in a comment in
   `asus_camera.urdf.xacro`, but didn't add a corresponding bridge config
   since it's unused by go2; add one if you rely on the generic camera.

## Suggested verification checklist

```bash
# 1. Build
cd <your_ws>
colcon build --symlink-install
source install/setup.bash

# 2. RViz-only (no sim) — sanity check the description/control stack alone
ros2 launch go2_config bringup.launch.py rviz:=true

# 3. Gazebo Harmonic + robot spawn
ros2 launch go2_config gazebo.launch.py
# In another shell:
gz topic -l                    # confirm imu, scan, odom/ground_truth, contact topics exist
ros2 topic list                # confirm ros_gz_bridge republished them
ros2 topic echo /imu/data --once
ros2 topic echo /scan --once

# 4. Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 5. Velodyne variant
ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true
ros2 topic echo /velodyne_points --once
```

If a step fails, the error message plus `gz topic -l` / `ros2 topic list`
output will usually point straight at whichever bridge/topic name needs
adjusting per the caveats above.
