# quad_config — CHAMP config for the SolidWorks quadruped

ROS 2 **Jazzy** + Gazebo **Harmonic** configuration for the custom robot exported
from SolidWorks (`src/quad`) and wired to the CHAMP controller.

## Packages

| Package | Role |
|---|---|
| `quad_description` | URDF/xacro, meshes, `ros2_control`, Gazebo plugins |
| `quad_config` | CHAMP joints/links/gait maps + bringup/gazebo launches |

The original SolidWorks package under `src/quad` is left as a reference and is
ignored by colcon (`COLCON_IGNORE`) because it is still a ROS 1 / catkin
export with spaces in the package name.

## Build

```bash
cd ~/quad_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select quad_description quad_config --symlink-install
source install/setup.bash
```

## Run

### View model in RViz (no physics)

```bash
ros2 launch quad_description display.launch.py
```

### CHAMP bringup (controller + RViz, no Gazebo)

```bash
ros2 launch quad_config bringup.launch.py rviz:=true
```

### Gazebo + CHAMP

```bash
ros2 launch quad_config gazebo.launch.py rviz:=true
```

### Teleop (second terminal)

```bash
source ~/quad_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## SolidWorks → CHAMP name map

| SolidWorks | CHAMP |
|---|---|
| `flj1/2/3`, `fll*`, `flf` | `lf_*` (left front) |
| `frj1`, `fr2`, `frj3`, `frl*`, `frf` | `rf_*` (right front) |
| `rlj1/2/3`, `rll*`, `rlf` | `lh_*` (left hind) |
| `rrj1/2/3`, `rrl*`, `rrf` | `rh_*` (right hind) |

Joint roles: `*_hip_joint` (abduction), `*_upper_leg_joint` (thigh),
`*_lower_leg_joint` (knee), `*_foot_joint` (fixed).

## Tuning checklist (expect to iterate)

1. **`config/gait/gait.yaml`**
   - `nominal_height` — hip height while walking (start ~0.16 m)
   - `knee_orientation` — try `">>"`, `"<<"`, `"><"`, `"<>"` if knees fold wrong
   - `swing_height`, velocity limits
2. **Joint limits** in `quad_description/urdf/quad_content.xacro` — SolidWorks
   left all limits at 0; placeholder values are filled in for typical servos.
3. **PID gains** in `config/ros_control/ros_control.yaml` (or
   `quad_description/config/ros_control/ros_control.yaml`) if legs sag or thrash.
4. **Stand pose** — `stand_hip` / `stand_thigh` / `stand_calf` in
   `xacro/ros2_control.xacro` if the zero pose is not a usable standing pose.

## Dependencies (same stack as go2)

```bash
sudo apt install ros-jazzy-gz-ros2-control ros-jazzy-ros-gz \
  ros-jazzy-xacro ros-jazzy-robot-localization \
  ros-jazzy-ros2-controllers ros-jazzy-ros2-control
```
