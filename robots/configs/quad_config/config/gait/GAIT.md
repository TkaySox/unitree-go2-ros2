# Gait parameters (`gait.yaml`)

**File:** `robots/configs/quad_config/config/gait/gait.yaml`  
**Used by:** CHAMP `quadruped_controller_node` and `state_estimation_node`  
**When applied:** at **node startup** only (edit + **restart launch**).  
**Runtime height:** use `/body_pose` (see below) — not this file live.

These settings control **how the legs step**, **how tall the robot stands**, and **how fast teleop can push it**. They do **not** change the URDF mesh or joint limits.

---

## What you should be able to do

| Goal | Main knobs |
|------|------------|
| Stand taller / crouch more | `nominal_height` (restart) or `/body_pose` (runtime) |
| Walk slower / smaller steps | lower `max_linear_velocity_x`, optionally lower `stance_duration` |
| Walk sideways slower | `max_linear_velocity_y` |
| Turn slower | `max_angular_velocity_z` |
| Feet lift less / more in the air | `swing_height` |
| Knees bend the wrong way | try other `knee_orientation` strings |
| Odometry drifts vs ground truth | tweak `odom_scaler` |
| Shift weight front/back | `com_x_translation` |

**Teleop** (`teleop_twist_keyboard`) only commands velocity up to these max limits. Full stick = max velocity.

---

## Parameter reference

### `knee_orientation` (string)

How the knees are bent in the IK model.

| Value | Meaning (dot = front of robot) |
|-------|--------------------------------|
| `">>"` | Both pairs of knees point **backward** (dog-like) — **current** |
| `"<<"` | Both pairs point **forward** |
| `"><"` | Front back, hind forward |
| `"<>"` | Front forward, hind back |

**If legs fold inside-out or walk looks mirrored**, cycle these and relaunch.

---

### `pantograph_leg` (bool)

| Value | Meaning |
|-------|---------|
| `false` | Normal 3-DOF legs (hip + thigh + calf) — **your robot** |
| `true` | Special pantograph linkage (not used here) |

Leave **`false`**.

---

### `nominal_height` (float, metres)

**Standing height:** roughly hip-to-ground while standing still (and the base for crouch/stand in IK).

- **Higher** → taller stand (legs more extended).  
- **Lower** → crouch.

**Practical range for this SolidWorks model:** about **0.11 – 0.21 m**.  
Export zero-stance is ~0.22 m; going too high can break IK or nearly lock the legs straight. Too low hits the controller’s crouch limit (~65% of full stretch).

**Current default in repo:** `0.13` (13 cm).

Restart launch after editing.

---

### `max_linear_velocity_x` (m/s)

Max **forward/back** speed from teleop / `cmd_vel`.

Also drives **step length** roughly as:

```text
step_length ≈ velocity × stance_duration
```

So lower max velocity ⇒ **smaller strides** at full stick.

---

### `max_linear_velocity_y` (m/s)

Max **sideways** (strafe) speed.

---

### `max_angular_velocity_z` (rad/s)

Max **yaw turn** rate (spin left/right).

---

### `stance_duration` (seconds)

How long each foot stays on the ground in a step cycle.

| Change | Effect |
|--------|--------|
| **Longer** | Longer steps at the same speed; slower cadence |
| **Shorter** | Shorter steps; quicker footfall |

Typical values: **0.15 – 0.30**.

---

### `swing_height` (metres)

How high the foot lifts during the **swing** phase.

| Too high | Too low |
|----------|---------|
| Legs flail, “walking in the air” | Feet scuff the ground |

For a small robot, **0.01 – 0.03 m** is a good band. Current: `0.015`.

---

### `stance_depth` (metres)

How far the foot is pushed **into the ground** during stance (virtual penetration for contact feel).

| Value | Effect |
|-------|--------|
| `0.0` | Neutral (recommended default for sim stability) |
| `> 0` | More “push into ground”; can add bounce if large |

Keep near **0** unless you know you need it.

---

### `com_x_translation` (metres)

Shifts the foot reference / CoM offset along **X** (front–back).

| Sign | Effect |
|------|--------|
| **Positive** | Shift reference forward |
| **Negative** | Shift reference backward |

Use if the robot is **nose-heavy or tail-heavy** (e.g. lidar on the front) and pitches under walk.

Start with **0.0**; try small steps like `±0.01`.

---

### `odom_scaler` (unitless)

Multiplies open-loop odometry from the gait.

| Value | Effect |
|-------|--------|
| `1.0` | No scale |
| `> 1` | Report more motion than actual (if odom undershoots) |
| `< 1` | Report less motion (if odom overshoots) |

Useful for dead-reckoning tuning; less critical if you use Gazebo ground-truth / EKF.

---

## Runtime control (not in `gait.yaml`)

### Standing height while running

Topic: **`/body_pose`** (`geometry_msgs/Pose`)

```text
actual_height ≈ nominal_height + body_pose.position.z
```

Examples:

```bash
# Crouch 2 cm below current nominal
ros2 topic pub --once /body_pose geometry_msgs/msg/Pose \
"{position: {x: 0.0, y: 0.0, z: -0.02},
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"

# Back to YAML nominal (offset 0)
ros2 topic pub --once /body_pose geometry_msgs/msg/Pose \
"{position: {x: 0.0, y: 0.0, z: 0.0},
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

Orientation fields can also pitch/roll/yaw the body relative to the feet (advanced).

### Walk / turn

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# or publish /cmd_vel (geometry_msgs/Twist)
```

CHAMP remaps smoothed cmd to the controller (`/cmd_vel` → internal smooth path depending on launch).

---

## Suggested tuning order

1. **`knee_orientation`** — legs must bend the right way.  
2. **`nominal_height`** — stable stand without collapse.  
3. **`max_linear_velocity_x`** — stride size with teleop.  
4. **`swing_height`** — clear the ground without flailing.  
5. **`stance_duration`** — cadence vs step length.  
6. **`com_x_translation`** — only if pitch/balance looks wrong.  
7. **PID / Gazebo contact** — if bounce/sag remains (`ros_control.yaml`, not this file).

---

## Related files

| File | Role |
|------|------|
| `config/gait/gait.yaml` | This document’s parameters |
| `config/joints/joints.yaml` | Joint name map for CHAMP |
| `config/links/links.yaml` | Link name map for CHAMP |
| `config/ros_control/ros_control.yaml` | Joint effort PID (sag / bounce) |
| `quad_description/.../ros_control.yaml` | Same gains used by Gazebo plugin |

---

## Apply changes

```bash
# After editing gait.yaml
source install/setup.bash
ros2 launch quad_config gazebo.launch.py headless:=true
```

No rebuild needed for YAML if the package was installed with `--symlink-install`.
