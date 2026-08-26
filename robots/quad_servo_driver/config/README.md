# quad_servo_driver

Feetech serial-bus hardware for the SolidWorks / CHAMP quadruped on a **Raspberry Pi 5**, via a **Waveshare USB** adapter (`/dev/ttyACM0` by default — not the Pi GPIO UART).

---

## What this package does

| Goal | How |
|------|-----|
| Talk to servos | Official Feetech **scservo_sdk** (same STS/SMS protocol as **SCServo_Linux**) |
| Match RViz to hardware | **Default: no offsets** — URDF/RViz rad → ticks directly (`apply_offsets:=false`) |
| Drive from CHAMP | Subscribe to `joint_states`, convert rad → ticks, write positions |
| Debug bus | Read-only **servo_diag** (pos, speed, V, temp, limits, …) |
| Bring-up motion test | **hw_joint_test**: remap, then **+20° all joints at once** (sync write) |
| Torque on/off | **servo_torque** — hold or limp mapped servos (no motion) |

---

## Hardware assumptions

| Joints | Model | Interface |
|--------|--------|-----------|
| Hips (`*_hip_joint`) | **FT5330M** | **PWM on Pi GPIO** (hobby-style pulse) |
| Upper / lower legs | **STS3215** | **USB serial bus** `/dev/ttyACM0` (Waveshare) |

**Serial (legs):** STS/SMS protocol, **4096 ticks/rev**, mid **2048** (verify on hardware). Baud **1 000 000**.

**PWM (hips):** 50 Hz frame, default pulse **500–2500 µs**, home **1500 µs**. Fill real **BCM** pin numbers in `joint_map.py`. Open-loop (no position feedback).

```bash
sudo apt install python3-gpiozero python3-lgpio
```

> Note: stock FT5330M docs are often serial-bus. This stack drives hips as **PWM**. If yours stay on the Feetech bus, set `interface="serial"` and a `servo_id` instead of `gpio_pin`.

---

## Package layout

| File | Role |
|------|------|
| `joint_map.py` | `JOINT_ID_MAP` (name → ID, model, direction, URDF home), tick math, `JOINT_ORDER` |
| `scservo_bus.py` | Low-level PortHandler / PacketHandler R/W (vendored SDK under `third_party/scservo_sdk/`) |
| `hardware_interface.py` | **`QuadServoHardware`**: connect, ping, calibrate/remap, torque, `command_rad()` |
| `champ_servo_driver.py` | ROS node: `joint_states` → hardware |
| `servo_diag.py` | Read-only diagnostics |
| `hw_joint_test.py` | Remap + simultaneous ±deg sweep (sync write; `simultaneous:=false` for sequential) |
| `servo_home.py` | Shortest-path home to tick 0 |
| `servo_torque.py` | Enable / disable torque on mapped servos |
| `third_party/VENDOR.md` | Where the SDK came from |

---

## Joint map (edit this)

File: `quad_servo_driver/joint_map.py`

```text
Serial IDs (STS3215 upper/lower):
  1 rf_lower   2 rf_upper   3 lf_lower   4 lf_upper
  5 rh_lower   6 rh_upper   7 lh_lower   8 lh_upper

Hips (FT5330M): PWM GPIO — edit gpio_pin in joint_map.py (defaults 12/13/18/19)
```

After changing IDs, **recalibrate** (pose at URDF home first):

```bash
ros2 run quad_servo_driver servo_calibrate
```

---

## Serial permissions

If you see `Permission denied: '/dev/ttyACM0'`:

```bash
sudo usermod -aG dialout $USER   # then log out / reboot
# one-shot until unplug:
sudo chmod 666 /dev/ttyACM0
```

---

## Mapping (default: no offsets)

If the physical robot pose already matches what you see in RViz/URDF, **do not
apply mechanical offsets**. That is the default:

```text
ticks = angle_to_ticks(urdf_angle)   # no +offset
```

**File:** `~/.ros/quad_servo_calibration.yaml` (identity / zeros by default)  
(override with `QUAD_SERVO_CALIB` or param `calibration_file`)

| Action | How |
|--------|-----|
| Normal (RViz matches robot) | Leave `apply_offsets:=false` (default on all nodes) |
| Write identity YAML | `ros2 run quad_servo_driver servo_calibrate` |
| Horns do NOT match RViz | Pose URDF home, then `servo_calibrate --ros-args -p apply_offsets:=true`, and run drivers with `-p apply_offsets:=true` |

Optional offset path (only when `apply_offsets:=true`):

```text
offset = raw_tick_at_URDF_home − ideal_home_ticks   # ideal ≈ 2048 at home
ticks  = angle_to_ticks(urdf_angle) + offset
```

---

## Nodes & how to run

Build once:

```bash
cd ~/quad_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select quad_servo_driver --symlink-install
source install/setup.bash
```

### A) `servo_diag` — read-only debug

Does **not** move servos. Pings IDs and prints position, speed, load, voltage, temperature, current, torque enable, mode, angle limits.

```bash
# One-shot scan IDs 1–20
ros2 run quad_servo_driver servo_diag --ros-args -p once:=true

# Continuous
ros2 run quad_servo_driver servo_diag --ros-args -p port:=/dev/ttyACM0 -p rate_hz:=2.0

# Only IDs in JOINT_ID_MAP
ros2 run quad_servo_driver servo_diag --ros-args -p mapped_only:=true -p once:=true
```

Useful fields from a good dump: `pos`, `V` (~7–12 V depending on supply), `T`, `torque_en`, `mode=0` (position), `ang_lim=[0,4095]`.

---

### B) `hw_joint_test` — +20° all at once (no offsets by default)

**Support the robot.**

```bash
# Default: URDF→ticks (no offsets), ALL online joints +20° together, hold, return home
ros2 run quad_servo_driver hw_joint_test
```

Options:

```bash
# Smaller / slower motion
ros2 run quad_servo_driver hw_joint_test --ros-args \
  -p delta_deg:=10.0 -p hold_sec:=2.0 -p settle_sec:=1.0

# Leave joints at +delta (do not return home)
ros2 run quad_servo_driver hw_joint_test --ros-args -p return_home:=false

# Old sequential (one joint at a time)
ros2 run quad_servo_driver hw_joint_test --ros-args -p simultaneous:=false

# Leave torque enabled at end
ros2 run quad_servo_driver hw_joint_test --ros-args -p disable_torque_at_end:=false

# Only if physical horns do not match RViz:
ros2 run quad_servo_driver hw_joint_test --ros-args \
  -p apply_offsets:=true -p force_recalibrate:=true
```

Sequence:

1. Open serial, ping mapped IDs  
2. Load mapping (identity / no offsets by default)  
3. Enable torque → all online joints to URDF home  
4. **All online joints +Δ° at the same time**, hold, return home together  
5. Disable torque (default) and close port  

---

### C) `champ_servo_driver` — live CHAMP bridge

With CHAMP publishing `joint_states` on hardware (`gazebo:=false`):

```bash
ros2 launch quad_config bringup.launch.py hardware_connected:=true
# other terminal — pose the robot where you want first:
ros2 run quad_servo_driver champ_servo_driver
```

**Default `capture_on_start:=true`:** on the first `/joint_states`, the driver
reads each servo’s **current ticks** and saves offsets so those URDF angles
map to the pose the robot is in **right now** (no yank). Then it follows CHAMP.

```bash
# Reuse last captured YAML (skip re-capture — can jump if pose changed)
ros2 run quad_servo_driver champ_servo_driver --ros-args -p capture_on_start:=false

# Raw URDF→ticks with no offsets (will move to ideal 2048-centered pose)
ros2 run quad_servo_driver champ_servo_driver --ros-args \
  -p capture_on_start:=false -p apply_offsets:=false
```

Ignores joint names not in `JOINT_ID_MAP`. Warns (does not crash) on failed writes. Closes the port on shutdown.

---

### D) `servo_torque` — enable / disable torque

Does **not** move joints. Turns holding torque on (stiff) or off (limp).

```bash
# Torque ON (hold)
ros2 run quad_servo_driver servo_torque --ros-args -p enable:=true

# Torque OFF (limp — safe to pose by hand)
ros2 run quad_servo_driver servo_torque --ros-args -p enable:=false
```

Shortcuts (after `source ~/.bashrc`):

```bash
qton     # torque on
qtoff    # torque off
```

> Stop `champ_servo_driver` / other nodes that own `/dev/ttyACM0` first, or this will fail to open the port.

---

## Shell shortcuts (`~/.bash_aliases`)

Reload with `source ~/.bashrc`, then `qhelp` for the full list.

| Shortcut | Command |
|----------|---------|
| `qsrc` | Source ROS + `~/quad_ws` |
| `qbuild` | `colcon build --symlink-install` |
| `qsim` / `qsimh` | Gazebo + RViz / headless |
| `qhw` / `qhwn` | Hardware CHAMP bringup (± RViz) |
| `qdrv` | `champ_servo_driver` |
| `qhome` | Shortest-path home → tick 0 |
| `qton` / `qtoff` | **Torque on / off** |
| `qtest` | `hw_joint_test` (+20°) |
| `qdiag` | `servo_diag` once |
| `qtele` | Keyboard teleop |
| `qcircle` / `qfwd` / `qstop` | Circle / forward / stop `cmd_vel` |
| `qdrop` / `qstand` | Crouch / reset `/body_pose` |

---

## Angle ↔ ticks

Start pose: **upper vertical (0)**, **lower 90° forward (π/2)** → servo **0° = tick 0**.

```text
ticks = (0 + direction * (angle_rad − urdf_home_rad) * (4096 / 2π)) mod 4096
# (+ optional calibration_offset only when apply_offsets:=true)
```

**Homing** (`servo_home` / `qhome`) uses the **shortest path** on the circle.
Example: at ~350° → moves 350→351→…→0 (wrap), not the long way through 180°.

≈ **227.6 ticks per 20°** at direction = +1.

Hip vs leg use different default **speed/acc** (`HIP_SPEED` / `DEFAULT_SPEED` in `joint_map.py`).

---

## Suggested bring-up order

1. `dialout` permissions + `ls -l /dev/ttyACM0`  
2. `servo_diag -p once:=true` — confirm which IDs answer  
3. Fix wiring / IDs for any missing joints; edit `JOINT_ID_MAP`  
4. Confirm RViz pose matches the physical robot → keep `apply_offsets:=false`  
5. `hw_joint_test` (+20° simultaneous sweep)  
6. If a joint moves the wrong way → set `direction=-1` for that joint  
7. Run CHAMP + `champ_servo_driver`  

---

## Related robot packages

| Path | Role |
|------|------|
| `robots/descriptions/quad_description/` | URDF / xacro used by CHAMP |
| `robots/configs/quad_config/` | Gait, launches (`gait/GAIT.md`) |
| `robots/configs/quad_config/config/gait/gait.yaml` | `nominal_height`, step size, etc. |
| `quad/` | Original SolidWorks export (reference; `COLCON_IGNORE`) |

Runtime stand height (CHAMP software crouch), separate from servo calib:

```bash
ros2 topic pub --once /body_pose geometry_msgs/msg/Pose \
"{position: {x: 0.0, y: 0.0, z: -0.02},
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
# actual ≈ nominal_height + z
```

See `robots/configs/quad_config/config/gait/GAIT.md` for gait parameters.
