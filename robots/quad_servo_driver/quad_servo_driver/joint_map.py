#!/usr/bin/env python3
"""
Shared joint map for the quad hardware.

Serial bus (Waveshare USB / QinHeng → /dev/ttyACM0): STS3215 upper+lower, IDs 1–8.
Hips (FT5330M): PWM on Pi GPIO — set gpio_pin to match your wiring.

User wiring (serial):
  1 rf_lower, 2 rf_upper, 3 lf_lower, 4 lf_upper,
  5 rr_lower, 6 rr_upper, 7 lr_lower, 8 lr_upper
  (rr = right hind / rh_*, lr = left hind / lh_*)
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Dict, Optional

SERIAL_PORT = os.environ.get("QUAD_SERVO_PORT", "/dev/ttyACM0")
BAUDRATE = 1_000_000

# STS3215: STS/SMS, 4096 ticks/rev.
TICKS_PER_REV = 4096
# Mechanical servo zero (tick 0) — see SERVO_ZERO_* below.
HOME_TICK = 0
CENTER_TICK = 2048  # legacy mid; not used for home anymore

DEFAULT_SPEED = 60
DEFAULT_ACC = 20
HIP_SPEED = 40
HIP_ACC = 15

# Homing / shortest-path stepping (ticks per sync hop around the wrap)
SHORTEST_PATH_STEP = 128

PWM_FRAME_HZ = 50
PWM_PULSE_MIN_US = 500
PWM_PULSE_MAX_US = 2500
PWM_PULSE_HOME_US = 1500
PWM_TRAVEL_RAD = math.pi

CALIBRATION_FILE = os.environ.get(
    "QUAD_SERVO_CALIB",
    os.path.expanduser("~/.ros/quad_servo_calibration.yaml"),
)
# Per-joint direction (+1 / -1) from servo_dir_config interactive tool.
DIRECTIONS_FILE = os.environ.get(
    "QUAD_SERVO_DIRS",
    os.path.expanduser("~/.ros/quad_servo_directions.yaml"),
)
# Absolute servo tick homes captured by qhome_champ (both links vertical).
# qhome moves to these ticks every time.
HOME_TICKS_FILE = os.environ.get(
    "QUAD_SERVO_HOME_TICKS",
    os.path.expanduser("~/.ros/quad_servo_home_ticks.yaml"),
)

MODEL_HIP = "FT5330M"
MODEL_LEG = "STS3215"

IFACE_SERIAL = "serial"
IFACE_PWM = "pwm"


@dataclass
class JointConfig:
    model: str
    interface: str  # "serial" | "pwm"
    direction: int = 1
    # Joint angle (CHAMP/URDF frame) at which the servo reads tick HOME_TICK (0).
    # = SERVO_ZERO pose for that joint — NOT the same as CHAMP_ZERO for lowers.
    urdf_home_rad: float = 0.0
    # servo_angle = joint_angle * gear_ratio (1.0 = direct drive); gated by APPLY_GEAR_RATIO
    gear_ratio: float = 1.0
    servo_id: Optional[int] = None
    gpio_pin: Optional[int] = None
    pulse_min_us: int = PWM_PULSE_MIN_US
    pulse_max_us: int = PWM_PULSE_MAX_US
    pulse_home_us: int = PWM_PULSE_HOME_US


# ---------------------------------------------------------------------------
# Pose model (CHAMP frame vs mechanical servo zero)
#
# CHAMP_ZERO (what CHAMP/gait treat as 0 on upper+lower):
#   both upper and lower links VERTICAL.
#
# SERVO_ZERO (tick 0 on the Feetech bus — how the horns are assembled):
#   upper = VERTICAL   → same as CHAMP 0
#   lower = HORIZONTAL (forward) → 90° away from CHAMP 0
#
# Mapping:
#   ticks = direction * (champ_angle − servo_zero_angle) * scale  (+ gear later)
# So when CHAMP commands lower=0 (vertical), the servo leaves tick 0 and
# rotates ~90° to make the lower link vertical.
# ---------------------------------------------------------------------------
CHAMP_ZERO_UPPER_RAD = 0.0  # vertical
CHAMP_ZERO_LOWER_RAD = 0.0  # vertical
CHAMP_ZERO_HIP_RAD = 0.0

SERVO_ZERO_UPPER_RAD = 0.0  # vertical at tick 0
# Horizontal forward at tick 0. Flip sign to -π/2 if your "forward" is opposite.
SERVO_ZERO_LOWER_RAD = math.pi / 2.0
SERVO_ZERO_HIP_RAD = 0.0

# Lower-link spur reduction: servo pinion Ø39.4 mm → driven gear Ø44.5 mm.
# Stored for later CHAMP→servo; off until QUAD_APPLY_GEAR_RATIO=1.
_LOWER_GEAR_PINION_MM = 39.4
_LOWER_GEAR_DRIVEN_MM = 44.5
_LOWER_GEAR_RATIO = _LOWER_GEAR_DRIVEN_MM / _LOWER_GEAR_PINION_MM  # ≈ 1.1294
APPLY_GEAR_RATIO = os.environ.get("QUAD_APPLY_GEAR_RATIO", "0") in (
    "1",
    "true",
    "True",
    "yes",
)

JOINT_ID_MAP: Dict[str, JointConfig] = {
    # --- Hips: PWM GPIO (FT5330M) — edit BCM pins ---
    "lf_hip_joint": JointConfig(
        MODEL_HIP, IFACE_PWM, gpio_pin=12, urdf_home_rad=SERVO_ZERO_HIP_RAD
    ),
    "rf_hip_joint": JointConfig(
        MODEL_HIP, IFACE_PWM, gpio_pin=13, urdf_home_rad=SERVO_ZERO_HIP_RAD
    ),
    "lh_hip_joint": JointConfig(
        MODEL_HIP, IFACE_PWM, gpio_pin=18, urdf_home_rad=SERVO_ZERO_HIP_RAD
    ),
    "rh_hip_joint": JointConfig(
        MODEL_HIP, IFACE_PWM, gpio_pin=19, urdf_home_rad=SERVO_ZERO_HIP_RAD
    ),
    # --- Upper / lower: USB serial (STS3215) ---
    # urdf_home_rad = joint angle at servo tick 0 (SERVO_ZERO), not CHAMP_ZERO.
    "rf_lower_leg_joint": JointConfig(
        MODEL_LEG,
        IFACE_SERIAL,
        servo_id=1,
        urdf_home_rad=SERVO_ZERO_LOWER_RAD,
        gear_ratio=_LOWER_GEAR_RATIO,
    ),
    "rf_upper_leg_joint": JointConfig(
        MODEL_LEG, IFACE_SERIAL, servo_id=2, urdf_home_rad=SERVO_ZERO_UPPER_RAD
    ),
    "lf_lower_leg_joint": JointConfig(
        MODEL_LEG,
        IFACE_SERIAL,
        servo_id=3,
        urdf_home_rad=SERVO_ZERO_LOWER_RAD,
        gear_ratio=_LOWER_GEAR_RATIO,
    ),
    "lf_upper_leg_joint": JointConfig(
        MODEL_LEG, IFACE_SERIAL, servo_id=4, urdf_home_rad=SERVO_ZERO_UPPER_RAD
    ),
    "rh_lower_leg_joint": JointConfig(
        MODEL_LEG,
        IFACE_SERIAL,
        servo_id=5,
        urdf_home_rad=SERVO_ZERO_LOWER_RAD,
        gear_ratio=_LOWER_GEAR_RATIO,
    ),
    "rh_upper_leg_joint": JointConfig(
        MODEL_LEG, IFACE_SERIAL, servo_id=6, urdf_home_rad=SERVO_ZERO_UPPER_RAD
    ),
    "lh_lower_leg_joint": JointConfig(
        MODEL_LEG,
        IFACE_SERIAL,
        servo_id=7,
        urdf_home_rad=SERVO_ZERO_LOWER_RAD,
        gear_ratio=_LOWER_GEAR_RATIO,
    ),
    "lh_upper_leg_joint": JointConfig(
        MODEL_LEG, IFACE_SERIAL, servo_id=8, urdf_home_rad=SERVO_ZERO_UPPER_RAD
    ),
}

JOINT_ORDER = [
    "lf_hip_joint",
    "lf_upper_leg_joint",
    "lf_lower_leg_joint",
    "rf_hip_joint",
    "rf_upper_leg_joint",
    "rf_lower_leg_joint",
    "lh_hip_joint",
    "lh_upper_leg_joint",
    "lh_lower_leg_joint",
    "rh_hip_joint",
    "rh_upper_leg_joint",
    "rh_lower_leg_joint",
]


def champ_zero_angle(joint_name: str) -> float:
    """CHAMP/gait zero: upper+lower vertical (0), hip 0."""
    if "lower_leg" in joint_name:
        return CHAMP_ZERO_LOWER_RAD
    if "upper_leg" in joint_name:
        return CHAMP_ZERO_UPPER_RAD
    if "hip" in joint_name:
        return CHAMP_ZERO_HIP_RAD
    return 0.0


def servo_zero_targets() -> Dict[str, float]:
    """Joint angles that correspond to servo tick 0 (upper vert, lower horiz)."""
    return {n: float(c.urdf_home_rad) for n, c in JOINT_ID_MAP.items()}


def champ_zero_targets() -> Dict[str, float]:
    """Joint angles for CHAMP zero (both links vertical)."""
    return {n: champ_zero_angle(n) for n in JOINT_ID_MAP}


def load_home_ticks(path: Optional[str] = None) -> Dict[str, int]:
    """Load captured home tick positions (from qhome_champ)."""
    import yaml

    path = path or HOME_TICKS_FILE
    if not os.path.isfile(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    ticks = data.get("home_ticks", data) or {}
    out: Dict[str, int] = {}
    for name, val in ticks.items():
        if name in JOINT_ID_MAP:
            out[name] = int(val) % TICKS_PER_REV
    return out


def save_home_ticks(
    home_ticks: Dict[str, int], path: Optional[str] = None
) -> str:
    """Persist tick homes captured at vertical (CHAMP) alignment."""
    import yaml

    path = path or HOME_TICKS_FILE
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    payload = {
        "note": (
            "Absolute servo ticks at CHAMP_ZERO pose (upper+lower VERTICAL). "
            "Captured by qhome_champ; used as targets by qhome."
        ),
        "home_ticks": {
            k: int(v) % TICKS_PER_REV for k, v in home_ticks.items()
        },
    }
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, default_flow_style=False, sort_keys=False)
    return path


def angle_to_ticks(
    angle_rad: float,
    direction: int,
    urdf_home_rad: float,
    gear_ratio: float = 1.0,
    apply_gear: Optional[bool] = None,
) -> int:
    """
    Map CHAMP/URDF joint angle → servo ticks.

    `urdf_home_rad` is the joint angle at SERVO_ZERO (tick 0), e.g. lower=π/2
    (horizontal). CHAMP commanding 0 (vertical) therefore moves the lower
    servo ~90° off tick 0.

    ticks = HOME_TICK + direction*(angle − servo_zero)*scale [* gear later]
    """
    if apply_gear is None:
        apply_gear = APPLY_GEAR_RATIO
    delta_joint = direction * (angle_rad - urdf_home_rad)
    ratio = float(gear_ratio) if apply_gear else 1.0
    delta_servo = delta_joint * ratio
    ticks = int(round(HOME_TICK + delta_servo * (TICKS_PER_REV / (2.0 * math.pi))))
    return ticks % TICKS_PER_REV


def config_angle_to_ticks(
    cfg: JointConfig, angle_rad: float, apply_gear: Optional[bool] = None
) -> int:
    """Use direction, home, and (optional) gear_ratio from JointConfig."""
    return angle_to_ticks(
        angle_rad,
        cfg.direction,
        cfg.urdf_home_rad,
        cfg.gear_ratio,
        apply_gear=apply_gear,
    )


def shortest_tick_delta(current: int, target: int) -> int:
    """Signed shortest delta on a 4096-tick circle (−2047..+2048)."""
    d = (int(target) - int(current)) % TICKS_PER_REV
    if d > TICKS_PER_REV // 2:
        d -= TICKS_PER_REV
    return d


def next_tick_toward(
    current: int, target: int, step: int = SHORTEST_PATH_STEP
) -> int:
    """One hop along the shortest path from current → target (wrap-aware)."""
    d = shortest_tick_delta(current, target)
    if d == 0:
        return int(target) % TICKS_PER_REV
    if abs(d) <= step:
        return int(target) % TICKS_PER_REV
    return (int(current) + (step if d > 0 else -step)) % TICKS_PER_REV


def angle_to_pulse_us(angle_rad: float, cfg: JointConfig) -> int:
    delta = cfg.direction * (angle_rad - cfg.urdf_home_rad)
    us_per_rad = (cfg.pulse_max_us - cfg.pulse_min_us) / PWM_TRAVEL_RAD
    pulse = cfg.pulse_home_us + delta * us_per_rad
    return int(round(max(cfg.pulse_min_us, min(cfg.pulse_max_us, pulse))))


def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def speed_acc_for_model(model: str):
    if model == MODEL_HIP:
        return HIP_SPEED, HIP_ACC
    return DEFAULT_SPEED, DEFAULT_ACC


def load_directions(path: Optional[str] = None) -> Dict[str, int]:
    """
    Apply saved direction signs (+1/−1) onto JOINT_ID_MAP.

    Written by `servo_dir_config` / `qdir`. Returns the loaded map (may be empty).
    """
    import yaml  # local import keeps joint_map light if unused

    path = path or DIRECTIONS_FILE
    if not os.path.isfile(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    directions = data.get("directions", data) or {}
    loaded: Dict[str, int] = {}
    for name, val in directions.items():
        if name not in JOINT_ID_MAP:
            continue
        d = int(val)
        if d not in (-1, 1):
            d = 1 if d >= 0 else -1
        JOINT_ID_MAP[name].direction = d
        loaded[name] = d
    return loaded


def save_directions(
    path: Optional[str] = None, directions: Optional[Dict[str, int]] = None
) -> str:
    """Persist direction signs to YAML. Returns the path written."""
    import yaml

    path = path or DIRECTIONS_FILE
    if directions is None:
        directions = {n: int(c.direction) for n, c in JOINT_ID_MAP.items()}
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    payload = {
        "note": (
            "direction = +1 or -1. Maps URDF/controller positive angle to servo "
            "rotation. Produced by: ros2 run quad_servo_driver servo_dir_config"
        ),
        "directions": {k: int(v) for k, v in directions.items()},
    }
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, default_flow_style=False, sort_keys=False)
    # Keep in-memory map in sync
    for name, d in directions.items():
        if name in JOINT_ID_MAP:
            JOINT_ID_MAP[name].direction = int(d)
    return path


# Load overrides at import so all tools see the configured signs.
load_directions()
