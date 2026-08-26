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
# Home / 0° = tick 0 (user setup: upper vertical, lower 90° forward → all at 0°).
TICKS_PER_REV = 4096
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

MODEL_HIP = "FT5330M"
MODEL_LEG = "STS3215"

IFACE_SERIAL = "serial"
IFACE_PWM = "pwm"


@dataclass
class JointConfig:
    model: str
    interface: str  # "serial" | "pwm"
    direction: int = 1
    urdf_home_rad: float = 0.0
    servo_id: Optional[int] = None
    gpio_pin: Optional[int] = None
    pulse_min_us: int = PWM_PULSE_MIN_US
    pulse_max_us: int = PWM_PULSE_MAX_US
    pulse_home_us: int = PWM_PULSE_HOME_US


# Start pose: upper vertical (0), lower 90° forward (π/2). Servos at tick 0 there.
_LOWER_HOME = math.pi / 2.0

JOINT_ID_MAP: Dict[str, JointConfig] = {
    # --- Hips: PWM GPIO (FT5330M) — edit BCM pins ---
    "lf_hip_joint": JointConfig(MODEL_HIP, IFACE_PWM, gpio_pin=12),
    "rf_hip_joint": JointConfig(MODEL_HIP, IFACE_PWM, gpio_pin=13),
    "lh_hip_joint": JointConfig(MODEL_HIP, IFACE_PWM, gpio_pin=18),
    "rh_hip_joint": JointConfig(MODEL_HIP, IFACE_PWM, gpio_pin=19),
    # --- Upper / lower: USB serial (STS3215), IDs per user ---
    "rf_lower_leg_joint": JointConfig(
        MODEL_LEG, IFACE_SERIAL, servo_id=1, urdf_home_rad=_LOWER_HOME
    ),
    "rf_upper_leg_joint": JointConfig(MODEL_LEG, IFACE_SERIAL, servo_id=2),
    "lf_lower_leg_joint": JointConfig(
        MODEL_LEG, IFACE_SERIAL, servo_id=3, urdf_home_rad=_LOWER_HOME
    ),
    "lf_upper_leg_joint": JointConfig(MODEL_LEG, IFACE_SERIAL, servo_id=4),
    "rh_lower_leg_joint": JointConfig(
        MODEL_LEG, IFACE_SERIAL, servo_id=5, urdf_home_rad=_LOWER_HOME
    ),
    "rh_upper_leg_joint": JointConfig(MODEL_LEG, IFACE_SERIAL, servo_id=6),
    "lh_lower_leg_joint": JointConfig(
        MODEL_LEG, IFACE_SERIAL, servo_id=7, urdf_home_rad=_LOWER_HOME
    ),
    "lh_upper_leg_joint": JointConfig(MODEL_LEG, IFACE_SERIAL, servo_id=8),
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


def angle_to_ticks(angle_rad: float, direction: int, urdf_home_rad: float) -> int:
    """Map URDF angle to servo ticks. Home / 0° → HOME_TICK (0), wraps 0..4095."""
    delta = direction * (angle_rad - urdf_home_rad)
    ticks = int(round(HOME_TICK + delta * (TICKS_PER_REV / (2.0 * math.pi))))
    return ticks % TICKS_PER_REV


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
