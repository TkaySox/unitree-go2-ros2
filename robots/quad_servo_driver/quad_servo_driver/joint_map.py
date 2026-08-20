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

# ASSUMPTION: STS3215 uses STS/SMS, 4096 ticks/rev, mid=2048. VERIFY on hardware.
TICKS_PER_REV = 4096
CENTER_TICK = 2048

DEFAULT_SPEED = 60
DEFAULT_ACC = 20
HIP_SPEED = 40
HIP_ACC = 15

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


JOINT_ID_MAP: Dict[str, JointConfig] = {
    # --- Hips: PWM GPIO (FT5330M) — edit BCM pins ---
    "lf_hip_joint": JointConfig(MODEL_HIP, IFACE_PWM, gpio_pin=12),
    "rf_hip_joint": JointConfig(MODEL_HIP, IFACE_PWM, gpio_pin=13),
    "lh_hip_joint": JointConfig(MODEL_HIP, IFACE_PWM, gpio_pin=18),
    "rh_hip_joint": JointConfig(MODEL_HIP, IFACE_PWM, gpio_pin=19),
    # --- Upper / lower: USB serial (STS3215), IDs per user ---
    "rf_lower_leg_joint": JointConfig(MODEL_LEG, IFACE_SERIAL, servo_id=1),
    "rf_upper_leg_joint": JointConfig(MODEL_LEG, IFACE_SERIAL, servo_id=2),
    "lf_lower_leg_joint": JointConfig(MODEL_LEG, IFACE_SERIAL, servo_id=3),
    "lf_upper_leg_joint": JointConfig(MODEL_LEG, IFACE_SERIAL, servo_id=4),
    "rh_lower_leg_joint": JointConfig(MODEL_LEG, IFACE_SERIAL, servo_id=5),
    "rh_upper_leg_joint": JointConfig(MODEL_LEG, IFACE_SERIAL, servo_id=6),
    "lh_lower_leg_joint": JointConfig(MODEL_LEG, IFACE_SERIAL, servo_id=7),
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
    delta = direction * (angle_rad - urdf_home_rad)
    return int(round(CENTER_TICK + delta * (TICKS_PER_REV / (2.0 * math.pi))))


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
