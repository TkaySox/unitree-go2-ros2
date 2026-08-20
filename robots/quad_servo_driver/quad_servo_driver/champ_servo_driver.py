#!/usr/bin/env python3
"""
CHAMP → Feetech serial-bus servo bridge (ROS 2 / rclpy).

Subscribes to CHAMP joint angles (sensor_msgs/JointState on `joint_states`),
converts radians → servo ticks, applies per-joint mechanical calibration
offsets, and writes positions over USB (Waveshare USB serial bus adapter).

Calibration (once, then cached in YAML):
  Pose the robot in the URDF "home" / stand pose, start this node with no
  calibration file (or delete the file to force re-cal). The node reads each
  servo's raw tick, compares to the tick that URDF home would map to, and
  stores (raw - ideal_home) per joint. Later boots reload that YAML.

Fill in JOINT_ID_MAP / SERIAL_PORT before running on hardware.
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState

# =============================================================================
# USER CONFIG — edit these
# =============================================================================

# Waveshare USB serial bus servo adapter (NOT the Pi's GPIO UART / ttyAMA0).
SERIAL_PORT = os.environ.get("QUAD_SERVO_PORT", "/dev/ttyUSB0")
BAUDRATE = 1_000_000  # common Feetech default; change if your bus differs

# Where calibration offsets are saved/loaded (absolute or relative path OK).
CALIBRATION_FILE = os.environ.get(
    "QUAD_SERVO_CALIB",
    os.path.expanduser("~/.ros/quad_servo_calibration.yaml"),
)

# Topic CHAMP uses when not in Gazebo (publish_joint_states:=true, gazebo:=false).
JOINT_STATES_TOPIC = "joint_states"

# Ticks: ASSUMPTION — both FT5330M (hips) and STS3215 use the STS/SCS protocol
# with 4096 ticks per revolution and mid-position 2048 = 0 rad. VERIFY on
# hardware (datasheets / SDK examples); FT5330M may differ — update TICKS_PER_REV
# / CENTER_TICK or split per-model if needed.
TICKS_PER_REV = 4096
CENTER_TICK = 2048

# Default motion profile (WritePosEx). Hips can use HIP_* below.
DEFAULT_SPEED = 60
DEFAULT_ACC = 20
HIP_SPEED = 40
HIP_ACC = 15

# Model tags for documentation / future per-model tuning.
MODEL_HIP = "FT5330M"  # 35 kg-cm — hip joints
MODEL_LEG = "STS3215"  # upper + lower leg joints


@dataclass
class JointConfig:
    servo_id: int
    model: str
    direction: int = 1  # +1 or -1 if servo axis is opposite URDF
    urdf_home_rad: float = 0.0  # URDF neutral angle for this joint (rad)


# URDF joint name → servo config. FILL IN real bus IDs.
# Hip joints → FT5330M; upper/lower → STS3215 (per your hardware plan).
JOINT_ID_MAP: Dict[str, JointConfig] = {
    # Left front
    "lf_hip_joint": JointConfig(servo_id=1, model=MODEL_HIP, direction=1, urdf_home_rad=0.0),
    "lf_upper_leg_joint": JointConfig(servo_id=2, model=MODEL_LEG, direction=1, urdf_home_rad=0.0),
    "lf_lower_leg_joint": JointConfig(servo_id=3, model=MODEL_LEG, direction=1, urdf_home_rad=0.0),
    # Right front
    "rf_hip_joint": JointConfig(servo_id=4, model=MODEL_HIP, direction=1, urdf_home_rad=0.0),
    "rf_upper_leg_joint": JointConfig(servo_id=5, model=MODEL_LEG, direction=1, urdf_home_rad=0.0),
    "rf_lower_leg_joint": JointConfig(servo_id=6, model=MODEL_LEG, direction=1, urdf_home_rad=0.0),
    # Left hind
    "lh_hip_joint": JointConfig(servo_id=7, model=MODEL_HIP, direction=1, urdf_home_rad=0.0),
    "lh_upper_leg_joint": JointConfig(servo_id=8, model=MODEL_LEG, direction=1, urdf_home_rad=0.0),
    "lh_lower_leg_joint": JointConfig(servo_id=9, model=MODEL_LEG, direction=1, urdf_home_rad=0.0),
    # Right hind
    "rh_hip_joint": JointConfig(servo_id=10, model=MODEL_HIP, direction=1, urdf_home_rad=0.0),
    "rh_upper_leg_joint": JointConfig(servo_id=11, model=MODEL_LEG, direction=1, urdf_home_rad=0.0),
    "rh_lower_leg_joint": JointConfig(servo_id=12, model=MODEL_LEG, direction=1, urdf_home_rad=0.0),
}

# =============================================================================
# SDK import (Feetech packages vary by install name)
# =============================================================================


def _import_scservo_sdk():
    """
    Try common Feetech SDK package names. API expected:
      PortHandler, sms_sts (or equivalent), COMM_SUCCESS,
      portHandler.openPort/setBaudRate/closePort,
      packetHandler.ReadPos / WritePosEx
    """
    errors = []
    for mod_name in ("scservo_sdk", "STservo_sdk", "stservo_sdk", "feetech_servo_sdk"):
        try:
            mod = __import__(mod_name)
            return mod
        except ImportError as e:
            errors.append(f"{mod_name}: {e}")
    raise ImportError(
        "Could not import a Feetech servo SDK. Tried: "
        + ", ".join(errors)
        + ". Install scservo_sdk / STservo_sdk and ensure it is on PYTHONPATH."
    )


# =============================================================================
# Angle ↔ tick helpers
# =============================================================================


def angle_to_ticks(angle_rad: float, direction: int, urdf_home_rad: float) -> int:
    """
    Map URDF joint angle (rad) to ideal servo ticks (before calibration offset).

    CENTER_TICK (2048) corresponds to urdf_home_rad when direction=+1 and
    angle == urdf_home_rad → result is CENTER_TICK.
    """
    delta = direction * (angle_rad - urdf_home_rad)
    ticks = CENTER_TICK + delta * (TICKS_PER_REV / (2.0 * math.pi))
    return int(round(ticks))


def clamp_ticks(ticks: int) -> int:
    return max(0, min(TICKS_PER_REV - 1, ticks))


def speed_acc_for_model(model: str) -> tuple:
    if model == MODEL_HIP:
        return HIP_SPEED, HIP_ACC
    return DEFAULT_SPEED, DEFAULT_ACC


# =============================================================================
# Node
# =============================================================================


class ChampServoDriver(Node):
    def __init__(self) -> None:
        super().__init__("champ_servo_driver")

        self._sdk = _import_scservo_sdk()
        self._port = None
        self._packet = None
        self._comm_success = getattr(self._sdk, "COMM_SUCCESS", 0)

        # joint_name -> tick offset (raw_at_cal - ideal_home_ticks)
        self._offsets: Dict[str, int] = {}

        self._open_port()
        self._load_or_run_calibration()

        self._sub = self.create_subscription(
            JointState,
            JOINT_STATES_TOPIC,
            self._on_joint_states,
            10,
        )
        self.get_logger().info(
            f"Listening on '{JOINT_STATES_TOPIC}' for {len(JOINT_ID_MAP)} mapped joints."
        )

    # ----- serial -------------------------------------------------------------

    def _open_port(self) -> None:
        PortHandler = self._sdk.PortHandler
        # sms_sts is the usual STS/SCS packet handler class name
        handler_cls = getattr(self._sdk, "sms_sts", None) or getattr(
            self._sdk, "sms", None
        )
        if handler_cls is None:
            raise RuntimeError(
                "SDK has no sms_sts/sms packet handler — check Feetech SDK version."
            )

        self._port = PortHandler(SERIAL_PORT)
        if not self._port.openPort():
            raise RuntimeError(f"Failed to open serial port {SERIAL_PORT}")
        if not self._port.setBaudRate(BAUDRATE):
            self._port.closePort()
            raise RuntimeError(f"Failed to set baudrate {BAUDRATE} on {SERIAL_PORT}")

        self._packet = handler_cls(self._port)
        self.get_logger().info(f"Opened {SERIAL_PORT} @ {BAUDRATE}")

    def _close_port(self) -> None:
        if self._port is not None:
            try:
                self._port.closePort()
                self.get_logger().info("Serial port closed.")
            except Exception as e:
                self.get_logger().warning(f"Error closing serial port: {e}")
            self._port = None

    def _read_pos(self, servo_id: int) -> Optional[int]:
        pos, result, error = self._packet.ReadPos(servo_id)
        if result != self._comm_success:
            self.get_logger().warning(
                f"ReadPos failed id={servo_id} result={result} error={error}"
            )
            return None
        return int(pos)

    def _write_pos(self, servo_id: int, ticks: int, model: str) -> bool:
        speed, acc = speed_acc_for_model(model)
        ticks = clamp_ticks(ticks)
        result, error = self._packet.WritePosEx(servo_id, ticks, speed, acc)
        if result != self._comm_success:
            self.get_logger().warning(
                f"WritePosEx failed id={servo_id} ticks={ticks} "
                f"result={result} error={error}"
            )
            return False
        return True

    # ----- calibration --------------------------------------------------------

    def _load_or_run_calibration(self) -> None:
        """
        If CALIBRATION_FILE exists → load offsets.
        Else → read each servo (robot must be in URDF home pose), compute
        offset = raw - ideal_home_ticks, save YAML, use those offsets.
        """
        if os.path.isfile(CALIBRATION_FILE):
            self._load_calibration(CALIBRATION_FILE)
            return

        self.get_logger().warn(
            f"No calibration file at {CALIBRATION_FILE}. "
            "Reading servos now — robot should already be in URDF HOME pose."
        )
        self._calibrate_from_servos()
        self._save_calibration(CALIBRATION_FILE)

    def _calibrate_from_servos(self) -> None:
        for name, cfg in JOINT_ID_MAP.items():
            ideal_home = angle_to_ticks(cfg.urdf_home_rad, cfg.direction, cfg.urdf_home_rad)
            # With home mapping above, ideal_home == CENTER_TICK always.
            raw = self._read_pos(cfg.servo_id)
            if raw is None:
                self.get_logger().error(
                    f"Calibration skipped for {name} (id={cfg.servo_id}) — using offset 0."
                )
                self._offsets[name] = 0
                continue
            offset = raw - ideal_home
            self._offsets[name] = offset
            self.get_logger().info(
                f"CAL {name} (id={cfg.servo_id}, {cfg.model}): "
                f"raw={raw} ideal_home={ideal_home} offset={offset}"
            )

    def _load_calibration(self, path: str) -> None:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        offsets = data.get("offsets", data)
        for name in JOINT_ID_MAP:
            if name not in offsets:
                self.get_logger().warning(
                    f"Calibration file missing '{name}' — using offset 0."
                )
                self._offsets[name] = 0
            else:
                self._offsets[name] = int(offsets[name])
            self.get_logger().info(
                f"LOAD CAL {name}: offset={self._offsets[name]}"
            )
        self.get_logger().info(f"Loaded calibration from {path}")

    def _save_calibration(self, path: str) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
        payload = {
            "note": (
                "offset = raw_tick_at_URDF_home - ideal_home_ticks "
                f"(ideal_home uses CENTER_TICK={CENTER_TICK})"
            ),
            "serial_port": SERIAL_PORT,
            "offsets": {k: int(v) for k, v in self._offsets.items()},
            "models": {k: v.model for k, v in JOINT_ID_MAP.items()},
            "servo_ids": {k: v.servo_id for k, v in JOINT_ID_MAP.items()},
        }
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(payload, f, default_flow_style=False, sort_keys=False)
        self.get_logger().info(f"Saved calibration to {path}")

    # ----- CHAMP callback -----------------------------------------------------

    def _on_joint_states(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        n = min(len(msg.name), len(msg.position))
        for i in range(n):
            name = msg.name[i]
            cfg = JOINT_ID_MAP.get(name)
            if cfg is None:
                # CHAMP may publish extras (feet, etc.) — ignore quietly.
                continue
            angle = float(msg.position[i])
            ideal = angle_to_ticks(angle, cfg.direction, cfg.urdf_home_rad)
            target = ideal + self._offsets.get(name, 0)
            self._write_pos(cfg.servo_id, target, cfg.model)

    # ----- shutdown -----------------------------------------------------------

    def destroy_node(self) -> bool:
        self._close_port()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ChampServoDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # Ensure serial closes even on init failure after port open.
        print(f"champ_servo_driver fatal: {e}", file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
