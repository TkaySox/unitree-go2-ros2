#!/usr/bin/env python3
"""
Read-only Feetech servo diagnostics (no motion commands).

Uses the same scservo_sdk / SCServo_Linux STS register map as the driver.
Useful to verify USB port, IDs, bus health, and live position/voltage/temp.

Examples:
  ros2 run quad_servo_driver servo_diag
  ros2 run quad_servo_driver servo_diag --ros-args -p port:=/dev/ttyACM0
  ros2 run quad_servo_driver servo_diag --ros-args -p scan_max:=20 -p once:=true
  QUAD_SERVO_PORT=/dev/ttyUSB0 ros2 run quad_servo_driver servo_diag
"""

from __future__ import annotations

import os
from typing import List

import rclpy
from rclpy.node import Node

from quad_servo_driver.joint_map import JOINT_ID_MAP
from quad_servo_driver.scservo_bus import (
    ADDR_MAX_ANGLE_LIMIT,
    ADDR_MAX_TORQUE,
    ADDR_MIN_ANGLE_LIMIT,
    ADDR_MODE,
    ADDR_MOVING,
    ADDR_PRESENT_CURRENT,
    ADDR_PRESENT_LOAD,
    ADDR_PRESENT_TEMPERATURE,
    ADDR_PRESENT_VOLTAGE,
    ADDR_TORQUE_ENABLE,
    ScsBus,
)


class ServoDiag(Node):
    def __init__(self) -> None:
        super().__init__("servo_diag")

        self.declare_parameter("port", os.environ.get("QUAD_SERVO_PORT", "/dev/ttyACM0"))
        self.declare_parameter("baudrate", 1_000_000)
        self.declare_parameter("scan_min", 1)
        self.declare_parameter("scan_max", 20)
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("once", False)
        # If true, only probe IDs listed in JOINT_ID_MAP (skip full scan).
        self.declare_parameter("mapped_only", False)

        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baudrate").value)
        self._once = bool(self.get_parameter("once").value)
        self._scan_min = int(self.get_parameter("scan_min").value)
        self._scan_max = int(self.get_parameter("scan_max").value)
        self._mapped_only = bool(self.get_parameter("mapped_only").value)

        from quad_servo_driver.joint_map import IFACE_PWM, IFACE_SERIAL

        self._id_to_joint = {
            cfg.servo_id: name
            for name, cfg in JOINT_ID_MAP.items()
            if cfg.interface == IFACE_SERIAL and cfg.servo_id is not None
        }
        for name, cfg in JOINT_ID_MAP.items():
            if cfg.interface == IFACE_PWM:
                self.get_logger().info(
                    f"PWM hip (no serial read): {name} BCM{cfg.gpio_pin} ({cfg.model})"
                )

        self._bus = ScsBus(port, baud, protocol_end=0)
        self._bus.open()
        self.get_logger().info(f"Diag open {port} @ {baud} (serial read-only)")

        self._ids: List[int] = self._discover()
        if not self._ids:
            self.get_logger().error(
                "No serial servos responded to ping. Check power/USB/IDs."
            )
        else:
            self.get_logger().info(f"Found servo IDs: {self._ids}")

        self._did_once = False
        if self._once:
            self._print_all()
            self._did_once = True
            return

        period = 1.0 / max(0.1, float(self.get_parameter("rate_hz").value))
        self.create_timer(period, self._print_all)

    def _discover(self) -> List[int]:
        from quad_servo_driver.joint_map import IFACE_SERIAL

        found = []
        if self._mapped_only:
            candidates = sorted(
                {
                    c.servo_id
                    for c in JOINT_ID_MAP.values()
                    if c.interface == IFACE_SERIAL and c.servo_id is not None
                }
            )
        else:
            candidates = list(range(self._scan_min, self._scan_max + 1))
        for sid in candidates:
            model, result, _err = self._bus.ping(sid)
            if model is not None and result == self._bus.ok:
                joint = self._id_to_joint.get(sid, "?")
                self.get_logger().info(
                    f"  ping OK id={sid} model_num={model} joint={joint}"
                )
                found.append(sid)
        return found

    def _print_all(self) -> None:
        if not self._ids:
            return
        lines = ["----- servo diag -----"]
        for sid in self._ids:
            joint = self._id_to_joint.get(sid, "?")
            block = self._read_one(sid, joint)
            lines.append(block)
        self.get_logger().info("\n".join(lines))

    def _read_one(self, sid: int, joint: str) -> str:
        pos, speed, r, e = self._bus.read_pos_speed(sid)
        if pos is None:
            return (
                f"id={sid:3d} ({joint}): READ FAIL "
                f"{self._bus.result_str(r)} {self._bus.error_str(e)}"
            )

        load, _, _ = self._bus.read_u16(sid, ADDR_PRESENT_LOAD)
        volt, _, _ = self._bus.read_u8(sid, ADDR_PRESENT_VOLTAGE)
        temp, _, _ = self._bus.read_u8(sid, ADDR_PRESENT_TEMPERATURE)
        moving, _, _ = self._bus.read_u8(sid, ADDR_MOVING)
        current, _, _ = self._bus.read_u16(sid, ADDR_PRESENT_CURRENT)
        torque_en, _, _ = self._bus.read_u8(sid, ADDR_TORQUE_ENABLE)
        mode, _, _ = self._bus.read_u8(sid, ADDR_MODE)
        amin, _, _ = self._bus.read_u16(sid, ADDR_MIN_ANGLE_LIMIT)
        amax, _, _ = self._bus.read_u16(sid, ADDR_MAX_ANGLE_LIMIT)
        tmax, _, _ = self._bus.read_u16(sid, ADDR_MAX_TORQUE)

        # Load is often signed in high bits; show raw for debug.
        volt_v = (volt * 0.1) if volt is not None else float("nan")
        return (
            f"id={sid:3d} ({joint}): "
            f"pos={pos:4d} spd={speed:5d} load_raw={load} "
            f"V={volt_v:4.1f} T={temp}C moving={moving} I_raw={current} "
            f"torque_en={torque_en} mode={mode} "
            f"ang_lim=[{amin},{amax}] max_torque={tmax}"
        )

    def destroy_node(self) -> bool:
        try:
            self._bus.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ServoDiag()
        if not getattr(node, "_did_once", False):
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
