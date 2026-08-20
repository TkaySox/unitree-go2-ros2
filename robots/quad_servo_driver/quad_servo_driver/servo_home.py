#!/usr/bin/env python3
"""
Move joints to calibrated URDF home (mapped locations).

Does not recalibrate. Uses existing ~/.ros/quad_servo_calibration.yaml.

  # Legs only (skip PWM hips) — default
  ros2 run quad_servo_driver servo_home

  # Include hips too (needs gpiozero)
  ros2 run quad_servo_driver servo_home --ros-args -p include_hips:=true
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from quad_servo_driver.hardware_interface import QuadServoHardware
from quad_servo_driver.joint_map import CALIBRATION_FILE, SERIAL_PORT


class ServoHome(Node):
    def __init__(self) -> None:
        super().__init__("servo_home")
        self.declare_parameter("port", SERIAL_PORT)
        self.declare_parameter("calibration_file", CALIBRATION_FILE)
        self.declare_parameter("hold_sec", 2.0)
        self.declare_parameter("disable_torque_at_end", False)
        port = str(self.get_parameter("port").value)
        calib = str(self.get_parameter("calibration_file").value)
        hold = float(self.get_parameter("hold_sec").value)
        disable_end = bool(self.get_parameter("disable_torque_at_end").value)

        # All mapped serial joints (IDs 1–8 today; 9–12 skipped if offline).
        hw = QuadServoHardware(
            port=port,
            calibration_file=calib,
            logger=self.get_logger(),
            enable_pwm=False,
            enable_serial=True,
        )
        try:
            hw.connect()
            online = hw.ping_mapped()
            if not online:
                self.get_logger().error(
                    "No online joints. Is another node using the serial port? "
                    "Stop champ_servo_driver and check servo power."
                )
                return
            hw.load_or_calibrate(force=False)
            for name in online:
                hw.enable_torque(name, True)
            self.get_logger().info(f"Moving to mapped home: {online}")
            for name in online:
                home = hw.joint_config(name).urdf_home_rad
                ok = hw.command_rad(name, home)
                ticks = hw.read_ticks(name)
                self.get_logger().info(
                    f"  {name}: home_rad={home} ok={ok} ticks={ticks}"
                )
            time.sleep(hold)
            self.get_logger().info("At mapped home.")
            if disable_end:
                for name in online:
                    hw.enable_torque(name, False)
        finally:
            try:
                hw.disconnect()
            except Exception as e:
                self.get_logger().warning(f"Cleanup: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ServoHome()
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
