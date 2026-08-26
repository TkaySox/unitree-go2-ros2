#!/usr/bin/env python3
"""
Move joints to URDF home along the shortest servo path.

Home = tick 0 at the start pose (upper vertical, lower 90° forward).
If a servo is at ~350°, it wraps 350→351→…→0 instead of the long way.

  ros2 run quad_servo_driver servo_home
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
        self.declare_parameter("apply_offsets", False)
        self.declare_parameter("capture_first", False)
        self.declare_parameter("hold_sec", 1.0)
        self.declare_parameter("disable_torque_at_end", False)
        self.declare_parameter("step_ticks", 128)

        port = str(self.get_parameter("port").value)
        calib = str(self.get_parameter("calibration_file").value)
        apply_offsets = bool(self.get_parameter("apply_offsets").value)
        capture_first = bool(self.get_parameter("capture_first").value)
        hold = float(self.get_parameter("hold_sec").value)
        disable_end = bool(self.get_parameter("disable_torque_at_end").value)
        step = int(self.get_parameter("step_ticks").value)

        hw = QuadServoHardware(
            port=port,
            calibration_file=calib,
            logger=self.get_logger(),
            enable_pwm=False,
            enable_serial=True,
            apply_offsets=apply_offsets,
        )
        try:
            hw.connect()
            online = hw.ping_mapped()
            if not online:
                self.get_logger().error(
                    "No online joints. Is another node using the serial port?"
                )
                return

            if capture_first:
                # Bind current ticks to current URDF home angles (no motion).
                angles = {n: hw.joint_config(n).urdf_home_rad for n in online}
                hw.calibrate_from_angles(angles, save=True)
            else:
                hw.load_or_calibrate(force=False)

            for name in online:
                hw.enable_torque(name, True)

            self.get_logger().warn(
                f"Shortest-path home for {online} (0° = tick 0). "
                "Support the robot."
            )
            for name in online:
                self.get_logger().info(
                    f"  before {name}: ticks={hw.read_ticks(name)} "
                    f"home_rad={hw.joint_config(name).urdf_home_rad:.4f}"
                )

            results = hw.home_shortest(online, step=step)
            time.sleep(hold)

            for name in online:
                self.get_logger().info(
                    f"  after  {name}: ticks={hw.read_ticks(name)} "
                    f"ok={results.get(name)}"
                )
            self.get_logger().info("Shortest-path home complete.")

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
