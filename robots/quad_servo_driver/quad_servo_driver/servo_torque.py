#!/usr/bin/env python3
"""
Enable or disable torque on mapped Feetech serial servos.

Does not move joints — only torque enable (hold) / disable (limp).

Examples:
  ros2 run quad_servo_driver servo_torque --ros-args -p enable:=true
  ros2 run quad_servo_driver servo_torque --ros-args -p enable:=false

  # aliases (after source ~/.bashrc):
  qton     # torque on
  qtoff    # torque off
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from quad_servo_driver.hardware_interface import QuadServoHardware
from quad_servo_driver.joint_map import CALIBRATION_FILE, SERIAL_PORT


class ServoTorque(Node):
    def __init__(self) -> None:
        super().__init__("servo_torque")
        self.declare_parameter("port", SERIAL_PORT)
        self.declare_parameter("calibration_file", CALIBRATION_FILE)
        self.declare_parameter("enable", True)
        self.declare_parameter("mapped_only", True)

        port = str(self.get_parameter("port").value)
        calib = str(self.get_parameter("calibration_file").value)
        enable = bool(self.get_parameter("enable").value)

        action = "ENABLE" if enable else "DISABLE"
        self.get_logger().warn(
            f"Torque {action} on mapped serial servos. "
            + ("Joints will hold position." if enable else "Joints will go limp.")
        )

        hw = QuadServoHardware(
            port=port,
            calibration_file=calib,
            logger=self.get_logger(),
            enable_pwm=False,
            enable_serial=True,
            apply_offsets=False,
        )
        try:
            hw.connect()
            online = hw.ping_mapped()
            if not online:
                self.get_logger().error(
                    "No mapped servos online — is the port free / powered?"
                )
                return

            ok_n = 0
            for name in online:
                if hw.enable_torque(name, enable):
                    ok_n += 1
                    self.get_logger().info(
                        f"  {name} id={hw.joint_config(name).servo_id}: "
                        f"torque={'ON' if enable else 'OFF'}"
                    )
                else:
                    self.get_logger().warning(f"  {name}: torque write failed")

            self.get_logger().info(
                f"Torque {action} done ({ok_n}/{len(online)} joints)."
            )
        finally:
            try:
                # Leave torque state as set; only close the serial port.
                hw.disconnect()
            except Exception as e:
                self.get_logger().warning(f"Cleanup: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ServoTorque()
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
