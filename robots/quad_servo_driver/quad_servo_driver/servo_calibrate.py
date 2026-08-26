#!/usr/bin/env python3
"""
Calibration helper (no motion sweep).

Default (apply_offsets:=false): writes an identity YAML (all offsets 0).
Use this when the physical pose already matches RViz/URDF.

Only measure mechanical offsets if horns do NOT match RViz:
  ros2 run quad_servo_driver servo_calibrate --ros-args -p apply_offsets:=true
  (robot must be in URDF HOME pose)

Writes ~/.ros/quad_servo_calibration.yaml (or QUAD_SERVO_CALIB / -p calibration_file).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from quad_servo_driver.hardware_interface import QuadServoHardware
from quad_servo_driver.joint_map import CALIBRATION_FILE, SERIAL_PORT


class ServoCalibrate(Node):
    def __init__(self) -> None:
        super().__init__("servo_calibrate")
        self.declare_parameter("port", SERIAL_PORT)
        self.declare_parameter("calibration_file", CALIBRATION_FILE)
        self.declare_parameter("apply_offsets", False)

        port = str(self.get_parameter("port").value)
        calib = str(self.get_parameter("calibration_file").value)
        apply_offsets = bool(self.get_parameter("apply_offsets").value)

        if apply_offsets:
            self.get_logger().warn(
                "RECALIBRATE offsets: robot must already be in URDF HOME pose. "
                "No joints will be moved."
            )
        else:
            self.get_logger().info(
                "Writing identity calibration (offsets=0). URDF angles → ticks directly."
            )

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
                self.get_logger().error("No mapped servos online — abort.")
                return
            hw.load_or_calibrate(force=True)
            self.get_logger().info(
                f"Calibration saved to {calib} "
                f"(apply_offsets={apply_offsets}). Online: {online}"
            )
        finally:
            try:
                hw.disconnect()
            except Exception as e:
                self.get_logger().warning(f"Cleanup: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ServoCalibrate()
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
