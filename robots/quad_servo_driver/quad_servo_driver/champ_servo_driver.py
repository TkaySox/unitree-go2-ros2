#!/usr/bin/env python3
"""
CHAMP → Feetech hardware bridge (ROS 2 / rclpy).

Uses QuadServoHardware for calibration + bus I/O. Subscribes to joint_states
and commands each mapped servo.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from quad_servo_driver.hardware_interface import QuadServoHardware
from quad_servo_driver.joint_map import CALIBRATION_FILE, JOINT_ID_MAP, SERIAL_PORT

JOINT_STATES_TOPIC = "joint_states"


class ChampServoDriver(Node):
    def __init__(self) -> None:
        super().__init__("champ_servo_driver")
        self.declare_parameter("port", SERIAL_PORT)
        self.declare_parameter("calibration_file", CALIBRATION_FILE)
        self.declare_parameter("force_recalibrate", False)

        port = str(self.get_parameter("port").value)
        calib = str(self.get_parameter("calibration_file").value)
        force = bool(self.get_parameter("force_recalibrate").value)

        self._hw = QuadServoHardware(
            port=port, calibration_file=calib, logger=self.get_logger()
        )
        self._hw.connect()
        self._hw.ping_mapped()
        self._hw.load_or_calibrate(force=force)
        self._hw.enable_all_torque(True)

        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joint_states, 10)
        self.get_logger().info(
            f"HW ready — listening on '{JOINT_STATES_TOPIC}' "
            f"({len(self._hw.online_joints)}/{len(JOINT_ID_MAP)} online)."
        )

    def _on_joint_states(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        n = min(len(msg.name), len(msg.position))
        for i in range(n):
            name = msg.name[i]
            if name not in JOINT_ID_MAP:
                continue
            if self._hw.online_joints and name not in self._hw.online_joints:
                continue
            self._hw.command_rad(name, float(msg.position[i]))

    def destroy_node(self) -> bool:
        try:
            self._hw.enable_all_torque(False)
            self._hw.disconnect()
        except Exception as e:
            self.get_logger().warning(f"Shutdown HW error: {e}")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ChampServoDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
