#!/usr/bin/env python3
"""
CHAMP → Feetech hardware bridge (ROS 2 / rclpy).

On start (default): wait for the first /joint_states, read every online
servo's current ticks, and save offsets so those URDF angles map to the
pose the robot is in right now (no yank). Then follow joint_states.
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
        # Capture live ticks ↔ first joint_states so the robot does not jump.
        self.declare_parameter("capture_on_start", True)
        # If capture_on_start is false: load YAML offsets (apply_offsets true)
        # or raw URDF→ticks (apply_offsets false).
        self.declare_parameter("apply_offsets", True)
        self.declare_parameter("force_recalibrate", False)

        port = str(self.get_parameter("port").value)
        calib = str(self.get_parameter("calibration_file").value)
        self._capture = bool(self.get_parameter("capture_on_start").value)
        apply_offsets = bool(self.get_parameter("apply_offsets").value)
        force = bool(self.get_parameter("force_recalibrate").value)

        self._hw = QuadServoHardware(
            port=port,
            calibration_file=calib,
            logger=self.get_logger(),
            apply_offsets=True if self._capture else apply_offsets,
        )
        self._hw.connect()
        self._hw.ping_mapped()

        self._ready = False
        if self._capture:
            # Do NOT enable torque or command until we capture current pose.
            self.get_logger().warn(
                "Waiting for first /joint_states to capture current servo "
                "ticks as calibration (robot should already be in the pose "
                "you want — it will not be moved for capture)."
            )
        else:
            self._hw.load_or_calibrate(force=force)
            self._hw.enable_all_torque(True)
            self._ready = True
            self.get_logger().info(
                f"HW ready (no capture) — listening on '{JOINT_STATES_TOPIC}' "
                f"({len(self._hw.online_joints)}/{len(JOINT_ID_MAP)} online)."
            )

        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joint_states, 10)

    def _capture_from_msg(self, msg: JointState) -> None:
        angles = {}
        n = min(len(msg.name), len(msg.position))
        for i in range(n):
            name = msg.name[i]
            if name in JOINT_ID_MAP:
                angles[name] = float(msg.position[i])

        if not angles:
            self.get_logger().warning(
                "First joint_states had no mapped joints — still waiting…"
            )
            return

        self.get_logger().info(
            f"Capturing calibration from current ticks for {len(angles)} joints…"
        )
        self._hw.calibrate_from_angles(angles, save=True)
        self._hw.enable_all_torque(True)
        self._ready = True
        self._capture = False
        self.get_logger().info(
            "Capture done — offsets saved. Following joint_states "
            "(commanding the captured pose holds still)."
        )

    def _on_joint_states(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return

        if not self._ready:
            if self._capture:
                self._capture_from_msg(msg)
                if not self._ready:
                    return
            else:
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
