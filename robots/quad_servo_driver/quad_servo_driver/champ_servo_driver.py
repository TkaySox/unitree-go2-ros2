#!/usr/bin/env python3
"""
CHAMP → Feetech hardware bridge (ROS 2 / rclpy).

Pose model (see joint_map.py):
  CHAMP 0 on upper+lower = both links VERTICAL
  Servo tick 0           = upper VERTICAL, lower HORIZONTAL

So when CHAMP stands at 0/0, lowers are commanded ~90° off mechanical zero.

Default: use that geometric map (no live capture). Optional capture still
available if you really want to lock whatever pose you're in.
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
        # Default OFF: use SERVO_ZERO vs CHAMP_ZERO geometry in joint_map.
        self.declare_parameter("capture_on_start", False)
        self.declare_parameter("apply_offsets", False)
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
            self.get_logger().warn(
                "capture_on_start:=true — waiting for first /joint_states to "
                "bind current ticks (overrides geometric SERVO_ZERO map)."
            )
        else:
            self._hw.load_or_calibrate(force=force)
            self._hw.enable_all_torque(True)
            self._ready = True
            self.get_logger().info(
                "HW ready — CHAMP angles → servos with SERVO_ZERO map "
                "(upper vert / lower horiz at tick 0; CHAMP 0 = both vertical). "
                f"Listening on '{JOINT_STATES_TOPIC}' "
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
            "Capture done — offsets saved. Following joint_states."
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
