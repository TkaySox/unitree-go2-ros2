#!/usr/bin/env python3
"""
Compute calibration offsets from live RViz/CHAMP joint_states + servo ticks.

  offset = raw_ticks_now − angle_to_ticks(joint_angle_now)

Put the physical robot in the pose that matches what RViz shows, then run:

  # Terminal 1: qhw   (CHAMP + RViz publishing /joint_states)
  # Terminal 2:
  ros2 run quad_servo_driver servo_sync_offsets

Or assume stand (all 0) and current ticks (default if no joint_states yet
with -p assume_stand:=true).

Shortcut idea: qsync
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from quad_servo_driver.hardware_interface import QuadServoHardware
from quad_servo_driver.joint_map import (
    CALIBRATION_FILE,
    IFACE_SERIAL,
    JOINT_ID_MAP,
    JOINT_ORDER,
    SERIAL_PORT,
    config_angle_to_ticks,
)


class ServoSyncOffsets(Node):
    def __init__(self) -> None:
        super().__init__("servo_sync_offsets")
        self.declare_parameter("port", SERIAL_PORT)
        self.declare_parameter("calibration_file", CALIBRATION_FILE)
        self.declare_parameter("timeout_sec", 10.0)
        # If True and no joint_states arrive, use angle=0 (RViz/CHAMP stand).
        self.declare_parameter("assume_stand", True)
        self.declare_parameter("save", True)

        port = str(self.get_parameter("port").value)
        calib = str(self.get_parameter("calibration_file").value)
        timeout = float(self.get_parameter("timeout_sec").value)
        assume_stand = bool(self.get_parameter("assume_stand").value)
        do_save = bool(self.get_parameter("save").value)

        self._angles: dict[str, float] | None = None
        self.create_subscription(JointState, "joint_states", self._on_js, 10)

        self.get_logger().info(
            "Waiting for /joint_states (from qhw / CHAMP) to pair with servo ticks…"
        )
        t0 = time.time()
        while self._angles is None and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)

        if self._angles is None:
            if assume_stand:
                self.get_logger().warn(
                    "No /joint_states — assuming RViz/CHAMP stand = all joints 0 rad."
                )
                self._angles = {
                    n: 0.0
                    for n, c in JOINT_ID_MAP.items()
                    if c.interface == IFACE_SERIAL
                }
            else:
                self.get_logger().error(
                    "No /joint_states. Start qhw first, or use -p assume_stand:=true"
                )
                return

        hw = QuadServoHardware(
            port=port,
            calibration_file=calib,
            logger=self.get_logger(),
            enable_pwm=False,
            enable_serial=True,
            apply_offsets=True,
        )
        try:
            hw.connect()
            online = hw.ping_mapped()
            if not online:
                self.get_logger().error("No servos online.")
                return

            print()
            print("RViz/CHAMP angle  |  servo ticks  |  ideal  |  offset=raw−ideal")
            print("-" * 72)
            offsets: dict[str, int] = {n: 0 for n in JOINT_ORDER}
            for name in JOINT_ORDER:
                cfg = JOINT_ID_MAP[name]
                if cfg.interface != IFACE_SERIAL:
                    continue
                if name not in online:
                    self.get_logger().warning(f"SKIP {name} (offline)")
                    continue
                angle = float(self._angles.get(name, cfg.urdf_home_rad))
                ideal = config_angle_to_ticks(cfg, angle)
                raw = hw.read_ticks(name)
                if raw is None:
                    self.get_logger().warning(f"SKIP {name} (no read)")
                    continue
                off = int(raw) - int(ideal)
                offsets[name] = off
                print(
                    f"{name:22s}  ang={angle:+7.4f}  "
                    f"raw={raw:4d}  ideal={ideal:4d}  offset={off:+5d}"
                )

            hw.offsets = {**{n: 0 for n in JOINT_ORDER}, **offsets}
            hw.apply_offsets = True
            if do_save:
                hw._save_calibration()
                self.get_logger().info(f"Saved offsets → {calib}")
                self.get_logger().info(
                    "Restart qdrv with: "
                    "ros2 run … champ_servo_driver --ros-args "
                    "-p capture_on_start:=false -p apply_offsets:=true"
                )
            print()
        finally:
            try:
                hw.disconnect()
            except Exception as e:
                self.get_logger().warning(f"Cleanup: {e}")

    def _on_js(self, msg: JointState) -> None:
        if self._angles is not None:
            return
        if not msg.name or not msg.position:
            return
        angles = {}
        n = min(len(msg.name), len(msg.position))
        for i in range(n):
            name = msg.name[i]
            if name in JOINT_ID_MAP:
                angles[name] = float(msg.position[i])
        if angles:
            self._angles = angles
            self.get_logger().info(f"Got joint_states for {len(angles)} mapped joints.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ServoSyncOffsets()
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
