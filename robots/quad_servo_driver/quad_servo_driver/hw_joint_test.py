#!/usr/bin/env python3
"""
Hardware bring-up test for the quad Feetech bus.

1) Ping mapped servos
2) Load mapping (default: no offsets — URDF rad → ticks directly)
3) Enable torque, go to home
4) Move ALL online joints +N degrees at the same time, hold, return home together

Examples:
  ros2 run quad_servo_driver hw_joint_test
  ros2 run quad_servo_driver hw_joint_test --ros-args -p delta_deg:=10.0
  # old one-at-a-time behavior:
  ros2 run quad_servo_driver hw_joint_test --ros-args -p simultaneous:=false
  # only if horns do NOT match RViz (rare):
  ros2 run quad_servo_driver hw_joint_test --ros-args -p apply_offsets:=true -p force_recalibrate:=true
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from quad_servo_driver.hardware_interface import QuadServoHardware
from quad_servo_driver.joint_map import (
    CALIBRATION_FILE,
    JOINT_ORDER,
    SERIAL_PORT,
    deg_to_rad,
)


class HwJointTest(Node):
    def __init__(self) -> None:
        super().__init__("hw_joint_test")

        self.declare_parameter("port", SERIAL_PORT)
        self.declare_parameter("calibration_file", CALIBRATION_FILE)
        self.declare_parameter("force_recalibrate", False)
        self.declare_parameter("apply_offsets", False)
        self.declare_parameter("delta_deg", 20.0)
        self.declare_parameter("hold_sec", 2.0)
        self.declare_parameter("settle_sec", 1.0)
        self.declare_parameter("return_home", True)
        self.declare_parameter("disable_torque_at_end", True)
        self.declare_parameter("simultaneous", True)

        port = str(self.get_parameter("port").value)
        calib = str(self.get_parameter("calibration_file").value)
        force = bool(self.get_parameter("force_recalibrate").value)
        apply_offsets = bool(self.get_parameter("apply_offsets").value)
        delta_deg = float(self.get_parameter("delta_deg").value)
        hold = float(self.get_parameter("hold_sec").value)
        settle = float(self.get_parameter("settle_sec").value)
        return_home = bool(self.get_parameter("return_home").value)
        disable_end = bool(self.get_parameter("disable_torque_at_end").value)
        simultaneous = bool(self.get_parameter("simultaneous").value)

        delta_rad = deg_to_rad(delta_deg)

        mode = "ALL AT ONCE" if simultaneous else "one-at-a-time"
        offset_mode = "with offsets" if apply_offsets else "no offsets (URDF direct)"
        self.get_logger().warn(
            f"HW TEST: +{delta_deg:.1f}° ({mode}, {offset_mode}). "
            "Support the robot / keep clear of pinch points."
        )

        hw = QuadServoHardware(
            port=port,
            calibration_file=calib,
            logger=self.get_logger(),
            enable_pwm=False,
            enable_serial=True,
            apply_offsets=apply_offsets,
        )
        self._hw = hw

        try:
            hw.connect()
            online = hw.ping_mapped()
            if not online:
                self.get_logger().error("No mapped servos online — abort.")
                return

            hw.load_or_calibrate(force=force)
            self.get_logger().info(
                f"Mapping ready ({offset_mode}). Online joints ({len(online)}): {online}"
            )

            # Home all along shortest tick path (wrap-aware, e.g. 350°→0°)
            hw.enable_all_torque(True)
            home_targets = {
                name: hw.joint_config(name).urdf_home_rad for name in online
            }
            self.get_logger().info(
                f"Shortest-path home for {len(online)} joints (0° = tick 0)…"
            )
            home_ok = hw.move_to_angles_shortest(home_targets)
            for name, ok in home_ok.items():
                self.get_logger().info(
                    f"  home {name} id={hw.joint_config(name).servo_id} "
                    f"ok={ok} ticks={hw.read_ticks(name)}"
                )
            time.sleep(settle + 0.5)

            if simultaneous:
                # One SyncWrite: every online joint to home+delta together
                move_targets = {
                    name: hw.joint_config(name).urdf_home_rad + delta_rad
                    for name in online
                }
                self.get_logger().info(
                    f"MOVE ALL +{delta_deg:.1f}° simultaneously (sync write): {online}"
                )
                for name in online:
                    cfg = hw.joint_config(name)
                    self.get_logger().info(
                        f"  {name} id={cfg.servo_id}: "
                        f"target={move_targets[name]:.4f} rad "
                        f"ticks_now={hw.read_ticks(name)}"
                    )
                move_ok = hw.command_rad_many(move_targets)
                for name, ok in move_ok.items():
                    self.get_logger().info(f"  sync cmd {name} ok={ok}")
                time.sleep(hold)

                for name in online:
                    self.get_logger().info(
                        f"  {name} arrived ticks={hw.read_ticks(name)}"
                    )

                if return_home:
                    self.get_logger().info(
                        "Returning ALL to home (shortest path)…"
                    )
                    home_ok = hw.move_to_angles_shortest(home_targets)
                    time.sleep(settle)
                    for name in online:
                        self.get_logger().info(
                            f"  {name} home ok={home_ok.get(name)} "
                            f"ticks={hw.read_ticks(name)}"
                        )
            else:
                # Sequential (legacy)
                for name in JOINT_ORDER:
                    if name not in online:
                        self.get_logger().warning(f"SKIP {name} (not online)")
                        continue
                    cfg = hw.joint_config(name)
                    home = cfg.urdf_home_rad
                    target = home + delta_rad
                    before = hw.read_ticks(name)
                    self.get_logger().info(
                        f"MOVE {name} (id={cfg.servo_id}): "
                        f"+{delta_deg:.1f}° ticks_now={before}"
                    )
                    hw.command_rad(name, target)
                    time.sleep(hold)
                    self.get_logger().info(
                        f"  arrived ticks={hw.read_ticks(name)}"
                    )
                    if return_home:
                        hw.move_to_angles_shortest({name: home})
                        time.sleep(settle)

            self.get_logger().info("Sweep complete.")
        finally:
            try:
                if disable_end:
                    self.get_logger().info("Disabling torque.")
                    hw.enable_all_torque(False)
                hw.disconnect()
            except Exception as e:
                self.get_logger().warning(f"Cleanup: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = HwJointTest()
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
