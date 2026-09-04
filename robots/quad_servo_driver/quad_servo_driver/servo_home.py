#!/usr/bin/env python3
"""
Home helpers.

  qhome_champ / home_mode:=capture_champ
      Torque OFF → you physically set upper+lower VERTICAL → Enter
      → reads servo ticks → saves ~/.ros/quad_servo_home_ticks.yaml

  qhome / home_mode:=goto_home  (default)
      Moves every online joint to those saved ticks (shortest path).
      Run qhome_champ at least once first.

  ros2 run quad_servo_driver servo_home --ros-args -p home_mode:=capture_champ
  ros2 run quad_servo_driver servo_home
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node

from quad_servo_driver.hardware_interface import QuadServoHardware
from quad_servo_driver.joint_map import (
    CALIBRATION_FILE,
    HOME_TICKS_FILE,
    SERIAL_PORT,
    load_home_ticks,
    save_home_ticks,
)


def _ask(prompt: str) -> str:
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return "q"


class ServoHome(Node):
    def __init__(self) -> None:
        super().__init__("servo_home")
        self.declare_parameter("port", SERIAL_PORT)
        self.declare_parameter("calibration_file", CALIBRATION_FILE)
        self.declare_parameter("home_ticks_file", HOME_TICKS_FILE)
        self.declare_parameter("apply_offsets", False)
        # capture_champ | goto_home
        self.declare_parameter("home_mode", "goto_home")
        self.declare_parameter("hold_sec", 1.0)
        self.declare_parameter("disable_torque_at_end", False)
        self.declare_parameter("hold_after_capture", True)

        port = str(self.get_parameter("port").value)
        calib = str(self.get_parameter("calibration_file").value)
        ticks_file = str(self.get_parameter("home_ticks_file").value)
        apply_offsets = bool(self.get_parameter("apply_offsets").value)
        home_mode = str(self.get_parameter("home_mode").value).strip().lower()
        hold = float(self.get_parameter("hold_sec").value)
        disable_end = bool(self.get_parameter("disable_torque_at_end").value)
        hold_after = bool(self.get_parameter("hold_after_capture").value)

        # Normalize aliases
        if home_mode in (
            "champ",
            "champ_zero",
            "capture",
            "capture_champ",
            "vertical",
            "champ0",
        ):
            home_mode = "capture_champ"
        else:
            home_mode = "goto_home"

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
                    "No online joints. Stop qdrv / free the serial port."
                )
                return

            hw.load_or_calibrate(force=False)

            if home_mode == "capture_champ":
                self._capture_champ(hw, online, ticks_file, hold_after)
            else:
                self._goto_home(hw, online, ticks_file, hold, disable_end)
        finally:
            try:
                hw.disconnect()
            except Exception as e:
                self.get_logger().warning(f"Cleanup: {e}")

    def _capture_champ(
        self,
        hw: QuadServoHardware,
        online: list,
        ticks_file: str,
        hold_after: bool,
    ) -> None:
        print()
        print("=" * 60)
        print("CAPTURE CHAMP HOME (both links VERTICAL)")
        print("  1) Torque will turn OFF (joints limp)")
        print("  2) Physically set EVERY upper+lower link VERTICAL")
        print("  3) Press Enter — ticks are read and saved")
        print("  4) Those ticks become the target for every `qhome`")
        print(f"  File: {ticks_file}")
        print("=" * 60)

        # Limp so user can place
        for name in online:
            hw.enable_torque(name, False)
        self.get_logger().warn("Torque OFF — place upper+lower VERTICAL now.")

        ans = _ask("When aligned vertical, press Enter (q=abort): ")
        if ans.startswith("q"):
            self.get_logger().info("Capture aborted.")
            return

        home_ticks: dict[str, int] = {}
        print()
        print("Captured ticks:")
        for name in online:
            ticks = hw.read_ticks(name)
            if ticks is None:
                self.get_logger().warning(f"  {name}: READ FAIL — skipped")
                continue
            home_ticks[name] = int(ticks)
            cfg = hw.joint_config(name)
            print(f"  {name}  id={cfg.servo_id}  ticks={ticks}")

        if not home_ticks:
            self.get_logger().error("No ticks captured — nothing saved.")
            return

        path = save_home_ticks(home_ticks, ticks_file)
        self.get_logger().info(f"Saved home ticks → {path}")
        print(f"\nSaved. From now on, `qhome` moves to these ticks.\n")

        if hold_after:
            for name in online:
                hw.enable_torque(name, True)
            # Hold current pose (already at capture pose)
            hw.move_to_ticks_shortest(home_ticks)
            self.get_logger().info("Torque ON — holding captured vertical pose.")
        else:
            self.get_logger().info("Leaving torque OFF.")

    def _goto_home(
        self,
        hw: QuadServoHardware,
        online: list,
        ticks_file: str,
        hold: float,
        disable_end: bool,
    ) -> None:
        saved = load_home_ticks(ticks_file)
        if not saved:
            self.get_logger().error(
                f"No home ticks at {ticks_file}.\n"
                "Run once:  qhome_champ\n"
                "  (limp → place both links VERTICAL → Enter)"
            )
            return

        targets = {n: saved[n] for n in online if n in saved}
        missing = [n for n in online if n not in saved]
        if missing:
            self.get_logger().warning(
                f"No saved home ticks for: {missing} — skipping those"
            )
        if not targets:
            self.get_logger().error("No overlapping online joints with saved homes.")
            return

        for name in targets:
            hw.enable_torque(name, True)

        self.get_logger().warn(
            f"qhome → moving to captured CHAMP vertical ticks: {list(targets)}"
        )
        for name, t in targets.items():
            self.get_logger().info(
                f"  before {name}: now={hw.read_ticks(name)} → goal={t}"
            )

        results = hw.move_to_ticks_shortest(targets)
        time.sleep(hold)

        for name, t in targets.items():
            self.get_logger().info(
                f"  after  {name}: now={hw.read_ticks(name)} "
                f"goal={t} ok={results.get(name)}"
            )
        self.get_logger().info("qhome complete.")

        if disable_end:
            for name in online:
                hw.enable_torque(name, False)


def main(args=None) -> None:
    try:
        sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
    except Exception:
        pass

    rclpy.init(args=args)
    node = None
    try:
        node = ServoHome()
    except KeyboardInterrupt:
        print("\nInterrupted.")
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
