#!/usr/bin/env python3
"""
Interactive SERVO ROTATION DIRECTION configurator.

Default (lower joints): nudge **+250 ticks** from the current position
(one joint at a time). You watch and report which way each moved.

  ros2 run quad_servo_driver servo_dir_config
  # or: qdir

  # Custom tick nudge:
  ros2 run quad_servo_driver servo_dir_config --ros-args -p lower_delta_ticks:=250

  # Include uppers too (+10° by default):
  ros2 run quad_servo_driver servo_dir_config --ros-args -p joints_filter:=all
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node

from quad_servo_driver.hardware_interface import QuadServoHardware
from quad_servo_driver.joint_map import (
    CALIBRATION_FILE,
    DIRECTIONS_FILE,
    IFACE_SERIAL,
    JOINT_ID_MAP,
    JOINT_ORDER,
    SERIAL_PORT,
    TICKS_PER_REV,
    deg_to_rad,
    save_directions,
    shortest_tick_delta,
    speed_acc_for_model,
)


def _ask(prompt: str) -> str:
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return "q"


class ServoDirConfig(Node):
    def __init__(self) -> None:
        super().__init__("servo_dir_config")
        self.declare_parameter("port", SERIAL_PORT)
        self.declare_parameter("calibration_file", CALIBRATION_FILE)
        self.declare_parameter("directions_file", DIRECTIONS_FILE)
        self.declare_parameter("delta_deg", 10.0)  # uppers (if included)
        self.declare_parameter("lower_delta_ticks", 250)  # lowers: tick nudge
        self.declare_parameter("hold_sec", 2.5)
        self.declare_parameter("settle_sec", 1.0)
        self.declare_parameter("apply_offsets", False)
        self.declare_parameter("include_hips", False)
        self.declare_parameter("confirm_moved", True)
        # lower | upper | all
        self.declare_parameter("joints_filter", "lower")

        port = str(self.get_parameter("port").value)
        calib = str(self.get_parameter("calibration_file").value)
        dirs_file = str(self.get_parameter("directions_file").value)
        delta_deg = float(self.get_parameter("delta_deg").value)
        lower_ticks = int(self.get_parameter("lower_delta_ticks").value)
        hold = float(self.get_parameter("hold_sec").value)
        settle = float(self.get_parameter("settle_sec").value)
        apply_offsets = bool(self.get_parameter("apply_offsets").value)
        include_hips = bool(self.get_parameter("include_hips").value)
        confirm_moved = bool(self.get_parameter("confirm_moved").value)
        joints_filter = str(self.get_parameter("joints_filter").value).strip().lower()
        delta_rad = deg_to_rad(delta_deg)

        joints = []
        for name in JOINT_ORDER:
            cfg = JOINT_ID_MAP[name]
            if cfg.interface != IFACE_SERIAL and not include_hips:
                continue
            if joints_filter in ("lower", "lowers") and "lower_leg" not in name:
                continue
            if joints_filter in ("upper", "uppers") and "upper_leg" not in name:
                continue
            joints.append(name)

        if not joints:
            self.get_logger().error(
                f"No joints selected (filter={joints_filter})."
            )
            return

        self.get_logger().warn(
            "SERVO DIRECTION SETUP — one joint at a time.\n"
            f"  Lowers: +{lower_ticks} ticks from current position\n"
            f"  Uppers: +{delta_deg:.0f}° (if included)\n"
            f"  Filter: {joints_filter} → {joints}\n"
            f"  Support the robot. Stop other serial nodes first.\n"
            f"  Saving to: {dirs_file}"
        )
        ans = _ask("Ready? [Enter]=start  q=quit: ")
        if ans.startswith("q"):
            self.get_logger().info("Aborted.")
            return

        hw = QuadServoHardware(
            port=port,
            calibration_file=calib,
            logger=self.get_logger(),
            enable_pwm=include_hips,
            enable_serial=True,
            apply_offsets=apply_offsets,
        )
        try:
            hw.connect()
            online = hw.ping_mapped()
            if not online:
                self.get_logger().error("No mapped servos online — abort.")
                return

            hw.load_or_calibrate(force=False)
            hw.enable_all_torque(True)

            # Park at SERVO_ZERO (upper vert, lower horiz / tick ~0)
            self.get_logger().info(
                "Moving online joints to SERVO_ZERO (shortest path)…"
            )
            home_targets = {
                n: JOINT_ID_MAP[n].urdf_home_rad for n in online
            }
            hw.move_to_angles_shortest(home_targets)
            time.sleep(settle)

            results: dict[str, int] = {
                n: int(JOINT_ID_MAP[n].direction) for n in JOINT_ORDER
            }

            for name in joints:
                if name not in online:
                    self.get_logger().warning(f"SKIP {name} (not online)")
                    continue

                cfg = JOINT_ID_MAP[name]
                is_lower = "lower_leg" in name
                home_ang = cfg.urdf_home_rad

                while True:
                    before = hw.read_ticks(name)
                    print()
                    print("=" * 60)
                    print(
                        f"Joint: {name}  id={cfg.servo_id}  "
                        f"direction={cfg.direction:+d}  ticks_now={before}"
                    )
                    if is_lower:
                        signed = cfg.direction * lower_ticks
                        print(
                            f"Will nudge SERVO by {signed:+d} ticks "
                            f"(dir={cfg.direction:+d} × {lower_ticks}) "
                            f"from current position."
                        )
                    else:
                        print(
                            f"Will nudge +{delta_deg:.0f}° from SERVO_ZERO "
                            f"(dir={cfg.direction:+d})."
                        )
                    print("Watch this link / servo only.")
                    print("=" * 60)
                    go = _ask("[Enter]=move  s=skip  q=quit: ")
                    if go.startswith("q"):
                        self.get_logger().info("Quit — saving.")
                        save_directions(dirs_file, results)
                        return
                    if go.startswith("s"):
                        results[name] = int(cfg.direction)
                        break

                    before = hw.read_ticks(name)
                    if is_lower:
                        if before is None:
                            self.get_logger().error(f"No tick read for {name}")
                            break
                        # Multi-turn goal so it takes the short way for ±250
                        mt_goal = int(before) + int(cfg.direction) * lower_ticks
                        speed, acc = speed_acc_for_model(cfg.model)
                        self.get_logger().info(
                            f"MOVE {name} ticks {before} → multi-turn {mt_goal} "
                            f"(Δ={cfg.direction * lower_ticks:+d})"
                        )
                        if hw.bus is not None:
                            hw.bus.write_position(
                                cfg.servo_id, mt_goal, speed, acc
                            )
                        time.sleep(hold)
                        after = hw.read_ticks(name)
                    else:
                        target = home_ang + cfg.direction * delta_rad
                        # With direction already in angle_to_ticks, command
                        # home + delta in joint space using |direction| via
                        # flipping target relative to home:
                        # Actually angle_to_ticks already multiplies direction.
                        # Positive test = increase joint angle from servo_zero.
                        target = home_ang + delta_rad
                        self.get_logger().info(
                            f"MOVE {name} +{delta_deg:.0f}° ticks_now={before}"
                        )
                        hw.move_to_angles_shortest({name: target})
                        time.sleep(hold)
                        after = hw.read_ticks(name)

                    tick_delta = (
                        None
                        if before is None or after is None
                        else shortest_tick_delta(before, after)
                    )
                    self.get_logger().info(
                        f"  arrived ticks={after}  shortest_Δ={tick_delta}"
                    )
                    print(
                        f"Result: {name} id={cfg.servo_id}: "
                        f"{before} → {after}  (Δ≈{tick_delta})"
                    )

                    if confirm_moved:
                        print()
                        print(
                            f"Did {name} MOVE AT ALL? "
                            f"(ticks {before} → {after})"
                        )
                        print("  y = yes")
                        print("  n = no — retry")
                        print("  s = skip joint")
                        print("  q = quit and save")
                        moved = _ask("Moved? [y/n/s/q]: ")
                        if moved.startswith("q"):
                            self._return_home(hw, name, home_ang, is_lower, before)
                            save_directions(dirs_file, results)
                            return
                        if moved.startswith("s"):
                            self._return_home(hw, name, home_ang, is_lower, before)
                            time.sleep(settle)
                            results[name] = int(cfg.direction)
                            break
                        if moved.startswith("n") or moved == "no":
                            self._return_home(hw, name, home_ang, is_lower, before)
                            time.sleep(settle)
                            again = _ask("Retry? [Enter]=yes  s=skip: ")
                            if again.startswith("s"):
                                results[name] = int(cfg.direction)
                                break
                            continue
                        if not (moved.startswith("y") or moved in ("", "yes")):
                            self._return_home(hw, name, home_ang, is_lower, before)
                            time.sleep(settle)
                            continue

                    print()
                    print(
                        f"Note the rotation for {name} (id={cfg.servo_id}). "
                        f"Was this the CORRECT positive direction?"
                    )
                    print("  y = yes — keep sign")
                    print("  n = no — flip sign and re-test")
                    print("  r = repeat")
                    print("  s = skip / keep and continue")
                    choice = _ask("Direction OK? [y/n/r/s]: ")

                    self._return_home(hw, name, home_ang, is_lower, before)
                    time.sleep(settle)

                    if choice.startswith("y") or choice in ("", "yes"):
                        results[name] = int(cfg.direction)
                        self.get_logger().info(
                            f"KEEP {name} direction={cfg.direction:+d}"
                        )
                        break
                    if choice.startswith("n") or choice == "no":
                        cfg.direction = -1 if cfg.direction >= 0 else 1
                        results[name] = int(cfg.direction)
                        self.get_logger().warn(
                            f"FLIPPED {name} → {cfg.direction:+d} (re-test)"
                        )
                        continue
                    if choice.startswith("r"):
                        continue
                    if choice.startswith("s"):
                        results[name] = int(cfg.direction)
                        break
                    print("Unknown — repeating.")

            path = save_directions(dirs_file, results)
            print()
            print(f"Saved directions → {path}")
            for name in joints:
                if name in results:
                    print(f"  {name}: {results[name]:+d}")
            self.get_logger().info("Done. Tell me which joint did what.")
        finally:
            try:
                hw.enable_all_torque(False)
                hw.disconnect()
            except Exception as e:
                self.get_logger().warning(f"Cleanup: {e}")

    @staticmethod
    def _return_home(hw, name, home_ang, is_lower, start_ticks) -> None:
        """Return toward SERVO_ZERO / start ticks."""
        if is_lower and start_ticks is not None and hw.bus is not None:
            cfg = JOINT_ID_MAP[name]
            # Shortest path back to the tick we started from (usually ~0)
            cur = hw.read_ticks(name)
            if cur is None:
                return
            mt = cur + shortest_tick_delta(cur, int(start_ticks) % TICKS_PER_REV)
            speed, acc = speed_acc_for_model(cfg.model)
            hw.bus.write_position(cfg.servo_id, mt, speed, acc)
        else:
            hw.move_to_angles_shortest({name: home_ang})


def main(args=None) -> None:
    try:
        sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
    except Exception:
        pass

    rclpy.init(args=args)
    node = None
    try:
        node = ServoDirConfig()
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
