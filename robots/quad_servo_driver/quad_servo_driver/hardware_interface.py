#!/usr/bin/env python3
"""
Quad hardware interface — dual backend:

  * STS3215 legs  → USB serial bus (/dev/ttyACM0)
  * FT5330M hips  → Raspberry Pi GPIO PWM

rclpy loggers only accept a single string (use f-strings).
"""

from __future__ import annotations

import logging
import os
import time
from typing import Dict, List, Optional

import yaml

from quad_servo_driver.joint_map import (
    BAUDRATE,
    CALIBRATION_FILE,
    IFACE_PWM,
    IFACE_SERIAL,
    JOINT_ID_MAP,
    JOINT_ORDER,
    SERIAL_PORT,
    TICKS_PER_REV,
    JointConfig,
    config_angle_to_ticks,
    shortest_tick_delta,
    speed_acc_for_model,
)
from quad_servo_driver.pwm_hips import PwmHipBank
from quad_servo_driver.scservo_bus import ScsBus

_log = logging.getLogger("quad_hw")


class QuadServoHardware:
    def __init__(
        self,
        port: str = SERIAL_PORT,
        baudrate: int = BAUDRATE,
        calibration_file: str = CALIBRATION_FILE,
        logger=None,
        enable_serial: bool = True,
        enable_pwm: bool = True,
        apply_offsets: bool = False,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.calibration_file = calibration_file
        self.log = logger or _log
        self.enable_serial = enable_serial
        self.enable_pwm = enable_pwm
        # When False (default): URDF rad → ticks with no mechanical offset.
        # Use True only if horns do not match the RViz/URDF pose.
        self.apply_offsets = apply_offsets

        self.bus: Optional[ScsBus] = None
        self.pwm = PwmHipBank(logger=self.log)
        self.offsets: Dict[str, int] = {}  # serial tick offsets (optional)
        self.pwm_homes: Dict[str, int] = {}  # calibrated pulse_home_us
        self.online_joints: List[str] = []

    def connect(self) -> None:
        if self.enable_serial and any(
            c.interface == IFACE_SERIAL for c in JOINT_ID_MAP.values()
        ):
            self.bus = ScsBus(self.port, self.baudrate, protocol_end=0)
            self.bus.open()
            self.log.info(f"Serial HW open {self.port} @ {self.baudrate}")

        if self.enable_pwm:
            for name, cfg in JOINT_ID_MAP.items():
                if cfg.interface == IFACE_PWM:
                    try:
                        self.pwm.attach(name, cfg)
                    except Exception as e:
                        self.log.error(f"PWM attach {name}: {e}")

    def disconnect(self) -> None:
        try:
            self.pwm.detach_all()
        except Exception:
            pass
        if self.bus is not None:
            self.bus.close()
            self.bus = None
        self.log.info("HW disconnected (serial + PWM)")

    def ping_mapped(self) -> List[str]:
        online: List[str] = []
        for name in JOINT_ORDER:
            cfg = JOINT_ID_MAP[name]
            if cfg.interface == IFACE_SERIAL:
                if not self.enable_serial or self.bus is None:
                    continue
                model, result, _ = self.bus.ping(cfg.servo_id)
                if model is not None and result == self.bus.ok:
                    online.append(name)
                    self.log.info(
                        f"ping OK {name} serial id={cfg.servo_id} model_num={model}"
                    )
                else:
                    self.log.warning(
                        f"ping FAIL {name} serial id={cfg.servo_id} — skip"
                    )
            elif cfg.interface == IFACE_PWM:
                if not self.enable_pwm:
                    continue
                # Open-loop PWM: treat as online if pin attached
                if name in self.pwm._servos:
                    online.append(name)
                    self.log.info(
                        f"PWM ready {name} BCM{cfg.gpio_pin} ({cfg.model})"
                    )
                else:
                    self.log.warning(f"PWM not attached {name} — skip")
        self.online_joints = online
        return online

    def load_or_calibrate(self, force: bool = False) -> None:
        """
        Default (apply_offsets=False): identity mapping — all offsets 0.
        RViz/URDF angles are sent straight to the servos.

        If apply_offsets=True: load YAML offsets, or (re)measure them when
        force=True / file missing.
        """
        if not self.apply_offsets:
            for name in JOINT_ORDER:
                self.offsets[name] = 0
                cfg = JOINT_ID_MAP[name]
                if cfg.interface == IFACE_PWM:
                    self.pwm_homes[name] = cfg.pulse_home_us
                    if name in self.pwm._servos:
                        self.pwm.set_home_pulse(name, cfg.pulse_home_us)
            self.log.info(
                "Using URDF angles directly (apply_offsets=false) — no tick offsets."
            )
            # Keep a zeroed YAML so tools see a consistent file on disk.
            if force or not os.path.isfile(self.calibration_file):
                self._save_calibration()
            return

        if not force and os.path.isfile(self.calibration_file):
            self._load_calibration()
            return
        self.log.warning(
            "Calibrating mechanical offsets (URDF HOME pose required). "
            f"file={self.calibration_file} force={force}"
        )
        self._calibrate_from_hardware()
        self._save_calibration()

    def _calibrate_from_hardware(self) -> None:
        """Legacy: assume robot is at each joint's urdf_home_rad."""
        angles = {
            name: JOINT_ID_MAP[name].urdf_home_rad for name in JOINT_ORDER
        }
        self.calibrate_from_angles(angles, save=False)

    def calibrate_from_angles(
        self, angles: Dict[str, float], save: bool = True
    ) -> None:
        """
        Bind current servo ticks to the given URDF angles (no motion).

        offset = raw_ticks_now − angle_to_ticks(angle_now)

        After this, commanding those same angles holds the robot still.
        """
        self.apply_offsets = True
        for name in JOINT_ORDER:
            cfg = JOINT_ID_MAP[name]
            angle = float(angles.get(name, cfg.urdf_home_rad))

            if name not in self.online_joints and self.online_joints:
                self.offsets[name] = 0
                continue

            if cfg.interface == IFACE_SERIAL:
                if self.bus is None:
                    self.offsets[name] = 0
                    continue
                ideal = config_angle_to_ticks(cfg, angle)
                pos, _spd, result, error = self.bus.read_pos_speed(cfg.servo_id)
                if pos is None:
                    self.log.error(
                        f"CAL fail {name} id={cfg.servo_id}: "
                        f"{self.bus.result_str(result)} {self.bus.error_str(error)}"
                    )
                    self.offsets[name] = 0
                    continue
                self.offsets[name] = int(pos) - ideal
                self.log.info(
                    f"CAL serial {name} id={cfg.servo_id}: "
                    f"angle={angle:.4f} rad gear={cfg.gear_ratio:.4f} "
                    f"raw={pos} ideal={ideal} offset={self.offsets[name]}"
                )

            elif cfg.interface == IFACE_PWM:
                self.pwm_homes[name] = cfg.pulse_home_us
                if name in self.pwm._servos:
                    self.pwm.set_home_pulse(name, cfg.pulse_home_us)
                self.offsets[name] = 0
                self.log.info(
                    f"CAL pwm {name} BCM{cfg.gpio_pin}: "
                    f"pulse_home_us={cfg.pulse_home_us} (open-loop)"
                )

        if save:
            self._save_calibration()

    def _load_calibration(self) -> None:
        with open(self.calibration_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        offsets = data.get("offsets", {})
        pwm_homes = data.get("pwm_homes", {})
        for name in JOINT_ORDER:
            self.offsets[name] = int(offsets.get(name, 0))
            cfg = JOINT_ID_MAP[name]
            if cfg.interface == IFACE_PWM:
                home = int(pwm_homes.get(name, cfg.pulse_home_us))
                self.pwm_homes[name] = home
                cfg.pulse_home_us = home
                if name in self.pwm._servos:
                    self.pwm.set_home_pulse(name, home)
            self.log.info(
                f"LOAD CAL {name}: offset={self.offsets[name]} "
                f"pwm_home={self.pwm_homes.get(name, '-')}"
            )
        self.log.info(f"Loaded calibration from {self.calibration_file}")

    def _save_calibration(self) -> None:
        path = self.calibration_file
        os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
        payload = {
            "note": (
                "serial offset = raw_ticks_at_capture − angle_to_ticks(angle_at_capture). "
                "champ_servo_driver captures this from the first /joint_states by default. "
                "pwm_homes = pulse_us at URDF home (open-loop)."
            ),
            "apply_offsets": bool(self.apply_offsets),
            "serial_port": self.port,
            "offsets": {k: int(v) for k, v in self.offsets.items()},
            "pwm_homes": {
                n: int(self.pwm_homes.get(n, JOINT_ID_MAP[n].pulse_home_us))
                for n, c in JOINT_ID_MAP.items()
                if c.interface == IFACE_PWM
            },
            "interfaces": {k: v.interface for k, v in JOINT_ID_MAP.items()},
            "servo_ids": {
                k: v.servo_id for k, v in JOINT_ID_MAP.items() if v.servo_id is not None
            },
            "gpio_pins": {
                k: v.gpio_pin for k, v in JOINT_ID_MAP.items() if v.gpio_pin is not None
            },
        }
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(payload, f, default_flow_style=False, sort_keys=False)
        self.log.info(f"Saved calibration to {path}")

    def enable_torque(self, joint_name: str, enable: bool = True) -> bool:
        cfg = JOINT_ID_MAP[joint_name]
        if cfg.interface == IFACE_SERIAL:
            if self.bus is None:
                return False
            result, error = self.bus.set_torque_enable(cfg.servo_id, enable)
            if result != self.bus.ok:
                self.log.warning(
                    f"torque_en fail {joint_name}: "
                    f"{self.bus.result_str(result)} {self.bus.error_str(error)}"
                )
                return False
            return True
        # PWM: "torque off" = stop pulses
        if not enable and joint_name in self.pwm._servos:
            try:
                self.pwm._servos[joint_name].value = None
            except Exception:
                pass
        return True

    def enable_all_torque(self, enable: bool = True) -> None:
        for name in self.online_joints or JOINT_ORDER:
            self.enable_torque(name, enable)

    def _ticks_for_angle(self, joint_name: str, angle_rad: float) -> int:
        cfg = JOINT_ID_MAP[joint_name]
        ideal = config_angle_to_ticks(cfg, angle_rad)
        if self.apply_offsets:
            ideal = ideal + self.offsets.get(joint_name, 0)
        return int(ideal) % TICKS_PER_REV

    def _goal_ticks_shortest(self, current: Optional[int], target: int) -> int:
        """
        Multi-turn goal so the servo travels the short way on the circle.

        Example: current=3982 (~350°), target=0 → goal=4096 (not 0), so it
        advances 3982→4095→0 instead of reversing the long way.
        """
        target = int(target) % TICKS_PER_REV
        if current is None:
            return target
        current = int(current) % TICKS_PER_REV
        return current + shortest_tick_delta(current, target)

    def command_rad(
        self, joint_name: str, angle_rad: float, shortest: bool = True
    ) -> bool:
        cfg = JOINT_ID_MAP[joint_name]
        if cfg.interface == IFACE_SERIAL:
            if self.bus is None:
                return False
            target = self._ticks_for_angle(joint_name, angle_rad)
            if shortest:
                target = self._goal_ticks_shortest(self.read_ticks(joint_name), target)
            speed, acc = speed_acc_for_model(cfg.model)
            result, error = self.bus.write_position(cfg.servo_id, target, speed, acc)
            if result != self.bus.ok:
                self.log.warning(
                    f"cmd fail {joint_name} rad={angle_rad:.4f} ticks={target}: "
                    f"{self.bus.result_str(result)} {self.bus.error_str(error)}"
                )
                return False
            return True

        if cfg.interface == IFACE_PWM:
            return self.pwm.command_rad(joint_name, angle_rad)

        return False

    def command_rad_many(
        self, targets: Dict[str, float], shortest: bool = True
    ) -> Dict[str, bool]:
        """
        Command many joints at once. Serial joints use one SyncWrite packet so
        they start moving together; PWM hips are written immediately after.

        With shortest=True, goals that need wrap (e.g. 350°→0°) get the next
        hop on the short arc so the servo does not take the long way.
        """
        results: Dict[str, bool] = {name: False for name in targets}
        sync_goals = []
        sync_names: List[str] = []

        for name, angle_rad in targets.items():
            cfg = JOINT_ID_MAP[name]
            if cfg.interface == IFACE_SERIAL:
                if self.bus is None:
                    continue
                ticks = self._ticks_for_angle(name, angle_rad)
                if shortest:
                    ticks = self._goal_ticks_shortest(self.read_ticks(name), ticks)
                speed, acc = speed_acc_for_model(cfg.model)
                sync_goals.append((cfg.servo_id, ticks, speed, acc))
                sync_names.append(name)
            elif cfg.interface == IFACE_PWM:
                results[name] = self.pwm.command_rad(name, angle_rad)

        if sync_goals and self.bus is not None:
            result = self.bus.sync_write_positions(sync_goals)
            ok = result == self.bus.ok
            if not ok:
                self.log.warning(
                    f"sync_write fail ({len(sync_goals)} servos): "
                    f"{self.bus.result_str(result)} — falling back per-joint"
                )
                for name, angle_rad in targets.items():
                    if name in sync_names:
                        results[name] = self.command_rad(
                            name, angle_rad, shortest=shortest
                        )
            else:
                for name in sync_names:
                    results[name] = True

        return results

    def move_to_angles_shortest(
        self,
        targets: Dict[str, float],
        settle_sec: float = 0.5,
        timeout_sec: float = 15.0,
        tol_ticks: int = 12,
        **_ignored,
    ) -> Dict[str, bool]:
        """
        Drive joints to target URDF angles along the shortest tick arc.

        Example: current ≈ 350° (tick ~3982), home tick 0 → multi-turn goal
        4096 so the servo advances 350→351→…→0 (not the long way).
        """
        final_ticks: Dict[str, int] = {}
        sync_goals = []
        for name, angle in targets.items():
            cfg = JOINT_ID_MAP[name]
            if cfg.interface != IFACE_SERIAL:
                self.command_rad(name, angle, shortest=False)
                continue
            goal = self._ticks_for_angle(name, angle)
            final_ticks[name] = goal
            cur = self.read_ticks(name)
            mt_goal = self._goal_ticks_shortest(cur, goal)
            speed, acc = speed_acc_for_model(cfg.model)
            sync_goals.append((cfg.servo_id, mt_goal, speed, acc))
            self.log.info(
                f"shortest {name} id={cfg.servo_id}: "
                f"now={cur} → multi-turn {mt_goal} (mod goal {goal}, "
                f"Δ={0 if cur is None else shortest_tick_delta(cur, goal)})"
            )

        if sync_goals and self.bus is not None:
            result = self.bus.sync_write_positions(sync_goals)
            if result != self.bus.ok:
                self.log.warning(
                    f"shortest sync_write: {self.bus.result_str(result)} — per-joint"
                )
                for name, angle in targets.items():
                    if name in final_ticks:
                        self.command_rad(name, angle, shortest=True)

        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            time.sleep(settle_sec)
            pending = False
            for name, goal in final_ticks.items():
                cur = self.read_ticks(name)
                if cur is None or abs(shortest_tick_delta(cur, goal)) > tol_ticks:
                    pending = True
                    break
            if not pending:
                break

        out: Dict[str, bool] = {}
        for name in targets:
            if name not in final_ticks:
                out[name] = True
                continue
            goal = final_ticks[name]
            cur = self.read_ticks(name)
            ok = cur is not None and abs(shortest_tick_delta(cur, goal)) <= tol_ticks
            out[name] = ok
            if not ok:
                self.log.warning(
                    f"shortest {name}: end ticks={cur} goal={goal} ok={ok}"
                )
        return out

    def home_shortest(
        self,
        joint_names: Optional[List[str]] = None,
        **kwargs,
    ) -> Dict[str, bool]:
        """Home listed joints (default: all online) via shortest path to URDF home."""
        names = joint_names or list(self.online_joints)
        targets = {
            n: JOINT_ID_MAP[n].urdf_home_rad for n in names if n in JOINT_ID_MAP
        }
        self.log.info(
            f"Homing {len(targets)} joints shortest-path to tick home "
            f"(0° = tick 0 at URDF home angles)…"
        )
        return self.move_to_angles_shortest(targets, **kwargs)

    def move_to_ticks_shortest(
        self,
        tick_targets: Dict[str, int],
        settle_sec: float = 0.5,
        timeout_sec: float = 15.0,
        tol_ticks: int = 12,
        **_ignored,
    ) -> Dict[str, bool]:
        """Drive joints to absolute tick goals along the shortest arc."""
        final_ticks: Dict[str, int] = {}
        sync_goals = []
        for name, goal in tick_targets.items():
            cfg = JOINT_ID_MAP[name]
            if cfg.interface != IFACE_SERIAL:
                continue
            goal_i = int(goal) % TICKS_PER_REV
            final_ticks[name] = goal_i
            cur = self.read_ticks(name)
            mt_goal = self._goal_ticks_shortest(cur, goal_i)
            speed, acc = speed_acc_for_model(cfg.model)
            sync_goals.append((cfg.servo_id, mt_goal, speed, acc))
            self.log.info(
                f"tick-home {name} id={cfg.servo_id}: "
                f"now={cur} → multi-turn {mt_goal} (goal {goal_i})"
            )

        if sync_goals and self.bus is not None:
            result = self.bus.sync_write_positions(sync_goals)
            if result != self.bus.ok:
                self.log.warning(
                    f"tick-home sync_write: {self.bus.result_str(result)}"
                )
                for name, goal_i in final_ticks.items():
                    cfg = JOINT_ID_MAP[name]
                    cur = self.read_ticks(name)
                    mt = self._goal_ticks_shortest(cur, goal_i)
                    speed, acc = speed_acc_for_model(cfg.model)
                    self.bus.write_position(cfg.servo_id, mt, speed, acc)

        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            time.sleep(settle_sec)
            pending = False
            for name, goal in final_ticks.items():
                cur = self.read_ticks(name)
                if cur is None or abs(shortest_tick_delta(cur, goal)) > tol_ticks:
                    pending = True
                    break
            if not pending:
                break

        out: Dict[str, bool] = {}
        for name, goal in final_ticks.items():
            cur = self.read_ticks(name)
            ok = cur is not None and abs(shortest_tick_delta(cur, goal)) <= tol_ticks
            out[name] = ok
            if not ok:
                self.log.warning(
                    f"tick-home {name}: end={cur} goal={goal} ok={ok}"
                )
        return out

    def read_ticks(self, joint_name: str) -> Optional[int]:
        cfg = JOINT_ID_MAP[joint_name]
        if cfg.interface != IFACE_SERIAL or self.bus is None:
            return None
        pos, _spd, result, _ = self.bus.read_pos_speed(cfg.servo_id)
        return pos if pos is not None else None

    def joint_config(self, joint_name: str) -> JointConfig:
        return JOINT_ID_MAP[joint_name]
