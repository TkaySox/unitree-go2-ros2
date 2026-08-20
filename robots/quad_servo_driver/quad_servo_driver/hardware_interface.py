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
    JointConfig,
    angle_to_ticks,
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
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.calibration_file = calibration_file
        self.log = logger or _log
        self.enable_serial = enable_serial
        self.enable_pwm = enable_pwm

        self.bus: Optional[ScsBus] = None
        self.pwm = PwmHipBank(logger=self.log)
        self.offsets: Dict[str, int] = {}  # serial tick offsets
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
        if not force and os.path.isfile(self.calibration_file):
            self._load_calibration()
            return
        self.log.warning(
            "Calibrating (URDF HOME pose required for serial joints). "
            f"file={self.calibration_file} force={force}"
        )
        self._calibrate_from_hardware()
        self._save_calibration()

    def _calibrate_from_hardware(self) -> None:
        for name in JOINT_ORDER:
            cfg = JOINT_ID_MAP[name]
            if name not in self.online_joints and self.online_joints:
                self.offsets[name] = 0
                continue

            if cfg.interface == IFACE_SERIAL:
                if self.bus is None:
                    self.offsets[name] = 0
                    continue
                ideal = angle_to_ticks(
                    cfg.urdf_home_rad, cfg.direction, cfg.urdf_home_rad
                )
                pos, _spd, result, error = self.bus.read_pos_speed(cfg.servo_id)
                if pos is None:
                    self.log.error(
                        f"CAL fail {name} id={cfg.servo_id}: "
                        f"{self.bus.result_str(result)} {self.bus.error_str(error)}"
                    )
                    self.offsets[name] = 0
                    continue
                self.offsets[name] = pos - ideal
                self.log.info(
                    f"CAL serial {name} id={cfg.servo_id}: "
                    f"raw={pos} ideal={ideal} offset={self.offsets[name]}"
                )

            elif cfg.interface == IFACE_PWM:
                # No feedback: keep configured pulse_home_us as calibrated home.
                # Pose the hip horn where you want zero; we store that pulse as home.
                self.pwm_homes[name] = cfg.pulse_home_us
                self.pwm.set_home_pulse(name, cfg.pulse_home_us)
                self.offsets[name] = 0
                self.log.info(
                    f"CAL pwm {name} BCM{cfg.gpio_pin}: "
                    f"pulse_home_us={cfg.pulse_home_us} (open-loop, set in joint_map)"
                )

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
                "serial offset = raw_tick_at_home - 2048; "
                "pwm_homes = pulse_us at URDF home (open-loop)"
            ),
            "serial_port": self.port,
            "offsets": {k: int(v) for k, v in self.offsets.items()},
            "pwm_homes": {
                n: int(JOINT_ID_MAP[n].pulse_home_us)
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

    def command_rad(self, joint_name: str, angle_rad: float) -> bool:
        cfg = JOINT_ID_MAP[joint_name]
        if cfg.interface == IFACE_SERIAL:
            if self.bus is None:
                return False
            ideal = angle_to_ticks(angle_rad, cfg.direction, cfg.urdf_home_rad)
            target = ideal + self.offsets.get(joint_name, 0)
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

    def command_rad_many(self, targets: Dict[str, float]) -> Dict[str, bool]:
        """
        Command many joints at once. Serial joints use one SyncWrite packet so
        they start moving together; PWM hips are written immediately after.
        """
        results: Dict[str, bool] = {name: False for name in targets}
        sync_goals = []
        sync_names: List[str] = []

        for name, angle_rad in targets.items():
            cfg = JOINT_ID_MAP[name]
            if cfg.interface == IFACE_SERIAL:
                if self.bus is None:
                    continue
                ideal = angle_to_ticks(angle_rad, cfg.direction, cfg.urdf_home_rad)
                ticks = ideal + self.offsets.get(name, 0)
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
                        results[name] = self.command_rad(name, angle_rad)
            else:
                for name in sync_names:
                    results[name] = True

        return results

    def read_ticks(self, joint_name: str) -> Optional[int]:
        cfg = JOINT_ID_MAP[joint_name]
        if cfg.interface != IFACE_SERIAL or self.bus is None:
            return None
        pos, _spd, result, _ = self.bus.read_pos_speed(cfg.servo_id)
        return pos if pos is not None else None

    def joint_config(self, joint_name: str) -> JointConfig:
        return JOINT_ID_MAP[joint_name]
