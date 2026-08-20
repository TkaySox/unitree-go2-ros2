#!/usr/bin/env python3
"""
PWM hobby-style hip driver for Raspberry Pi GPIO (FT5330M hips).

Requires: sudo apt install python3-gpiozero python3-lgpio
(Pi 5 uses the lgpio pin factory with gpiozero.)

Open-loop only — no position feedback. Calibration stores pulse_home_us.
"""

from __future__ import annotations

from typing import Dict, Optional

from quad_servo_driver.joint_map import JointConfig, PWM_FRAME_HZ, angle_to_pulse_us


class PwmHipBank:
    def __init__(self, logger=None) -> None:
        self.log = logger
        self._servos: Dict[str, object] = {}
        self._cfg: Dict[str, JointConfig] = {}

    def _info(self, msg: str) -> None:
        if self.log:
            self.log.info(msg)
        else:
            print(msg)

    def _warn(self, msg: str) -> None:
        if self.log:
            self.log.warning(msg)
        else:
            print("WARN:", msg)

    def attach(self, joint_name: str, cfg: JointConfig) -> bool:
        if cfg.gpio_pin is None:
            self._warn(f"PWM hip {joint_name}: no gpio_pin set — skip")
            return False
        try:
            from gpiozero import Servo
            from gpiozero.pins.lgpio import LGPIOFactory
        except ImportError as e:
            raise RuntimeError(
                "PWM hips need gpiozero + lgpio. Install:\n"
                "  sudo apt install python3-gpiozero python3-lgpio\n"
                f"Import error: {e}"
            ) from e

        factory = LGPIOFactory()
        servo = Servo(
            cfg.gpio_pin,
            initial_value=None,  # don't jump until we command
            min_pulse_width=cfg.pulse_min_us / 1_000_000.0,
            max_pulse_width=cfg.pulse_max_us / 1_000_000.0,
            frame_width=1.0 / PWM_FRAME_HZ,
            pin_factory=factory,
        )
        # gpiozero Servo uses -1..+1; we'll drive via value from pulse.
        self._servos[joint_name] = servo
        self._cfg[joint_name] = cfg
        self._info(
            f"PWM hip {joint_name}: BCM{cfg.gpio_pin} "
            f"pulse {cfg.pulse_min_us}-{cfg.pulse_max_us} µs home={cfg.pulse_home_us}"
        )
        return True

    def _pulse_to_value(self, cfg: JointConfig, pulse_us: int) -> float:
        """Map pulse µs → gpiozero Servo.value in [-1, 1]."""
        span = cfg.pulse_max_us - cfg.pulse_min_us
        if span <= 0:
            return 0.0
        t = (pulse_us - cfg.pulse_min_us) / span  # 0..1
        return max(-1.0, min(1.0, t * 2.0 - 1.0))

    def command_rad(self, joint_name: str, angle_rad: float) -> bool:
        servo = self._servos.get(joint_name)
        cfg = self._cfg.get(joint_name)
        if servo is None or cfg is None:
            return False
        pulse = angle_to_pulse_us(angle_rad, cfg)
        try:
            servo.value = self._pulse_to_value(cfg, pulse)
            return True
        except Exception as e:
            self._warn(f"PWM cmd fail {joint_name}: {e}")
            return False

    def set_home_pulse(self, joint_name: str, pulse_us: Optional[int] = None) -> int:
        """Record / set home pulse for calibration. Returns pulse_home_us used."""
        cfg = self._cfg[joint_name]
        if pulse_us is not None:
            cfg.pulse_home_us = int(pulse_us)
        return cfg.pulse_home_us

    def detach_all(self) -> None:
        for name, servo in list(self._servos.items()):
            try:
                servo.value = None  # stop PWM pulses
                servo.close()
            except Exception:
                pass
        self._servos.clear()
