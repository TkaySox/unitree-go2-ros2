#!/usr/bin/env python3
"""
Thin wrapper around the official Feetech Python scservo_sdk (same protocol as
SCServo_Linux C++ SMS_STS examples: PortHandler + PacketHandler, register R/W).

ASSUMPTION: FT5330M (hips) and STS3215 share STS/SMS register map and 4096
ticks/rev. VERIFY on hardware — if FT5330M differs, split handlers per model.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional, Tuple

# Prefer system/pip install; fall back to vendored copy under third_party/.
_VENDOR = Path(__file__).resolve().parents[1] / "third_party"
if _VENDOR.is_dir() and str(_VENDOR) not in sys.path:
    sys.path.insert(0, str(_VENDOR))

from scservo_sdk import (  # noqa: E402
    COMM_SUCCESS,
    COMM_TX_FAIL,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
    SCS_HIBYTE,
    SCS_HIWORD,
    SCS_LOBYTE,
    SCS_LOWORD,
    SCS_TOHOST,
)

# Control table (STS/SMS) — matches SCServo_Linux SMS_STS.h / Python examples
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_ACC = 41
ADDR_GOAL_POSITION = 42
ADDR_GOAL_SPEED = 46
ADDR_TORQUE_LIMIT = 48
ADDR_PRESENT_POSITION = 56  # 4 bytes: pos L/H + speed L/H
ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_MOVING = 66
ADDR_PRESENT_CURRENT = 69

# EEPROM (useful for diag)
ADDR_ID = 5
ADDR_BAUD_RATE = 6
ADDR_MIN_ANGLE_LIMIT = 9
ADDR_MAX_ANGLE_LIMIT = 11
ADDR_MAX_TORQUE = 16
ADDR_MODE = 33
ADDR_MODEL_L = 3


class ScsBus:
    """USB serial bus to Feetech STS/SMS servos (Waveshare USB adapter)."""

    def __init__(self, port: str, baudrate: int = 1_000_000, protocol_end: int = 0):
        """
        protocol_end: 0 for STS/SMS (FT5330M / STS3215), 1 for SCS series.
        """
        self.port_name = port
        self.baudrate = baudrate
        self._port = PortHandler(port)
        self._packet = PacketHandler(protocol_end)
        self._open = False

    def open(self) -> None:
        try:
            ok = self._port.openPort()
        except Exception as e:
            raise RuntimeError(
                f"Failed to open {self.port_name}: {e}\n"
                "Fix permissions (recommended):\n"
                "  sudo usermod -aG dialout $USER\n"
                "  # then log out and back in (or reboot)\n"
                "Or one-shot:  sudo chmod 666 /dev/ttyACM0\n"
                "Check device:  ls -l /dev/ttyACM*"
            ) from e
        if not ok:
            raise RuntimeError(
                f"Failed to open serial port {self.port_name}. "
                "Is the Waveshare adapter plugged in? Try: ls -l /dev/ttyACM* /dev/ttyUSB*"
            )
        if not self._port.setBaudRate(self.baudrate):
            self._port.closePort()
            raise RuntimeError(f"Failed to set baudrate {self.baudrate} on {self.port_name}")
        self._open = True

    def close(self) -> None:
        if self._open:
            try:
                self._port.closePort()
            finally:
                self._open = False

    def ping(self, servo_id: int) -> Tuple[Optional[int], int, int]:
        model, result, error = self._packet.ping(self._port, servo_id)
        if result != COMM_SUCCESS:
            return None, result, error
        return int(model), result, error

    def read_pos_speed(self, servo_id: int) -> Tuple[Optional[int], Optional[int], int, int]:
        """Present position + speed (one 4-byte read at ADDR 56), like the SDK example."""
        data, result, error = self._packet.read4ByteTxRx(
            self._port, servo_id, ADDR_PRESENT_POSITION
        )
        if result != COMM_SUCCESS:
            return None, None, result, error
        pos = SCS_LOWORD(data)
        speed = SCS_TOHOST(SCS_HIWORD(data), 15)
        return int(pos), int(speed), result, error

    def read_u8(self, servo_id: int, addr: int) -> Tuple[Optional[int], int, int]:
        data, result, error = self._packet.read1ByteTxRx(self._port, servo_id, addr)
        if result != COMM_SUCCESS:
            return None, result, error
        return int(data), result, error

    def read_u16(self, servo_id: int, addr: int) -> Tuple[Optional[int], int, int]:
        data, result, error = self._packet.read2ByteTxRx(self._port, servo_id, addr)
        if result != COMM_SUCCESS:
            return None, result, error
        return int(data), result, error

    def set_torque_enable(self, servo_id: int, enable: bool) -> Tuple[int, int]:
        return self._packet.write1ByteTxRx(
            self._port, servo_id, ADDR_TORQUE_ENABLE, 1 if enable else 0
        )

    def write_position(
        self, servo_id: int, ticks: int, speed: int = 0, acc: int = 0
    ) -> Tuple[int, int]:
        """
        Write goal acc, speed, then position (same sequence as scsservo_sdk_example/read_write.py).
        ticks: 0..4095 for STS/SMS.
        """
        ticks = max(0, min(4095, int(ticks)))
        r1, e1 = self._packet.write1ByteTxRx(self._port, servo_id, ADDR_GOAL_ACC, int(acc))
        if r1 != COMM_SUCCESS:
            return r1, e1
        r2, e2 = self._packet.write2ByteTxRx(self._port, servo_id, ADDR_GOAL_SPEED, int(speed))
        if r2 != COMM_SUCCESS:
            return r2, e2
        return self._packet.write2ByteTxRx(
            self._port, servo_id, ADDR_GOAL_POSITION, ticks
        )

    def sync_write_positions(
        self,
        goals: list,
    ) -> int:
        """
        One bus packet: start all servos together (SMS/STS SyncWritePosEx layout).

        goals: iterable of (servo_id, ticks, speed, acc)
        Returns COMM_SUCCESS or a tx result code.
        """
        # 7 bytes from ADDR_GOAL_ACC: ACC + POS(2) + TIME(2)=0 + SPEED(2)
        gsw = GroupSyncWrite(self._port, self._packet, ADDR_GOAL_ACC, 7)
        for servo_id, ticks, speed, acc in goals:
            ticks = max(0, min(4095, int(ticks)))
            speed = int(speed)
            acc = int(acc)
            data = [
                acc & 0xFF,
                SCS_LOBYTE(ticks),
                SCS_HIBYTE(ticks),
                0,
                0,
                SCS_LOBYTE(speed),
                SCS_HIBYTE(speed),
            ]
            if not gsw.addParam(int(servo_id), data):
                return COMM_TX_FAIL
        result = gsw.txPacket()
        gsw.clearParam()
        return result

    def result_str(self, result: int) -> str:
        return self._packet.getTxRxResult(result)

    def error_str(self, error: int) -> str:
        return self._packet.getRxPacketError(error)

    @property
    def ok(self) -> int:
        return COMM_SUCCESS
