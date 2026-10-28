"""Low-level STS3215 protocol driver.

The driver is transport-agnostic: pass in a SerialTransport or TcpTransport
(or any object that satisfies the same interface).
"""

import threading
import time
from typing import Optional

from .constants import (
    STS_HEADER, STS_BROADCAST_ID,
    INST_PING, INST_READ, INST_WRITE, INST_SYNC_WRITE,
    REG_ID, REG_EEPROM_MAX_SPEED, REG_TORQUE_ENABLE, REG_LOCK,
    REG_GOAL_ACC, REG_PRESENT_POSITION, REG_MOVING,
    REG_PRESENT_TEMP, REG_PRESENT_VOLTAGE,
    CALIBRATE_CMD,
    POS_MIN, POS_MAX, DEG_MAX,
)
from .transport import SerialTransport, TcpTransport


class STS3215Driver:
    def __init__(self):
        self._transport = None
        self.lock = threading.Lock()

    # ── connection ──────────────────────────────────────────────────────────
    def connect_serial(self, port: str, baud: int = 1_000_000) -> bool:
        t = SerialTransport()
        if t.connect(port, baud):
            self._transport = t
            return True
        return False

    def connect_tcp(self, host: str, port: int = 8888) -> bool:
        t = TcpTransport()
        if t.connect(host, port):
            self._transport = t
            return True
        return False

    def disconnect(self):
        if self._transport:
            self._transport.disconnect()
        self._transport = None

    @property
    def connected(self) -> bool:
        return self._transport is not None and self._transport.connected

    # ── packet helpers ──────────────────────────────────────────────────────
    @staticmethod
    def _checksum(data: list) -> int:
        return (~sum(data)) & 0xFF

    def _build_packet(self, servo_id: int, instruction: int, params: list) -> bytes:
        length = len(params) + 2
        data = [servo_id, length, instruction] + params
        chk = self._checksum(data)
        return bytes([STS_HEADER, STS_HEADER] + data + [chk])

    def _send(self, packet: bytes):
        if self.connected:
            self._transport.reset_input_buffer()
            self._transport.write(packet)

    def _read_response(self) -> Optional[list]:
        try:
            hdr = self._transport.read(2)
            if len(hdr) < 2 or hdr[0] != STS_HEADER or hdr[1] != STS_HEADER:
                return None
            meta = self._transport.read(3)
            if len(meta) < 3:
                return None
            length = meta[1]
            params = self._transport.read(length - 2)
            self._transport.read(1)
            return list(params)
        except Exception:
            return None

    # ── register I/O ────────────────────────────────────────────────────────
    def write_byte(self, servo_id: int, reg: int, value: int) -> bool:
        with self.lock:
            pkt = self._build_packet(servo_id, INST_WRITE, [reg, value & 0xFF])
            self._send(pkt)
            time.sleep(0.005)
            return True

    def write_word(self, servo_id: int, reg: int, value: int) -> bool:
        with self.lock:
            pkt = self._build_packet(servo_id, INST_WRITE,
                                     [reg, value & 0xFF, (value >> 8) & 0xFF])
            self._send(pkt)
            time.sleep(0.005)
            return True

    def read_word(self, servo_id: int, reg: int) -> Optional[int]:
        with self.lock:
            pkt = self._build_packet(servo_id, INST_READ, [reg, 2])
            self._send(pkt)
            resp = self._read_response()
            if resp and len(resp) >= 2:
                return resp[0] | (resp[1] << 8)
            return None

    def read_byte(self, servo_id: int, reg: int) -> Optional[int]:
        with self.lock:
            pkt = self._build_packet(servo_id, INST_READ, [reg, 1])
            self._send(pkt)
            resp = self._read_response()
            if resp and len(resp) >= 1:
                return resp[0]
            return None

    def ping(self, servo_id: int) -> bool:
        with self.lock:
            pkt = self._build_packet(servo_id, INST_PING, [])
            self._send(pkt)
            return self._read_response() is not None

    def _write_byte_raw(self, servo_id: int, reg: int, value: int):
        pkt = self._build_packet(servo_id, INST_WRITE, [reg, value & 0xFF])
        self._send(pkt)
        time.sleep(0.01)

    # ── high-level commands ─────────────────────────────────────────────────
    def set_id(self, current_id: int, new_id: int) -> bool:
        with self.lock:
            self._write_byte_raw(current_id, REG_LOCK, 0)
            self._write_byte_raw(current_id, REG_ID, new_id)
            self._write_byte_raw(current_id, REG_LOCK, 1)
        time.sleep(0.05)
        return True

    def clear_eeprom_speed_limit(self, servo_id: int) -> bool:
        with self.lock:
            self._write_byte_raw(servo_id, REG_LOCK, 0)
            pkt = self._build_packet(servo_id, INST_WRITE,
                                     [REG_EEPROM_MAX_SPEED, 0x00, 0x00])
            self._send(pkt)
            time.sleep(0.01)
            self._write_byte_raw(servo_id, REG_LOCK, 1)
        time.sleep(0.05)
        return True

    def calibrate_zero(self, servo_id: int) -> bool:
        with self.lock:
            self._write_byte_raw(servo_id, REG_LOCK, 0)
            self._write_byte_raw(servo_id, REG_TORQUE_ENABLE, CALIBRATE_CMD)
            self._write_byte_raw(servo_id, REG_LOCK, 1)
        time.sleep(0.1)
        return True

    def set_torque(self, servo_id: int, enable: bool) -> bool:
        return self.write_byte(servo_id, REG_TORQUE_ENABLE, 1 if enable else 0)

    def set_goal_position(self, servo_id: int, position: int,
                          speed: int = 500, acc: int = 50) -> bool:
        pos_lo = position & 0xFF
        pos_hi = (position >> 8) & 0xFF
        spd_lo = speed & 0xFF
        spd_hi = (speed >> 8) & 0xFF
        payload = [REG_GOAL_ACC, acc & 0xFF,
                   pos_lo, pos_hi, 0, 0, spd_lo, spd_hi]
        with self.lock:
            pkt = self._build_packet(servo_id, INST_WRITE, payload)
            self._send(pkt)
            time.sleep(0.005)
        return True

    def sync_write_positions(self, id_pos_speed: list) -> None:
        data_len = 7
        params = [REG_GOAL_ACC, data_len]
        for sid, pos, spd, acc in id_pos_speed:
            params += [sid, acc & 0xFF,
                       pos & 0xFF, (pos >> 8) & 0xFF,
                       0, 0,
                       spd & 0xFF, (spd >> 8) & 0xFF]
        with self.lock:
            pkt = self._build_packet(STS_BROADCAST_ID, INST_SYNC_WRITE, params)
            self._send(pkt)
            time.sleep(0.005)

    # ── sensor reads ────────────────────────────────────────────────────────
    def get_present_position(self, servo_id: int) -> Optional[int]:
        return self.read_word(servo_id, REG_PRESENT_POSITION)

    def get_temperature(self, servo_id: int) -> Optional[int]:
        return self.read_byte(servo_id, REG_PRESENT_TEMP)

    def get_voltage(self, servo_id: int) -> Optional[int]:
        return self.read_byte(servo_id, REG_PRESENT_VOLTAGE)

    def is_moving(self, servo_id: int) -> Optional[bool]:
        """Return True if the servo is still moving toward its target.
        Returns None if the servo does not respond."""
        val = self.read_byte(servo_id, REG_MOVING)
        if val is None:
            return None
        return bool(val)

    # ── unit conversion helpers ──────────────────────────────────────────────
    @staticmethod
    def deg_to_pos(deg: float) -> int:
        return max(POS_MIN, min(POS_MAX, int((deg / DEG_MAX) * POS_MAX)))

    @staticmethod
    def pos_to_deg(pos: int) -> float:
        return round((pos / POS_MAX) * DEG_MAX, 2)
