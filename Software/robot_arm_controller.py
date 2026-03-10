#!/usr/bin/env python3
"""
6DOF Robot Arm Controller - Feetech STS3215 Servos
Features: servo setup, angle control, sequence playback, teach mode,
          real-time monitoring, dark/light theme toggle.
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import time
import json
import serial
import serial.tools.list_ports
from dataclasses import dataclass, field, asdict
from typing import Optional

# ─────────────────────────────────────────────
#  STS3215 Protocol Constants
# ─────────────────────────────────────────────
STS_HEADER = 0xFF
STS_BROADCAST_ID = 0xFE

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_WRITE = 0x83

REG_ID = 0x05
REG_EEPROM_MIN_ANGLE = 0x09   # EEPROM min angle limit (2 bytes)
REG_EEPROM_MAX_ANGLE = 0x0B   # EEPROM max angle limit (2 bytes)
# EEPROM max speed limit (2 bytes) — persists across reboots
REG_EEPROM_MAX_SPEED = 0x0F
# decimal 40 — also used for one-key calibration (write 128)
REG_TORQUE_ENABLE = 0x28
REG_LOCK = 0x37   # decimal 55 — RAM lock: 0=EEPROM writes allowed, 1=locked
# decimal 41 — acceleration (1 byte); WritePos starts here
REG_GOAL_ACC = 0x29
# decimal 42 — goal position (2 bytes, little-endian)
REG_GOAL_POSITION = 0x2A
REG_GOAL_SPEED = 0x2E   # decimal 46 — goal speed   (2 bytes, little-endian)
REG_PRESENT_POSITION = 0x38   # decimal 56
REG_PRESENT_TEMP = 0x3F   # decimal 63
REG_PRESENT_VOLTAGE = 0x3E   # decimal 62

# One-key calibration: write 128 to REG_TORQUE_ENABLE (after EEPROM unlock)
# This tells the servo to store current physical position as center (2048).
CALIBRATE_CMD = 128

POS_MIN = 0
POS_MAX = 4095
DEG_MAX = 360.0
POS_CENTER = 2047   # maps to 0° logical

JOINT_NAMES = [f"Joint {i+1}" for i in range(6)]

# ─────────────────────────────────────────────
#  Theme palettes
# ─────────────────────────────────────────────
THEMES = {
    "dark": {
        "BG":         "#1a1a2e",
        "PANEL":      "#16213e",
        "ACCENT":     "#0f3460",
        "ORANGE":     "#e94560",
        "GREEN":      "#4ecca3",
        "TEXT":       "#eaeaea",
        "SUBTEXT":    "#a0a0b0",
        "ENTRY":      "#0d2137",
        "DANGER_BTN": "#8b0000",
        "GREEN_BTN":  "#1a6b4a",
        "SEL_BG":     "#0f3460",
    },
    "light": {
        "BG":         "#f0f2f5",
        "PANEL":      "#dde1ea",
        "ACCENT":     "#4a90d9",
        "ORANGE":     "#d0392b",
        "GREEN":      "#217a52",
        "TEXT":       "#1a1a2a",
        "SUBTEXT":    "#555577",
        "ENTRY":      "#ffffff",
        "DANGER_BTN": "#c0392b",
        "GREEN_BTN":  "#217a52",
        "SEL_BG":     "#4a90d9",
    },
}


# ─────────────────────────────────────────────
#  Low-level serial driver
# ─────────────────────────────────────────────
class STS3215Driver:
    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()

    def connect(self, port: str, baud: int = 1000000) -> bool:
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(0.1)
            return True
        except Exception as e:
            print(f"[Driver] Connect error: {e}")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    @property
    def connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    # ── packet helpers ───────────────────────
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
            self.ser.reset_input_buffer()
            self.ser.write(packet)

    def _read_response(self) -> Optional[list]:
        try:
            hdr = self.ser.read(2)
            if len(hdr) < 2 or hdr[0] != STS_HEADER or hdr[1] != STS_HEADER:
                return None
            meta = self.ser.read(3)
            if len(meta) < 3:
                return None
            length = meta[1]
            params = self.ser.read(length - 2)
            self.ser.read(1)  # checksum byte
            return list(params)
        except Exception:
            return None

    # ── register I/O ────────────────────────
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

    # ── low-level unlocked writes (only call while holding self.lock) ──
    def _write_byte_raw(self, servo_id: int, reg: int, value: int):
        pkt = self._build_packet(servo_id, INST_WRITE, [reg, value & 0xFF])
        self._send(pkt)
        time.sleep(0.01)

    # ── high-level commands ──────────────────
    def set_id(self, current_id: int, new_id: int) -> bool:
        """Write new ID to EEPROM persistently.

        STS3215 EEPROM lock is at RAM register 0x37 (decimal 55).
        Sequence:
          1. Unlock EEPROM  (write 0 to REG_LOCK, addressed to current_id)
          2. Write new ID   (write new_id to REG_ID,  addressed to current_id)
          3. Re-lock EEPROM (write 1 to REG_LOCK, addressed to current_id —
                             must use current_id here because the servo still
                             responds on its old ID until power-cycled)
          4. Power-cycle the servo for the new ID to take effect.

        NOTE: After calling this, the servo still responds on current_id until
        it is power-cycled.  Position/torque commands will continue to use
        current_id correctly until then.
        """
        with self.lock:
            self._write_byte_raw(current_id, REG_LOCK, 0)   # unlock EEPROM
            # write new ID to EEPROM
            self._write_byte_raw(current_id, REG_ID, new_id)
            # re-lock (still current_id!)
            self._write_byte_raw(current_id, REG_LOCK, 1)
        time.sleep(0.05)
        return True

    def clear_eeprom_speed_limit(self, servo_id: int) -> bool:
        """Clear the persistent EEPROM speed limit (REG_EEPROM_MAX_SPEED = 0x0F).
        Writing 0 removes any cap set by external tools like FD software.
        Requires EEPROM unlock/lock sequence.
        """
        with self.lock:
            self._write_byte_raw(servo_id, REG_LOCK, 0)
            # write 0 to max speed (2 bytes at 0x0F)
            pkt = self._build_packet(servo_id, INST_WRITE,
                                     [REG_EEPROM_MAX_SPEED, 0x00, 0x00])
            self._send(pkt)
            time.sleep(0.01)
            self._write_byte_raw(servo_id, REG_LOCK, 1)
        time.sleep(0.05)
        return True

    def calibrate_zero(self, servo_id: int) -> bool:
        """Persistently set the servo's current physical position as its center (2048).

        Uses the STS3215 one-key calibration: unlock EEPROM, write 128 to
        REG_TORQUE_ENABLE (address 40), re-lock. The servo stores the current
        position as its new zero reference in EEPROM — survives power cycles.
        The servo must NOT be under torque load when this is called.
        """
        with self.lock:
            self._write_byte_raw(servo_id, REG_LOCK, 0)
            self._write_byte_raw(servo_id, REG_TORQUE_ENABLE, CALIBRATE_CMD)
            self._write_byte_raw(servo_id, REG_LOCK, 1)
        time.sleep(0.1)
        return True

    def set_torque(self, servo_id: int, enable: bool) -> bool:
        return self.write_byte(servo_id, REG_TORQUE_ENABLE, 1 if enable else 0)

    def set_goal_position(self, servo_id: int, position: int, speed: int = 500, acc: int = 50) -> bool:
        """Single-servo position command using the 6-byte WritePos format.

        Writes 6 bytes starting at REG_GOAL_ACC (0x29):
          [acc, pos_lo, pos_hi, time_lo(0), time_hi(0), spd_lo, spd_hi]
        This matches the official SMS_STS WritePos() exactly.
        'time' bytes are 0 — speed control is used, not time control.
        """
        pos_lo = position & 0xFF
        pos_hi = (position >> 8) & 0xFF
        spd_lo = speed & 0xFF
        spd_hi = (speed >> 8) & 0xFF
        payload = [REG_GOAL_ACC, acc & 0xFF,
                   pos_lo, pos_hi,
                   0, 0,          # time = 0 (use speed, not time control)
                   spd_lo, spd_hi]
        with self.lock:
            pkt = self._build_packet(servo_id, INST_WRITE, payload)
            self._send(pkt)
            time.sleep(0.005)
        return True

    def get_present_position(self, servo_id: int) -> Optional[int]:
        return self.read_word(servo_id, REG_PRESENT_POSITION)

    def get_temperature(self, servo_id: int) -> Optional[int]:
        return self.read_byte(servo_id, REG_PRESENT_TEMP)

    def get_voltage(self, servo_id: int) -> Optional[int]:
        return self.read_byte(servo_id, REG_PRESENT_VOLTAGE)

    def sync_write_positions(self, id_pos_speed: list) -> None:
        """Sync-write positions to multiple servos in one broadcast packet.

        id_pos_speed: [(servo_id, raw_position, speed, acc), ...]

        Writes 7 data bytes per servo starting at REG_GOAL_ACC (0x29):
          [acc(1), pos_lo(1), pos_hi(1), time_lo(1), time_hi(1), spd_lo(1), spd_hi(1)]
        data_len = 7  (bytes per servo, NOT counting the ID byte).
        """
        data_len = 7   # acc + pos_lo + pos_hi + time_lo + time_hi + spd_lo + spd_hi
        params = [REG_GOAL_ACC, data_len]
        for sid, pos, spd, acc in id_pos_speed:
            params += [sid,
                       acc & 0xFF,
                       pos & 0xFF, (pos >> 8) & 0xFF,
                       0, 0,                            # time bytes = 0
                       spd & 0xFF, (spd >> 8) & 0xFF]
        with self.lock:
            pkt = self._build_packet(STS_BROADCAST_ID, INST_SYNC_WRITE, params)
            self._send(pkt)
            time.sleep(0.005)

    # ── unit helpers ────────────────────────
    @staticmethod
    def deg_to_pos(deg: float) -> int:
        return max(POS_MIN, min(POS_MAX, int((deg / DEG_MAX) * POS_MAX)))

    @staticmethod
    def pos_to_deg(pos: int) -> float:
        return round((pos / POS_MAX) * DEG_MAX, 2)


# ─────────────────────────────────────────────
#  Data structures
# ─────────────────────────────────────────────
@dataclass
class JointConfig:
    servo_id:    int = 1
    zero_offset: int = POS_CENTER
    min_angle:   float = -90.0
    max_angle:   float = 90.0
    home_angle:  float = 0.0
    speed:       int = 30
    acc:         int = 10


@dataclass
class RobotConfig:
    joints: list = field(
        default_factory=lambda: [JointConfig(servo_id=i+1) for i in range(6)]
    )
    port: str = ""
    baud: int = 1000000


@dataclass
class Waypoint:
    name:     str
    angles:   list   # logical degrees (for display/editing)
    raw_pos:  list   # raw servo counts (what actually gets sent)
    duration: float = 1.0


# ─────────────────────────────────────────────
#  Robot Controller
# ─────────────────────────────────────────────
class RobotController:
    def __init__(self):
        self.driver = STS3215Driver()
        self.config = RobotConfig()
        self.sequence: list = []

        self.live_positions: list = [None] * 6   # logical degrees
        self.live_raw:       list = [None] * 6   # raw counts

        self._poll_running = False
        self._seq_running = False
        self._poll_thread = None
        self._seq_thread = None

        self.on_position_update = None   # callback(joint_idx, deg)

    # ── connection ──────────────────────────
    def connect(self, port: str, baud: int = 1000000) -> bool:
        self.config.port = port
        self.config.baud = baud
        ok = self.driver.connect(port, baud)
        if ok:
            self._start_polling()
        return ok

    def disconnect(self):
        self._stop_polling()
        self.driver.disconnect()

    # ── polling ─────────────────────────────
    def _start_polling(self):
        self._poll_running = True
        self._poll_thread = threading.Thread(
            target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def _stop_polling(self):
        self._poll_running = False
        if self._poll_thread:
            self._poll_thread.join(timeout=2)

    def _poll_loop(self):
        while self._poll_running:
            for i, jcfg in enumerate(self.config.joints):
                raw = self.driver.get_present_position(jcfg.servo_id)
                if raw is not None:
                    self.live_raw[i] = raw
                    deg = self._raw_to_logical(i, raw)
                    self.live_positions[i] = deg
                    if self.on_position_update:
                        self.on_position_update(i, deg)
            time.sleep(0.05)   # ~20 Hz

    # ── angle ↔ raw ──────────────────────────
    def _raw_to_logical(self, joint_idx: int, raw: int) -> float:
        offset = raw - self.config.joints[joint_idx].zero_offset
        deg = STS3215Driver.pos_to_deg(offset + POS_CENTER) - 180.0
        return round(-deg, 2)

    def _logical_to_raw(self, joint_idx: int, deg: float) -> int:
        jcfg = self.config.joints[joint_idx]
        # enforce joint limits
        deg = max(jcfg.min_angle, min(jcfg.max_angle, deg))
        pos = STS3215Driver.deg_to_pos(-deg + 180.0)
        raw = pos - POS_CENTER + jcfg.zero_offset
        return max(POS_MIN, min(POS_MAX, raw))

    def angles_to_raw(self, angles: list) -> list:
        return [self._logical_to_raw(i, deg) for i, deg in enumerate(angles)]

    # ── joint control ────────────────────────
    def set_joint_angle(self, joint_idx: int, deg: float):
        jcfg = self.config.joints[joint_idx]
        raw = self._logical_to_raw(joint_idx, deg)
        self.driver.set_goal_position(jcfg.servo_id, raw, jcfg.speed, jcfg.acc)

    def set_all_angles(self, angles: list) -> None:
        """Convert logical degrees → raw and sync-write all joints."""
        id_pos_speed = [
            (self.config.joints[i].servo_id,
             self._logical_to_raw(i, deg),
             self.config.joints[i].speed,
             self.config.joints[i].acc)
            for i, deg in enumerate(angles)
        ]
        self.driver.sync_write_positions(id_pos_speed)

    def set_all_raw(self, raw_positions: list) -> None:
        """Send pre-computed raw positions directly (used by sequence playback)."""
        id_pos_speed = [
            (self.config.joints[i].servo_id, raw,
             self.config.joints[i].speed,
             self.config.joints[i].acc)
            for i, raw in enumerate(raw_positions)
        ]
        self.driver.sync_write_positions(id_pos_speed)

    def go_home(self) -> None:
        """Send all servos to their configured home angle."""
        angles = [jcfg.home_angle for jcfg in self.config.joints]
        self.set_all_angles(angles)

    def torque_all(self, enable: bool):
        for jcfg in self.config.joints:
            self.driver.set_torque(jcfg.servo_id, enable)

    # ── teach mode ──────────────────────────
    def enter_teach_mode(self):
        self.torque_all(False)

    def exit_teach_mode(self):
        self.torque_all(True)

    def capture_position(self, name: str) -> Waypoint:
        """Snapshot current live positions into a Waypoint (stores both angles and raw)."""
        angles = [p if p is not None else 0.0 for p in self.live_positions]
        raw_pos = [r if r is not None else POS_CENTER for r in self.live_raw]
        wp = Waypoint(name=name, angles=angles, raw_pos=raw_pos)
        self.sequence.append(wp)
        return wp

    # ── sequence playback ────────────────────
    def play_sequence(self, loop: bool = False, progress_cb=None, done_cb=None):
        """Play the stored sequence using raw positions — no angle conversion needed."""
        self._seq_running = True

        def _run():
            repeat = True
            while repeat and self._seq_running:
                for idx, wp in enumerate(self.sequence):
                    if not self._seq_running:
                        break
                    if progress_cb:
                        progress_cb(idx, wp)
                    # Send raw positions directly to servos
                    self.set_all_raw(wp.raw_pos)
                    time.sleep(wp.duration)
                repeat = loop and self._seq_running
            self._seq_running = False
            if done_cb:
                done_cb()

        self._seq_thread = threading.Thread(target=_run, daemon=True)
        self._seq_thread.start()

    def stop_sequence(self):
        self._seq_running = False

    # ── setup helpers ────────────────────────
    def set_servo_id(self, current_id: int, new_id: int) -> bool:
        return self.driver.set_id(current_id, new_id)

    def set_zero_here(self, joint_idx: int) -> bool:
        """Persistently calibrate the servo's current position as its hardware zero.

        Calls the STS3215 one-key calibration which writes the current physical
        position as center (2048) into EEPROM. This survives power cycles.
        Also resets the software zero_offset to POS_CENTER so logical angles
        remain consistent with the new hardware zero.
        """
        jcfg = self.config.joints[joint_idx]
        ok = self.driver.calibrate_zero(jcfg.servo_id)
        if ok:
            # Reset software offset — hardware is now the true zero
            jcfg.zero_offset = POS_CENTER
        return ok

    def clear_speed_limit(self, joint_idx: int) -> bool:
        return self.driver.clear_eeprom_speed_limit(
            self.config.joints[joint_idx].servo_id)

    def ping_servo(self, servo_id: int) -> bool:
        return self.driver.ping(servo_id)

    def get_servo_info(self, servo_id: int) -> dict:
        return {
            "position":    self.driver.get_present_position(servo_id),
            "temperature": self.driver.get_temperature(servo_id),
            "voltage":     self.driver.get_voltage(servo_id),
        }

    # ── persistence ──────────────────────────
    def save_config(self, path: str):
        data = {"port": self.config.port, "baud": self.config.baud,
                "joints": [asdict(j) for j in self.config.joints]}
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

    def load_config(self, path: str):
        with open(path) as f:
            data = json.load(f)
        self.config.port = data.get("port", "")
        self.config.baud = data.get("baud", 1000000)
        for i, jd in enumerate(data.get("joints", [])[:6]):
            jd.setdefault("acc",        10)
            jd.setdefault("home_angle", 0.0)
            jd.setdefault("min_angle",  -90.0)
            jd.setdefault("max_angle",   90.0)
            self.config.joints[i] = JointConfig(**jd)

    def save_sequence(self, path: str):
        data = [{"name": w.name, "angles": w.angles,
                 "raw_pos": w.raw_pos, "duration": w.duration}
                for w in self.sequence]
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

    def load_sequence(self, path: str):
        with open(path) as f:
            data = json.load(f)
        self.sequence = []
        for d in data:
            # Back-compat: old files without raw_pos compute it from angles
            if "raw_pos" not in d:
                d["raw_pos"] = self.angles_to_raw(d["angles"])
            self.sequence.append(Waypoint(**d))


# ─────────────────────────────────────────────
#  GUI
# ─────────────────────────────────────────────
class RobotGUI:

    def __init__(self, root: tk.Tk):
        self.root = root
        self.ctrl = RobotController()
        self.ctrl.on_position_update = self._on_position_update

        self._teach_mode = False
        self._theme_name = "dark"
        self.T = dict(THEMES["dark"])

        # per-joint Tk variables
        self.pos_vars:        list = [
            tk.StringVar(value="—") for _ in range(6)]
        self.target_vars:     list = [
            tk.DoubleVar(value=0.0) for _ in range(6)]
        self.id_vars:         list = [tk.IntVar(value=i+1) for i in range(6)]
        self.speed_vars:      list = [tk.IntVar(value=30) for _ in range(6)]
        self.acc_vars:        list = [tk.IntVar(value=10) for _ in range(6)]
        self.min_angle_vars:  list = [
            tk.DoubleVar(value=-90.0) for _ in range(6)]
        self.max_angle_vars:  list = [
            tk.DoubleVar(value=90.0) for _ in range(6)]
        self.home_angle_vars: list = [
            tk.DoubleVar(value=0.0) for _ in range(6)]

        self._build_ui()
        self.root.after(200, self._refresh_ports)

    # ─────────────────────────────────────────
    #  Theme
    # ─────────────────────────────────────────
    def _apply_theme(self):
        T = self.T
        style = ttk.Style()
        style.theme_use("clam")

        style.configure(".",
                        background=T["BG"], foreground=T["TEXT"],
                        fieldbackground=T["ENTRY"], troughcolor=T["ACCENT"],
                        selectbackground=T["SEL_BG"], selectforeground=T["TEXT"])
        style.configure("TNotebook",
                        background=T["BG"],         borderwidth=0)
        style.configure(
            "TNotebook.Tab",      background=T["ACCENT"],     foreground=T["TEXT"], padding=[12, 4])
        style.map("TNotebook.Tab",            background=[
                  ("selected", T["ORANGE"])])
        style.configure("TFrame",             background=T["BG"])
        style.configure(
            "TLabelframe",        background=T["BG"],         foreground=T["TEXT"], bordercolor=T["ACCENT"])
        style.configure("TLabelframe.Label",
                        background=T["BG"],         foreground=T["GREEN"])
        style.configure("TButton",            background=T["ACCENT"],
                        foreground=T["TEXT"], borderwidth=0, relief="flat", padding=[8, 4])
        style.map("TButton",                  background=[
                  ("active", T["ORANGE"]), ("pressed", T["ORANGE"])])
        style.configure("Danger.TButton",
                        background=T["DANGER_BTN"], foreground=T["TEXT"])
        style.map("Danger.TButton",           background=[
                  ("active", T["ORANGE"])])
        style.configure("Green.TButton",
                        background=T["GREEN_BTN"],  foreground=T["TEXT"])
        style.map("Green.TButton",            background=[
                  ("active", T["GREEN"])])
        style.configure(
            "TScale",             background=T["BG"],         troughcolor=T["ACCENT"])
        style.configure("TCombobox",
                        fieldbackground=T["ENTRY"], foreground=T["TEXT"])
        style.configure(
            "TEntry",             fieldbackground=T["ENTRY"], foreground=T["TEXT"])
        style.configure("TCheckbutton",
                        background=T["BG"],         foreground=T["TEXT"])
        style.configure("TSeparator",         background=T["ACCENT"])

        self.root.configure(bg=T["BG"])
        self._recolour(self.root)

    def _recolour(self, widget):
        """Recursively update plain tk widget colours."""
        T = self.T
        cls = widget.__class__.__name__
        try:
            if cls == "Frame":
                widget.configure(bg=T["BG"])
            elif cls == "Label":
                # inherit parent bg where possible
                try:
                    pbg = widget.master.cget("bg")
                except Exception:
                    pbg = T["BG"]
                widget.configure(bg=pbg, fg=T["TEXT"])
            elif cls == "Text":
                widget.configure(bg=T["ENTRY"], fg=T["GREEN"],
                                 insertbackground=T["TEXT"])
            elif cls == "Listbox":
                widget.configure(bg=T["ENTRY"], fg=T["TEXT"],
                                 selectbackground=T["SEL_BG"],
                                 selectforeground=T["TEXT"])
            elif cls == "Button":
                widget.configure(bg=T["ACCENT"], fg=T["TEXT"],
                                 activebackground=T["ORANGE"],
                                 activeforeground=T["TEXT"])
        except Exception:
            pass
        for child in widget.winfo_children():
            self._recolour(child)
        # Fix special named frames
        try:
            self.header_frame.configure(bg=T["PANEL"])
            self.topbar_frame.configure(bg=T["ORANGE"])
            self.sb_frame.configure(bg=T["PANEL"])
            self.status_label.configure(bg=T["PANEL"])
            self.sb_label.configure(bg=T["PANEL"], fg=T["SUBTEXT"])
            self.theme_btn.configure(bg=T["ACCENT"], fg=T["TEXT"],
                                     activebackground=T["ORANGE"])
        except Exception:
            pass

    def _toggle_theme(self):
        self._theme_name = "light" if self._theme_name == "dark" else "dark"
        self.T = dict(THEMES[self._theme_name])
        self.theme_btn.configure(
            text="☀  Light" if self._theme_name == "dark" else "🌙  Dark")
        self._apply_theme()
        # restore connection status colour
        conn_fg = self.T["GREEN"] if self.ctrl.driver.connected else self.T["ORANGE"]
        self.status_label.configure(fg=conn_fg)
        # restore teach status colour
        if self._teach_mode:
            self.teach_status.configure(fg=self.T["ORANGE"])

    # ─────────────────────────────────────────
    #  UI construction
    # ─────────────────────────────────────────
    def _build_ui(self):
        T = self.T
        self.root.title("6DOF Robot Arm Controller  •  Feetech STS3215")
        self.root.configure(bg=T["BG"])
        self.root.resizable(True, True)
        self.root.minsize(920, 720)

        self._apply_theme()

        # ── top accent bar ──────────────────
        self.topbar_frame = tk.Frame(self.root, bg=T["ORANGE"], height=4)
        self.topbar_frame.pack(fill="x")

        # ── header ──────────────────────────
        self.header_frame = tk.Frame(self.root, bg=T["PANEL"], pady=8)
        self.header_frame.pack(fill="x", padx=4, pady=(0, 4))

        tk.Label(self.header_frame, text="⚙  6DOF Robot Arm Controller",
                 bg=T["PANEL"], fg=T["TEXT"],
                 font=("Helvetica", 16, "bold")).pack(side="left", padx=16)

        self.theme_btn = tk.Button(
            self.header_frame, text="☀  Light",
            bg=T["ACCENT"], fg=T["TEXT"],
            activebackground=T["ORANGE"], activeforeground=T["TEXT"],
            relief="flat", padx=10, pady=3, cursor="hand2",
            command=self._toggle_theme)
        self.theme_btn.pack(side="right", padx=10)

        self.estop_btn = tk.Button(
            self.header_frame, text="⛔  E-STOP",
            bg="#cc0000", fg="white",
            activebackground="#ff0000", activeforeground="white",
            font=("Helvetica", 11, "bold"),
            relief="flat", padx=14, pady=4, cursor="hand2",
            command=self._emergency_stop)
        self.estop_btn.pack(side="right", padx=10)

        self.status_label = tk.Label(
            self.header_frame, text="● DISCONNECTED",
            bg=T["PANEL"], fg=T["ORANGE"],
            font=("Helvetica", 10, "bold"))
        self.status_label.pack(side="right", padx=16)

        # ── notebook ────────────────────────
        self.nb = ttk.Notebook(self.root)
        self.nb.pack(fill="both", expand=True, padx=6, pady=4)

        self.tab_conn = ttk.Frame(self.nb)
        self.nb.add(self.tab_conn,  text="  🔌 Connection  ")
        self.tab_ctrl = ttk.Frame(self.nb)
        self.nb.add(self.tab_ctrl,  text="  🕹 Control  ")
        self.tab_setup = ttk.Frame(self.nb)
        self.nb.add(self.tab_setup, text="  ⚙ Setup  ")
        self.tab_seq = ttk.Frame(self.nb)
        self.nb.add(self.tab_seq,   text="  📋 Sequence  ")

        self._build_conn_tab()
        self._build_ctrl_tab()
        self._build_setup_tab()
        self._build_seq_tab()

        # ── status bar ──────────────────────
        self.sb_frame = tk.Frame(self.root, bg=T["PANEL"], height=26)
        self.sb_frame.pack(fill="x", side="bottom")
        self.log_var = tk.StringVar(value="Ready.")
        self.sb_label = tk.Label(self.sb_frame, textvariable=self.log_var,
                                 bg=T["PANEL"], fg=T["SUBTEXT"],
                                 font=("Courier", 9))
        self.sb_label.pack(side="left", padx=10)

    # ── Connection tab ───────────────────────
    def _build_conn_tab(self):
        f = self.tab_conn
        f.columnconfigure(0, weight=1)
        f.columnconfigure(1, weight=1)
        T = self.T

        lf = ttk.LabelFrame(f, text=" Serial Port ", padding=12)
        lf.grid(row=0, column=0, padx=12, pady=12, sticky="nsew")

        tk.Label(lf, text="Port:", bg=T["BG"], fg=T["SUBTEXT"]).grid(
            row=0, column=0, sticky="w")
        self.port_cb = ttk.Combobox(lf, width=18, state="readonly")
        self.port_cb.grid(row=0, column=1, padx=6, pady=3)

        tk.Label(lf, text="Baud:", bg=T["BG"], fg=T["SUBTEXT"]).grid(
            row=1, column=0, sticky="w")
        self.baud_cb = ttk.Combobox(lf, values=["1000000", "500000", "115200", "57600"],
                                    width=18, state="readonly")
        self.baud_cb.current(0)
        self.baud_cb.grid(row=1, column=1, padx=6, pady=3)

        bf = tk.Frame(lf, bg=T["BG"])
        bf.grid(row=2, column=0, columnspan=2, pady=8)
        ttk.Button(bf, text="↺ Refresh", command=self._refresh_ports).pack(
            side="left", padx=4)
        self.conn_btn = ttk.Button(bf, text="Connect", style="Green.TButton",
                                   command=self._toggle_connect)
        self.conn_btn.pack(side="left", padx=4)

        lf2 = ttk.LabelFrame(f, text=" Robot Config ", padding=12)
        lf2.grid(row=0, column=1, padx=12, pady=12, sticky="nsew")
        ttk.Button(lf2, text="💾  Save Config",
                   command=self._save_config).pack(fill="x", pady=3)
        ttk.Button(lf2, text="📂  Load Config",
                   command=self._load_config).pack(fill="x", pady=3)

        lf3 = ttk.LabelFrame(f, text=" Servo Scanner ", padding=12)
        lf3.grid(row=1, column=0, columnspan=2, padx=12, pady=4, sticky="nsew")
        f.rowconfigure(1, weight=1)

        st = tk.Frame(lf3, bg=T["BG"])
        st.pack(fill="x")
        ttk.Button(st, text="🔍  Scan IDs 1–20",
                   command=self._scan_servos).pack(side="left")
        self.scan_result = tk.Text(lf3, height=6, bg=T["ENTRY"], fg=T["GREEN"],
                                   font=("Courier", 9), relief="flat")
        self.scan_result.pack(fill="both", expand=True, pady=6)

    # ── Control tab ──────────────────────────
    def _build_ctrl_tab(self):
        f = self.tab_ctrl
        f.columnconfigure(0, weight=1)
        T = self.T

        hdr = tk.Frame(f, bg=T["PANEL"], pady=6)
        hdr.pack(fill="x", padx=8, pady=(8, 0))
        tk.Label(hdr, text="LIVE JOINT POSITIONS", bg=T["PANEL"],
                 fg=T["GREEN"], font=("Helvetica", 10, "bold")).pack(side="left", padx=10)

        jf = ttk.LabelFrame(f, text=" Joints ", padding=8)
        jf.pack(fill="x", padx=8, pady=6)

        for c, (h, w) in enumerate(zip(
                ["Joint", "Current °", "Target °", "−", "Slider", "+", "Set"],
                [10,       10,          10,           3,   30,       3,   6])):
            tk.Label(jf, text=h, bg=T["BG"], fg=T["SUBTEXT"],
                     font=("Helvetica", 9, "bold"), width=w).grid(row=0, column=c, padx=3)

        self.sliders: list = []
        for i, name in enumerate(JOINT_NAMES):
            r = i + 1
            jcfg = self.ctrl.config.joints[i]
            tk.Label(jf, text=name, bg=T["BG"], fg=T["TEXT"],
                     font=("Helvetica", 10), width=10, anchor="w").grid(row=r, column=0, padx=4, pady=3)
            tk.Label(jf, textvariable=self.pos_vars[i], bg=T["BG"],
                     fg=T["GREEN"], font=("Courier", 11, "bold"), width=10).grid(row=r, column=1)
            ttk.Entry(jf, textvariable=self.target_vars[i], width=8).grid(
                row=r, column=2, padx=4)
            ttk.Button(jf, text="−", width=2,
                       command=lambda idx=i: self._nudge_joint(idx, +1)).grid(row=r, column=3, padx=(4, 0))
            sld = ttk.Scale(jf, from_=jcfg.min_angle, to=jcfg.max_angle, orient="horizontal",
                            variable=self.target_vars[i], length=220,
                            command=lambda v, idx=i: self.target_vars[idx].set(round(float(v), 2)))
            sld.grid(row=r, column=4, padx=2)
            self.sliders.append(sld)
            ttk.Button(jf, text="+", width=2,
                       command=lambda idx=i: self._nudge_joint(idx, -1)).grid(row=r, column=5, padx=(0, 4))
            ttk.Button(jf, text="▶", width=3,
                       command=lambda idx=i: self._send_joint(idx)).grid(row=r, column=6)

        bc = tk.Frame(f, bg=T["BG"])
        bc.pack(fill="x", padx=12, pady=8)
        ttk.Button(bc, text="▶  Send ALL",   style="Green.TButton",
                   command=self._send_all).pack(side="left", padx=4)
        ttk.Button(bc, text="⏹  Torque OFF", style="Danger.TButton",
                   command=lambda: self.ctrl.torque_all(False)).pack(side="left", padx=4)
        ttk.Button(bc, text="⚡  Torque ON",
                   command=lambda: self.ctrl.torque_all(True)).pack(side="left", padx=4)
        ttk.Button(bc, text="🏠  Home (0°)",
                   command=self._go_home).pack(side="left", padx=4)

        tf = ttk.LabelFrame(f, text=" Teach Mode ", padding=10)
        tf.pack(fill="x", padx=8, pady=6)

        tm = tk.Frame(tf, bg=T["BG"])
        tm.pack(fill="x")
        self.teach_btn = ttk.Button(tm, text="🖐  Enter Teach Mode",
                                    command=self._toggle_teach)
        self.teach_btn.pack(side="left", padx=4)
        tk.Label(tm, text="Position name:", bg=T["BG"], fg=T["SUBTEXT"]).pack(
            side="left", padx=8)
        self.pos_name_var = tk.StringVar(value="pose_1")
        ttk.Entry(tm, textvariable=self.pos_name_var,
                  width=14).pack(side="left")
        ttk.Button(tm, text="📍  Capture", style="Green.TButton",
                   command=self._capture_pose).pack(side="left", padx=6)

        self.teach_status = tk.Label(tf, text="", bg=T["BG"], fg=T["ORANGE"],
                                     font=("Helvetica", 9, "italic"))
        self.teach_status.pack()

    # ── Setup tab ────────────────────────────
    def _build_setup_tab(self):
        f = self.tab_setup
        f.columnconfigure(0, weight=1)
        f.columnconfigure(1, weight=1)
        T = self.T

        lf = ttk.LabelFrame(f, text=" Change Servo ID ", padding=12)
        lf.grid(row=0, column=0, padx=12, pady=12, sticky="nsew")

        tk.Label(lf, text="Current ID:", bg=T["BG"], fg=T["SUBTEXT"]).grid(
            row=0, column=0, sticky="w")
        self.cur_id_var = tk.IntVar(value=1)
        ttk.Entry(lf, textvariable=self.cur_id_var, width=8).grid(
            row=0, column=1, padx=6, pady=3)
        tk.Label(lf, text="New ID:", bg=T["BG"], fg=T["SUBTEXT"]).grid(
            row=1, column=0, sticky="w")
        self.new_id_var = tk.IntVar(value=2)
        ttk.Entry(lf, textvariable=self.new_id_var, width=8).grid(
            row=1, column=1, padx=6, pady=3)
        ttk.Button(lf, text="✎  Change ID", command=self._change_id).grid(
            row=2, column=0, columnspan=2, pady=8)

        lf2 = ttk.LabelFrame(f, text=" Joint Configuration ", padding=12)
        lf2.grid(row=0, column=1, padx=12, pady=12, sticky="nsew")

        headers = ["Joint", "Servo ID", "Speed", "ACC", "Min°",
                   "Max°", "Home°", "Set Zero", "Ping", "Clr Spd"]
        for c, col in enumerate(headers):
            tk.Label(lf2, text=col, bg=T["BG"], fg=T["SUBTEXT"],
                     font=("Helvetica", 9, "bold")).grid(row=0, column=c, padx=4)

        for i, name in enumerate(JOINT_NAMES):
            r = i + 1
            tk.Label(lf2, text=name, bg=T["BG"], fg=T["TEXT"],
                     width=8, anchor="w").grid(row=r, column=0, padx=4, pady=3)

            sb_id = ttk.Spinbox(lf2, textvariable=self.id_vars[i],
                                from_=1, to=253, width=5)
            sb_id.grid(row=r, column=1, padx=3)
            sb_id.bind("<FocusOut>", lambda e, idx=i: self._sync_id(idx))
            sb_id.bind("<Return>", lambda e, idx=i: self._sync_id(idx))

            sb_spd = ttk.Spinbox(lf2, textvariable=self.speed_vars[i],
                                 from_=0, to=4095, increment=5, width=5)
            sb_spd.grid(row=r, column=2, padx=3)
            for ev in ("<FocusOut>", "<Return>", "<<Increment>>", "<<Decrement>>"):
                sb_spd.bind(ev, lambda e, idx=i: self._sync_speed_acc(idx))

            sb_acc = ttk.Spinbox(lf2, textvariable=self.acc_vars[i],
                                 from_=0, to=254, increment=5, width=5)
            sb_acc.grid(row=r, column=3, padx=3)
            for ev in ("<FocusOut>", "<Return>", "<<Increment>>", "<<Decrement>>"):
                sb_acc.bind(ev, lambda e, idx=i: self._sync_speed_acc(idx))

            sb_min = ttk.Spinbox(lf2, textvariable=self.min_angle_vars[i],
                                 from_=-180, to=180, increment=5, width=6)
            sb_min.grid(row=r, column=4, padx=3)
            for ev in ("<FocusOut>", "<Return>", "<<Increment>>", "<<Decrement>>"):
                sb_min.bind(ev, lambda e, idx=i: self._sync_limits(idx))

            sb_max = ttk.Spinbox(lf2, textvariable=self.max_angle_vars[i],
                                 from_=-180, to=180, increment=5, width=6)
            sb_max.grid(row=r, column=5, padx=3)
            for ev in ("<FocusOut>", "<Return>", "<<Increment>>", "<<Decrement>>"):
                sb_max.bind(ev, lambda e, idx=i: self._sync_limits(idx))

            sb_home = ttk.Spinbox(lf2, textvariable=self.home_angle_vars[i],
                                  from_=-180, to=180, increment=5, width=6)
            sb_home.grid(row=r, column=6, padx=3)
            for ev in ("<FocusOut>", "<Return>", "<<Increment>>", "<<Decrement>>"):
                sb_home.bind(ev, lambda e, idx=i: self._sync_limits(idx))

            ttk.Button(lf2, text="⊙ Zero", width=7,
                       command=lambda idx=i: self._set_zero(idx)).grid(row=r, column=7, padx=2)
            ttk.Button(lf2, text="Ping", width=5,
                       command=lambda idx=i: self._ping_joint(idx)).grid(row=r, column=8, padx=2)
            ttk.Button(lf2, text="Clr Spd", width=7, style="Danger.TButton",
                       command=lambda idx=i: self._clear_eeprom_speed(idx)).grid(row=r, column=9, padx=2)

        apply_row = len(JOINT_NAMES) + 1
        ttk.Button(lf2, text="✔  Apply Speed & ACC to all joints",
                   command=self._apply_speed_acc).grid(
            row=apply_row, column=0, columnspan=10, pady=8, sticky="ew")

        lf3 = ttk.LabelFrame(f, text=" Servo Diagnostics ", padding=10)
        lf3.grid(row=1, column=0, columnspan=2, padx=12, pady=6, sticky="nsew")
        f.rowconfigure(1, weight=1)

        tk.Label(lf3, text="Select joint:",
                 bg=T["BG"], fg=T["SUBTEXT"]).pack(side="left")
        self.diag_joint_cb = ttk.Combobox(
            lf3, values=[f"{i}: {n}" for i, n in enumerate(JOINT_NAMES)],
            state="readonly", width=18)
        self.diag_joint_cb.current(0)
        self.diag_joint_cb.pack(side="left", padx=6)
        ttk.Button(lf3, text="📊 Read",
                   command=self._read_diagnostics).pack(side="left")

        self.diag_text = tk.Text(lf3, height=5, bg=T["ENTRY"], fg=T["GREEN"],
                                 font=("Courier", 9), relief="flat")
        self.diag_text.pack(fill="both", expand=True, pady=6, padx=4)

    # ── Sequence tab ─────────────────────────
    def _build_seq_tab(self):
        f = self.tab_seq
        f.columnconfigure(0, weight=1)
        f.columnconfigure(1, weight=3)
        f.rowconfigure(0, weight=1)
        T = self.T

        lf = ttk.LabelFrame(f, text=" Waypoints ", padding=6)
        lf.grid(row=0, column=0, padx=8, pady=8, sticky="nsew")

        self.wp_list = tk.Listbox(lf, bg=T["ENTRY"], fg=T["TEXT"],
                                  selectbackground=T["SEL_BG"],
                                  font=("Courier", 10), relief="flat",
                                  activestyle="none")
        self.wp_list.pack(fill="both", expand=True)
        self.wp_list.bind("<<ListboxSelect>>", self._on_wp_select)

        br = tk.Frame(lf, bg=T["BG"])
        br.pack(fill="x", pady=4)
        ttk.Button(br, text="↑", width=3, command=self._wp_up).pack(
            side="left", padx=2)
        ttk.Button(br, text="↓", width=3, command=self._wp_down).pack(
            side="left", padx=2)
        ttk.Button(br, text="✕", width=3, style="Danger.TButton",
                   command=self._wp_delete).pack(side="left", padx=2)

        lf2 = ttk.LabelFrame(f, text=" Waypoint Editor ", padding=10)
        lf2.grid(row=0, column=1, padx=8, pady=8, sticky="nsew")

        self.wp_name_var = tk.StringVar()
        self.wp_dur_var = tk.DoubleVar(value=1.0)

        tk.Label(lf2, text="Name:",         bg=T["BG"], fg=T["SUBTEXT"]).grid(
            row=0, column=0, sticky="w")
        ttk.Entry(lf2, textvariable=self.wp_name_var, width=18).grid(
            row=0, column=1, padx=6, pady=3)
        tk.Label(lf2, text="Duration (s):", bg=T["BG"], fg=T["SUBTEXT"]).grid(
            row=1, column=0, sticky="w")
        ttk.Entry(lf2, textvariable=self.wp_dur_var,   width=10).grid(
            row=1, column=1, padx=6, pady=3)

        self.wp_angle_vars: list = [tk.DoubleVar() for _ in range(6)]
        for i, name in enumerate(JOINT_NAMES):
            r = i + 2
            tk.Label(lf2, text=f"{name}:", bg=T["BG"], fg=T["SUBTEXT"],
                     width=12, anchor="w").grid(row=r, column=0, sticky="w")
            ttk.Entry(lf2, textvariable=self.wp_angle_vars[i], width=10).grid(
                row=r, column=1, padx=6, pady=2)

        br2 = tk.Frame(lf2, bg=T["BG"])
        br2.grid(row=9, column=0, columnspan=2, pady=8)
        ttk.Button(br2, text="💾 Update",  command=self._wp_update).pack(
            side="left", padx=4)
        ttk.Button(br2, text="➕ Add New", style="Green.TButton",
                   command=self._wp_add_new).pack(side="left", padx=4)
        ttk.Button(br2, text="▶ Go To",   command=self._wp_goto).pack(
            side="left", padx=4)

        pb = ttk.LabelFrame(f, text=" Playback ", padding=10)
        pb.grid(row=1, column=0, columnspan=2, padx=8, pady=4, sticky="ew")

        self.loop_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(pb, text="Loop", variable=self.loop_var).pack(
            side="left", padx=6)

        self.play_btn = ttk.Button(pb, text="▶  Play Sequence",
                                   style="Green.TButton", command=self._play_sequence)
        self.play_btn.pack(side="left", padx=6)
        ttk.Button(pb, text="⏹  Stop", style="Danger.TButton",
                   command=self._stop_sequence).pack(side="left", padx=6)

        self.seq_progress = tk.Label(pb, text="", bg=T["BG"], fg=T["SUBTEXT"],
                                     font=("Helvetica", 9))
        self.seq_progress.pack(side="left", padx=10)

        # Sequence file buttons on the right side of playback bar
        ttk.Separator(pb, orient="vertical").pack(
            side="left", fill="y", padx=8)
        ttk.Button(pb, text="💾 Save Sequence",
                   command=self._save_sequence).pack(side="left", padx=4)
        ttk.Button(pb, text="📂 Load Sequence",
                   command=self._load_sequence).pack(side="left", padx=4)

    # ─────────────────────────────────────────
    #  Callbacks
    # ─────────────────────────────────────────
    def _emergency_stop(self):
        self.ctrl.stop_sequence()
        if self.ctrl.driver.connected:
            self.ctrl.torque_all(False)
        self.play_btn.configure(state="normal")
        self.seq_progress.configure(text="E-STOP")
        self._log("!!! EMERGENCY STOP — torque disabled on all joints !!!")

    def _log(self, msg: str):
        self.log_var.set(msg)
        print(f"[GUI] {msg}")

    def _on_position_update(self, idx: int, deg: float):
        self.pos_vars[idx].set(f"{deg:+.1f}°")

    # ── Connection ───────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports:
            self.port_cb.current(0)

    def _toggle_connect(self):
        if self.ctrl.driver.connected:
            self.ctrl.disconnect()
            self.conn_btn.configure(text="Connect", style="Green.TButton")
            self.status_label.configure(
                text="● DISCONNECTED", fg=self.T["ORANGE"])
            self._log("Disconnected.")
        else:
            port = self.port_cb.get()
            baud = int(self.baud_cb.get())
            if not port:
                messagebox.showerror("Error", "Select a port first.")
                return
            if self.ctrl.connect(port, baud):
                self.conn_btn.configure(
                    text="Disconnect", style="Danger.TButton")
                self.status_label.configure(
                    text=f"● CONNECTED  {port}", fg=self.T["GREEN"])
                self._log(f"Connected to {port} @ {baud}")
            else:
                messagebox.showerror("Connection failed",
                                     f"Could not open {port}")

    def _scan_servos(self):
        if not self.ctrl.driver.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return
        self.scan_result.delete("1.0", "end")
        self.scan_result.insert("end", "Scanning IDs 1–20 ...\n")
        self.root.update()

        def scan():
            found = []
            for sid in range(1, 21):
                if self.ctrl.ping_servo(sid):
                    info = self.ctrl.get_servo_info(sid)
                    found.append(f"ID {sid:2d}  pos={info['position']}  "
                                 f"temp={info['temperature']}°C  "
                                 f"volt={info['voltage']}×0.1V")
            result = "\n".join(found) if found else "No servos found."
            self.root.after(
                0, lambda: self.scan_result.insert("end", result + "\n"))

        threading.Thread(target=scan, daemon=True).start()

    # ── Control ──────────────────────────────
    def _send_joint(self, idx: int):
        if not self.ctrl.driver.connected:
            self._log("Not connected")
            return
        jcfg = self.ctrl.config.joints[idx]
        deg = max(jcfg.min_angle, min(
            jcfg.max_angle, self.target_vars[idx].get()))
        self.target_vars[idx].set(round(deg, 2))
        self.ctrl.set_joint_angle(idx, deg)
        self._log(f"Sent {JOINT_NAMES[idx]} → {deg:.2f}°")

    def _nudge_joint(self, idx: int, delta: float):
        """Increment or decrement the target by 1° and send immediately."""
        jcfg = self.ctrl.config.joints[idx]
        current = self.target_vars[idx].get()
        new_val = round(max(jcfg.min_angle, min(
            jcfg.max_angle, current + delta)), 2)
        self.target_vars[idx].set(new_val)
        if self.ctrl.driver.connected:
            self.ctrl.set_joint_angle(idx, new_val)

    def _send_all(self):
        if not self.ctrl.driver.connected:
            self._log("Not connected")
            return
        angles = [v.get() for v in self.target_vars]
        self.ctrl.set_all_angles(angles)
        self._log(f"Sent all: {[round(a, 1) for a in angles]}")

    def _go_home(self):
        if not self.ctrl.driver.connected:
            self._log("Not connected")
            return
        self.ctrl.go_home()
        for i, v in enumerate(self.target_vars):
            v.set(self.ctrl.config.joints[i].home_angle)
        self._log("Home: all joints → home angles")

    def _toggle_teach(self):
        if not self._teach_mode:
            self.ctrl.enter_teach_mode()
            self._teach_mode = True
            self.teach_btn.configure(
                text="✋  Exit Teach Mode", style="Danger.TButton")
            self.teach_status.configure(
                text="Teach mode ON – torque disabled. Move arm freely.",
                fg=self.T["ORANGE"])
            self._log("Teach mode entered.")
        else:
            self.ctrl.exit_teach_mode()
            self._teach_mode = False
            self.teach_btn.configure(
                text="🖐  Enter Teach Mode", style="TButton")
            self.teach_status.configure(text="")
            self._log("Teach mode exited.")

    def _capture_pose(self):
        name = self.pos_name_var.get().strip(
        ) or f"pose_{len(self.ctrl.sequence)+1}"
        wp = self.ctrl.capture_position(name)
        self._refresh_wp_list()
        parts = name.rsplit("_", 1)
        if len(parts) == 2 and parts[1].isdigit():
            self.pos_name_var.set(f"{parts[0]}_{int(parts[1])+1}")
        self._log(f"Captured '{name}': {[round(a, 1) for a in wp.angles]}")

    # ── Setup ────────────────────────────────
    def _sync_id(self, idx: int):
        try:
            self.ctrl.config.joints[idx].servo_id = int(
                self.id_vars[idx].get())
        except Exception:
            pass

    def _sync_limits(self, idx: int):
        try:
            self.ctrl.config.joints[idx].min_angle = float(
                self.min_angle_vars[idx].get())
        except Exception:
            pass
        try:
            self.ctrl.config.joints[idx].max_angle = float(
                self.max_angle_vars[idx].get())
        except Exception:
            pass
        try:
            self.ctrl.config.joints[idx].home_angle = float(
                self.home_angle_vars[idx].get())
        except Exception:
            pass
        # keep slider range in sync
        jcfg = self.ctrl.config.joints[idx]
        self.sliders[idx].configure(from_=jcfg.min_angle, to=jcfg.max_angle)

    def _clear_eeprom_speed(self, idx: int):
        if not self.ctrl.driver.connected:
            self._log("Not connected")
            return
        ok = self.ctrl.clear_speed_limit(idx)
        self._log(
            f"{'EEPROM speed limit cleared for' if ok else 'Failed to clear'} {JOINT_NAMES[idx]}")

    def _sync_speed_acc(self, idx: int):
        try:
            self.ctrl.config.joints[idx].speed = int(
                self.speed_vars[idx].get())
        except Exception:
            pass
        try:
            self.ctrl.config.joints[idx].acc = int(self.acc_vars[idx].get())
        except Exception:
            pass

    def _apply_speed_acc(self):
        """Bulk-apply all Speed & ACC spinbox values to the joint config."""
        for i in range(6):
            self._sync_speed_acc(i)
        self._log("Speed & ACC applied to all joints.")

    def _change_id(self):
        if not self.ctrl.driver.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return
        cur = self.cur_id_var.get()
        new = self.new_id_var.get()
        if cur == new:
            messagebox.showwarning(
                "Same ID", "Current and new ID are the same.")
            return
        if not (1 <= new <= 253):
            messagebox.showerror(
                "Invalid ID", "New ID must be between 1 and 253.")
            return

        ok = self.ctrl.set_servo_id(cur, new)
        if ok:
            # The servo still responds on 'cur' until power-cycled.
            # We do NOT update the joint config yet — that happens after reboot.
            msg = (f"ID {cur} -> {new} written to EEPROM.\n\n"
                   f"IMPORTANT: Power-cycle (unplug/replug) the servo now.\n"
                   f"The servo still responds on ID {cur} until rebooted.\n\n"
                   f"After rebooting, update the Servo ID field in the Setup\n"
                   f"tab for the matching joint to {new}.")
        else:
            msg = f"Failed to change ID {cur} -> {new}"
        self._log(msg.splitlines()[0])
        messagebox.showinfo("ID Change — Action Required", msg)

    def _set_zero(self, idx: int):
        if not self.ctrl.driver.connected:
            self._log("Not connected")
            return
        ok = self.ctrl.set_zero_here(idx)
        self._log(
            f"{'Zero set for' if ok else 'Failed to zero'} {JOINT_NAMES[idx]}")

    def _ping_joint(self, idx: int):
        if not self.ctrl.driver.connected:
            self._log("Not connected")
            return
        sid = self.ctrl.config.joints[idx].servo_id
        ok = self.ctrl.ping_servo(sid)
        self._log(f"Ping ID {sid}: {'OK ✓' if ok else 'NO RESPONSE ✗'}")

    def _read_diagnostics(self):
        if not self.ctrl.driver.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return
        idx = self.diag_joint_cb.current()
        jcfg = self.ctrl.config.joints[idx]
        info = self.ctrl.get_servo_info(jcfg.servo_id)
        live = self.ctrl.live_positions[idx]
        raw = self.ctrl.live_raw[idx]
        text = (f"Joint       : {JOINT_NAMES[idx]}\n"
                f"Servo ID    : {jcfg.servo_id}\n"
                f"Raw position: {raw}  →  {live if live is not None else '—'}°\n"
                f"Temperature : {info['temperature']} °C\n"
                f"Voltage     : {info['voltage']} (×0.1 V)\n"
                f"Zero offset : {jcfg.zero_offset}")
        self.diag_text.delete("1.0", "end")
        self.diag_text.insert("end", text)

    # ── Sequence ─────────────────────────────
    def _refresh_wp_list(self):
        self.wp_list.delete(0, "end")
        for i, wp in enumerate(self.ctrl.sequence):
            self.wp_list.insert("end", f"{i:02d}  {wp.name}  ({wp.duration}s)")

    def _on_wp_select(self, _=None):
        sel = self.wp_list.curselection()
        if not sel:
            return
        wp = self.ctrl.sequence[sel[0]]
        self.wp_name_var.set(wp.name)
        self.wp_dur_var.set(wp.duration)
        for i, a in enumerate(wp.angles):
            self.wp_angle_vars[i].set(round(a, 2))

    def _wp_update(self):
        sel = self.wp_list.curselection()
        if not sel:
            return
        wp = self.ctrl.sequence[sel[0]]
        wp.name = self.wp_name_var.get()
        wp.duration = self.wp_dur_var.get()
        wp.angles = [v.get() for v in self.wp_angle_vars]
        # Recompute raw positions from edited angles
        wp.raw_pos = self.ctrl.angles_to_raw(wp.angles)
        self._refresh_wp_list()
        self.wp_list.selection_set(sel[0])
        self._log(f"Updated '{wp.name}'  raw={wp.raw_pos}")

    def _wp_add_new(self):
        angles = [v.get() for v in self.wp_angle_vars]
        raw_pos = self.ctrl.angles_to_raw(angles)
        wp = Waypoint(
            name=self.wp_name_var.get() or f"pose_{len(self.ctrl.sequence)+1}",
            angles=angles,
            raw_pos=raw_pos,
            duration=self.wp_dur_var.get()
        )
        self.ctrl.sequence.append(wp)
        self._refresh_wp_list()
        self.wp_list.selection_set(len(self.ctrl.sequence) - 1)

    def _wp_delete(self):
        sel = self.wp_list.curselection()
        if not sel:
            return
        del self.ctrl.sequence[sel[0]]
        self._refresh_wp_list()

    def _wp_up(self):
        sel = self.wp_list.curselection()
        if not sel or sel[0] == 0:
            return
        i = sel[0]
        self.ctrl.sequence[i], self.ctrl.sequence[i-1] = \
            self.ctrl.sequence[i-1], self.ctrl.sequence[i]
        self._refresh_wp_list()
        self.wp_list.selection_set(i - 1)

    def _wp_down(self):
        sel = self.wp_list.curselection()
        if not sel or sel[0] >= len(self.ctrl.sequence) - 1:
            return
        i = sel[0]
        self.ctrl.sequence[i], self.ctrl.sequence[i+1] = \
            self.ctrl.sequence[i+1], self.ctrl.sequence[i]
        self._refresh_wp_list()
        self.wp_list.selection_set(i + 1)

    def _wp_goto(self):
        if not self.ctrl.driver.connected:
            self._log("Not connected")
            return
        sel = self.wp_list.curselection()
        if sel:
            wp = self.ctrl.sequence[sel[0]]
            self.ctrl.set_all_raw(wp.raw_pos)
            self._log(f"Moved to '{wp.name}'")
        else:
            angles = [v.get() for v in self.wp_angle_vars]
            self.ctrl.set_all_angles(angles)

    def _play_sequence(self):
        if not self.ctrl.driver.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return
        if not self.ctrl.sequence:
            messagebox.showwarning("Empty", "No waypoints in sequence.")
            return

        self.play_btn.configure(state="disabled")

        def on_progress(idx, wp):
            self.root.after(0, lambda: self.seq_progress.configure(
                text=f"▶  {idx+1}/{len(self.ctrl.sequence)}  {wp.name}"))

        def on_done():
            self.root.after(0, lambda: [
                self.play_btn.configure(state="normal"),
                self.seq_progress.configure(text="Done ✓")])

        self.ctrl.play_sequence(
            loop=self.loop_var.get(),
            progress_cb=on_progress,
            done_cb=on_done)
        self._log("Sequence started.")

    def _stop_sequence(self):
        self.ctrl.stop_sequence()
        self.play_btn.configure(state="normal")
        self.seq_progress.configure(text="Stopped.")
        self._log("Sequence stopped.")

    # ── Persistence ──────────────────────────
    def _save_config(self):
        path = filedialog.asksaveasfilename(defaultextension=".json",
                                            filetypes=[("JSON", "*.json")])
        if path:
            # Flush all spinbox values to config before writing
            for i in range(6):
                self._sync_id(i)
                self._sync_speed_acc(i)
                self._sync_limits(i)
            self.ctrl.save_config(path)
            self._log(f"Config saved: {path}")

    def _load_config(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if path:
            self.ctrl.load_config(path)
            for i, jcfg in enumerate(self.ctrl.config.joints):
                self.id_vars[i].set(jcfg.servo_id)
                self.speed_vars[i].set(jcfg.speed)
                self.acc_vars[i].set(jcfg.acc)
                self.min_angle_vars[i].set(jcfg.min_angle)
                self.max_angle_vars[i].set(jcfg.max_angle)
                self.home_angle_vars[i].set(jcfg.home_angle)
            self._log(f"Config loaded: {path}")

    def _save_sequence(self):
        path = filedialog.asksaveasfilename(defaultextension=".json",
                                            filetypes=[("JSON", "*.json")])
        if path:
            self.ctrl.save_sequence(path)
            self._log(
                f"Sequence saved ({len(self.ctrl.sequence)} waypoints): {path}")

    def _load_sequence(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if path:
            self.ctrl.load_sequence(path)
            self._refresh_wp_list()
            self._log(f"Sequence loaded: {len(self.ctrl.sequence)} waypoints")


# ─────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────
def main():
    root = tk.Tk()
    app = RobotGUI(root)

    def on_close():
        app.ctrl.stop_sequence()
        app.ctrl.disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
