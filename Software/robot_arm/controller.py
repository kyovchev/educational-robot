"""High-level robot controller: kinematics, sequencing, persistence."""

import json
import threading
import time
from dataclasses import asdict
from typing import Callable, Optional

from .constants import POS_CENTER, POS_MIN, POS_MAX, NUM_JOINTS
from .driver import STS3215Driver
from .kinematics import Kinematics
from .models import JointConfig, RobotConfig, Waypoint


class RobotController:
    def __init__(self, urdf_path: str = "robot.urdf"):
        self.driver   = STS3215Driver()
        self.config   = RobotConfig()
        self.sequence: list[Waypoint] = []
        self.kinematics = Kinematics(urdf_path)

        self.live_positions: list[Optional[float]] = [None] * NUM_JOINTS
        self.live_raw:       list[Optional[int]]   = [None] * NUM_JOINTS

        self._poll_running = False
        self._seq_running  = False
        self._poll_thread: Optional[threading.Thread] = None
        self._seq_thread:  Optional[threading.Thread] = None

        # callbacks
        self.on_position_update: Optional[Callable[[int, float], None]] = None

    # ── connection ──────────────────────────────────────────────────────────
    def connect(self, port: str, baud: int = 1_000_000) -> bool:
        self.config.port = port
        self.config.baud = baud
        ok = self.driver.connect_serial(port, baud)
        if ok:
            self._start_polling()
        return ok

    def connect_tcp(self, host: str, port: int = 8888) -> bool:
        self.config.tcp_host = host
        self.config.tcp_port = port
        ok = self.driver.connect_tcp(host, port)
        if ok:
            self._start_polling()
        return ok

    def disconnect(self):
        self._stop_polling()
        self.driver.disconnect()

    # ── polling ─────────────────────────────────────────────────────────────
    def _start_polling(self):
        self._poll_running = True
        self._poll_thread  = threading.Thread(target=self._poll_loop, daemon=True)
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
            time.sleep(0.05)

    # ── angle ↔ raw conversion ──────────────────────────────────────────────
    def _raw_to_logical(self, joint_idx: int, raw: int) -> float:
        offset = raw - self.config.joints[joint_idx].zero_offset
        deg    = STS3215Driver.pos_to_deg(offset + POS_CENTER) - 180.0
        return round(-deg, 2)

    def _logical_to_raw(self, joint_idx: int, deg: float) -> int:
        jcfg = self.config.joints[joint_idx]
        deg  = max(jcfg.min_angle, min(jcfg.max_angle, deg))
        pos  = STS3215Driver.deg_to_pos(-deg + 180.0)
        raw  = pos - POS_CENTER + jcfg.zero_offset
        return max(POS_MIN, min(POS_MAX, raw))

    def angles_to_raw(self, angles: list) -> list:
        return [self._logical_to_raw(i, deg) for i, deg in enumerate(angles)]

    # ── joint control ────────────────────────────────────────────────────────
    def set_joint_angle(self, joint_idx: int, deg: float):
        jcfg = self.config.joints[joint_idx]
        raw  = self._logical_to_raw(joint_idx, deg)
        self.driver.set_goal_position(jcfg.servo_id, raw, jcfg.speed, jcfg.acc)

    def set_all_angles(self, angles: list) -> None:
        id_pos_speed = [
            (self.config.joints[i].servo_id,
             self._logical_to_raw(i, deg),
             self.config.joints[i].speed,
             self.config.joints[i].acc)
            for i, deg in enumerate(angles)
        ]
        self.driver.sync_write_positions(id_pos_speed)

    def go_home(self) -> None:
        angles = [jcfg.home_angle for jcfg in self.config.joints]
        self.set_all_angles(angles)

    def torque_all(self, enable: bool):
        for jcfg in self.config.joints:
            self.driver.set_torque(jcfg.servo_id, enable)

    # ── FK helper (uses live positions, substitutes 0 for unread joints) ────
    def compute_fk(self):
        """Return FK pose using current live positions (None → 0.0)."""
        angles = [p if p is not None else 0.0 for p in self.live_positions]
        return self.kinematics.fk(angles)

    # ── IK helper ────────────────────────────────────────────────────────────
    def compute_ik(self, x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg):
        """
        Compute IK for a Cartesian target.
        Returns list of NUM_ARM_JOINTS angles in degrees, or None.
        """
        pose = self.kinematics.pose_from_components(
            x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg)
        seed = [p if p is not None else 0.0 for p in self.live_positions]
        return self.kinematics.ik(pose, seed_angles_deg=seed)

    # ── teach mode ───────────────────────────────────────────────────────────
    def enter_teach_mode(self):
        self.torque_all(False)

    def exit_teach_mode(self):
        self.torque_all(True)

    def capture_position(self, name: str) -> Waypoint:
        angles = [p if p is not None else 0.0 for p in self.live_positions]
        wp = Waypoint(name=name, angles=angles)
        self.sequence.append(wp)
        return wp

    # ── sequence playback ────────────────────────────────────────────────────
    def play_sequence(self, loop: bool = False,
                      progress_cb: Optional[Callable] = None,
                      done_cb: Optional[Callable] = None):
        self._seq_running = True

        def _run():
            repeat = True
            while repeat and self._seq_running:
                for idx, wp in enumerate(self.sequence):
                    if not self._seq_running:
                        break
                    if progress_cb:
                        progress_cb(idx, wp)
                    self.set_all_angles(wp.angles)
                    time.sleep(wp.duration)
                repeat = loop and self._seq_running
            self._seq_running = False
            if done_cb:
                done_cb()

        self._seq_thread = threading.Thread(target=_run, daemon=True)
        self._seq_thread.start()

    def stop_sequence(self):
        self._seq_running = False

    # ── setup helpers ────────────────────────────────────────────────────────
    def set_servo_id(self, current_id: int, new_id: int) -> bool:
        return self.driver.set_id(current_id, new_id)

    def set_zero_here(self, joint_idx: int) -> bool:
        jcfg = self.config.joints[joint_idx]
        ok = self.driver.calibrate_zero(jcfg.servo_id)
        if ok:
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

    # ── persistence ──────────────────────────────────────────────────────────
    def save_config(self, path: str):
        data = {
            "port":     self.config.port,
            "baud":     self.config.baud,
            "tcp_host": self.config.tcp_host,
            "tcp_port": self.config.tcp_port,
            "joints":   [asdict(j) for j in self.config.joints],
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

    def load_config(self, path: str):
        with open(path) as f:
            data = json.load(f)
        self.config.port     = data.get("port", "")
        self.config.baud     = data.get("baud", 1_000_000)
        self.config.tcp_host = data.get("tcp_host", "192.168.1.100")
        self.config.tcp_port = data.get("tcp_port", 8888)
        for i, jd in enumerate(data.get("joints", [])[:NUM_JOINTS]):
            jd.setdefault("acc",        10)
            jd.setdefault("home_angle", 0.0)
            jd.setdefault("min_angle",  -90.0)
            jd.setdefault("max_angle",   90.0)
            self.config.joints[i] = JointConfig(**jd)

    def save_sequence(self, path: str):
        """Save sequence as angles-only JSON (human-editable)."""
        data = [
            {"name": w.name, "angles": [round(a, 4) for a in w.angles],
             "duration": w.duration}
            for w in self.sequence
        ]
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

    def load_sequence(self, path: str):
        """Load sequence from angles-only JSON."""
        with open(path) as f:
            data = json.load(f)
        self.sequence = []
        for d in data:
            # back-compat: silently ignore legacy raw_pos field
            d.pop("raw_pos", None)
            self.sequence.append(Waypoint(**d))
