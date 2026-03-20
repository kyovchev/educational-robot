"""
STS Robot API  —  XArmAPI-compatible interface for a 6-DOF robot
                  built with Feetech STS3215 servos.

Drop-in replacement for XArmAPI / XArmEmulator.
Kinematics are handled by roboticstoolbox + a URDF file.

Usage (TCP / Wi-Fi ESP32 bridge)
---------------------------------
    from sts_robot_api import StsRobotAPI as XArmAPI

    robot = StsRobotAPI("192.168.1.100")          # TCP, default port 8888
    robot.motion_enable(enable=True)
    robot.set_mode(0)
    robot.set_state(state=0)
    code, pos = robot.get_position()              # [x, y, z, roll, pitch, yaw]
    robot.set_position(x=250, y=0, z=300, roll=180, pitch=0, yaw=0, wait=True)

Usage (USB serial)
------------------
    robot = StsRobotAPI(port="COM3")              # or "/dev/ttyUSB0"
    robot.motion_enable(enable=True)
    ...

Usage (offline emulator)
------------------------
    from sts_robot_api import StsRobotEmulator as XArmAPI
    robot = StsRobotEmulator()
    ...

Config file (JSON)
------------------
    robot = StsRobotAPI("192.168.1.100", config="robot_config.json")

Return codes
------------
    CODE_SUCCESS =  0
    CODE_WARN    =  1
    CODE_ERROR   = -1

All methods that return a status code follow the XArmAPI convention:
    (code, value)  for getters
    code           for setters / commands
"""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import asdict
from typing import Callable, List, Optional, Tuple

# ── internal modules ─────────────────────────────────────────────────────────
from robot_arm.constants import (POS_CENTER, POS_MIN, POS_MAX,
                                 NUM_JOINTS, NUM_ARM_JOINTS)
from robot_arm.driver import STS3215Driver
from robot_arm.kinematics import Kinematics
from robot_arm.models import JointConfig, RobotConfig, Waypoint

# ── return codes (mirrors XArmAPI) ───────────────────────────────────────────
CODE_SUCCESS = 0
CODE_WARN = 1
CODE_ERROR = -1

# ── robot states ─────────────────────────────────────────────────────────────
STATE_READY = 0
STATE_MOVING = 1
STATE_PAUSED = 3
STATE_ERROR = 4

# ── default gripper open/close range (degrees) ───────────────────────────────
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 90.0


# ─────────────────────────────────────────────────────────────────────────────
#  StsRobotAPI
# ─────────────────────────────────────────────────────────────────────────────
class StsRobotAPI:
    """
    XArmAPI-compatible controller for a 6-DOF STS3215 servo robot.

    Parameters
    ----------
    ip : str | None
        ESP32 IP address for TCP/Wi-Fi connection.
        Pass None (or omit) when using a serial port.
    port : str | None
        Serial port string, e.g. "COM3" or "/dev/ttyUSB0".
        Ignored when `ip` is provided.
    tcp_port : int
        TCP port on the ESP32 bridge (default 8888).
    baud : int
        Serial baud rate (default 1 000 000).
    urdf_path : str
        Path to the URDF file for FK/IK.
    config : str | None
        Path to a JSON config file (joint IDs, limits, offsets …).
    is_radian : bool
        If True, angle arguments/returns use radians instead of degrees.
        Mirrors XArmAPI convention (default False).
    """

    CODE_SUCCESS = CODE_SUCCESS
    CODE_WARN = CODE_WARN
    CODE_ERROR = CODE_ERROR

    STATE_READY = STATE_READY
    STATE_MOVING = STATE_MOVING
    STATE_PAUSED = STATE_PAUSED
    STATE_ERROR = STATE_ERROR

    def __init__(self,
                 ip:        Optional[str] = None,
                 port:      Optional[str] = None,
                 tcp_port:  int = 8888,
                 baud:      int = 1_000_000,
                 urdf_path: str = "robot.urdf",
                 config:    Optional[str] = None,
                 is_radian: bool = False):

        self.ip = ip
        self._serial_port = port
        self._tcp_port = tcp_port
        self._baud = baud
        self.is_radian = is_radian

        self._driver = STS3215Driver()
        self._kinematics = Kinematics(urdf_path)
        self._cfg = RobotConfig()

        if config:
            self._load_config(config)

        # live state
        self._live_positions: List[Optional[float]] = [None] * NUM_JOINTS
        self._live_raw:       List[Optional[int]] = [None] * NUM_JOINTS
        self._target_angles:  List[float] = [0.0] * NUM_JOINTS

        self._state = STATE_READY
        self._error_code = 0
        self._motion_enabled = False
        self._mode = 0
        self._connected = False

        self._poll_running = False
        self._poll_thread:  Optional[threading.Thread] = None
        self._move_lock = threading.Lock()

        # user callbacks
        self.on_position_update: Optional[Callable[[int, float], None]] = None

        # auto-connect
        self._auto_connect()

    # ── auto connect ─────────────────────────────────────────────────────────
    def _auto_connect(self):
        if self.ip:
            ok = self._driver.connect_tcp(self.ip, self._tcp_port)
        elif self._serial_port:
            ok = self._driver.connect_serial(self._serial_port, self._baud)
        else:
            print("[StsRobotAPI] No ip or port given — not connected.")
            return
        if ok:
            self._connected = True
            self._start_polling()
            print(f"[StsRobotAPI] Connected to "
                  f"{'TCP ' + self.ip + ':' + str(self._tcp_port) if self.ip else self._serial_port}")
        else:
            print("[StsRobotAPI] Connection failed.")

    # ── XArmAPI: connect / disconnect ────────────────────────────────────────
    def connect(self) -> int:
        if self._connected:
            return CODE_SUCCESS
        self._auto_connect()
        return CODE_SUCCESS if self._connected else CODE_ERROR

    def disconnect(self) -> int:
        self._stop_polling()
        self._driver.disconnect()
        self._connected = False
        print("[StsRobotAPI] Disconnected.")
        return CODE_SUCCESS

    # ── XArmAPI: enable / mode / state ───────────────────────────────────────
    def motion_enable(self, enable: bool = True,
                      servo_id: Optional[int] = None) -> int:
        self._motion_enabled = enable
        if self._connected:
            for i, jcfg in enumerate(self._cfg.joints):
                if servo_id is None or jcfg.servo_id == servo_id:
                    self._driver.set_torque(jcfg.servo_id, enable)
        status = "enabled" if enable else "disabled"
        print(f"[StsRobotAPI] Motion {status}.")
        return CODE_SUCCESS

    def set_mode(self, mode: int) -> int:
        """
        0 = position mode  (normal operation)
        1 = servo/joint mode
        2 = teach / zero-torque mode
        """
        self._mode = mode
        if mode == 2:
            self.motion_enable(False)
        names = {0: "Position", 1: "Servo", 2: "Teach"}
        print(f"[StsRobotAPI] Mode: {names.get(mode, str(mode))}")
        return CODE_SUCCESS

    def set_state(self, state: int) -> int:
        """0=ready, 3=pause, 4=stop."""
        self._state = state
        if state == 4:                          # stop
            self._motion_enabled = False
        names = {0: "Ready", 3: "Pause", 4: "Stop"}
        print(f"[StsRobotAPI] State: {names.get(state, str(state))}")
        return CODE_SUCCESS

    def get_state(self) -> Tuple[int, int]:
        return (CODE_SUCCESS, self._state)

    # ── XArmAPI: position getters ─────────────────────────────────────────────
    def get_position(self,
                     is_radian: Optional[bool] = None) -> Tuple[int, List[float]]:
        """
        Returns (code, [x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg]).
        Computed from FK using current joint angles.
        """
        angles = self._get_arm_angles_deg()
        if self._kinematics.available:
            pose = self._kinematics.fk(angles)
            c = self._kinematics.pose_components(pose)
            pos = [c["x"], c["y"], c["z"], c["roll"], c["pitch"], c["yaw"]]
        else:
            pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        if self._use_radian(is_radian):
            pos = pos[:3] + [math.radians(a) for a in pos[3:]]
        return (CODE_SUCCESS, pos)

    def get_servo_angle(self,
                        servo_id:  Optional[int] = None,
                        is_radian: Optional[bool] = None
                        ) -> Tuple[int, List[float]]:
        """
        Returns (code, [j1…j6]) — all 6 arm joint angles in degrees (or radians).
        `servo_id` is ignored (returns all joints, same as XArmAPI 7-DOF default).
        """
        angles = self._get_arm_angles_deg()
        if self._use_radian(is_radian):
            angles = [math.radians(a) for a in angles]
        return (CODE_SUCCESS, angles)

    def get_gripper_position(self) -> Tuple[int, float]:
        """Returns (code, gripper_angle_deg).  0=open, 90=closed (default range)."""
        idx = NUM_ARM_JOINTS           # Gripper is joint index 6
        val = self._live_positions[idx]
        return (CODE_SUCCESS, val if val is not None else 0.0)

    # ── XArmAPI: position setters ─────────────────────────────────────────────
    def set_position(self,
                     x:       Optional[float] = None,
                     y:       Optional[float] = None,
                     z:       Optional[float] = None,
                     roll:    Optional[float] = None,
                     pitch:   Optional[float] = None,
                     yaw:     Optional[float] = None,
                     speed:   Optional[float] = None,
                     mvacc:   Optional[float] = None,
                     mvtime:  Optional[float] = None,
                     relative: bool = False,
                     wait:    bool = False,
                     timeout: Optional[float] = None,
                     **kwargs) -> int:
        """
        Cartesian move via IK.

        x, y, z  : mm
        roll, pitch, yaw : degrees  (or radians if is_radian=True at construction)
        speed    : mm/s  → mapped to servo speed steps
        relative : if True, values are relative to current position
        wait     : block until servos reach target (polls state)
        """
        if not self._motion_enabled:
            print("[StsRobotAPI] Motion not enabled.")
            return CODE_ERROR
        if not self._connected:
            return CODE_ERROR

        # get current Cartesian pose
        _, cur = self.get_position()

        # build absolute target
        def _resolve(cur_val, new_val, is_rel):
            if new_val is None:
                return cur_val
            v = new_val
            if self.is_radian and new_val is not None:
                v = math.degrees(new_val)
            return (cur_val + v) if is_rel else v

        tx = _resolve(cur[0], x,     relative)
        ty = _resolve(cur[1], y,     relative)
        tz = _resolve(cur[2], z,     relative)
        tr = _resolve(cur[3], roll,  relative)
        tp = _resolve(cur[4], pitch, relative)
        tyw = _resolve(cur[5], yaw,   relative)

        # optional speed override
        if speed is not None:
            self._apply_speed(speed)

        # IK
        if not self._kinematics.available:
            print("[StsRobotAPI] Kinematics not available — cannot set_position.")
            return CODE_ERROR

        seed = self._get_arm_angles_deg()
        pose = self._kinematics.pose_from_components(tx, ty, tz, tr, tp, tyw)
        arm_angles = self._kinematics.ik(pose, seed_angles_deg=seed)
        if arm_angles is None:
            print(f"[StsRobotAPI] IK failed for target "
                  f"[{tx:.1f}, {ty:.1f}, {tz:.1f}, {tr:.1f}, {tp:.1f}, {tyw:.1f}]")
            return CODE_ERROR

        # keep current gripper
        gripper_angle = self._live_positions[NUM_ARM_JOINTS] or 0.0
        full_angles = list(arm_angles[:NUM_ARM_JOINTS]) + [gripper_angle]

        self._send_all_angles(full_angles)
        print(f"[StsRobotAPI] set_position → "
              f"xyz=[{tx:.1f},{ty:.1f},{tz:.1f}] "
              f"rpy=[{tr:.1f},{tp:.1f},{tyw:.1f}]")

        if wait:
            # wait only for the arm joints (0..NUM_ARM_JOINTS-1); gripper was not commanded
            self._wait_for_motion(timeout or 10.0,
                                  joint_indices=list(range(NUM_ARM_JOINTS)))
        return CODE_SUCCESS

    def set_servo_angle(self,
                        servo_id:  Optional[int] = None,
                        angle:     Optional[float] = None,
                        speed:     Optional[float] = None,
                        wait:      bool = False,
                        angles:    Optional[List[float]] = None,
                        is_radian: Optional[bool] = None,
                        **kwargs) -> int:
        """
        Set one or all arm joint angles.

        servo_id : 1-based joint index (1…6).  None = set all joints.
        angle    : target angle in degrees (or radians).
        angles   : list of 6 angles — used when servo_id is None.
        """
        if not self._motion_enabled:
            print("[StsRobotAPI] Motion not enabled.")
            return CODE_ERROR
        if not self._connected:
            return CODE_ERROR

        use_rad = self._use_radian(is_radian)

        if speed is not None:
            self._apply_speed(speed)

        if servo_id is not None:
            idx = int(servo_id) - 1      # 1-based → 0-based
            if not (0 <= idx < NUM_ARM_JOINTS):
                return CODE_ERROR
            deg = math.degrees(angle) if use_rad else angle
            self._set_single_angle(idx, deg)
            print(f"[StsRobotAPI] Joint {servo_id} → {deg:.2f}°")
            if wait:
                self._wait_for_motion(kwargs.get("timeout", 10.0),
                                      joint_indices=[idx])
        else:
            src = angle if angle is not None else []
            if len(src) < NUM_ARM_JOINTS:
                print("[StsRobotAPI] Need 6 angles.")
                return CODE_ERROR
            degs = [math.degrees(
                a) if use_rad else a for a in src[:NUM_ARM_JOINTS]]
            gripper = self._live_positions[NUM_ARM_JOINTS] or 0.0
            self._send_all_angles(degs + [gripper])
            print(f"[StsRobotAPI] All joints → {[round(d, 2) for d in degs]}")
            if wait:
                # wait only for arm joints — gripper was not commanded to move
                self._wait_for_motion(kwargs.get("timeout", 10.0),
                                      joint_indices=list(range(NUM_ARM_JOINTS)))
        return CODE_SUCCESS

    def set_gripper_position(self,
                             pos:   float,
                             wait:  bool = False,
                             speed: Optional[float] = None,
                             **kwargs) -> int:
        """
        Set gripper opening.

        pos : 0 = fully open, 800 = fully closed  (XArmAPI units).
              Internally mapped to the Gripper joint's min/max angle.
        """
        if not self._connected:
            return CODE_ERROR
        jcfg = self._cfg.joints[NUM_ARM_JOINTS]
        # Map XArmAPI 0-800 range to joint min-max angle
        t = max(0.0, min(1.0, pos / 800.0))
        deg = jcfg.min_angle + t * (jcfg.max_angle - jcfg.min_angle)
        raw = self._logical_to_raw(NUM_ARM_JOINTS, deg)
        if speed is not None:
            self._apply_speed(speed)
        # update _target_angles so _wait_for_motion has a reference
        self._target_angles[NUM_ARM_JOINTS] = max(jcfg.min_angle,
                                                  min(jcfg.max_angle, deg))
        self._driver.set_goal_position(
            jcfg.servo_id, raw, jcfg.speed, jcfg.acc)
        print(f"[StsRobotAPI] Gripper → {deg:.1f}° (pos={pos})")
        if wait:
            self._wait_for_motion(kwargs.get("timeout", 10.0),
                                  joint_indices=[NUM_ARM_JOINTS])
        return CODE_SUCCESS

    def set_vacuum_gripper(self, on: bool, wait: bool = False,
                           timeout: float = 3, delay_sec=None,
                           sync: bool = True,
                           hardware_version: int = 1) -> int:
        """Toggle gripper: True=close (full), False=open."""
        pos = 800 if on else 0
        return self.set_gripper_position(pos, wait=wait)

    def get_vacuum_gripper(self, hardware_version: int = 1) -> int:
        code, angle = self.get_gripper_position()
        jcfg = self._cfg.joints[NUM_ARM_JOINTS]
        mid = (jcfg.min_angle + jcfg.max_angle) / 2
        return 1 if angle > mid else 0

    # ── XArmAPI: emergency / errors ──────────────────────────────────────────
    def emergency_stop(self) -> int:
        self._motion_enabled = False
        self._state = STATE_ERROR
        for jcfg in self._cfg.joints:
            self._driver.set_torque(jcfg.servo_id, False)
        print("[StsRobotAPI] ⛔ EMERGENCY STOP")
        return CODE_SUCCESS

    def clean_error(self) -> int:
        self._error_code = 0
        self._state = STATE_READY
        print("[StsRobotAPI] Errors cleared.")
        return CODE_SUCCESS

    def get_err_warn_code(self) -> Tuple[int, List[int]]:
        return (CODE_SUCCESS, [self._error_code, 0])

    def reset(self, wait: bool = True) -> int:
        """Return to home position."""
        angles = [jcfg.home_angle for jcfg in self._cfg.joints]
        self._send_all_angles(angles)
        if wait:
            self._wait_for_motion(10.0)
        print("[StsRobotAPI] Reset to home.")
        return CODE_SUCCESS

    # ── XArmAPI: info ────────────────────────────────────────────────────────
    def get_version(self) -> Tuple[int, str]:
        return (CODE_SUCCESS, "v1.0.0-sts3215")

    def get_statistics(self) -> dict:
        angles = self._get_arm_angles_deg()
        _, pos = self.get_position()
        return {
            "connected":      self._connected,
            "motion_enabled": self._motion_enabled,
            "state":          self._state,
            "mode":           self._mode,
            "joint_angles":   angles,
            "position":       pos,
            "live_raw":       self._live_raw.copy(),
        }

    def reset_statistics(self) -> int:
        print("[StsRobotAPI] reset_statistics — no-op (no persistent stats).")
        return CODE_SUCCESS

    # ── XArmAPI: gripper mode/enable stubs ──────────────────────────────────
    def set_gripper_mode(self, mode) -> int:
        print(f"[StsRobotAPI] set_gripper_mode({mode}) — no-op")
        return CODE_SUCCESS

    def set_gripper_enable(self, enable) -> int:
        print(f"[StsRobotAPI] set_gripper_enable({enable}) — no-op")
        return CODE_SUCCESS

    # ── Config persistence ───────────────────────────────────────────────────
    def save_config(self, path: str) -> int:
        data = {
            "tcp_host": self.ip or "",
            "tcp_port": self._tcp_port,
            "port":     self._serial_port or "",
            "baud":     self._baud,
            "joints":   [asdict(j) for j in self._cfg.joints],
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"[StsRobotAPI] Config saved: {path}")
        return CODE_SUCCESS

    def _load_config(self, path: str):
        with open(path) as f:
            data = json.load(f)
        self.ip = data.get("tcp_host") or self.ip
        self._tcp_port = data.get("tcp_port", self._tcp_port)
        self._serial_port = data.get("port") or self._serial_port
        self._baud = data.get("baud", self._baud)
        for i, jd in enumerate(data.get("joints", [])[:NUM_JOINTS]):
            jd.setdefault("acc",        10)
            jd.setdefault("home_angle", 0.0)
            jd.setdefault("min_angle",  -90.0)
            jd.setdefault("max_angle",   90.0)
            self._cfg.joints[i] = JointConfig(**jd)
        print(f"[StsRobotAPI] Config loaded: {path}")

    # ── internal polling ─────────────────────────────────────────────────────
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
            for i, jcfg in enumerate(self._cfg.joints):
                raw = self._driver.get_present_position(jcfg.servo_id)
                if raw is not None:
                    self._live_raw[i] = raw
                    deg = self._raw_to_logical(i, raw)
                    self._live_positions[i] = deg
                    if self.on_position_update:
                        self.on_position_update(i, deg)
            # auto-clear moving state when servos settle
            if self._state == STATE_MOVING:
                self._state = STATE_READY
            time.sleep(0.05)

    # ── internal motion helpers ──────────────────────────────────────────────
    def _raw_to_logical(self, joint_idx: int, raw: int) -> float:
        offset = raw - self._cfg.joints[joint_idx].zero_offset
        deg = STS3215Driver.pos_to_deg(offset + POS_CENTER) - 180.0
        return round(-deg, 2)

    def _logical_to_raw(self, joint_idx: int, deg: float) -> int:
        jcfg = self._cfg.joints[joint_idx]
        deg = max(jcfg.min_angle, min(jcfg.max_angle, deg))
        pos = STS3215Driver.deg_to_pos(-deg + 180.0)
        raw = pos - POS_CENTER + jcfg.zero_offset
        return max(POS_MIN, min(POS_MAX, raw))

    def _send_all_angles(self, angles: List[float]):
        # store clamped targets so _wait_for_motion can compare against live data
        for i, deg in enumerate(angles[:NUM_JOINTS]):
            jcfg = self._cfg.joints[i]
            self._target_angles[i] = max(
                jcfg.min_angle, min(jcfg.max_angle, deg))
        id_pos_speed = [
            (self._cfg.joints[i].servo_id,
             self._logical_to_raw(i, deg),
             self._cfg.joints[i].speed,
             self._cfg.joints[i].acc)
            for i, deg in enumerate(angles[:NUM_JOINTS])
        ]
        self._driver.sync_write_positions(id_pos_speed)

    def _set_single_angle(self, joint_idx: int, deg: float):
        """Send a single joint to `deg` and record it in _target_angles."""
        jcfg = self._cfg.joints[joint_idx]
        clamped = max(jcfg.min_angle, min(jcfg.max_angle, deg))
        self._target_angles[joint_idx] = clamped
        self._driver.set_goal_position(
            jcfg.servo_id,
            self._logical_to_raw(joint_idx, clamped),
            jcfg.speed,
            jcfg.acc)

    def _get_arm_angles_deg(self) -> List[float]:
        return [p if p is not None else 0.0
                for p in self._live_positions[:NUM_ARM_JOINTS]]

    def _apply_speed(self, speed_mm_s: float):
        """Convert mm/s to servo speed steps and apply to all arm joints."""
        # rough mapping: 4096 steps = 360°, arm link ~150mm → 1 step ≈ 0.13 mm
        steps = max(1, int(speed_mm_s / 0.13))
        for jcfg in self._cfg.joints[:NUM_ARM_JOINTS]:
            jcfg.speed = min(steps, 256)

    def _wait_for_motion(self, timeout: float = 10.0,
                         settle_time: float = 0.10,
                         joint_indices: Optional[List[int]] = None):
        """
        Block until the specified joints (or all joints if None) report
        REG_MOVING == 0, then wait `settle_time` seconds for mechanical settle.

        Falls back to position-threshold comparison for any servo that does
        not respond to the moving register read (e.g. disconnected joint).

        timeout       : maximum seconds to wait before giving up
        settle_time   : extra pause after all servos stop (mechanical damping)
        joint_indices : list of 0-based joint indices to watch; None = all joints
        """
        indices = joint_indices if joint_indices is not None else list(
            range(NUM_JOINTS))

        # Wait for servos to actually start moving before polling REG_MOVING.
        # The STS3215 needs a few ms after receiving the command before the
        # moving register transitions to 1.  Without this pause the first poll
        # sees 0 and returns immediately even though the servo hasn't moved yet.
        # We use a two-phase approach:
        #   Phase 1 (startup): wait until at least one servo reports moving=1,
        #                      OR until the position has visibly changed,
        #                      OR until startup_timeout expires (very slow move).
        #   Phase 2 (running): poll until all servos report moving=0.
        startup_timeout = 0.3   # seconds to wait for motion to begin
        startup_deadline = time.time() + startup_timeout

        # Snapshot starting positions for the position-change fallback
        start_positions = {i: self._live_positions[i] for i in indices}

        # ── Phase 1: wait for motion to begin ────────────────────────────────
        motion_started = False
        while time.time() < startup_deadline:
            for i in indices:
                jcfg = self._cfg.joints[i]
                moving = self._driver.is_moving(jcfg.servo_id)
                if moving:
                    motion_started = True
                    break
                # fallback: check if position has changed from snapshot
                live = self._live_positions[i]
                start = start_positions[i]
                if live is not None and start is not None:
                    if abs(live - start) > 0.5:
                        motion_started = True
                        break
            if motion_started:
                break
            time.sleep(0.02)

        # ── Phase 2: wait for motion to complete ──────────────────────────────
        deadline = time.time() + timeout
        while time.time() < deadline:
            any_moving = False

            for i in indices:
                jcfg = self._cfg.joints[i]
                moving = self._driver.is_moving(jcfg.servo_id)

                if moving is None:
                    # servo not responding — fall back to position vs target
                    live = self._live_positions[i]
                    if live is not None:
                        if abs(live - self._target_angles[i]) > 2.0:
                            any_moving = True
                            break
                elif moving:
                    any_moving = True
                    break

            if not any_moving:
                time.sleep(settle_time)   # let mechanics settle
                self._state = STATE_READY
                return

            time.sleep(0.02)

        # timed out
        self._state = STATE_READY

    def _use_radian(self, override: Optional[bool]) -> bool:
        return self.is_radian if override is None else override

    # ── context manager support ──────────────────────────────────────────────
    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.disconnect()

    def __del__(self):
        try:
            self.disconnect()
        except Exception:
            pass

    def __repr__(self):
        state_names = {0: "ready", 1: "moving", 3: "paused", 4: "error"}
        conn = (f"TCP {self.ip}:{self._tcp_port}" if self.ip
                else (self._serial_port or "disconnected"))
        return (f"StsRobotAPI({conn}, "
                f"state={state_names.get(self._state, self._state)}, "
                f"motion={'on' if self._motion_enabled else 'off'})")


# ─────────────────────────────────────────────────────────────────────────────
#  StsRobotEmulator  —  offline stand-in, no hardware needed
# ─────────────────────────────────────────────────────────────────────────────
class StsRobotEmulator:
    """
    Pure-software emulator of StsRobotAPI.  No serial port or socket needed.
    Mirrors the XArmEmulator interface.

    Usage
    -----
        from sts_robot_api import StsRobotEmulator as XArmAPI
        robot = StsRobotAPI("192.168.1.100")
    """

    CODE_SUCCESS = CODE_SUCCESS
    CODE_WARN = CODE_WARN
    CODE_ERROR = CODE_ERROR

    STATE_READY = STATE_READY
    STATE_MOVING = STATE_MOVING
    STATE_PAUSED = STATE_PAUSED
    STATE_ERROR = STATE_ERROR

    def __init__(self,
                 ip:        Optional[str] = "127.0.0.1",
                 urdf_path: str = "robot.urdf",
                 is_radian: bool = False):

        self.ip = ip
        self.is_radian = is_radian

        self._kinematics = Kinematics(urdf_path)
        self._cfg = RobotConfig()
        self._connected = True
        self._motion_enabled = False
        self._mode = 0
        self._state = STATE_READY
        self._error_code = 0

        # joint angles in degrees, index 6 = gripper
        self._joint_angles: List[float] = [0.0] * NUM_JOINTS

        self._move_lock = threading.Lock()
        self._move_thread: Optional[threading.Thread] = None
        self._stop_move = False

        # callbacks
        self.on_position_update: Optional[Callable[[int, float], None]] = None

        print(f"[StsRobotEmulator] Running (ip={ip})")

    # ── connect / disconnect ─────────────────────────────────────────────────
    def connect(self) -> int:
        self._connected = True
        print("[StsRobotEmulator] Connected.")
        return CODE_SUCCESS

    def disconnect(self) -> int:
        self._stop_move = True
        self._connected = False
        print("[StsRobotEmulator] Disconnected.")
        return CODE_SUCCESS

    # ── enable / mode / state ────────────────────────────────────────────────
    def motion_enable(self, enable: bool = True,
                      servo_id: Optional[int] = None) -> int:
        self._motion_enabled = enable
        print(
            f"[StsRobotEmulator] Motion {'enabled' if enable else 'disabled'}.")
        return CODE_SUCCESS

    def set_mode(self, mode: int) -> int:
        self._mode = mode
        names = {0: "Position", 1: "Servo", 2: "Teach"}
        print(f"[StsRobotEmulator] Mode: {names.get(mode, str(mode))}")
        return CODE_SUCCESS

    def set_state(self, state: int) -> int:
        self._state = state
        names = {0: "Ready", 3: "Pause", 4: "Stop"}
        print(f"[StsRobotEmulator] State: {names.get(state, str(state))}")
        return CODE_SUCCESS

    def get_state(self) -> Tuple[int, int]:
        return (CODE_SUCCESS, self._state)

    # ── getters ──────────────────────────────────────────────────────────────
    def get_position(self,
                     is_radian: Optional[bool] = None
                     ) -> Tuple[int, List[float]]:
        arm = self._joint_angles[:NUM_ARM_JOINTS]
        if self._kinematics.available:
            pose = self._kinematics.fk(arm)
            c = self._kinematics.pose_components(pose)
            pos = [c["x"], c["y"], c["z"], c["roll"], c["pitch"], c["yaw"]]
        else:
            pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if self._use_radian(is_radian):
            pos = pos[:3] + [math.radians(a) for a in pos[3:]]
        return (CODE_SUCCESS, pos)

    def get_servo_angle(self,
                        servo_id:  Optional[int] = None,
                        is_radian: Optional[bool] = None
                        ) -> Tuple[int, List[float]]:
        angles = list(self._joint_angles[:NUM_ARM_JOINTS])
        if self._use_radian(is_radian):
            angles = [math.radians(a) for a in angles]
        return (CODE_SUCCESS, angles)

    def get_gripper_position(self) -> Tuple[int, float]:
        return (CODE_SUCCESS, self._joint_angles[NUM_ARM_JOINTS])

    # ── setters ──────────────────────────────────────────────────────────────
    def set_position(self,
                     x=None, y=None, z=None,
                     roll=None, pitch=None, yaw=None,
                     speed=None, mvacc=None, mvtime=None,
                     relative: bool = False,
                     wait: bool = False,
                     timeout: Optional[float] = None,
                     **kwargs) -> int:
        if not self._motion_enabled:
            print("[StsRobotEmulator] Motion not enabled.")
            return CODE_ERROR

        _, cur = self.get_position()

        def _r(cv, nv, rel):
            if nv is None:
                return cv
            v = math.degrees(nv) if self.is_radian else nv
            return (cv + v) if rel else v

        tx, ty, tz = _r(cur[0], x, relative), _r(
            cur[1], y, relative), _r(cur[2], z, relative)
        tr, tp, ty2 = _r(cur[3], roll, relative), _r(
            cur[4], pitch, relative), _r(cur[5], yaw, relative)

        if self._kinematics.available:
            seed = self._joint_angles[:NUM_ARM_JOINTS]
            pose = self._kinematics.pose_from_components(
                tx, ty, tz, tr, tp, ty2)
            sol = self._kinematics.ik(pose, seed_angles_deg=seed)
            if sol:
                self._animate_joints(
                    sol[:NUM_ARM_JOINTS], wait, timeout or 2.0)
                print(f"[StsRobotEmulator] set_position → "
                      f"[{tx:.1f},{ty:.1f},{tz:.1f},{tr:.1f},{tp:.1f},{ty2:.1f}]")
                return CODE_SUCCESS
            print("[StsRobotEmulator] IK failed.")
            return CODE_ERROR
        else:
            print(f"[StsRobotEmulator] set_position (no kinematics) → "
                  f"[{tx:.1f},{ty:.1f},{tz:.1f}]")
            return CODE_SUCCESS

    def set_servo_angle(self,
                        servo_id=None, angle=None, speed=None,
                        wait: bool = False,
                        angles: Optional[List[float]] = None,
                        is_radian: Optional[bool] = None,
                        **kwargs) -> int:
        use_rad = self._use_radian(is_radian)
        if servo_id is not None:
            idx = int(servo_id) - 1
            deg = math.degrees(angle) if use_rad else angle
            self._joint_angles[idx] = deg
            print(f"[StsRobotEmulator] Joint {servo_id} → {deg:.2f}°")
        elif angles:
            degs = [math.degrees(
                a) if use_rad else a for a in angles[:NUM_ARM_JOINTS]]
            self._joint_angles[:NUM_ARM_JOINTS] = degs
            print(
                f"[StsRobotEmulator] All joints → {[round(d, 2) for d in degs]}")
        if wait:
            time.sleep(0.5)
        return CODE_SUCCESS

    def set_gripper_position(self, pos: float, wait: bool = False,
                             speed=None, **kwargs) -> int:
        jcfg = self._cfg.joints[NUM_ARM_JOINTS]
        t = max(0.0, min(1.0, pos / 800.0))
        deg = jcfg.min_angle + t * (jcfg.max_angle - jcfg.min_angle)
        self._joint_angles[NUM_ARM_JOINTS] = deg
        print(f"[StsRobotEmulator] Gripper → {deg:.1f}° (pos={pos})")
        if wait:
            time.sleep(0.3)
        return CODE_SUCCESS

    def set_vacuum_gripper(self, on, wait=False, timeout=3,
                           delay_sec=None, sync=True,
                           hardware_version=1) -> int:
        return self.set_gripper_position(800 if on else 0, wait=wait)

    def get_vacuum_gripper(self, hardware_version=1) -> int:
        _, angle = self.get_gripper_position()
        jcfg = self._cfg.joints[NUM_ARM_JOINTS]
        return 1 if angle > (jcfg.min_angle + jcfg.max_angle) / 2 else 0

    # ── emergency / errors ───────────────────────────────────────────────────
    def emergency_stop(self) -> int:
        self._stop_move = True
        self._motion_enabled = False
        self._state = STATE_ERROR
        print("[StsRobotEmulator] ⛔ EMERGENCY STOP")
        return CODE_SUCCESS

    def clean_error(self) -> int:
        self._error_code = 0
        self._state = STATE_READY
        print("[StsRobotEmulator] Errors cleared.")
        return CODE_SUCCESS

    def get_err_warn_code(self) -> Tuple[int, List[int]]:
        return (CODE_SUCCESS, [self._error_code, 0])

    def reset(self, wait: bool = True) -> int:
        self._joint_angles = [0.0] * NUM_JOINTS
        print("[StsRobotEmulator] Reset.")
        return CODE_SUCCESS

    # ── info ─────────────────────────────────────────────────────────────────
    def get_version(self) -> Tuple[int, str]:
        return (CODE_SUCCESS, "v1.0.0-sts3215-emulator")

    def get_statistics(self) -> dict:
        _, pos = self.get_position()
        return {
            "connected":      self._connected,
            "motion_enabled": self._motion_enabled,
            "state":          self._state,
            "joint_angles":   self._joint_angles.copy(),
            "position":       pos,
        }

    def reset_statistics(self) -> int:
        print("[StsRobotEmulator] Statistics reset.")
        return CODE_SUCCESS

    def set_gripper_mode(self, _) -> int:
        return CODE_SUCCESS

    def set_gripper_enable(self, _) -> int:
        return CODE_SUCCESS

    # ── animation helper ─────────────────────────────────────────────────────
    def _animate_joints(self, target_angles: List[float],
                        wait: bool, duration: float):
        """Smoothly interpolate joint angles over `duration` seconds."""
        self._stop_move = False
        self._state = STATE_MOVING
        start = list(self._joint_angles[:NUM_ARM_JOINTS])

        def _run():
            steps = max(10, int(duration * 50))
            delay = duration / steps
            for s in range(steps + 1):
                if self._stop_move:
                    break
                t = s / steps
                for i in range(NUM_ARM_JOINTS):
                    self._joint_angles[i] = start[i] + \
                        (target_angles[i] - start[i]) * t
                    if self.on_position_update:
                        self.on_position_update(i, self._joint_angles[i])
                time.sleep(delay)
            self._state = STATE_READY

        self._move_thread = threading.Thread(target=_run, daemon=True)
        self._move_thread.start()
        if wait:
            self._move_thread.join()

    def _use_radian(self, override):
        return self.is_radian if override is None else override

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.disconnect()

    def __del__(self):
        try:
            self.disconnect()
        except Exception:
            pass

    def __repr__(self):
        return (f"StsRobotEmulator(ip={self.ip}, "
                f"motion={'on' if self._motion_enabled else 'off'})")


# ── drop-in alias ─────────────────────────────────────────────────────────────
XArmAPI = StsRobotAPI
