"""
Forward and inverse kinematics via roboticstoolbox-python + a URDF file.

Usage
-----
    km = Kinematics("robot.urdf")
    if km.available:
        pose   = km.fk([0, 0, 0, 0, 0, 0])   # SE3
        angles = km.ik(pose)                   # list[float] | None

The module degrades gracefully: if roboticstoolbox / spatialmath are not
installed, `km.available` is False and both methods return None.

Only the first NUM_ARM_JOINTS joints from the URDF are used for FK/IK;
the Gripper joint is excluded from kinematics.
"""

from __future__ import annotations
import os
from typing import Optional

from .constants import NUM_ARM_JOINTS

# ── optional imports ─────────────────────────────────────────────────────────
try:
    import roboticstoolbox as rtb
    from spatialmath import SE3
    import numpy as np
    _RTB_OK = True
except ImportError:
    _RTB_OK = False


class Kinematics:
    def __init__(self, urdf_path: str = "robot.urdf"):
        self.available = False
        self._robot = None
        self._urdf_path = urdf_path

        if not _RTB_OK:
            print("[Kinematics] roboticstoolbox not installed — FK/IK disabled.")
            return

        if not os.path.isfile(urdf_path):
            print(
                f"[Kinematics] URDF not found: {urdf_path} — FK/IK disabled.")
            return

        try:
            self._robot = rtb.ERobot.URDF(os.path.abspath(urdf_path))
            self.available = True
            print(f"[Kinematics] Loaded '{urdf_path}'  "
                  f"({len(self._robot.links)} links, "
                  f"{self._robot.n} DOF)")
        except Exception as e:
            print(f"[Kinematics] Failed to load URDF: {e}")

    # ── public API ────────────────────────────────────────────────────────────
    def fk(self, joint_angles_deg: list) -> Optional[object]:
        """
        Forward kinematics.

        Parameters
        ----------
        joint_angles_deg : list of float, length >= NUM_ARM_JOINTS
            Joint angles in degrees.  Only the first NUM_ARM_JOINTS values
            are used; the Gripper angle is ignored.

        Returns
        -------
        SE3 pose of the end-effector, or None on failure.
        """
        if not self.available:
            return None
        try:
            q = _deg_to_rad(joint_angles_deg[:NUM_ARM_JOINTS])
            return self._robot.fkine(q)
        except Exception as e:
            print(f"[Kinematics] FK error: {e}")
            return None

    def ik(self, target_pose,
           seed_angles_deg: Optional[list] = None) -> Optional[list]:
        """
        Inverse kinematics.

        Handles both old API (sol.success / sol.q) and new tuple API
        (q, success, iterations, searches, residual).

        Returns list of NUM_ARM_JOINTS angles in degrees, or None on failure.
        """
        if not self.available or target_pose is None:
            return None
        try:
            q0 = _deg_to_rad(seed_angles_deg[:NUM_ARM_JOINTS]) \
                if seed_angles_deg else np.zeros(self._robot.n)
            for solver in (
                lambda: self._robot.ikine_LM(
                    target_pose, q0=q0, tol=1e-3, ilimit=500),
                lambda: self._robot.ikine_NR(target_pose, q0=q0),
            ):
                result = solver()
                print(result)
                q, success = _unpack_ik(result)
                if success and q is not None:
                    return _rad_to_deg(q)

            print("[Kinematics] IK did not converge.")
            return None
        except Exception as e:
            print(f"[Kinematics] IK error: {e}")
            return None

    def pose_components(self, pose) -> dict:
        """
        Decompose an SE3 pose into xyz (mm) and rpy (deg).

        Returns dict with keys x, y, z, roll, pitch, yaw  — all floats.
        """
        if pose is None:
            return dict(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0)
        try:
            import numpy as np
            t = pose.t                         # (3,) translation in metres
            # (3,) roll-pitch-yaw in deg
            rpy = pose.rpy(order="xyz", unit="deg")
            return dict(
                x=float(t[0] * 1000),   # → mm
                y=float(t[1] * 1000),
                z=float(t[2] * 1000),
                roll=float(rpy[0]),
                pitch=float(rpy[1]),
                yaw=float(rpy[2]),
            )
        except Exception as e:
            print(f"[Kinematics] pose_components error: {e}")
            return dict(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0)

    def pose_from_components(self, x_mm: float, y_mm: float, z_mm: float,
                             roll_deg: float, pitch_deg: float,
                             yaw_deg: float):
        """Build an SE3 pose from xyz (mm) and rpy (deg)."""
        if not self.available:
            return None
        try:
            from spatialmath import SE3
            import numpy as np
            t = np.array([x_mm / 1000, y_mm / 1000, z_mm / 1000])
            rot = SE3.RPY([roll_deg, pitch_deg, yaw_deg],
                          order="xyz", unit="deg")
            return SE3.Rt(rot.R, t)
        except Exception as e:
            print(f"[Kinematics] pose_from_components error: {e}")
            return None


# ── helpers ──────────────────────────────────────────────────────────────────
def _deg_to_rad(angles_deg):
    import math
    return [math.radians(a) for a in angles_deg]


def _rad_to_deg(angles_rad):
    import math
    return [round(math.degrees(a), 3) for a in angles_rad]


def _unpack_ik(result):
    """
    Normalise the two different return styles of roboticstoolbox IK solvers:
      - Old style: solution object with .success and .q attributes
      - New style: tuple (q, success, iterations, searches, residual)
    Returns (q, success) in both cases.
    """
    if isinstance(result, tuple):
        # new-style: (q, success, ...)
        q, success = result[0], bool(result[1])
        return q, success
    else:
        # old-style: solution object
        return getattr(result, "q", None), bool(getattr(result, "success", False))
