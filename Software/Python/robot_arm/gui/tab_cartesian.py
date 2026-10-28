"""Cartesian control tab — move end-effector in XYZ (1 mm steps) and RPY (1° steps)."""

import tkinter as tk
from tkinter import ttk, messagebox

from ..constants import JOINT_NAMES, NUM_ARM_JOINTS


class CartesianTabMixin:

    def _build_cartesian_tab(self):
        f = self.tab_cartesian
        T = self.T

        if not self.ctrl.kinematics.available:
            tk.Label(f, text="⚠  Cartesian control requires roboticstoolbox\n"
                              "and robot.urdf to be present.\n\n"
                              "Install:  pip install roboticstoolbox-python",
                     bg=T["BG"], fg=T["ORANGE"],
                     font=("Helvetica", 12), justify="center").pack(expand=True)
            return

        # ── Current pose display ─────────────────────────────────────────────
        pose_frame = ttk.LabelFrame(f, text=" Current End-Effector Pose (from FK) ", padding=10)
        pose_frame.pack(fill="x", padx=12, pady=(12, 4))

        self._cart_cur_vars = {}
        cur_inner = tk.Frame(pose_frame, bg=T["BG"])
        cur_inner.pack(fill="x")
        for col, (lbl, unit) in enumerate([("X","mm"),("Y","mm"),("Z","mm"),
                                            ("Roll","°"),("Pitch","°"),("Yaw","°")]):
            tk.Label(cur_inner, text=f"{lbl} ({unit})", bg=T["BG"], fg=T["SUBTEXT"],
                     font=("Helvetica", 9, "bold")).grid(row=0, column=col, padx=10)
            var = tk.StringVar(value="—")
            self._cart_cur_vars[lbl] = var
            tk.Label(cur_inner, textvariable=var, bg=T["BG"],
                     fg=T["GREEN"], font=("Courier", 11, "bold"),
                     width=9).grid(row=1, column=col, padx=10)

        ttk.Button(pose_frame, text="⟳  Refresh from live positions",
                   command=self._cartesian_refresh_fk).pack(pady=(6, 0))

        # ── Target pose entry ────────────────────────────────────────────────
        tgt_frame = ttk.LabelFrame(f, text=" Target Pose ", padding=10)
        tgt_frame.pack(fill="x", padx=12, pady=4)

        self._cart_tgt_vars = {}
        defaults = {"X": 200.0, "Y": 0.0, "Z": 200.0,
                    "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}
        units    = {"X": "mm", "Y": "mm", "Z": "mm",
                    "Roll": "°", "Pitch": "°", "Yaw": "°"}
        steps    = {"X": 1.0, "Y": 1.0, "Z": 1.0,
                    "Roll": 1.0, "Pitch": 1.0, "Yaw": 1.0}

        tgt_grid = tk.Frame(tgt_frame, bg=T["BG"])
        tgt_grid.pack(fill="x")

        for col, key in enumerate(["X", "Y", "Z", "Roll", "Pitch", "Yaw"]):
            unit = units[key]
            step = steps[key]
            var  = tk.DoubleVar(value=defaults[key])
            self._cart_tgt_vars[key] = var

            tk.Label(tgt_grid, text=f"{key} ({unit})", bg=T["BG"], fg=T["SUBTEXT"],
                     font=("Helvetica", 9, "bold")).grid(row=0, column=col, padx=8, pady=2)

            cell = tk.Frame(tgt_grid, bg=T["BG"])
            cell.grid(row=1, column=col, padx=6, pady=2)

            ttk.Button(cell, text="−", width=2,
                       command=lambda k=key, s=step: self._cart_nudge(k, -s)
                       ).pack(side="left")
            ttk.Entry(cell, textvariable=var, width=8,
                      justify="center").pack(side="left", padx=2)
            ttk.Button(cell, text="+", width=2,
                       command=lambda k=key, s=step: self._cart_nudge(k, +s)
                       ).pack(side="left")

        # step size selector
        step_row = tk.Frame(tgt_frame, bg=T["BG"])
        step_row.pack(fill="x", pady=(6, 0))
        tk.Label(step_row, text="Step size:", bg=T["BG"],
                 fg=T["SUBTEXT"]).pack(side="left", padx=6)
        self._cart_step_var = tk.DoubleVar(value=1.0)
        for s in [0.1, 0.5, 1.0, 5.0, 10.0]:
            ttk.Radiobutton(step_row, text=str(s),
                            variable=self._cart_step_var, value=s,
                            command=self._cart_update_steps).pack(side="left", padx=4)

        # ── IK result ────────────────────────────────────────────────────────
        ik_frame = ttk.LabelFrame(f, text=" IK Solution (joint angles °) ", padding=8)
        ik_frame.pack(fill="x", padx=12, pady=4)

        self._cart_ik_vars = [tk.StringVar(value="—") for _ in range(NUM_ARM_JOINTS)]
        ik_inner = tk.Frame(ik_frame, bg=T["BG"])
        ik_inner.pack(fill="x")
        for i in range(NUM_ARM_JOINTS):
            tk.Label(ik_inner, text=JOINT_NAMES[i], bg=T["BG"], fg=T["SUBTEXT"],
                     font=("Helvetica", 9)).grid(row=0, column=i, padx=8)
            tk.Label(ik_inner, textvariable=self._cart_ik_vars[i],
                     bg=T["BG"], fg=T["GREEN"],
                     font=("Courier", 10, "bold"), width=8).grid(row=1, column=i, padx=8)

        # ── Action buttons ───────────────────────────────────────────────────
        act_row = tk.Frame(f, bg=T["BG"])
        act_row.pack(fill="x", padx=12, pady=8)

        ttk.Button(act_row, text="🔢  Solve IK",
                   command=self._cart_solve_ik).pack(side="left", padx=4)
        ttk.Button(act_row, text="▶  Solve & Move", style="Green.TButton",
                   command=self._cart_solve_and_move).pack(side="left", padx=4)
        ttk.Button(act_row, text="📋  Copy to Control Tab",
                   command=self._cart_copy_to_ctrl).pack(side="left", padx=4)

        # ── Realtime sending ─────────────────────────────────────────────────
        rt_row = tk.Frame(f, bg=T["BG"])
        rt_row.pack(fill="x", padx=12, pady=(0, 4))

        self._cart_realtime_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(rt_row, text="🔴  Realtime send  (solve IK & move on every nudge)",
                        variable=self._cart_realtime_var).pack(side="left")
        tk.Label(rt_row, text="⚠ robot must be connected",
                 bg=T["BG"], fg=T["SUBTEXT"],
                 font=("Helvetica", 8, "italic")).pack(side="left", padx=8)

        self._cart_status = tk.Label(f, text="", bg=T["BG"], fg=T["SUBTEXT"],
                                      font=("Helvetica", 9, "italic"))
        self._cart_status.pack(padx=12, anchor="w")

        # store step references so they can be updated
        self._cart_step_refs = steps

    # ── helpers ───────────────────────────────────────────────────────────────
    def _cart_update_steps(self):
        s = self._cart_step_var.get()
        for key in self._cart_step_refs:
            self._cart_step_refs[key] = s

    def _cart_nudge(self, key: str, delta: float):
        step = self._cart_step_var.get()
        var  = self._cart_tgt_vars[key]
        var.set(round(var.get() + delta * step / abs(delta) if delta != 0 else 0, 4))
        if getattr(self, "_cart_realtime_var", None) and self._cart_realtime_var.get():
            self._cart_solve_and_move()

    def _cartesian_refresh_fk(self):
        if not self.ctrl.kinematics.available:
            return
        pose = self.ctrl.compute_fk()
        c = self.ctrl.kinematics.pose_components(pose)
        self._cart_cur_vars["X"].set(f"{c['x']:+.2f}")
        self._cart_cur_vars["Y"].set(f"{c['y']:+.2f}")
        self._cart_cur_vars["Z"].set(f"{c['z']:+.2f}")
        self._cart_cur_vars["Roll"].set(f"{c['roll']:+.2f}")
        self._cart_cur_vars["Pitch"].set(f"{c['pitch']:+.2f}")
        self._cart_cur_vars["Yaw"].set(f"{c['yaw']:+.2f}")
        # also pre-fill target with current pose
        for k, ck in [("X","x"),("Y","y"),("Z","z"),
                      ("Roll","roll"),("Pitch","pitch"),("Yaw","yaw")]:
            self._cart_tgt_vars[k].set(round(c[ck], 2))

    def _cart_solve_ik(self) -> list | None:
        x   = self._cart_tgt_vars["X"].get()
        y   = self._cart_tgt_vars["Y"].get()
        z   = self._cart_tgt_vars["Z"].get()
        ro  = self._cart_tgt_vars["Roll"].get()
        pi  = self._cart_tgt_vars["Pitch"].get()
        ya  = self._cart_tgt_vars["Yaw"].get()

        angles = self.ctrl.compute_ik(x, y, z, ro, pi, ya)
        if angles is None:
            self._cart_status.configure(
                text="✗  IK did not converge for this target.", fg=self.T["ORANGE"])
            for v in self._cart_ik_vars:
                v.set("✗")
            return None

        for i, a in enumerate(angles[:NUM_ARM_JOINTS]):
            self._cart_ik_vars[i].set(f"{a:+.2f}")
        self._cart_status.configure(
            text=f"✓  IK solved: {[round(a,2) for a in angles]}", fg=self.T["GREEN"])
        return angles

    def _cart_solve_and_move(self):
        if not self.ctrl.driver.connected:
            self._log("Not connected"); return
        angles = self._cart_solve_ik()
        if angles is None:
            return
        # Build full joint angle list: IK result + keep current gripper angle
        full = list(angles[:NUM_ARM_JOINTS])
        gripper_idx = NUM_ARM_JOINTS  # index 6
        gripper_cur = self.ctrl.live_positions[gripper_idx]
        full.append(gripper_cur if gripper_cur is not None else 0.0)
        self.ctrl.set_all_angles(full)
        # update target sliders
        for i, a in enumerate(full):
            self.target_vars[i].set(round(a, 2))
        self._log(f"Cartesian move → {[round(a,1) for a in full]}")

    def _cart_copy_to_ctrl(self):
        angles = self._cart_solve_ik()
        if angles is None:
            return
        for i, a in enumerate(angles[:NUM_ARM_JOINTS]):
            self.target_vars[i].set(round(a, 2))
        self._cart_status.configure(
            text="Angles copied to Control tab.", fg=self.T["GREEN"])
