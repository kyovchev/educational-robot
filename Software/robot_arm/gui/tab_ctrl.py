"""Control tab — live joint sliders, nudge buttons, FK display, teach mode."""

import tkinter as tk
from tkinter import ttk

from ..constants import JOINT_NAMES, NUM_JOINTS


class CtrlTabMixin:
    # ── build ────────────────────────────────────────────────────────────────
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
                     font=("Helvetica", 10), width=10, anchor="w").grid(
                         row=r, column=0, padx=4, pady=3)
            tk.Label(jf, textvariable=self.pos_vars[i], bg=T["BG"],
                     fg=T["GREEN"], font=("Courier", 11, "bold"), width=10).grid(
                         row=r, column=1)
            ttk.Entry(jf, textvariable=self.target_vars[i], width=8).grid(
                row=r, column=2, padx=4)
            # − decrements (moves in negative direction), + increments
            ttk.Button(jf, text="−", width=2,
                       command=lambda idx=i: self._nudge_joint(idx, -1)).grid(
                           row=r, column=3, padx=(4, 0))
            sld = ttk.Scale(jf, from_=jcfg.min_angle, to=jcfg.max_angle,
                            orient="horizontal", variable=self.target_vars[i], length=220,
                            command=lambda v, idx=i: self.target_vars[idx].set(
                                round(float(v), 2)))
            sld.grid(row=r, column=4, padx=2)
            self.sliders.append(sld)
            ttk.Button(jf, text="+", width=2,
                       command=lambda idx=i: self._nudge_joint(idx, +1)).grid(
                           row=r, column=5, padx=(0, 4))
            ttk.Button(jf, text="▶", width=3,
                       command=lambda idx=i: self._send_joint(idx)).grid(
                           row=r, column=6)

        bc = tk.Frame(f, bg=T["BG"])
        bc.pack(fill="x", padx=12, pady=8)
        ttk.Button(bc, text="▶  Send ALL", style="Green.TButton",
                   command=self._send_all).pack(side="left", padx=4)
        ttk.Button(bc, text="⏹  Torque OFF", style="Danger.TButton",
                   command=lambda: self.ctrl.torque_all(False)).pack(side="left", padx=4)
        ttk.Button(bc, text="⚡  Torque ON",
                   command=lambda: self.ctrl.torque_all(True)).pack(side="left", padx=4)
        ttk.Button(bc, text="🏠  Home (0°)",
                   command=self._go_home).pack(side="left", padx=4)

        # ── FK display ────────────────────────────────────────────────────────
        fk_frame = ttk.LabelFrame(f, text=" Forward Kinematics — End-Effector Pose ", padding=8)
        fk_frame.pack(fill="x", padx=8, pady=4)

        fk_inner = tk.Frame(fk_frame, bg=T["BG"])
        fk_inner.pack(fill="x")

        self._fk_vars = {}
        labels = [("X", "mm"), ("Y", "mm"), ("Z", "mm"),
                  ("Roll", "°"), ("Pitch", "°"), ("Yaw", "°")]
        for col, (lbl, unit) in enumerate(labels):
            tk.Label(fk_inner, text=lbl, bg=T["BG"], fg=T["SUBTEXT"],
                     font=("Helvetica", 9, "bold")).grid(row=0, column=col*2, padx=(10,2))
            var = tk.StringVar(value="—")
            self._fk_vars[lbl] = var
            tk.Label(fk_inner, textvariable=var, bg=T["BG"],
                     fg=T["GREEN"], font=("Courier", 10, "bold"),
                     width=8).grid(row=0, column=col*2+1, padx=(0,6))

        fk_btn_row = tk.Frame(fk_frame, bg=T["BG"])
        fk_btn_row.pack(fill="x", pady=(4,0))
        if self.ctrl.kinematics.available:
            ttk.Button(fk_btn_row, text="⟳  Compute FK",
                       command=self._update_fk).pack(side="left", padx=4)
            tk.Label(fk_btn_row, text="(auto-updates from live positions)",
                     bg=T["BG"], fg=T["SUBTEXT"], font=("Helvetica", 8)).pack(side="left")
        else:
            tk.Label(fk_btn_row,
                     text="⚠  roboticstoolbox not installed or robot.urdf not found — FK/IK disabled",
                     bg=T["BG"], fg=T["ORANGE"], font=("Helvetica", 8)).pack(side="left", padx=4)

        # ── Teach Mode ────────────────────────────────────────────────────────
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
        ttk.Entry(tm, textvariable=self.pos_name_var, width=14).pack(side="left")
        ttk.Button(tm, text="📍  Capture", style="Green.TButton",
                   command=self._capture_pose).pack(side="left", padx=6)

        self.teach_status = tk.Label(tf, text="", bg=T["BG"], fg=T["ORANGE"],
                                     font=("Helvetica", 9, "italic"))
        self.teach_status.pack()

    # ── FK update ─────────────────────────────────────────────────────────────
    def _update_fk(self):
        if not self.ctrl.kinematics.available:
            return
        pose = self.ctrl.compute_fk()
        if pose is None:
            for v in self._fk_vars.values():
                v.set("err")
            return
        c = self.ctrl.kinematics.pose_components(pose)
        self._fk_vars["X"].set(f"{c['x']:+.1f}")
        self._fk_vars["Y"].set(f"{c['y']:+.1f}")
        self._fk_vars["Z"].set(f"{c['z']:+.1f}")
        self._fk_vars["Roll"].set(f"{c['roll']:+.1f}")
        self._fk_vars["Pitch"].set(f"{c['pitch']:+.1f}")
        self._fk_vars["Yaw"].set(f"{c['yaw']:+.1f}")

    # ── callbacks ────────────────────────────────────────────────────────────
    def _on_position_update(self, idx: int, deg: float):
        self.pos_vars[idx].set(f"{deg:+.1f}°")
        # auto-refresh FK whenever any joint updates
        if self.ctrl.kinematics.available:
            self.root.after(0, self._update_fk)

    def _send_joint(self, idx: int):
        if not self.ctrl.driver.connected:
            self._log("Not connected"); return
        jcfg = self.ctrl.config.joints[idx]
        deg  = max(jcfg.min_angle, min(jcfg.max_angle, self.target_vars[idx].get()))
        self.target_vars[idx].set(round(deg, 2))
        self.ctrl.set_joint_angle(idx, deg)
        self._log(f"Sent {JOINT_NAMES[idx]} → {deg:.2f}°")

    def _nudge_joint(self, idx: int, delta: float):
        """+ increments angle, − decrements angle, then sends immediately."""
        jcfg    = self.ctrl.config.joints[idx]
        current = self.target_vars[idx].get()
        new_val = round(max(jcfg.min_angle, min(jcfg.max_angle, current + delta)), 2)
        self.target_vars[idx].set(new_val)
        if self.ctrl.driver.connected:
            self.ctrl.set_joint_angle(idx, new_val)

    def _send_all(self):
        if not self.ctrl.driver.connected:
            self._log("Not connected"); return
        angles = [v.get() for v in self.target_vars]
        self.ctrl.set_all_angles(angles)
        self._log(f"Sent ALL → {[round(a, 1) for a in angles]}")

    def _go_home(self):
        if not self.ctrl.driver.connected:
            self._log("Not connected"); return
        self.ctrl.go_home()
        for i, jcfg in enumerate(self.ctrl.config.joints):
            self.target_vars[i].set(jcfg.home_angle)
        self._log("Moved to home position.")

    def _toggle_teach(self):
        if self._teach_mode:
            self.ctrl.exit_teach_mode()
            self._teach_mode = False
            self.teach_btn.configure(text="🖐  Enter Teach Mode")
            self.teach_status.configure(text="")
            self._log("Teach mode OFF — torque restored.")
        else:
            if not self.ctrl.driver.connected:
                self._log("Not connected"); return
            self.ctrl.enter_teach_mode()
            self._teach_mode = True
            self.teach_btn.configure(text="✋  Exit Teach Mode")
            self.teach_status.configure(
                text="Teach mode active — manually pose the arm, then Capture.")
            self._log("Teach mode ON — torque disabled.")

    def _capture_pose(self):
        if not self.ctrl.driver.connected:
            self._log("Not connected"); return
        name = self.pos_name_var.get() or f"pose_{len(self.ctrl.sequence)+1}"
        wp = self.ctrl.capture_position(name)
        self._log(f"Captured '{wp.name}'  angles={[round(a, 1) for a in wp.angles]}")
        self._refresh_wp_list()
