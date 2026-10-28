"""Setup tab — servo ID change, joint configuration table, diagnostics."""

import tkinter as tk
from tkinter import ttk, messagebox

from ..constants import JOINT_NAMES, NUM_JOINTS


class SetupTabMixin:
    # ── build ────────────────────────────────────────────────────────────────
    def _build_setup_tab(self):
        f = self.tab_setup
        f.columnconfigure(0, weight=1)
        f.columnconfigure(1, weight=1)
        T = self.T

        # ── Appearance bar ────────────────────────────────────────────────────
        app_bar = tk.Frame(f, bg=T["PANEL"], pady=6)
        app_bar.grid(row=0, column=0, columnspan=2, sticky="ew", padx=0, pady=(0, 4))

        tk.Label(app_bar, text="Appearance:", bg=T["PANEL"],
                 fg=T["SUBTEXT"], font=("Helvetica", 9, "bold")).pack(side="left", padx=12)

        self.theme_btn = tk.Button(
            app_bar, text="☀  Light mode",
            bg=T["ACCENT"], fg=T["TEXT"],
            activebackground=T["ORANGE"], activeforeground=T["TEXT"],
            relief="flat", padx=10, pady=3, cursor="hand2",
            command=self._toggle_theme)
        self.theme_btn.pack(side="left", padx=6)

        # ── Change Servo ID ──────────────────────────────────────────────────
        lf = ttk.LabelFrame(f, text=" Change Servo ID ", padding=12)
        lf.grid(row=1, column=0, padx=12, pady=12, sticky="nsew")

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

        # ── Joint Configuration ──────────────────────────────────────────────
        lf2 = ttk.LabelFrame(f, text=" Joint Configuration ", padding=12)
        lf2.grid(row=1, column=1, padx=12, pady=12, sticky="nsew")

        headers = ["Joint", "Servo ID", "Speed", "ACC",
                   "Min°", "Max°", "Home°", "Set Zero", "Ping", "Clr Spd"]
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
            sb_id.bind("<Return>",   lambda e, idx=i: self._sync_id(idx))

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
                       command=lambda idx=i: self._set_zero(idx)).grid(
                           row=r, column=7, padx=2)
            ttk.Button(lf2, text="Ping", width=5,
                       command=lambda idx=i: self._ping_joint(idx)).grid(
                           row=r, column=8, padx=2)
            ttk.Button(lf2, text="Clr Spd", width=7, style="Danger.TButton",
                       command=lambda idx=i: self._clear_eeprom_speed(idx)).grid(
                           row=r, column=9, padx=2)

        apply_row = len(JOINT_NAMES) + 1
        ttk.Button(lf2, text="✔  Apply Speed & ACC to all joints",
                   command=self._apply_speed_acc).grid(
                       row=apply_row, column=0, columnspan=10, pady=8, sticky="ew")

        # ── Diagnostics ──────────────────────────────────────────────────────
        lf3 = ttk.LabelFrame(f, text=" Servo Diagnostics ", padding=10)
        lf3.grid(row=2, column=0, columnspan=2, padx=12, pady=6, sticky="nsew")
        f.rowconfigure(2, weight=1)

        tk.Label(lf3, text="Select joint:", bg=T["BG"], fg=T["SUBTEXT"]).pack(side="left")
        self.diag_joint_cb = ttk.Combobox(
            lf3, values=[f"{i}: {n}" for i, n in enumerate(JOINT_NAMES)],
            state="readonly", width=18)
        self.diag_joint_cb.current(0)
        self.diag_joint_cb.pack(side="left", padx=6)
        ttk.Button(lf3, text="📊 Read", command=self._read_diagnostics).pack(side="left")

        self.diag_text = tk.Text(lf3, height=5, bg=T["ENTRY"], fg=T["GREEN"],
                                 font=("Courier", 9), relief="flat")
        self.diag_text.pack(fill="both", expand=True, pady=6, padx=4)

    # ── config-sync helpers ──────────────────────────────────────────────────
    def _sync_id(self, idx: int):
        try:
            self.ctrl.config.joints[idx].servo_id = int(self.id_vars[idx].get())
        except Exception:
            pass

    def _sync_speed_acc(self, idx: int):
        try:
            self.ctrl.config.joints[idx].speed = int(self.speed_vars[idx].get())
        except Exception:
            pass
        try:
            self.ctrl.config.joints[idx].acc = int(self.acc_vars[idx].get())
        except Exception:
            pass

    def _sync_limits(self, idx: int):
        try:
            self.ctrl.config.joints[idx].min_angle  = float(self.min_angle_vars[idx].get())
        except Exception:
            pass
        try:
            self.ctrl.config.joints[idx].max_angle  = float(self.max_angle_vars[idx].get())
        except Exception:
            pass
        try:
            self.ctrl.config.joints[idx].home_angle = float(self.home_angle_vars[idx].get())
        except Exception:
            pass
        # keep slider range in sync with new limits
        jcfg = self.ctrl.config.joints[idx]
        self.sliders[idx].configure(from_=jcfg.min_angle, to=jcfg.max_angle)

    def _apply_speed_acc(self):
        for i in range(NUM_JOINTS):
            self._sync_speed_acc(i)
        self._log("Speed & ACC applied to all joints.")

    # ── callbacks ────────────────────────────────────────────────────────────
    def _change_id(self):
        if not self.ctrl.driver.connected:
            self._log("Not connected"); return
        try:
            cur = int(self.cur_id_var.get())
            new = int(self.new_id_var.get())
        except ValueError:
            messagebox.showerror("Invalid", "IDs must be integers.")
            return
        if cur == new:
            messagebox.showwarning("Same ID", "Current and new ID are the same.")
            return
        if not (1 <= new <= 253):
            messagebox.showerror("Invalid ID", "New ID must be between 1 and 253.")
            return

        ok = self.ctrl.set_servo_id(cur, new)
        if ok:
            msg = (f"ID {cur} → {new} written to EEPROM.\n\n"
                   f"IMPORTANT: Power-cycle (unplug/replug) the servo now.\n"
                   f"The servo still responds on ID {cur} until rebooted.\n\n"
                   f"After rebooting, update the Servo ID field in the Setup\n"
                   f"tab for the matching joint to {new}.")
        else:
            msg = f"Failed to change ID {cur} → {new}"
        self._log(msg.splitlines()[0])
        messagebox.showinfo("ID Change — Action Required", msg)

    def _set_zero(self, idx: int):
        if not self.ctrl.driver.connected:
            self._log("Not connected"); return
        ok = self.ctrl.set_zero_here(idx)
        self._log(f"{'Zero set for' if ok else 'Failed to zero'} {JOINT_NAMES[idx]}")

    def _ping_joint(self, idx: int):
        if not self.ctrl.driver.connected:
            self._log("Not connected"); return
        sid = self.ctrl.config.joints[idx].servo_id
        ok  = self.ctrl.ping_servo(sid)
        self._log(f"Ping ID {sid}: {'OK ✓' if ok else 'NO RESPONSE ✗'}")

    def _clear_eeprom_speed(self, idx: int):
        if not self.ctrl.driver.connected:
            self._log("Not connected"); return
        ok = self.ctrl.clear_speed_limit(idx)
        self._log(
            f"{'EEPROM speed limit cleared for' if ok else 'Failed to clear'} "
            f"{JOINT_NAMES[idx]}")

    def _read_diagnostics(self):
        if not self.ctrl.driver.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return
        idx  = self.diag_joint_cb.current()
        jcfg = self.ctrl.config.joints[idx]
        info = self.ctrl.get_servo_info(jcfg.servo_id)
        live = self.ctrl.live_positions[idx]
        raw  = self.ctrl.live_raw[idx]
        text = (f"Joint       : {JOINT_NAMES[idx]}\n"
                f"Servo ID    : {jcfg.servo_id}\n"
                f"Raw position: {raw}  →  {live if live is not None else '—'}°\n"
                f"Temperature : {info['temperature']} °C\n"
                f"Voltage     : {info['voltage']} (×0.1 V)\n"
                f"Zero offset : {jcfg.zero_offset}")
        self.diag_text.delete("1.0", "end")
        self.diag_text.insert("end", text)
