"""Sequence tab — waypoint list, editor, playback, file I/O."""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

from ..constants import JOINT_NAMES, NUM_JOINTS
from ..models import Waypoint


class SeqTabMixin:
    # ── build ────────────────────────────────────────────────────────────────
    def _build_seq_tab(self):
        f = self.tab_seq
        f.columnconfigure(0, weight=1)
        f.columnconfigure(1, weight=3)
        f.rowconfigure(0, weight=1)
        T = self.T

        # ── Waypoint list ────────────────────────────────────────────────────
        lf = ttk.LabelFrame(f, text=" Waypoints ", padding=6)
        lf.grid(row=0, column=0, padx=8, pady=8, sticky="nsew")

        self.wp_list = tk.Listbox(
            lf, bg=T["ENTRY"], fg=T["TEXT"],
            selectbackground=T["SEL_BG"],
            font=("Courier", 10), relief="flat", activestyle="none")
        self.wp_list.pack(fill="both", expand=True)
        self.wp_list.bind("<<ListboxSelect>>", self._on_wp_select)

        br = tk.Frame(lf, bg=T["BG"])
        br.pack(fill="x", pady=4)
        ttk.Button(br, text="↑", width=3, command=self._wp_up).pack(side="left", padx=2)
        ttk.Button(br, text="↓", width=3, command=self._wp_down).pack(side="left", padx=2)
        ttk.Button(br, text="✕", width=3, style="Danger.TButton",
                   command=self._wp_delete).pack(side="left", padx=2)

        # ── Waypoint editor ──────────────────────────────────────────────────
        lf2 = ttk.LabelFrame(f, text=" Waypoint Editor ", padding=10)
        lf2.grid(row=0, column=1, padx=8, pady=8, sticky="nsew")

        self.wp_name_var = tk.StringVar()
        self.wp_dur_var  = tk.DoubleVar(value=1.0)

        tk.Label(lf2, text="Name:",         bg=T["BG"], fg=T["SUBTEXT"]).grid(
            row=0, column=0, sticky="w")
        ttk.Entry(lf2, textvariable=self.wp_name_var, width=18).grid(
            row=0, column=1, padx=6, pady=3)
        tk.Label(lf2, text="Duration (s):", bg=T["BG"], fg=T["SUBTEXT"]).grid(
            row=1, column=0, sticky="w")
        ttk.Entry(lf2, textvariable=self.wp_dur_var, width=10).grid(
            row=1, column=1, padx=6, pady=3)

        self.wp_angle_vars: list = [tk.DoubleVar() for _ in range(NUM_JOINTS)]
        for i, name in enumerate(JOINT_NAMES):
            r = i + 2
            tk.Label(lf2, text=f"{name}:", bg=T["BG"], fg=T["SUBTEXT"],
                     width=12, anchor="w").grid(row=r, column=0, sticky="w")
            ttk.Entry(lf2, textvariable=self.wp_angle_vars[i], width=10).grid(
                row=r, column=1, padx=6, pady=2)

        br2 = tk.Frame(lf2, bg=T["BG"])
        br2.grid(row=NUM_JOINTS + 2, column=0, columnspan=2, pady=8)
        ttk.Button(br2, text="💾 Update",  command=self._wp_update).pack(
            side="left", padx=4)
        ttk.Button(br2, text="➕ Add New", style="Green.TButton",
                   command=self._wp_add_new).pack(side="left", padx=4)
        ttk.Button(br2, text="▶ Go To",   command=self._wp_goto).pack(
            side="left", padx=4)

        # ── Playback bar ─────────────────────────────────────────────────────
        pb = ttk.LabelFrame(f, text=" Playback ", padding=10)
        pb.grid(row=1, column=0, columnspan=2, padx=8, pady=4, sticky="ew")

        self.loop_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(pb, text="Loop", variable=self.loop_var).pack(
            side="left", padx=6)
        self.play_btn = ttk.Button(pb, text="▶  Play Sequence",
                                   style="Green.TButton",
                                   command=self._play_sequence)
        self.play_btn.pack(side="left", padx=6)
        ttk.Button(pb, text="⏹  Stop", style="Danger.TButton",
                   command=self._stop_sequence).pack(side="left", padx=6)

        self.seq_progress = tk.Label(pb, text="", bg=T["BG"], fg=T["SUBTEXT"],
                                     font=("Helvetica", 9))
        self.seq_progress.pack(side="left", padx=10)

        ttk.Separator(pb, orient="vertical").pack(side="left", fill="y", padx=8)
        ttk.Button(pb, text="💾 Save Sequence",
                   command=self._save_sequence).pack(side="left", padx=4)
        ttk.Button(pb, text="📂 Load Sequence",
                   command=self._load_sequence).pack(side="left", padx=4)

    # ── list helpers ─────────────────────────────────────────────────────────
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

    # ── editor callbacks ──────────────────────────────────────────────────────
    def _wp_update(self):
        sel = self.wp_list.curselection()
        if not sel:
            return
        wp = self.ctrl.sequence[sel[0]]
        wp.name     = self.wp_name_var.get()
        wp.duration = self.wp_dur_var.get()
        wp.angles   = [v.get() for v in self.wp_angle_vars]
        self._refresh_wp_list()
        self.wp_list.selection_set(sel[0])
        self._log(f"Updated '{wp.name}'")

    def _wp_add_new(self):
        angles = [v.get() for v in self.wp_angle_vars]
        wp = Waypoint(
            name=self.wp_name_var.get() or f"pose_{len(self.ctrl.sequence)+1}",
            angles=angles,
            duration=self.wp_dur_var.get(),
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
        self.ctrl.sequence[i], self.ctrl.sequence[i - 1] = \
            self.ctrl.sequence[i - 1], self.ctrl.sequence[i]
        self._refresh_wp_list()
        self.wp_list.selection_set(i - 1)

    def _wp_down(self):
        sel = self.wp_list.curselection()
        if not sel or sel[0] >= len(self.ctrl.sequence) - 1:
            return
        i = sel[0]
        self.ctrl.sequence[i], self.ctrl.sequence[i + 1] = \
            self.ctrl.sequence[i + 1], self.ctrl.sequence[i]
        self._refresh_wp_list()
        self.wp_list.selection_set(i + 1)

    def _wp_goto(self):
        if not self.ctrl.driver.connected:
            self._log("Not connected"); return
        sel = self.wp_list.curselection()
        if sel:
            wp = self.ctrl.sequence[sel[0]]
            self.ctrl.set_all_angles(wp.angles)
            self._log(f"Moved to '{wp.name}'")
        else:
            angles = [v.get() for v in self.wp_angle_vars]
            self.ctrl.set_all_angles(angles)

    # ── playback callbacks ───────────────────────────────────────────────────
    def _play_sequence(self):
        if not self.ctrl.driver.connected:
            messagebox.showwarning("Not connected", "Connect first."); return
        if not self.ctrl.sequence:
            messagebox.showwarning("Empty", "No waypoints in sequence."); return

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

    # ── file I/O ──────────────────────────────────────────────────────────────
    def _save_sequence(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".json", filetypes=[("JSON", "*.json")])
        if path:
            self.ctrl.save_sequence(path)
            self._log(f"Sequence saved ({len(self.ctrl.sequence)} waypoints): {path}")

    def _load_sequence(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if path:
            self.ctrl.load_sequence(path)
            self._refresh_wp_list()
            self._log(f"Sequence loaded: {len(self.ctrl.sequence)} waypoints")
