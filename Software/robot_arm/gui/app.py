"""Main GUI class — composes all tab mixins, handles theme and top-level layout."""

import tkinter as tk
from tkinter import ttk

from ..constants import JOINT_NAMES, THEMES, NUM_JOINTS
from ..controller import RobotController
from .tab_conn import ConnTabMixin
from .tab_ctrl import CtrlTabMixin
from .tab_setup import SetupTabMixin
from .tab_seq import SeqTabMixin
from .tab_cartesian import CartesianTabMixin


class RobotGUI(ConnTabMixin, CtrlTabMixin, SetupTabMixin, SeqTabMixin, CartesianTabMixin):

    def __init__(self, root: tk.Tk, urdf_path: str = "robot.urdf"):
        self.root = root
        self.ctrl = RobotController(urdf_path=urdf_path)
        self.ctrl.on_position_update = self._on_position_update

        self._teach_mode = False
        self._theme_name = "dark"
        self.T = dict(THEMES["dark"])

        # per-joint Tk variables (shared across all tabs)
        self.pos_vars:        list = [tk.StringVar(
            value="—") for _ in range(NUM_JOINTS)]
        self.target_vars:     list = [tk.DoubleVar(
            value=0.0) for _ in range(NUM_JOINTS)]
        self.id_vars:         list = [
            tk.IntVar(value=i + 1) for i in range(NUM_JOINTS)]
        self.speed_vars:      list = [
            tk.IntVar(value=1000) for _ in range(NUM_JOINTS)]
        self.acc_vars:        list = [
            tk.IntVar(value=200) for _ in range(NUM_JOINTS)]
        self.min_angle_vars:  list = [tk.DoubleVar(
            value=-130.0) for _ in range(NUM_JOINTS)]
        self.max_angle_vars:  list = [tk.DoubleVar(
            value=130.0) for _ in range(NUM_JOINTS)]
        self.home_angle_vars: list = [tk.DoubleVar(
            value=0.0) for _ in range(NUM_JOINTS)]

        self._build_ui()
        self.root.after(200, self._refresh_ports)

    # ─────────────────────────────────────────────────────────────────────────
    #  Theme
    # ─────────────────────────────────────────────────────────────────────────
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
            "TNotebook.Tab",     background=T["ACCENT"],     foreground=T["TEXT"], padding=[12, 4])
        style.map("TNotebook.Tab",           background=[
                  ("selected", T["ORANGE"])])
        style.configure("TFrame",            background=T["BG"])
        style.configure(
            "TLabelframe",       background=T["BG"],         foreground=T["TEXT"],    bordercolor=T["ACCENT"])
        style.configure("TLabelframe.Label",
                        background=T["BG"],         foreground=T["GREEN"])
        style.configure("TButton",           background=T["ACCENT"],
                        foreground=T["TEXT"],    borderwidth=0, relief="flat", padding=[8, 4])
        style.map("TButton",                 background=[
                  ("active", T["ORANGE"]), ("pressed", T["ORANGE"])])
        style.configure("Danger.TButton",
                        background=T["DANGER_BTN"], foreground=T["TEXT"])
        style.map("Danger.TButton",          background=[
                  ("active", T["ORANGE"])])
        style.configure("Green.TButton",
                        background=T["GREEN_BTN"],  foreground=T["TEXT"])
        style.map("Green.TButton",           background=[
                  ("active", T["GREEN"])])
        style.configure(
            "TScale",            background=T["BG"],         troughcolor=T["ACCENT"])
        style.configure("TCombobox",
                        fieldbackground=T["ENTRY"], foreground=T["TEXT"])
        style.configure(
            "TEntry",            fieldbackground=T["ENTRY"], foreground=T["TEXT"])
        style.configure("TCheckbutton",
                        background=T["BG"],         foreground=T["TEXT"])
        style.configure("TSeparator",        background=T["ACCENT"])

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
        try:
            self.header_frame.configure(bg=T["PANEL"])
            self.topbar_frame.configure(bg=T["ORANGE"])
            self.sb_frame.configure(bg=T["PANEL"])
            self.status_label.configure(bg=T["PANEL"])
            self.sb_label.configure(bg=T["PANEL"], fg=T["SUBTEXT"])
            if hasattr(self, "theme_btn"):
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
        conn_fg = self.T["GREEN"] if self.ctrl.driver.connected else self.T["ORANGE"]
        self.status_label.configure(fg=conn_fg)
        if self._teach_mode:
            self.teach_status.configure(fg=self.T["ORANGE"])

    # ─────────────────────────────────────────────────────────────────────────
    #  Top-level UI construction
    # ─────────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        T = self.T
        self.root.title("6DOF Robot Arm Controller  •  Feetech STS3215")
        self.root.configure(bg=T["BG"])
        self.root.resizable(True, True)
        self.root.minsize(920, 720)
        self._apply_theme()

        # top accent bar
        self.topbar_frame = tk.Frame(self.root, bg=T["ORANGE"], height=4)
        self.topbar_frame.pack(fill="x")

        # header
        self.header_frame = tk.Frame(self.root, bg=T["PANEL"], pady=8)
        self.header_frame.pack(fill="x", padx=4, pady=(0, 4))

        tk.Label(self.header_frame, text="⚙  6DOF Robot Arm Controller",
                 bg=T["PANEL"], fg=T["TEXT"],
                 font=("Helvetica", 16, "bold")).pack(side="left", padx=16)

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

        # notebook tabs
        self.nb = ttk.Notebook(self.root)
        self.nb.pack(fill="both", expand=True, padx=6, pady=4)

        self.tab_conn = ttk.Frame(self.nb)
        self.nb.add(self.tab_conn,      text="  🔌 Connection  ")
        self.tab_ctrl = ttk.Frame(self.nb)
        self.nb.add(self.tab_ctrl,      text="  🕹 Control  ")
        self.tab_cartesian = ttk.Frame(self.nb)
        self.nb.add(self.tab_cartesian, text="  🎯 Cartesian  ")
        self.tab_seq = ttk.Frame(self.nb)
        self.nb.add(self.tab_seq,       text="  📋 Sequence  ")
        self.tab_setup = ttk.Frame(self.nb)
        self.nb.add(self.tab_setup,     text="  ⚙ Setup  ")

        self._build_conn_tab()
        self._build_ctrl_tab()
        self._build_cartesian_tab()
        self._build_seq_tab()
        self._build_setup_tab()

        # status bar
        self.sb_frame = tk.Frame(self.root, bg=T["PANEL"], height=26)
        self.sb_frame.pack(fill="x", side="bottom")
        self.log_var = tk.StringVar(value="Ready.")
        self.sb_label = tk.Label(self.sb_frame, textvariable=self.log_var,
                                 bg=T["PANEL"], fg=T["SUBTEXT"],
                                 font=("Courier", 9))
        self.sb_label.pack(side="left", padx=10)

    # ─────────────────────────────────────────────────────────────────────────
    #  Shared helpers
    # ─────────────────────────────────────────────────────────────────────────
    def _log(self, msg: str):
        self.log_var.set(msg)
        print(f"[GUI] {msg}")

    def _emergency_stop(self):
        self.ctrl.stop_sequence()
        if self.ctrl.driver.connected:
            self.ctrl.torque_all(False)
        self.play_btn.configure(state="normal")
        self.seq_progress.configure(text="E-STOP")
        self._log("!!! EMERGENCY STOP — torque disabled on all joints !!!")
