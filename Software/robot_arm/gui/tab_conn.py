"""Connection tab — serial port or TCP (ESP32 Wi-Fi) selection, scan, config I/O."""

import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import serial.tools.list_ports

from ..constants import JOINT_NAMES, NUM_JOINTS


class ConnTabMixin:
    # ── build ────────────────────────────────────────────────────────────────
    def _build_conn_tab(self):
        f = self.tab_conn
        f.columnconfigure(0, weight=1)
        f.columnconfigure(1, weight=1)
        T = self.T

        # ── Mode selector ────────────────────────────────────────────────────
        mode_frame = tk.Frame(f, bg=T["PANEL"], pady=6)
        mode_frame.grid(row=0, column=0, columnspan=2, padx=12, pady=(12, 0), sticky="ew")

        tk.Label(mode_frame, text="Connection mode:", bg=T["PANEL"],
                 fg=T["SUBTEXT"], font=("Helvetica", 9, "bold")).pack(side="left", padx=10)

        self._conn_mode = tk.StringVar(value="serial")
        ttk.Radiobutton(mode_frame, text="USB Serial",
                        variable=self._conn_mode, value="serial",
                        command=self._on_mode_change).pack(side="left", padx=10)
        ttk.Radiobutton(mode_frame, text="Wi-Fi TCP (ESP32)",
                        variable=self._conn_mode, value="tcp",
                        command=self._on_mode_change).pack(side="left", padx=10)

        # ── Serial panel ─────────────────────────────────────────────────────
        self._serial_frame = ttk.LabelFrame(f, text=" USB Serial ", padding=12)
        self._serial_frame.grid(row=1, column=0, padx=12, pady=8, sticky="nsew")

        tk.Label(self._serial_frame, text="Port:", bg=T["BG"],
                 fg=T["SUBTEXT"]).grid(row=0, column=0, sticky="w")
        self.port_cb = ttk.Combobox(self._serial_frame, width=18, state="readonly")
        self.port_cb.grid(row=0, column=1, padx=6, pady=3)

        tk.Label(self._serial_frame, text="Baud:", bg=T["BG"],
                 fg=T["SUBTEXT"]).grid(row=1, column=0, sticky="w")
        self.baud_cb = ttk.Combobox(
            self._serial_frame,
            values=["1000000", "500000", "115200", "57600"],
            width=18, state="readonly")
        self.baud_cb.current(0)
        self.baud_cb.grid(row=1, column=1, padx=6, pady=3)

        bf = tk.Frame(self._serial_frame, bg=T["BG"])
        bf.grid(row=2, column=0, columnspan=2, pady=8)
        ttk.Button(bf, text="↺ Refresh",
                   command=self._refresh_ports).pack(side="left", padx=4)

        # ── TCP panel ────────────────────────────────────────────────────────
        self._tcp_frame = ttk.LabelFrame(f, text=" Wi-Fi TCP (ESP32) ", padding=12)
        self._tcp_frame.grid(row=1, column=0, padx=12, pady=8, sticky="nsew")

        tk.Label(self._tcp_frame, text="IP Address:", bg=T["BG"],
                 fg=T["SUBTEXT"]).grid(row=0, column=0, sticky="w")
        self._tcp_host_var = tk.StringVar(value=self.ctrl.config.tcp_host)
        ttk.Entry(self._tcp_frame, textvariable=self._tcp_host_var,
                  width=18).grid(row=0, column=1, padx=6, pady=3)

        tk.Label(self._tcp_frame, text="TCP Port:", bg=T["BG"],
                 fg=T["SUBTEXT"]).grid(row=1, column=0, sticky="w")
        self._tcp_port_var = tk.IntVar(value=self.ctrl.config.tcp_port)
        ttk.Entry(self._tcp_frame, textvariable=self._tcp_port_var,
                  width=10).grid(row=1, column=1, padx=6, pady=3)

        tk.Label(self._tcp_frame,
                 text="ESP32 must run a raw TCP↔UART bridge\n(WiFiServer forwarding to servo bus)",
                 bg=T["BG"], fg=T["SUBTEXT"],
                 font=("Helvetica", 8, "italic"), justify="left").grid(
                     row=2, column=0, columnspan=2, pady=(4, 0), sticky="w")

        # ── Connect button (shared) ──────────────────────────────────────────
        conn_row = tk.Frame(f, bg=T["BG"])
        conn_row.grid(row=2, column=0, columnspan=2, pady=6)
        self.conn_btn = ttk.Button(conn_row, text="Connect",
                                   style="Green.TButton",
                                   command=self._toggle_connect)
        self.conn_btn.pack(padx=4)

        # ── Robot Config panel ───────────────────────────────────────────────
        lf2 = ttk.LabelFrame(f, text=" Robot Config ", padding=12)
        lf2.grid(row=1, column=1, padx=12, pady=8, sticky="nsew")
        ttk.Button(lf2, text="💾  Save Config",
                   command=self._save_config).pack(fill="x", pady=3)
        ttk.Button(lf2, text="📂  Load Config",
                   command=self._load_config).pack(fill="x", pady=3)

        # ── Servo Scanner ────────────────────────────────────────────────────
        lf3 = ttk.LabelFrame(f, text=" Servo Scanner ", padding=12)
        lf3.grid(row=3, column=0, columnspan=2, padx=12, pady=4, sticky="nsew")
        f.rowconfigure(3, weight=1)

        st = tk.Frame(lf3, bg=T["BG"])
        st.pack(fill="x")
        ttk.Button(st, text="🔍  Scan IDs 1–20",
                   command=self._scan_servos).pack(side="left")
        self.scan_result = tk.Text(lf3, height=6, bg=T["ENTRY"], fg=T["GREEN"],
                                   font=("Courier", 9), relief="flat")
        self.scan_result.pack(fill="both", expand=True, pady=6)

        # show the correct panel for the initial mode
        self._on_mode_change()

    # ── mode switching ────────────────────────────────────────────────────────
    def _on_mode_change(self):
        if self._conn_mode.get() == "serial":
            self._tcp_frame.grid_remove()
            self._serial_frame.grid()
        else:
            self._serial_frame.grid_remove()
            self._tcp_frame.grid()

    # ── callbacks ────────────────────────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports:
            self.port_cb.current(0)

    def _toggle_connect(self):
        if self.ctrl.driver.connected:
            self.ctrl.disconnect()
            self.conn_btn.configure(text="Connect", style="Green.TButton")
            self.status_label.configure(text="● DISCONNECTED", fg=self.T["ORANGE"])
            self._log("Disconnected.")
            return

        mode = self._conn_mode.get()
        if mode == "serial":
            port = self.port_cb.get()
            baud = int(self.baud_cb.get())
            if not port:
                messagebox.showerror("Error", "Select a serial port first.")
                return
            ok  = self.ctrl.connect(port, baud)
            lbl = f"● CONNECTED  {port}"
            msg = f"Connected to {port} @ {baud}"
        else:
            host = self._tcp_host_var.get().strip()
            port = self._tcp_port_var.get()
            if not host:
                messagebox.showerror("Error", "Enter the ESP32 IP address.")
                return
            ok  = self.ctrl.connect_tcp(host, port)
            lbl = f"● CONNECTED  {host}:{port}  (TCP)"
            msg = f"Connected to {host}:{port} via TCP"

        if ok:
            self.conn_btn.configure(text="Disconnect", style="Danger.TButton")
            self.status_label.configure(text=lbl, fg=self.T["GREEN"])
            self._log(msg)
        else:
            dest = f"{self.port_cb.get()}" if mode == "serial" \
                   else f"{self._tcp_host_var.get()}:{self._tcp_port_var.get()}"
            messagebox.showerror("Connection failed", f"Could not connect to {dest}")

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
            self.root.after(0, lambda: self.scan_result.insert("end", result + "\n"))

        threading.Thread(target=scan, daemon=True).start()

    # ── persistence ───────────────────────────────────────────────────────────
    def _save_config(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".json", filetypes=[("JSON", "*.json")])
        if path:
            for i in range(NUM_JOINTS):
                self._sync_id(i)
                self._sync_speed_acc(i)
                self._sync_limits(i)
            # persist TCP fields too
            self.ctrl.config.tcp_host = self._tcp_host_var.get().strip()
            self.ctrl.config.tcp_port = self._tcp_port_var.get()
            self.ctrl.save_config(path)
            self._log(f"Config saved: {path}")

    def _load_config(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if path:
            self.ctrl.load_config(path)
            for i, jcfg in enumerate(self.ctrl.config.joints[:NUM_JOINTS]):
                self.id_vars[i].set(jcfg.servo_id)
                self.speed_vars[i].set(jcfg.speed)
                self.acc_vars[i].set(jcfg.acc)
                self.min_angle_vars[i].set(jcfg.min_angle)
                self.max_angle_vars[i].set(jcfg.max_angle)
                self.home_angle_vars[i].set(jcfg.home_angle)
            self._tcp_host_var.set(self.ctrl.config.tcp_host)
            self._tcp_port_var.set(self.ctrl.config.tcp_port)
            self._log(f"Config loaded: {path}")
