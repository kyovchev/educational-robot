#!/usr/bin/env python3
"""
6DOF Robot Arm Controller — Feetech STS3215
Entry point: run this file to launch the GUI.
"""

import tkinter as tk
from robot_arm.gui.app import RobotGUI


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
