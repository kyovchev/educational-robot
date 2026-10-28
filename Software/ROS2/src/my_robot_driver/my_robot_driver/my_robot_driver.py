"""
sts3215_ros2_driver.py
======================
ROS 2 node that bridges the STS3215 servo robot to the same topic interface
used by the Webots simulation controller.

  SUBSCRIBES  : /my_robot_cmd   (std_msgs/Float32MultiArray)
                  data[0]       = enable flag  (1 = active)
                  data[1..6]    = joint0..joint5 position commands (radians)

  PUBLISHES   : /joint0 .. /joint5  (std_msgs/Float32, radians)

Usage
-----
  ros2 run <your_pkg> sts3215_ros2_driver \
      --ros-args -p serial_port:=/dev/ttyUSB0 -p baud:=1000000

Parameters
----------
  serial_port     : serial device      (default "/dev/ttyUSB0")
  baud            : serial baud rate   (default 1000000)
  publish_rate_hz : state publish rate (default 50.0)
  cmd_timeout_sec : seconds before a stale command warning (default 1.0)
  urdf_path       : path to robot URDF (default "robot.urdf")
  config          : path to JSON config file (default "")
"""

import math
import time
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

from .sts_robot_api import StsRobotAPI

NUM_JOINTS = 6


class MyRobotDriver(Node):

    def __init__(self):
        super().__init__("sts3215_ros2_driver")

        # ── parameters ───────────────────────────────────────────────────
        self.declare_parameter("serial_port",     "/dev/ttyACM0")
        self.declare_parameter("baud",            1_000_000)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("cmd_timeout_sec", 1.0)

        serial_port      = self.get_parameter("serial_port").value
        baud             = self.get_parameter("baud").value
        pub_rate         = self.get_parameter("publish_rate_hz").value
        self._cmd_timeout = self.get_parameter("cmd_timeout_sec").value

        # ── state ────────────────────────────────────────────────────────
        self._last_cmd_time: float = 0.0

        # ── connect to real robot ─────────────────────────────────────────
        self._robot = StsRobotAPI(
            port=serial_port,
            baud=baud,
            urdf_path="",
            config="",
            is_radian=False,   # API works in degrees internally; we convert
        )
        self._robot.motion_enable(enable=True)
        self._robot.set_mode(0)
        self._robot.set_state(state=0)

        # ── publishers — one Float32 topic per joint ──────────────────────
        self._joint_pubs = [
            self.create_publisher(Float32, f"/joint{i}_sensor", 1)
            for i in range(NUM_JOINTS)
        ]

        # ── command subscriber ────────────────────────────────────────────
        self.create_subscription(
            Float32MultiArray,
            "/my_robot_cmd",
            self._cmd_callback,
            1,
        )

        # ── publish timer ─────────────────────────────────────────────────
        self.create_timer(1.0 / pub_rate, self._publish_state)

        self.get_logger().info(
            f"STS3215 ROS 2 driver ready on {serial_port} @ {baud} baud"
        )

    # ── command callback ─────────────────────────────────────────────────────
    def _cmd_callback(self, msg: Float32MultiArray):
        """
        data[0]   = enable flag (1 = move, 0 = hold)
        data[1-6] = joint0..joint5 target positions in radians
        """
        if len(msg.data) < NUM_JOINTS + 1:
            self.get_logger().warn(
                f"Command has {len(msg.data)} values, expected {NUM_JOINTS + 1}. "
                "Ignoring."
            )
            return

        self._last_cmd_time = time.monotonic()

        if not msg.data[0]:  # enable flag off — don't move
            return

        # Convert radians → degrees for the StsRobotAPI
        angles_deg: List[float] = [
            math.degrees(msg.data[i + 1]) for i in range(NUM_JOINTS)
        ]

        code = self._robot.set_servo_angle(
            angle=angles_deg,
            is_radian=False,
            wait=False,
        )
        if code != 0:
            self.get_logger().warn(f"set_servo_angle returned code {code}")

    # ── publish current joint positions ──────────────────────────────────────
    def _publish_state(self):
        # get_servo_angle() reads self._robot._live_positions which is kept
        # fresh by the driver's background polling thread (_poll_loop).
        # Returns degrees; we convert to radians for ros2_control.
        code, angles_deg = self._robot.get_servo_angle(is_radian=False)

        if code != 0:
            self.get_logger().warn(
                "get_servo_angle failed", throttle_duration_sec=2.0
            )
            return

        for i, pub in enumerate(self._joint_pubs):
            msg = Float32()
            msg.data = math.radians(angles_deg[i])
            pub.publish(msg)

        # stale command warning
        if self._last_cmd_time and self._cmd_timeout > 0:
            age = time.monotonic() - self._last_cmd_time
            if age > self._cmd_timeout:
                self.get_logger().warn(
                    f"No command for {age:.1f}s",
                    throttle_duration_sec=2.0,
                )

    # ── cleanup ──────────────────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info("Shutting down — disabling servos.")
        try:
            self._robot.motion_enable(enable=False)
            self._robot.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MyRobotDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()