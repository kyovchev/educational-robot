#!/usr/bin/env python3
"""ROS 2 node controlling a 9g (SG90-type) servo gripper on a Raspberry Pi 4.

Uses the pigpio daemon (pigpiod) for DMA-timed servo pulses, so the signal stays
steady even under CPU load (camera, MoveIt, web API ...).

Subscribes : /gripper_position (std_msgs/Int32) - plain integer position command.
             The integer range [min_value, max_value] (default 0..100) is mapped
             linearly onto the servo travel. Values outside the range are clamped.

Requires   : pigpiod running (sudo systemctl enable --now pigpiod) and the
             pigpio Python client (python3-pigpio or `pip install pigpio`).
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Int32
import pigpio


class GripperNode(Node):
    def __init__(self):
        super().__init__("gripper_node")

        # ---- parameters ----
        self.pin = int(self.declare_parameter("gpio_pin", 12).value)      # BCM numbering
        self.min_pulse_us = float(self.declare_parameter("min_pulse_ms", 0.5).value) * 1000.0
        self.max_pulse_us = float(self.declare_parameter("max_pulse_ms", 2.5).value) * 1000.0

        # Range of the incoming integer command
        self.min_value = int(self.declare_parameter("min_value", 0).value)
        self.max_value = int(self.declare_parameter("max_value", 100).value)
        if self.max_value <= self.min_value:
            raise ValueError("max_value must be greater than min_value")

        # Usable part of the servo travel as a fraction (0.0 = servo min, 1.0 = servo max).
        # Tighten these so the servo never stalls against the gripper's end stops.
        self.limit_low = float(self.declare_parameter("limit_low", 0.0).value)
        self.limit_high = float(self.declare_parameter("limit_high", 1.0).value)

        initial_value = int(self.declare_parameter("initial_value", 50).value)
        # Stop sending pulses after this many seconds without a new command
        # (stops buzzing / heat, but the servo then holds no torque). 0 disables it.
        self.idle_timeout = float(self.declare_parameter("idle_timeout", 1.5).value)

        # ---- pigpio ----
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError(
                "Cannot connect to pigpiod. Start it with: sudo systemctl enable --now pigpiod"
            )
        self.pi.set_mode(self.pin, pigpio.OUTPUT)

        self.last_cmd_time = self.get_clock().now()
        self.holding = False
        self.set_position(initial_value)

        # ---- topic ----
        self.sub = self.create_subscription(
            Int32, "gripper_position", self.position_callback, 10
        )

        if self.idle_timeout > 0.0:
            self.idle_timer = self.create_timer(0.2, self.idle_check)

        self.get_logger().info(
            f"Gripper ready on GPIO{self.pin} (pigpio), listening on /gripper_position "
            f"(Int32, {self.min_value}..{self.max_value})"
        )

    # ---------- helpers ----------
    def set_position(self, value: int):
        value = max(self.min_value, min(self.max_value, value))
        frac = (value - self.min_value) / (self.max_value - self.min_value)
        frac = self.limit_low + frac * (self.limit_high - self.limit_low)
        pulse = self.min_pulse_us + frac * (self.max_pulse_us - self.min_pulse_us)
        pulse = int(round(max(500.0, min(2500.0, pulse))))  # pigpio servo range
        self.pi.set_servo_pulsewidth(self.pin, pulse)
        self.holding = True
        self.last_cmd_time = self.get_clock().now()
        self.get_logger().info(f"Gripper -> {value} ({pulse} us)")

    def release(self):
        self.pi.set_servo_pulsewidth(self.pin, 0)  # 0 = stop sending pulses
        self.holding = False

    # ---------- callbacks ----------
    def position_callback(self, msg: Int32):
        self.set_position(int(msg.data))

    def idle_check(self):
        if not self.holding:
            return
        idle = self.get_clock().now() - self.last_cmd_time
        if idle > Duration(seconds=self.idle_timeout):
            self.release()

    def shutdown(self):
        try:
            self.release()
            self.pi.stop()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"Servo cleanup failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()