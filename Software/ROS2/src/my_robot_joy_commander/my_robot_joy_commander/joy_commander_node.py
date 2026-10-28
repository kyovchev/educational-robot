#!/usr/bin/env python3
"""Joystick teleop that talks to the "commander" node and the gripper node through topics.

Publishes  : joint_command (JointsCommand), pose_command (PoseCommand),
             gripper_position (std_msgs/Int32)
Subscribes : is_moving (std_msgs/Bool), joy (sensor_msgs/Joy)
Current end-effector position is read from TF (base_frame -> ee_frame).

Controls:
  L1 (button 6)            toggle Start / Home
  R1 (button 7) + sticks   nudge the arm (axes 1, 0, 2 -> x, y, z)
  Button 0 + axis 1        open / close the gripper (arm nudging is suspended)
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int32
from my_robot_interfaces.msg import JointsCommand, PoseCommand

import tf2_ros
from tf2_ros import TransformException


class JoyCommanderNode(Node):
    def __init__(self):
        super().__init__("joy_commander_node")

        # ---- parameters ----
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.ee_frame = self.declare_parameter("ee_frame", "tool0").value
        self.step = self.declare_parameter("step", 0.01).value
        self.deadzone = self.declare_parameter("deadzone", 0.3).value
        self.home_joints = list(
            self.declare_parameter("home_joints", [0.0] * 6).value  # adjust to your "Home"
        )
        self.start_x = self.declare_parameter("start_x", 0.12).value
        self.start_y = self.declare_parameter("start_y", 0.00).value
        self.start_z = self.declare_parameter("start_z", 0.05).value
        self.fixed_roll = self.declare_parameter("roll", 3.14).value
        self.fixed_pitch = self.declare_parameter("pitch", 0.0).value
        self.fixed_yaw = self.declare_parameter("yaw", 0.0).value

        # Gripper: range must match the gripper node's min_value / max_value
        self.gripper_min = int(self.declare_parameter("gripper_min", 0).value)
        self.gripper_max = int(self.declare_parameter("gripper_max", 100).value)
        # Change per joy message at full stick deflection. Negative value flips direction.
        self.gripper_step = float(self.declare_parameter("gripper_step", 3.0).value)
        # Assumed starting value; should match the gripper node's initial_value
        self.gripper_pos = float(self.declare_parameter("gripper_initial", 50).value)

        # ---- state ----
        self.is_moving = False
        self.awaiting_start = False
        self.at_start_pose = False
        self.l1_prev = False
        self.last_sent = self.get_clock().now()
        self.gripper_last_sent = int(round(self.gripper_pos))

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- pub / sub ----
        self.joint_pub = self.create_publisher(JointsCommand, "joint_command", 1)
        self.pose_pub = self.create_publisher(PoseCommand, "pose_command", 1)
        self.gripper_pub = self.create_publisher(Int32, "gripper_position", 1)

        # QoS must match the commander's transient_local publisher
        moving_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.moving_sub = self.create_subscription(
            Bool, "is_moving", self.moving_callback, moving_qos
        )
        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 1)

        # Go to the start pose once, shortly after startup (lets discovery finish)
        self.init_timer = self.create_timer(1.0, self.init_callback)

    # ---------- helpers ----------
    def busy(self) -> bool:
        if self.is_moving:
            return True
        # Command was sent but the commander has not reported "moving" yet.
        # Timeout covers the case where planning failed and it never starts.
        if self.awaiting_start:
            if (self.get_clock().now() - self.last_sent) < Duration(seconds=2.0):
                return True
            self.awaiting_start = False
        return False

    def mark_sent(self):
        self.awaiting_start = True
        self.last_sent = self.get_clock().now()

    def make_pose(self, x, y, z, cartesian) -> PoseCommand:
        cmd = PoseCommand()
        cmd.x = float(x)
        cmd.y = float(y)
        cmd.z = float(z)
        cmd.roll = float(self.fixed_roll)
        cmd.pitch = float(self.fixed_pitch)
        cmd.yaw = float(self.fixed_yaw)
        cmd.cartesian_path = bool(cartesian)
        return cmd

    def send_start_pose(self):
        self.pose_pub.publish(
            self.make_pose(self.start_x, self.start_y, self.start_z, False)
        )
        self.at_start_pose = True
        self.mark_sent()
        self.get_logger().info("Sent: Start pose")

    def send_home(self):
        if len(self.home_joints) != 6:
            self.get_logger().error("home_joints must have 6 values")
            return
        cmd = JointsCommand()
        cmd.data = [float(j) for j in self.home_joints]
        self.joint_pub.publish(cmd)
        self.at_start_pose = False
        self.mark_sent()
        self.get_logger().info("Sent: Home joints")

    def handle_gripper(self, axis: float):
        """Integrate the stick deflection into a gripper position and publish it."""
        if abs(axis) <= self.deadzone:
            return
        self.gripper_pos += axis * self.gripper_step
        self.gripper_pos = max(
            float(self.gripper_min), min(float(self.gripper_max), self.gripper_pos)
        )
        value = int(round(self.gripper_pos))
        if value == self.gripper_last_sent:
            return  # nothing new (e.g. already at the limit)
        msg = Int32()
        msg.data = value
        self.gripper_pub.publish(msg)
        self.gripper_last_sent = value
        self.get_logger().info(f"Gripper -> {value}")

    # ---------- callbacks ----------
    def init_callback(self):
        self.init_timer.cancel()
        self.get_logger().info(
            "Ready. L1: Home/Start toggle, hold R1 to move, hold button 0 + axis 1 for gripper."
        )
        self.send_start_pose()

    def moving_callback(self, msg: Bool):
        self.is_moving = msg.data
        if msg.data:
            self.awaiting_start = False

    def joy_callback(self, msg: Joy):
        if len(msg.buttons) < 8 or len(msg.axes) < 3:
            return

        l1 = msg.buttons[6] == 1
        r1 = msg.buttons[7] == 1
        gripper_btn = msg.buttons[0] == 1

        # --- Toggle (L1), edge-triggered so holding it doesn't re-fire ---
        l1_pressed = l1 and not self.l1_prev
        self.l1_prev = l1

        # --- Gripper (button 0 held + axis 1) ---
        # Independent of the arm (works while it moves) and takes priority over
        # the arm nudge, since axis 1 is also used for X.
        if gripper_btn:
            self.handle_gripper(msg.axes[1])
            return

        if self.busy():
            return

        if l1_pressed:
            if self.at_start_pose:
                self.send_home()
            else:
                self.send_start_pose()
            return

        # --- Nudge (R1 held) ---
        if not r1:
            return

        ax, ay, az = msg.axes[1], msg.axes[0], msg.axes[2]
        mx = abs(ax) > self.deadzone
        my = abs(ay) > self.deadzone
        mz = abs(az) > self.deadzone
        if not (mx or my or mz):
            return

        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=2.0)
            return

        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z

        if mx:
            x += ax * self.step
        if my:
            y += ay * self.step
        if mz:
            z += az * self.step

        self.get_logger().info(f"Target position: x={x:.3f}, y={y:.3f}, z={z:.3f}")

        # Small straight-line nudge -> cartesian path
        self.pose_pub.publish(self.make_pose(x, y, z, True))
        self.at_start_pose = False
        self.mark_sent()


def main(args=None):
    rclpy.init(args=args)
    node = JoyCommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()