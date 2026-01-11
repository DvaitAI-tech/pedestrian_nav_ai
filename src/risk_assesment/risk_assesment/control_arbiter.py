#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class ControlArbiter(Node):
    def __init__(self):
        super().__init__('control_arbiter')

        # --- State ---
        self.latest_manual = Twist()
        self.decision = "GO"
        self.last_logged_decision = None

        # --- Subscribers ---
        self.create_subscription(
            Twist,
            '/ego/cmd_vel_manual',
            self.manual_callback,
            10
        )

        self.create_subscription(
            String,
            '/av/decision_state',
            self.decision_callback,
            10
        )

        # --- Publisher ---
        self.pub_cmd = self.create_publisher(
            Twist,
            '/ego/cmd_vel',
            10
        )

        # Timer
        self.create_timer(0.05, self.publish_control)

        self.get_logger().info("🧠 Control Arbiter Started (Decision Override Enabled)")

    def manual_callback(self, msg: Twist):
        self.latest_manual = msg

    def decision_callback(self, msg: String):
        self.decision = msg.data.split('|')[0]
        print(f"Received Decision: {self.decision}")

    def publish_control(self):
        cmd = Twist()

        # Always allow steering
        cmd.angular = self.latest_manual.angular

        # --- Decision Logic ---
        if self.decision == "STOP":
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0

        elif self.decision == "SLOW":
            cmd.linear.y = min(0.7, self.latest_manual.linear.y)

        else:  # GO
            cmd.linear = self.latest_manual.linear

        self.pub_cmd.publish(cmd)

        # --- LOGGING (only when decision changes) ---
        if self.decision != self.last_logged_decision:
            self.get_logger().warn(
                f"DECISION = {self.decision} | "
                f"manual_v = {self.latest_manual.linear.y:.2f} m/s | "
                f"final_v = {cmd.linear.y:.2f} m/s"
            )
            self.last_logged_decision = self.decision


def main(args=None):
    rclpy.init(args=args)
    node = ControlArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
