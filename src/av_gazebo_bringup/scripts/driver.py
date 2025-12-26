#!/usr/bin/env python3
import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Driver(Node):
    def __init__(self):
        super().__init__('manual_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def drive(self, linear_x=0.5, angular_z=0.0, duration=2.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        end = time.time() + duration
        while time.time() < end and rclpy.ok():
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)

def main():
    rclpy.init()
    d = Driver()
    # example: go forward for 4s while slight turn
    d.drive(1.0, 0.05, 4.0)
    d.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
