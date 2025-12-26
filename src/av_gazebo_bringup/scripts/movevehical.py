#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# === SETTINGS ===
SPEED = 2.0        # Meters per second
TURN_SPEED = 1.0   # Radians per second

# Instructions printed to screen
msg = """
Control Your Vehicle!
---------------------------
Moving around:
        W
   A    S    D

Space: FORCE STOP
w/s : increase/decrease linear speed
a/d : increase/decrease angular speed

CTRL-C to quit
"""

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        
        # Publisher to /ego/cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/ego/cmd_vel', 10)
        self.get_logger().info('Keyboard Controller Started')

    def publish_velocity(self, linear_y, angular_z):
        twist = Twist()
        
        # NOTE: Using linear.y because your robot moves forward on Y
        # Standard ROS robots usually use linear.x
        twist.linear.x = 0.0
        twist.linear.y = float(linear_y)
        twist.linear.z = 0.0
        
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(angular_z)
        
        self.publisher_.publish(twist)

def getKey(settings):
    # Reads a single key press from stdin without requiring "Enter"
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    
    settings = termios.tcgetattr(sys.stdin)
    
    target_linear_vel = 0.0
    target_angular_vel = 0.0

    try:
        print(msg)
        while True:
            key = getKey(settings)
            
            # === MOVEMENT LOGIC ===
            if key == 'w':
                target_linear_vel = SPEED
                print(f"Forward (Speed: {SPEED})")
            elif key == 's':
                target_linear_vel = -SPEED
                print(f"Backward (Speed: {-SPEED})")
            elif key == 'a':
                target_angular_vel = TURN_SPEED
                print("Turning Left")
            elif key == 'd':
                target_angular_vel = -TURN_SPEED
                print("Turning Right")
            elif key == ' ':
                target_linear_vel = 0.0
                target_angular_vel = 0.0
                print("STOPPING")
            elif key == '\x03': # CTRL+C
                break

            # Publish the command
            node.publish_velocity(target_linear_vel, target_angular_vel)

            # Reset angular velocity immediately (so it doesn't spin forever)
            # Remove this line if you want continuous turning
            target_angular_vel = 0.0 

    except Exception as e:
        print(e)

    finally:
        # Stop the robot before exiting
        node.publish_velocity(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()