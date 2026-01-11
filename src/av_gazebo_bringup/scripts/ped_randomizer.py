#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import random
import time

# === CONFIGURATION ===
NUM_PEDESTRIANS = 20       # How many people to spawn
AREA_X_MIN = -300.0        # Range near your car
AREA_X_MAX =  300.0
AREA_Y_MIN = -10.0         # Road width range
AREA_Y_MAX = 10.0
# =====================

class PedestrianManager(Node):
    def __init__(self):
        super().__init__('pedestrian_manager')
        
        # 1. Generate URDF Description
        pkg_share = get_package_share_directory('av_gazebo_bringup')
        xacro_file = os.path.join(pkg_share, 'urdf', 'pedestrian.xacro')
        doc = xacro.process_file(xacro_file)
        self.robot_desc = doc.toxml()

        # 2. Setup Spawner Client
        self.spawner = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawner.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        # 3. Spawn Pedestrians
        self.publishers_ = []
        self.spawn_pedestrians()

        # 4. Timer to update movement every 2 seconds
        self.timer = self.create_timer(2.0, self.move_pedestrians)

    def spawn_pedestrians(self):
        for i in range(NUM_PEDESTRIANS):
            name = f'pedestrian_{i}'
            namespace = f'ped_{i}'
            
            # Random Pose
            x = random.uniform(AREA_X_MIN, AREA_X_MAX)
            y = random.uniform(AREA_Y_MIN, AREA_Y_MAX)
            
            req = SpawnEntity.Request()
            req.name = name
            req.xml = self.robot_desc
            req.robot_namespace = namespace
            req.initial_pose.position.x = x
            req.initial_pose.position.y = y
            req.initial_pose.position.z = 0.2
            
            future = self.spawner.call_async(req)
            # We don't wait for result to avoid blocking main thread too long
            
            # Create publisher for this pedestrian
            # Topic: /ped_0/cmd_vel
            pub = self.create_publisher(Twist, f'/{namespace}/cmd_vel', 10)
            self.publishers_.append(pub)
            
            self.get_logger().info(f'Spawning {name} at ({x:.1f}, {y:.1f})')
            time.sleep(0.2) # Small delay to prevent service overload

    def move_pedestrians(self):
        # Random Walk Logic
        for pub in self.publishers_:
            msg = Twist()
            
            # 70% chance to move, 30% chance to stop
            if random.random() > 0.1:
                msg.linear.x = random.uniform(1, 2.5) # Walk forward speed
                msg.angular.z = random.uniform(-2.0, 2.0) # Turn speed
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0

            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PedestrianManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()