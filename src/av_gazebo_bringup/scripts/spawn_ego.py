#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import os

class Spawner(Node):
    def __init__(self):
        super().__init__('ego_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
    def spawn_from_file(self, name, file_path, x=0.0, y=0.0, z=0.1):
        with open(file_path, 'r') as f:
            xml = f.read()
        req = SpawnEntity.Request()
        req.name = name
        req.xml = xml
        p = Pose()
        p.position.x = float(x); p.position.y = float(y); p.position.z = float(z)
        req.initial_pose = p
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() is not None:
            self.get_logger().info(f"Spawn result: {fut.result().status_message}")
        else:
            self.get_logger().error('Failed to spawn entity')

def main():
    rclpy.init()
    node = Spawner()
    pkg_path = os.path.expanduser('~/your_ws/src/your_pkg/urdf')  # change path
    file_path = os.path.join(pkg_path, 'ego_vehicle.xacro')      # or ego_vehicle.urdf
    node.spawn_from_file('ego_vehicle', file_path, x=0.0, y=0.0, z=0.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
