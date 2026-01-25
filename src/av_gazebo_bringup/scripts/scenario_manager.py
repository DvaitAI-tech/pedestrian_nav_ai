#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry

class ScenarioManager(Node):
    def __init__(self):
        super().__init__('scenario_manager')

        # 1. SCENARIO CONFIGURATION
        # You can add more scenarios here to test intersections or different road segments
        self.scenarios = {
            "straight_test": {
                "ego_start": {"x": 0.0, "y": -200.0, "yaw": 1.57}, # Moving North on Y
                "ped_start": {"x": 0.0, "y": 100.0, "yaw": 0.0},
                "goal": {"x": 0.0, "y": 200.0}
            },
            "crosswalk_test": {
                "ego_start": {"x": 0.0, "y": 50.0, "yaw": 1.57},
                "ped_start": {"x": 100.0, "y": 0.0, "yaw": 3.14},
                "goal": {"x": 0.0, "y": 150.0}
            }
        }

        # 2. GAZEBO SERVICE CLIENT
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Gazebo set_entity_state service...')

        # 3. POSITION MONITORING
        # Listening to ego odom to check if goal is reached
        self.create_subscription(Odometry, '/ego/odom', self.odom_callback, 10)
        
        self.active_goal = None
        self.goal_reached = False

        # Start with the first scenario
        self.run_scenario("straight_test")

    def run_scenario(self, name):
        """Teleports models to defined start points"""
        if name not in self.scenarios:
            self.get_logger().error(f"Scenario {name} not found!")
            return

        config = self.scenarios[name]
        self.active_goal = config["goal"]
        self.goal_reached = False
        
        # Teleport Ego Vehicle
        self.set_model_pose("ego_vehicle", config["ego_start"])
        
        # Teleport Main Pedestrian (Human Crossing)
        self.set_model_pose("giant_pedestrian", config["ped_start"])
        
        self.get_logger().info(f"🚀 Scenario '{name}' Initialized. Goal: {self.active_goal}")

    def set_model_pose(self, model_name, pose_dict):
        """Service call to Gazebo to move a model"""
        req = SetEntityState.Request()
        req.state.name = model_name
        req.state.pose.position.x = pose_dict["x"]
        req.state.pose.position.y = pose_dict["y"]
        req.state.pose.position.z = 0.05 
        
        # Simple Yaw to Quaternion conversion for Z-axis rotation
        req.state.pose.orientation.z = math.sin(pose_dict["yaw"] / 2.0)
        req.state.pose.orientation.w = math.cos(pose_dict["yaw"] / 2.0)
        
        future = self.client.call_async(req)
        return future

    def odom_callback(self, msg):
        """Checks if the vehicle has reached the goal coordinate"""
        if self.active_goal is None or self.goal_reached:
            return

        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        
        # Euclidean distance to goal
        dist = math.sqrt((curr_x - self.active_goal["x"])**2 + (curr_y - self.active_goal["y"])**2)
        
        if dist < 2.0: # 2 meter threshold
            self.goal_reached = True
            self.get_logger().warn(f"🏁 GOAL REACHED! Distance: {dist:.2f}m")
            # You can add logic here to trigger the next step (Metrics Logger)

def main(args=None):
    rclpy.init(args=args)
    node = ScenarioManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()