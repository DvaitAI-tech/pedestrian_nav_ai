#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import time
import csv
import os
import xacro

from gazebo_msgs.srv import SpawnEntity, SetEntityState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

class MasterScenarioEvaluator(Node):
    def __init__(self):
        super().__init__('master_scenario_evaluator')

        # 1. INTEGRATED DICTIONARY
        self.scenarios = {
            "high_speed_collision": {
                "desc": "Testing TTC at 30m/s with dynamic crossing pedestrians",
                "ego": {"x": -302.0, "y": 0.0, "yaw": -1.60, "vel": 20.0},
                "peds": {
                    "ped_1": {"x": -200.0, "y": 5.0,  "yaw": 0.0, "vel": 1.2, "type": "crossing"},
                    "ped_2": {"x": -150.0, "y": -5.0, "yaw": 0.0, "vel": 1.5, "type": "crossing"},
                    "ped_3": {"x": -100.0, "y": 2.0,  "yaw": 0.0, "vel": 1.0, "type": "static"}
                }
            },
            "edge_of_view_occlusion": {
                "desc": "Testing Track Persistence with crossing pedestrians",
                "ego": {"x": 0.0, "y": 0.0, "yaw": -1.57, "vel": 10.0},
                "peds": {
                    "ped_1": {"x": 8.0, "y": 40.0, "yaw": 3.14, "vel": 2.5},
                    "ped_2": {"x": 8.0, "y": 35.0, "yaw": 3.14, "vel": 2.5},
                    "ped_3": {"x": 8.0, "y": 30.0, "yaw": 3.14, "vel": 2.5}
                }
            }
        }

        # 2. GAZEBO CLIENTS
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        # 3. PUBLISHERS
        self.ego_pub = self.create_publisher(Twist, '/ego/cmd_vel_manual', 10)
        self.ped_pubs = {} # Dictionary to hold publishers for each spawned pedestrian
        
        # 4. METRICS LOGGING
        self.create_subscription(String, '/av/decision_state', self.decision_callback, 10)
        self.results_file = open('benchmark_results.csv', 'w', newline='')
        self.writer = csv.writer(self.results_file)
        self.writer.writerow(['Timestamp', 'Scenario', 'Decision', 'TTC'])

        # 5. EXECUTION STATE
        self.current_scenario_name = "high_speed_collision"
        self.ped_xml = self.load_pedestrian_model()
        
        # Start Master Control Loop (10Hz) to ensure movement 
        self.create_timer(0.1, self.master_control_loop)
        
        # Initialize Scenario
        self.init_scenario(self.current_scenario_name)

    def load_pedestrian_model(self):
        pkg_share = get_package_share_directory('av_gazebo_bringup')
        xacro_file = os.path.join(pkg_share, 'urdf', 'pedestrian.xacro')
        return xacro.process_file(xacro_file).toxml()

    def init_scenario(self, name):
        config = self.scenarios[name]
        self.get_logger().info(f"🚀 INITIALIZING: {name}")
        self.set_model_pose("ego_vehicle", config["ego"])
        
        for p_name, p_config in config["peds"].items():
            self.spawn_or_move_ped(p_name, p_config)
            if p_name not in self.ped_pubs:
                self.ped_pubs[p_name] = self.create_publisher(Twist, f"/{p_name}/cmd_vel", 10)
        
        time.sleep(2.0) # Registration Buffer

    def spawn_or_move_ped(self, name, p_config):
        spawn_req = SpawnEntity.Request()
        spawn_req.name = name
        spawn_req.xml = self.ped_xml
        spawn_req.robot_namespace = name 
        spawn_req.initial_pose.position.x = p_config["x"]
        spawn_req.initial_pose.position.y = p_config["y"]
        self.spawn_client.call_async(spawn_req)

    def set_model_pose(self, name, p):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = p["x"]
        req.state.pose.position.y = p["y"]
        req.state.pose.orientation.z = math.sin(p["yaw"]/2)
        req.state.pose.orientation.w = math.cos(p["yaw"]/2)
        self.set_state_client.call_async(req)

    def master_control_loop(self):
        config = self.scenarios[self.current_scenario_name]
        
        # Ego Motion
        ego_msg = Twist()
        ego_msg.linear.y = config["ego"]["vel"] 
        self.ego_pub.publish(ego_msg)
        
        # Pedestrian Motion (Simulating intent behavior)
        for p_name, p_config in config["peds"].items():
            if p_name in self.ped_pubs:
                ped_msg = Twist()
                if p_config["type"] == "crossing":
                    # Pedestrians cross the road (move along X)
                    ped_msg.linear.y = p_config["vel"]
                else:
                    ped_msg.linear.x = 0.0
                self.ped_pubs[p_name].publish(ped_msg)

    def decision_callback(self, msg):
        data = msg.data.split('|')
        self.writer.writerow([time.time(), self.current_scenario_name, data[0], data[1] if len(data)>1 else "N/A"])
        self.results_file.flush()

def main():
    rclpy.init()
    rclpy.spin(MasterScenarioEvaluator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()