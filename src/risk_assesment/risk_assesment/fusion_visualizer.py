#!/usr/bin/env python3

from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
import time

# Messages
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, Trigger

class TrackedPedestrian:
    """Helper class to manage individual track states"""
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.last_seen = time.time()

class FusionVisualizer(Node):
    def __init__(self):
        super().__init__('fusion_visualizer')

        self.sim_running = True
        self.ego_speed = 0.0
        self.current_decision = "GO"
        self.critical_id = -1  # Parsed from the decision string
        
        # --- HEALTH MONITORING ---
        self.topic_health = {
            "lidar": {"last_msg": 0.0, "alive": False},
            "perception": {"last_msg": 0.0, "alive": False}
        }
        
        self.create_service(SetBool, '/dashboard/start_stop', self.handle_start_stop)
        self.create_service(Trigger, '/dashboard/reset', self.handle_reset)

        # --- SUBSCRIBERS ---
        self.sub_peds = self.create_subscription(PoseArray, '/perception/pedestrians_world', self.ped_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/ego/lidar/scan', self.scan_callback, 10)
        self.create_subscription(String,'/av/decision_state',self.decision_callback,10)
        self.create_subscription(Twist, '/ego/cmd_vel',self.speed_callback,10)

        # --- SETTINGS ---
        self.window_name = "AV Perception Dashboard v2.0"
        self.img_h, self.img_w = 900, 1100
        self.scale = 15.0  
        # Shifted right to accommodate the health panel
        self.car_x, self.car_y = (self.img_w + 250) // 2, self.img_h - 100

        # DATA CONTAINERS
        self.tracked_peds = {} 
        self.scan_points = [] 

        self.create_timer(0.05, self.draw_loop) 
        self.get_logger().info("Advanced Dashboard Started!")

    def decision_callback(self, msg):
        # Parses "STOP|ID:5" -> decision="STOP", critical_id=5
        if '|' in msg.data:
            parts = msg.data.split('|')
            self.current_decision = parts[0]
            try:
                self.critical_id = int(parts[1].replace("ID:", ""))
            except:
                self.critical_id = -1
        else:
            self.current_decision = msg.data
            self.critical_id = -1

    def speed_callback(self, msg):
        # Vehicle moves on Y axis in your current coordinate setup
        self.ego_speed = abs(msg.linear.y)

    def ped_callback(self, msg):
        now = time.time()
        self.topic_health["perception"]["last_msg"] = now
        dist_threshold = 2.0  
        new_tracked_peds = {}
        
        for i, pose in enumerate(msg.poses):
            px, py = pose.position.x, pose.position.y
            assigned_id = None
            
            for old_id, old_p in self.tracked_peds.items():
                d = math.sqrt((px - old_p.x)**2 + (py - old_p.y)**2)
                if d < dist_threshold:
                    assigned_id = old_id
                    break
            
            if assigned_id is None:
                assigned_id = int(time.time() * 1000) % 10000 + i 
                
            new_p = TrackedPedestrian(assigned_id, px, py)
            new_tracked_peds[assigned_id] = new_p

        self.tracked_peds = new_tracked_peds

    def scan_callback(self, msg):
        self.topic_health["lidar"]["last_msg"] = time.time()
        points = []
        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if i % 5 == 0 and 1.0 < r < 50.0:
                points.append((r * math.cos(angle), r * math.sin(angle)))
            angle += msg.angle_increment
        self.scan_points = points

    def draw_loop(self):
        if not self.sim_running: return

        img = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
        img[:] = (15, 15, 15) 
        self.update_health()

        # 1. Panels and Grid
        cv2.rectangle(img, (0, 0), (250, self.img_h), (30, 30, 30), -1)
        self.draw_status_panel(img)
        self.draw_grid(img)

        # 2. Lidar Data
        lidar_color = (200, 200, 200) if self.topic_health["lidar"]["alive"] else (60, 60, 60)
        for px, py in self.scan_points:
            ix, iy = self.world_to_pixel_lidar(px, py)
            if 0 <= ix < self.img_w and 0 <= iy < self.img_h:
                cv2.circle(img, (ix, iy), 1, lidar_color, -1)

        # 3. Dynamic TTC & Decision Overlay
        # Only calculate TTC for the pedestrian triggering the safety logic
        ttc_val = float('inf')
        if self.critical_id in self.tracked_peds:
            cp = self.tracked_peds[self.critical_id]
            dist = math.sqrt(cp.x**2 + cp.y**2)
            if self.ego_speed > 0.1:
                ttc_val = dist / self.ego_speed

        # Draw Decision Text
        d_color = (0, 255, 0) if self.current_decision == "GO" else (0, 165, 255) if self.current_decision == "SLOW" else (0, 0, 255)
        cv2.putText(img, f"DECISION: {self.current_decision}", (300, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, d_color, 3)
        
        if ttc_val != float('inf'):
            cv2.putText(img, f"TTC: {ttc_val:.1f}s (ID:{self.critical_id})", (300, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

        # 4. Ego Vehicle
        self.draw_ego(img)

        # 5. Tracked Pedestrians
        for p_id, p in self.tracked_peds.items():
            ix, iy = self.world_to_pixel(p.x, p.y)
            # Highlight the critical pedestrian in Red, others in Yellow/Green
            p_color = (0, 0, 255) if p_id == self.critical_id else (0, 255, 255)
            cv2.circle(img, (ix, iy), 10, p_color, -1)
            cv2.putText(img, f"ID:{p_id}", (ix+12, iy-12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)

    def update_health(self):
        now = time.time()
        for key in self.topic_health:
            self.topic_health[key]["alive"] = (now - self.topic_health[key]["last_msg"]) < 0.5

    def draw_status_panel(self, img):
        cv2.putText(img, "SYSTEM HEALTH", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        for i, (name, data) in enumerate(self.topic_health.items()):
            color = (0, 255, 0) if data["alive"] else (0, 0, 255)
            cv2.circle(img, (30, 70 + i*30), 6, color, -1)
            cv2.putText(img, f"{name.upper()}", (50, 75 + i*30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

    def draw_ego(self, img):
        cw, cl = int(2.0 * self.scale), int(4.5 * self.scale)
        # Removed the redundant static "GO" logic from here to avoid overlap
        cv2.rectangle(img, (self.car_x - cw//2, self.car_y - cl), (self.car_x + cw//2, self.car_y), (255, 100, 0), -1)

    def world_to_pixel(self, wx, wy):
        # wx = Longitudinal (Forward distance from car)
        # wy = Lateral (Side distance from car)
        # Ensure this logic is applied identically to p.x, p.y AND px, py from lidar
        ix = int(self.car_x - (wx * self.scale))
        iy = int(self.car_y - (wy * self.scale))
        return ix, iy
    def world_to_pixel_lidar(self, wx, wy):

        # ROS X+ (Forward) -> Image Y- (Up)
        # ROS Y+ (Left)    -> Image X- (Left)
        ix = int(self.car_x - (wy * self.scale))
        iy = int(self.car_y - (wx * self.scale))
        return ix, iy

    def draw_grid(self, img):
        for dist in [10, 20, 30, 40, 50]:
            r = int(dist * self.scale)
            cv2.ellipse(img, (self.car_x, self.car_y), (r, r), 0, 180, 360, (50, 50, 50), 1)
            cv2.putText(img, f"{dist}m", (self.car_x + 5, self.car_y - r + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)

    def handle_start_stop(self, req, res):
        self.sim_running = req.data
        res.success = True
        return res

    def handle_reset(self, req, res):
        self.tracked_peds.clear()
        self.scan_points.clear()
        res.success = True
        return res

def main(args=None):
    rclpy.init(args=args)
    node = FusionVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()