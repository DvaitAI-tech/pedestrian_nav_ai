#!/usr/bin/env python3

from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
import time

# Messages
from geometry_msgs.msg import PoseArray, Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, Trigger

class TrackedPedestrian:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.last_seen = time.time()

class FusionVisualizer(Node):
    def __init__(self):
        super().__init__('fusion_visualizer')

        # --- State Variables ---
        self.sim_running = True
        self.ego_speed = 0.0
        self.current_decision = "GO"
        self.critical_id = -1 
        
        self.topic_health = {
            "lidar": {"last_msg": 0.0, "alive": False},
            "perception": {"last_msg": 0.0, "alive": False}
        }
        
        # --- WINDOW SETTINGS ---
        self.window_name = "AV Perception Dashboard v2.2"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL) 
        
        # Canvas Size & Layout
        self.img_h, self.img_w = 850, 1200
        self.side_panel_w = 250
        self.scale = 10.0  # 1 meter = 10 pixels (Ensures 60m = 600px fits in 850px height)
        
        # --- DATA CONTAINERS ---
        self.tracked_peds = {} 
        self.scan_points = [] 

        # ROS Infrastructure
        self.sub_peds = self.create_subscription(PoseArray, '/perception/pedestrians_world', self.ped_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/ego/lidar/scan', self.scan_callback, 10)
        self.sub_decision = self.create_subscription(String, '/av/decision_state', self.decision_callback, 10)
        self.sub_speed = self.create_subscription(Twist, '/ego/cmd_vel', self.speed_callback, 10)
        
        self.create_timer(0.05, self.draw_loop) 
        self.get_logger().info("AV Dashboard v2.2 - Centering Fix Applied")

    # =========================================================
    # DYNAMIC COORDINATE ANCHORING
    # =========================================================
    def get_ui_anchors(self):
        """Calculates the absolute center and base for the radar view"""
        # radar_area_width is the space to the right of the side panel
        radar_area_width = self.img_w - self.side_panel_w
        
        # Center X is the middle of that radar area
        cx = self.side_panel_w + (radar_area_width // 2)
        
        # Center Y (Ego Position) is 100 pixels up from the very bottom
        cy = self.img_h - 100 
        return cx, cy

    def world_to_pixel(self, wx, wy):
        """Standardized ROS (X-Forward, Y-Left) to Pixel mapping"""
        cx, cy = self.get_ui_anchors()
        # ROS Y+ (Left) -> Image X- (Left)
        ix = int(cx - (wy * self.scale))
        # ROS X+ (Forward) -> Image Y- (Up)
        iy = int(cy - (wx * self.scale))
        return ix, iy
    def world_to_pixel_track(self, wx, wy):
        """Standardized ROS (X-Forward, Y-Left) to Pixel mapping"""
        cx, cy = self.get_ui_anchors()
        # ROS Y+ (Left) -> Image X- (Left)
        ix = int(cx - (wx * self.scale))
        # ROS X+ (Forward) -> Image Y- (Up)
        iy = int(cy - (wy * self.scale))
        return ix, iy

    # =========================================================
    # CALLBACKS (Kept standard)
    # =========================================================
    def decision_callback(self, msg):
        if '|' in msg.data:
            parts = msg.data.split('|')
            self.current_decision = parts[0]
            try:
                self.critical_id = int(parts[1].replace("ID:", ""))
            except: self.critical_id = -1
        else:
            self.current_decision = msg.data
            self.critical_id = -1

    def speed_callback(self, msg):
        self.ego_speed = abs(msg.linear.y)

    def ped_callback(self, msg):
        now = time.time()
        self.topic_health["perception"]["last_msg"] = now
        new_tracks = {}
        for i, pose in enumerate(msg.poses):
            px, py = pose.position.x, pose.position.y
            assigned_id = None
            for old_id, old_p in self.tracked_peds.items():
                if math.sqrt((px - old_p.x)**2 + (py - old_p.y)**2) < 2.0:
                    assigned_id = old_id
                    break
            if assigned_id is None:
                assigned_id = int(time.time() * 1000) % 10000 + i 
            new_tracks[assigned_id] = TrackedPedestrian(assigned_id, px, py)
        self.tracked_peds = new_tracks

    def scan_callback(self, msg):
        self.topic_health["lidar"]["last_msg"] = time.time()
        points = []
        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if i % 5 == 0 and 1.0 < r < 70.0:
                points.append((r * math.cos(angle), r * math.sin(angle)))
            angle += msg.angle_increment
        self.scan_points = points

    # =========================================================
    # RENDER ENGINE
    # =========================================================
    def draw_loop(self):
        if not self.sim_running: return

        # Reset image frame
        img = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
        img[:] = (18, 18, 18) 
        
        cx, cy = self.get_ui_anchors()
        now = time.time()
        for key in self.topic_health:
            self.topic_health[key]["alive"] = (now - self.topic_health[key]["last_msg"]) < 0.5

        # 1. Static UI (Side Panel & Grid)
        cv2.rectangle(img, (0, 0), (self.side_panel_w, self.img_h), (35, 35, 35), -1)
        self.draw_status_panel(img)
        
        # Grid Rings (10m to 60m)
        for dist in [10, 20, 30, 40, 50, 60]:
            r = int(dist * self.scale)
            cv2.ellipse(img, (cx, cy), (r, r), 0, 180, 360, (65, 65, 65), 1)
            cv2.putText(img, f"{dist}m", (cx + 10, cy - r + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (130, 130, 130), 1)

        # 2. Lidar Rendering
        l_color = (230, 230, 230) if self.topic_health["lidar"]["alive"] else (70, 70, 70)
        for px, py in self.scan_points:
            ix, iy = self.world_to_pixel(px, py)
            if 0 <= ix < self.img_w and 0 <= iy < self.img_h:
                cv2.circle(img, (ix, iy), 1, l_color, -1)

        # 3. Decision & Ego HUD
        c_map = {"GO": (0, 255, 0), "SLOW": (0, 165, 255), "STOP": (0, 0, 255)}
        d_color = c_map.get(self.current_decision, (200, 200, 200))
        cv2.putText(img, f"DECISION: {self.current_decision}", (self.side_panel_w + 40, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, d_color, 3)

        # Draw Ego Vehicle (Blue Rectangle)
        ew, el = int(2.2 * self.scale), int(4.8 * self.scale)
        cv2.rectangle(img, (cx - ew//2, cy - el), (cx + ew//2, cy), (255, 140, 0), -1)

        # 4. Pedestrian Rendering
        for p_id, p in self.tracked_peds.items():
            ix, iy = self.world_to_pixel_track(p.x, p.y)
            p_color = (0, 0, 255) if p_id == self.critical_id else (0, 255, 255)
            if 0 <= ix < self.img_w and 0 <= iy < self.img_h:
                cv2.circle(img, (ix, iy), 10, p_color, -1)
                cv2.putText(img, f"ID:{p_id}", (ix+15, iy-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)

    def draw_status_panel(self, img):
        cv2.putText(img, "SYSTEM HEALTH", (25, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 220, 220), 2)
        for i, (name, data) in enumerate(self.topic_health.items()):
            c = (0, 255, 0) if data["alive"] else (0, 0, 255)
            cv2.circle(img, (40, 100 + i*45), 8, c, -1)
            cv2.putText(img, f"{name.upper()}", (65, 105 + i*45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

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
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()