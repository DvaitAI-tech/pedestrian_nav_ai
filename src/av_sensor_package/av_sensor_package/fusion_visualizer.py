#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math

# Messages
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan

class FusionVisualizer(Node):
    def __init__(self):
        super().__init__('fusion_visualizer')

        # --- SUBSCRIBERS ---
        self.sub_peds = self.create_subscription(
            PoseArray,
            '/perception/pedestrians_world',
            self.ped_callback,
            10
        )

        self.sub_scan = self.create_subscription(
            LaserScan,
            '/ego/lidar/scan',
            self.scan_callback,
            10
        )

        # --- SETTINGS ---
        self.window_name = "AV Perception Dashboard"
        self.img_h = 800
        self.img_w = 1000  # Wider to fit text panel
        self.scale = 15.0  # 1 meter = 15 pixels (Zoomed out slightly for full view)
        
        # Car Position (Bottom Center)
        self.car_x = self.img_w // 2 # Shifted right to make room for text panel
        self.car_y = self.img_h - 100

        # Data
        self.pedestrians = [] 
        self.scan_points = [] 

        # Timer
        self.create_timer(0.05, self.draw_loop) # 20 FPS
        self.get_logger().info("🚀 Advanced Dashboard Started!")

    def ped_callback(self, msg):
        self.pedestrians = []
        for pose in msg.poses:
            # X = Forward, Y = Left
            self.pedestrians.append((pose.position.x, pose.position.y))

    def scan_callback(self, msg):
        # Downsample lidar for performance (take every 3rd point)
        points = []
        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if i % 3 == 0 and 1.0 < r < 50.0:
                # Polar to Cartesian
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))
            angle += msg.angle_increment
        self.scan_points = points

    def draw_loop(self):
        # 1. Background (Dark Grey)
        img = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
        img[:] = (20, 20, 20) 

        # 2. Draw Side Panel (Left)
        cv2.rectangle(img, (0, 0), (250, self.img_h), (40, 40, 40), -1)
        cv2.putText(img, "TARGET LIST", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
        cv2.line(img, (20, 50), (230, 50), (100, 100, 100), 1)

        # 3. Draw Grid & FOV
        self.draw_grid(img)

        # 4. Draw Lidar Cloud (Faint White)
        # 4. Draw Lidar Cloud (Thicker & Brighter)
        for px, py in self.scan_points:
            ix, iy = self.world_to_pixel(px, py)
            
            # Check if point is inside the window
            if 0 <= ix < self.img_w and 0 <= iy < self.img_h:
                # cv2.circle(image, center, radius, color, thickness)
                # Color: (255, 255, 255) is White (BGR format)
                # Radius: 2 makes it a 4x4 pixel dot (Thicker)
                cv2.circle(img, (ix, iy), 2, (255, 255, 255), -1)

        # 5. Draw Ego Vehicle
        # Car body
        cw, cl = int(2.0 * self.scale), int(4.5 * self.scale)
        cv2.rectangle(img, (self.car_x - cw//2, self.car_y - cl), (self.car_x + cw//2, self.car_y), (255, 0, 0), -1)
        # Headlights
        cv2.circle(img, (self.car_x - cw//2 + 5, self.car_y - cl + 5), 3, (255, 255, 255), -1)
        cv2.circle(img, (self.car_x + cw//2 - 5, self.car_y - cl + 5), 3, (255, 255, 255), -1)

        # 6. Draw Pedestrians & HUD Info
        closest_dist = 999.0
        
        for i, (x, y) in enumerate(self.pedestrians):
            dist = math.sqrt(x**2 + y**2)
            if dist < closest_dist: closest_dist = dist

            ix, iy = self.world_to_pixel(x, y)

            # Draw Pedestrian (Red Circle)
            color = (0, 0, 255) # Red
            if dist > 15.0: color = (0, 255, 255) # Yellow if far

            cv2.circle(img, (ix, iy), 8, color, -1)
            cv2.circle(img, (ix, iy), 12, color, 1)

            # Label on Map
            label = f"{dist:.1f}m"
            cv2.putText(img, label, (ix + 15, iy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # Add to Side Panel List
            text_y = 90 + (i * 40)
            cv2.putText(img, f"PED #{i+1}", (20, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(img, f"X:{x:.1f}m Y:{y:.1f}m", (20, text_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # 7. Collision Warning System
        if closest_dist < 5.0:
            cv2.rectangle(img, (self.img_w//2 - 150, 50), (self.img_w//2 + 150, 120), (0, 0, 180), -1)
            cv2.putText(img, "WARNING", (self.img_w//2 - 110, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4)

        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)

    def draw_grid(self, img):
        # Draw arcs for 5m, 10m, 20m, 30m
        for dist in [5, 10, 20, 30]:
            r = int(dist * self.scale)
            cv2.ellipse(img, (self.car_x, self.car_y), (r, r), 0, 180, 360, (50, 50, 50), 1)
            cv2.putText(img, f"{dist}m", (self.car_x + 5, self.car_y - r + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)

        # Draw Camera FOV Lines (Approx 80 degrees)
        fov_len = 600
        angle = math.radians(40) # Half FOV
        x_off = int(fov_len * math.sin(angle))
        y_off = int(fov_len * math.cos(angle))
        
        # Left Line
        cv2.line(img, (self.car_x, self.car_y), (self.car_x - x_off, self.car_y - y_off), (0, 100, 100), 1)
        # Right Line
        cv2.line(img, (self.car_x, self.car_y), (self.car_x + x_off, self.car_y - y_off), (0, 100, 100), 1)
        cv2.putText(img, "Camera FOV", (self.car_x - 40, self.car_y - 150), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 100, 100), 1)

    def world_to_pixel(self, wx, wy):
        # ROS X+ (Forward) -> Image Y- (Up)
        # ROS Y+ (Left)    -> Image X- (Left)
        ix = int(self.car_x - (wy * self.scale))
        iy = int(self.car_y - (wx * self.scale))
        return ix, iy

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