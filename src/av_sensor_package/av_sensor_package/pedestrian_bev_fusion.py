#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import message_filters
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CameraInfo, LaserScan
from geometry_msgs.msg import PoseArray, Pose

class BEVFusionNode(Node):
    def __init__(self):
        super().__init__('pedestrian_bev_fusion')
        self.base_frame = 'base_link'
        
        # 1. Camera Intrinsics
        self.fx = 379.9  
        self.cx = 320.0
        
        # 2. Setup Subscribers
        self.info_sub = self.create_subscription(CameraInfo, '/ego/front_camera/camera_info', self.info_callback, 10)
        self.det_sub = message_filters.Subscriber(self, Detection2DArray, '/perception/tracked_pedestrians')
        self.scan_sub = message_filters.Subscriber(self, LaserScan, '/ego/lidar/scan')
        
        # Synchronization slop adjusted for 30m/s speed
        self.ts = message_filters.ApproximateTimeSynchronizer([self.det_sub, self.scan_sub], queue_size=20, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        self.pub_world = self.create_publisher(PoseArray, '/perception/pedestrians_world', 10)
        self.get_logger().info("✅ BEV Fusion Node: Final Axis Alignment Active")

    def info_callback(self, msg):
        self.fx = msg.p[0]
        self.cx = msg.p[2]

    def get_best_lidar_match(self, scan, target_angle, window=5):
        idx = int((target_angle - scan.angle_min) / scan.angle_increment)
        start = max(0, idx - window)
        end = min(len(scan.ranges), idx + window + 1)
        valid_ranges = [r for r in scan.ranges[start:end] if scan.range_min < r < scan.range_max]
        return min(valid_ranges) if valid_ranges else None

    def sync_callback(self, det_msg, scan_msg):
        pose_array = PoseArray()
        pose_array.header = det_msg.header
        pose_array.header.frame_id = self.base_frame

        for det in det_msg.detections:
            u_center = det.bbox.center.position.x
            # Calculate angle relative to camera center
            angle = -math.atan2((u_center - self.cx), self.fx)
            depth = self.get_best_lidar_match(scan_msg, angle)
            
            if depth:
                p = Pose()
                # --- FINAL COORDINATE MAPPING (MATCHING USER SYSTEM) ---
                # Forward (Longitudinal) is +Y
                p.position.y = float(depth * math.cos(angle))
                # Lateral (Side) is +X (Right) / -X (Left)
                # We flip the sign because math.sin(angle) increases to the left in camera space
                p.position.x = float(-depth * math.sin(angle))
                # -------------------------------------------------------
                
                p.position.z = 0.0
                p.orientation.w = 1.0
                pose_array.poses.append(p)

        if len(pose_array.poses) > 0:
            self.pub_world.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BEVFusionNode())
    rclpy.shutdown()