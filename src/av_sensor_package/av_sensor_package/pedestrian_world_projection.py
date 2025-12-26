#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import tf2_ros
from tf2_ros import TransformException
from rclpy.duration import Duration

# Messages
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CameraInfo, LaserScan
from geometry_msgs.msg import PoseArray, Pose, PointStamped

# TF Geometry (Specific import for the fix)
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_point

# Synchronization
import message_filters

class LidarCameraFusion(Node):
    def __init__(self):
        super().__init__('lidar_camera_fusion')

        self.camera_frame = 'camera_link'
        self.base_frame = 'base_link'

        # Subscribers
        self.sub_info = self.create_subscription(
            CameraInfo, '/ego/front_camera/camera_info', self.info_callback, 10)

        self.det_sub = message_filters.Subscriber(
            self, Detection2DArray, '/perception/tracked_pedestrians')
        self.scan_sub = message_filters.Subscriber(
            self, LaserScan, '/ego/lidar/scan')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.det_sub, self.scan_sub], queue_size=20, slop=0.3)
        self.ts.registerCallback(self.sync_callback)

        self.pub_world = self.create_publisher(
            PoseArray, '/perception/pedestrians_world', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.fx = None
        self.cx = None
        self.callback_count = 0

        self.get_logger().info("✅ Fusion Node Started.")

    def info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.p[0]
            self.cx = msg.p[2]
            self.get_logger().info(f"📸 Camera Intrinsics Loaded: fx={self.fx:.1f}")

    def sync_callback(self, det_msg, scan_msg):
        self.callback_count += 1
        if self.fx is None: 
            return

        pose_array = PoseArray()
        pose_array.header = det_msg.header
        pose_array.header.frame_id = self.base_frame

        success_count = 0

        for i, det in enumerate(det_msg.detections):
            u = det.bbox.center.position.x
            
            # Step A: Angle
            alpha = -math.atan2((u - self.cx), self.fx)
            
            # Step B: Lidar Depth
            depth = self.get_range_at_angle(scan_msg, alpha)
            if depth is None:
                continue

            # Step C: Coordinates in Camera Frame
            # X=Depth, Y=Left, Z=Up
            pt_cam = PointStamped()
            pt_cam.header.frame_id = self.camera_frame
            pt_cam.header.stamp = det_msg.header.stamp
            
            horizontal_offset = depth * math.tan(-alpha)
            
            pt_cam.point.x = float(depth)
            pt_cam.point.y = float(-horizontal_offset)
            pt_cam.point.z = 0.0

            try:
                # --- THE FIX: Look up transform first, then apply manually ---
                # This avoids the "Type not supported" error
                trans = self.tf_buffer.lookup_transform(
                    self.base_frame, 
                    self.camera_frame, 
                    det_msg.header.stamp, # Use precise time
                    Duration(seconds=0.1)
                )
                
                # Apply transform using the specific geometry_msgs function
                pt_base = do_transform_point(pt_cam, trans)
                
                pose = Pose()
                pose.position = pt_base.point
                pose.orientation.w = 1.0
                pose_array.poses.append(pose)
                success_count += 1
                
                # Debug log to verify coordinates
                # self.get_logger().info(f"Ped {i}: X={pt_base.point.x:.1f}, Y={pt_base.point.y:.1f}")

            except TransformException as e:
                self.get_logger().warn(f"TF Error: {e}", throttle_duration_sec=1)
                continue

        if success_count > 0:
            self.pub_world.publish(pose_array)
            # Only print every few seconds to avoid clutter
            self.get_logger().info(f"✅ Published {success_count} 3D Poses", throttle_duration_sec=2)

    def get_range_at_angle(self, scan, target_angle):
        if target_angle < scan.angle_min or target_angle > scan.angle_max:
            return None
        
        idx = int(round((target_angle - scan.angle_min) / scan.angle_increment))
        window = 3
        start = max(0, idx - window)
        end = min(len(scan.ranges), idx + window + 1)
        
        valid = [r for r in scan.ranges[start:end] if scan.range_min < r < scan.range_max]
        if not valid: return None
        return min(valid)

def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()