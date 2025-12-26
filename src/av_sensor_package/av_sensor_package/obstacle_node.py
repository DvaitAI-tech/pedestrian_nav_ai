#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

class LidarObstacleNode(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_node')

        # PARAMETERS
        self.declare_parameter('scan_topic', '/ego/lidar/scan')
        self.declare_parameter('stop_distance', 3.0)
        self.declare_parameter('front_angle_width', 0.6)  # radians (±0.3)
        self.declare_parameter('throttle_period', 2.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.front_angle_width = self.get_parameter('front_angle_width').value
        self.throttle_period = self.get_parameter('throttle_period').value

        self._last_info_time = self.get_clock().now()

        # SUBSCRIBE
        self.sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        # RViz marker
        self.marker_pub = self.create_publisher(Marker, 'lidar_obstacle_marker', 10)

        self.get_logger().info(f'Listening to: {self.scan_topic}')

    def _can_log_info(self):
        now = self.get_clock().now()
        if (now - self._last_info_time).nanoseconds * 1e-9 >= self.throttle_period:
            self._last_info_time = now
            return True
        return False

    def scan_callback(self, scan: LaserScan):
        half = self.front_angle_width / 2.0

        valid_ranges = []
        front_ranges = []

        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r):
                valid_ranges.append(r)
                if -half <= angle <= half:
                    front_ranges.append((r, angle))
            angle += scan.angle_increment

        # --- PRINT NON-INF VALUES (THROTTLED) ---
        if valid_ranges and self._can_log_info():
            min_all = min(valid_ranges)
            max_all = max(valid_ranges)
            avg_all = sum(valid_ranges) / len(valid_ranges)

            sample = ", ".join(f"{r:.2f}" for r in valid_ranges[:8])

            self.get_logger().info(
                f"VALID RAYS={len(valid_ranges)} | "
                f"min={min_all:.2f} m, avg={avg_all:.2f} m, max={max_all:.2f} m | "
                f"sample=[{sample}...]"
            )

        if not front_ranges:
            return

        # closest obstacle in front
        min_r, min_angle = min(front_ranges, key=lambda x: x[0])

        # WARN if too close
        if min_r < self.stop_distance:
            self.get_logger().warn(
                f"OBSTACLE CLOSE! dist={min_r:.2f} m @ angle={min_angle:.2f} rad"
            )

        # RViz marker
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = scan.header.stamp
        marker.ns = 'lidar_obstacle'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = min_r * math.cos(min_angle)
        marker.pose.position.y = min_r * math.sin(min_angle)
        marker.pose.position.z = 0.3
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.9

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
