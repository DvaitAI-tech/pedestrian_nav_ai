#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

class PedestrianViz(Node):
    def __init__(self):
        super().__init__('pedestrian_viz_node')

        self.sub = self.create_subscription(
            Detection2DArray,
            '/perception/tracked_pedestrians',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            MarkerArray,
            '/perception/pedestrian_markers',
            10
        )

    def callback(self, msg):
        markers = MarkerArray()

        for i, det in enumerate(msg.detections):
            x = det.bbox.center.position.x
            y = det.bbox.center.position.y
            pid = det.id

            # --- Sphere marker ---
            sphere = Marker()
            sphere.header = msg.header
            sphere.ns = 'pedestrians'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x * 0.01  # scale for visualization
            sphere.pose.position.y = y * 0.01
            sphere.pose.position.z = 0.0
            sphere.scale.x = 0.2
            sphere.scale.y = 0.2
            sphere.scale.z = 0.2
            sphere.color.r = 0.0
            sphere.color.g = 1.0
            sphere.color.b = 0.0
            sphere.color.a = 0.8

            # --- Text marker (ID) ---
            text = Marker()
            text.header = msg.header
            text.ns = 'pedestrian_ids'
            text.id = i + 1000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = sphere.pose.position.x
            text.pose.position.y = sphere.pose.position.y
            text.pose.position.z = 0.4
            text.scale.z = 0.3
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f'ID: {pid}'

            markers.markers.append(sphere)
            markers.markers.append(text)

        self.pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
