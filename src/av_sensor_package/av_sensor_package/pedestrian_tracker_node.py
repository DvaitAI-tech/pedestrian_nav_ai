#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray, Detection2D
import math
import time


def iou(b1, b2):
    """Intersection over Union for BoundingBox2D"""
    x1_min = b1.center.position.x - b1.size_x / 2
    y1_min = b1.center.position.y - b1.size_y / 2
    x1_max = b1.center.position.x + b1.size_x / 2
    y1_max = b1.center.position.y + b1.size_y / 2

    x2_min = b2.center.position.x - b2.size_x / 2
    y2_min = b2.center.position.y - b2.size_y / 2
    x2_max = b2.center.position.x + b2.size_x / 2
    y2_max = b2.center.position.y + b2.size_y / 2

    inter_xmin = max(x1_min, x2_min)
    inter_ymin = max(y1_min, y2_min)
    inter_xmax = min(x1_max, x2_max)
    inter_ymax = min(y1_max, y2_max)

    if inter_xmax < inter_xmin or inter_ymax < inter_ymin:
        return 0.0

    inter_area = (inter_xmax - inter_xmin) * (inter_ymax - inter_ymin)
    area1 = b1.size_x * b1.size_y
    area2 = b2.size_x * b2.size_y

    return inter_area / (area1 + area2 - inter_area)


class Track:
    def __init__(self, det, track_id):
        self.id = track_id
        self.bbox = det.bbox
        self.last_seen = time.time()
        self.age = 1


class PedestrianTracker(Node):
    def __init__(self):
        super().__init__('pedestrian_tracker_node')

        self.declare_parameter('iou_threshold', 0.3)
        self.declare_parameter('max_lost_time', 1.0)

        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.max_lost_time = self.get_parameter('max_lost_time').value

        self.sub = self.create_subscription(
            Detection2DArray,
            '/perception/pedestrian_detections',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            Detection2DArray,
            '/perception/tracked_pedestrians',
            10
        )

        self.tracks = []
        self.next_id = 1

        self.get_logger().info('Pedestrian tracker started.')

    def callback(self, msg):
        now = time.time()
        output = Detection2DArray()
        output.header = msg.header

        used_tracks = set()

        for det in msg.detections:
            best_iou = 0.0
            best_track = None

            for track in self.tracks:
                score = iou(det.bbox, track.bbox)
                if score > best_iou:
                    best_iou = score
                    best_track = track

            if best_iou > self.iou_threshold and best_track not in used_tracks:
                # Update existing track
                best_track.bbox = det.bbox
                best_track.last_seen = now
                best_track.age += 1
                used_tracks.add(best_track)

                det.id = str(best_track.id)
                output.detections.append(det)
            else:
                # Create new track
                new_track = Track(det, self.next_id)
                self.next_id += 1
                self.tracks.append(new_track)

                det.id = str(new_track.id)
                output.detections.append(det)

        # Remove lost tracks
        self.tracks = [
            t for t in self.tracks
            if now - t.last_seen < self.max_lost_time
        ]

        self.pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
