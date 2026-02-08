#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Import necessary messages
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
# We do not import Pose2D to avoid class conflict; we use the one inside BoundingBox2D

import cv2
from ultralytics import YOLO


class YoloCameraNode(Node):
    def __init__(self):
        super().__init__('yolo_camera_node')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('image_topic', '/ego/front_camera/image_raw')
        self.declare_parameter('conf_threshold', 0.4)
        self.declare_parameter('debug_view', False)
        self.declare_parameter('camera_frame', 'front_camera')

        self.image_topic = self.get_parameter('image_topic').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.debug_view = self.get_parameter('debug_view').value
        self.camera_frame = self.get_parameter('camera_frame').value

        # ---------------- UTILS ----------------
        self.bridge = CvBridge()

        # ---------------- YOLO MODEL ----------------
        self.get_logger().info('Loading YOLO model (yolov8n.pt)...')
        self.model = YOLO('models/yolov8n.pt')
        self.get_logger().info('YOLO model loaded successfully.')

        # ---------------- SUBSCRIBER ----------------
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        # ---------------- PUBLISHER ----------------
        self.det_pub = self.create_publisher(
            Detection2DArray,
            '/perception/pedestrian_detections',
            10
        )

        self.get_logger().info(f'Subscribed to image topic: {self.image_topic}')
        self.get_logger().info('Publishing pedestrian detections on /perception/pedestrian_detections')

    # ------------------------------------------------
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        results = self.model(frame, verbose=False)[0]

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header
        detections_msg.header.frame_id = self.camera_frame

        for box in results.boxes:
            conf = float(box.conf[0])
            if conf < self.conf_threshold:
                continue

            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]

            if label != 'person':
                continue

            # Extract coordinates
            x1, y1, x2, y2 = map(float, box.xyxy[0])
            
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            width = x2 - x1
            height = y2 - y1

            det = Detection2D()
            det.header = detections_msg.header

            # --- FIX: MODIFY IN-PLACE (No new Pose2D object) ---
            bbox = BoundingBox2D()
            # We access the existing .center object inside bbox
            bbox.center.position.x = float(center_x)
            bbox.center.position.y = float(center_y)
            bbox.center.theta = 0.0

            bbox.size_x = float(width)
            bbox.size_y = float(height)

            det.bbox = bbox

            # Add Hypothesis
            hyp_pose = ObjectHypothesisWithPose()
            hyp_pose.hypothesis.class_id = 'pedestrian'
            hyp_pose.hypothesis.score = float(conf)
            
            det.results.append(hyp_pose)
            detections_msg.detections.append(det)

            if self.debug_view:
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f'pedestrian {conf:.2f}', (int(x1), max(int(y1) - 5, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        self.det_pub.publish(detections_msg)

        if self.debug_view:
            cv2.imshow('YOLO Pedestrian Detections', frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloCameraNode()
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