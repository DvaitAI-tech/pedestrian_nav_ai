#!/usr/bin/python3
# generated from colcon_core/shell/template/prefix_chain_python_for_package.sh

# --- START MANUAL FIX ---
import sys
# Insert the Conda environment's package path as the first place to look
sys.path.insert(1, '/home/nk/anaconda3/envs/ros_yolo/lib/python3.10/site-packages')
# --- END MANUAL FIX ---

# This line loads the actual main function from your compiled package
import importlib.util
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from autoware_perception_msgs.msg import DetectedObject, DetectedObjects

from cv_bridge import CvBridge
import torch
import cv2
import numpy as np
import time

# --- Key Change: Use the installed 'ultralytics' package directly ---
from ultralytics import YOLO 
# -------------------------------------------------------------------

class YoloV5PedestrianNode(Node):
    def __init__(self):
        super().__init__('yolov5_pedestrian_node')
        self.bridge = CvBridge()
        self.publisher_img = self.create_publisher(Image, '/pedestrian/image_raw', 10)
        self.publisher_boxes = self.create_publisher(DetectedObjects, '/pedestrian/detections', 10)

        self.get_logger().info("🔄 Loading YOLOv5 model (using installed Ultralytics package)...")
        
        try:
            # Load YOLOv5 weights using the modern YOLO class (supports v5 weights)
            self.model = YOLO('yolov5s.pt') 
            
            # The 'ultralytics' library handles the downloading and PyTorch security patches internally.
            self.model.conf = 0.35  # confidence threshold
            self.model.classes = [0]  # only 'person' class (COCO class ID 0)
            self.get_logger().info("✅ YOLOv5 model loaded.")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load YOLOv5 model: {e}")
            rclpy.shutdown()
            return

        # Load video (Using webcam index 0)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("❌ Could not open video file or camera.")
            rclpy.shutdown()
            return

        self.timer = self.create_timer(0.05, self.process_frame)
        self.get_logger().info("✅ YOLOv5 Pedestrian Node Started")

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            # Loop for camera is not necessary, just wait for next frame
            self.get_logger().info("End of stream/Camera unavailable.")
            return

        # Run YOLOv5 inference using the modern predict method
        # We need the output to match the old results.xyxy[0] format
        results = self.model.predict(
            source=frame, 
            conf=self.model.conf, 
            classes=self.model.classes,
            verbose=False
        )

        # Publish raw image
        msg_img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg_img.header.stamp = self.get_clock().now().to_msg()
        msg_img.header.frame_id = "camera_link"
        self.publisher_img.publish(msg_img)

        # Prepare detections in Autoware format
        detections = DetectedObjects()
        detections.header.stamp = self.get_clock().now().to_msg()
        detections.header.frame_id = "camera_link"

        # Parsing results from the modern Ultralytics format (same as v8 logic)
        if results and len(results[0].boxes):
            boxes = results[0].boxes.xyxy.cpu().numpy()
            confs = results[0].boxes.conf.cpu().numpy()

            for box, conf in zip(boxes, confs):
                x1, y1, x2, y2 = box.astype(float)
                
                obj = DetectedObject()
                obj.existence_probability = float(conf)
                
                # Position (Center of the bounding box)
                obj.kinematics.pose_with_covariance.pose.position.x = (x1 + x2) / 2.0
                obj.kinematics.pose_with_covariance.pose.position.y = (y1 + y2) / 2.0
                
                # Dimensions (Width and Height)
                obj.shape.dimensions.x = x2 - x1
                obj.shape.dimensions.y = y2 - y1
                
                detections.objects.append(obj)

                # Draw bounding boxes
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f"person {conf:.2f}", (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Publish detection message
        self.publisher_boxes.publish(detections)

        # Show frame
        cv2.imshow("YOLOv5 Pedestrian Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = YoloV5PedestrianNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down node...')
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()