#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2

class YOLODebugNode(Node):
    def __init__(self):
        super().__init__('yolo_debug_node')

        # Load YOLOv5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.conf = 0.25  # Confidence threshold

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info("YOLO Debug Node is running...")

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLO
            results = self.model(frame)
            df = results.pandas().xyxy[0]

            if df.empty:
                self.get_logger().info("No objects detected.")
            else:
                for _, row in df.iterrows():
                    name = row['name']
                    conf = row['confidence']
                    self.get_logger().info(f"Detected {name} ({conf:.2f})")

                # OPTIONAL: Show the detection result in a window
                results.render()
                cv2.imshow("YOLO Detection", results.imgs[0])
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLODebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
