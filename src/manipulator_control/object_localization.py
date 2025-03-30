#!/usr/bin/env python3

import cv2
import torch
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from spatialmath import SE3
import roboticstoolbox as rtb
import ast
from interfaces import FrankaStateInterface

model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
bridge = CvBridge()

class ObjectLocalizationNode(Node):
    def __init__(self):
        super().__init__('object_localization_node')

        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/depth/color/points', self.depth_callback, 10)
        self.subscription = self.create_subscription(String, "/extracted_functions", self.callback, 10)
        self.coord_pub = self.create_publisher(String, '/object_coordinates', 10)

        self.latest_image = None
        self.latest_depth = None
        self.target_object = None

        self.panda = rtb.models.Panda()
        self.tool_offset = SE3([
            [-0.00271136, -0.99979483,  0.02007335,  0.05374745],
            [ 0.99996372, -0.00254864,  0.00812753, -0.04031328],
            [-0.0080747,   0.02009466,  0.99976547, -0.03746503],
            [ 0.0,         0.0,         0.0,         1.0]
        ])
        self.fsi = FrankaStateInterface(self)

        self.timer = self.create_timer(0.1, self.process_image)

    def parse_single_string_arg(self, arg_str):
        """Parses a single string literal inside parentheses, e.g. ('open')"""
        try:
            parsed = ast.parse(f"f{arg_str}", mode='eval')
            if isinstance(parsed.body, ast.Call) and len(parsed.body.args) == 1:
                val = parsed.body.args[0]
                if isinstance(val, ast.Str):
                    return val.s
            raise ValueError("Expected a single string literal")
        except Exception as e:
            self.node.get_logger().error(f"Failed to parse single string argument: {arg_str} — {e}")
            return None

    def dispatch_function_call(self, call_str):
        self.get_logger().info(f"Dispatching: {call_str}")

        if call_str.startswith("find_object"):
            args = self.parse_single_string_arg(call_str[len("find_object"):])
            if isinstance(args, str):
                self.target_callback(args)
            else:
                self.get_logger().error("Invalid arguments for find_object")
        else:
            self.get_logger().warn(f"Unknown function: {call_str}")
        
    def callback(self, msg):
        self.get_logger().info(f"Received function call string:\n{msg.data}")
        try:
            call_lines = [line.strip() for line in msg.data.strip().splitlines() if line.strip()]
            for call in call_lines:
                self.dispatch_function_call(call)
        except Exception as e:
            self.get_logger().error(f"Error processing function calls: {e}")
    
    def target_callback(self, msg):
        self.target_object = msg.lower()
        self.get_logger().info(f'Target object set to: {self.target_object}')

    def image_callback(self, msg):
        self.latest_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        self.latest_depth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_to_base(self):
        joint_positions = np.array(self.fsi.joint_positions)
        base_to_ee = self.panda.fkine(joint_positions)
        return base_to_ee * self.tool_offset

    def process_image(self):
        if self.latest_image is None or self.latest_depth is None or self.target_object is None:
            return

        results = model(self.latest_image)
        detected_objects = results.pandas().xyxy[0]

        for _, row in detected_objects.iterrows():
            if row['name'].lower() == self.target_object:
                x_center = int((row['xmin'] + row['xmax']) / 2)
                y_center = int((row['ymin'] + row['ymax']) / 2)

                depth_value = self.latest_depth[y_center, x_center]
                if np.isnan(depth_value) or depth_value == 0:
                    self.get_logger().warn(f'Invalid depth value for {self.target_object}')
                    continue

                # Convert pixel to 3D camera coordinates
                point_cam = np.array([x_center, y_center, depth_value])
                T_cam = self.camera_to_base()
                point_base = T_cam.A @ np.append(point_cam, 1)
                coords = tuple(point_base[:3])

                self.get_logger().info(f'Object {self.target_object} found at: {coords}')
                self.coord_pub.publish(String(data=f'{coords[0]},{coords[1]},{coords[2]}'))
                break

def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()