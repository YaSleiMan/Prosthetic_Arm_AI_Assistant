#!/usr/bin/env python3

import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def extract_function_calls(response):
    pattern = r"(\w+\(.*?\))"
    matches = re.findall(pattern, response)
    return matches

class FunctionCallExtractor(Node):
    def __init__(self):
        super().__init__("function_call_extractor_node")
        self.publisher = self.create_publisher(String, "/extracted_functions", 10)
        self.subscription = self.create_subscription(
            String,
            "/chatgpt_response",
            self.callback,
            10
        )
        self.get_logger().info("Function Call Extractor Node started.")

    def callback(self, msg):
        self.get_logger().info(f"Received response: {msg.data}")
        function_calls = extract_function_calls(msg.data)
        output = "\n".join(function_calls)
        self.get_logger().info(f"Extracted function calls:\n{output}")
        self.publisher.publish(String(data=output))

def main(args=None):
    rclpy.init(args=args)
    node = FunctionCallExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
