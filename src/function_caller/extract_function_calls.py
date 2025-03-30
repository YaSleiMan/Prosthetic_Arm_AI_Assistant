#!/usr/bin/env python3

import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def extract_function_calls(text):
    """
    Parses a string and extracts all top-level function calls with properly matched parentheses.
    """
    calls = []
    depth = 0
    start_idx = None

    for i, char in enumerate(text):
        if char == '(':
            if depth == 0:
                # Start of a function call: match the function name before the (
                match = re.search(r'\b\w+\($', text[:i+1])
                if match:
                    start_idx = match.start()
            depth += 1
        elif char == ')':
            depth -= 1
            if depth == 0 and start_idx is not None:
                calls.append(text[start_idx:i+1])
                start_idx = None

    return calls

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
        self.get_logger().info(f"Received response:\n{msg.data}")
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
