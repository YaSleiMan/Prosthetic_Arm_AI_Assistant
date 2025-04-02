#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PromptNode(Node):
    def __init__(self):
        super().__init__("prompt_node")
        self.publisher = self.create_publisher(String, "/prompt_input", 10)
        self.get_logger().info("Prompt node started. Type your command:")
        self.run_prompt_loop()

    def run_prompt_loop(self):
        try:
            while rclpy.ok():
                prompt = input(">> ")
                if prompt.lower() == "exit":
                    break
                msg = String()
                msg.data = prompt
                self.publisher.publish(msg)
                self.get_logger().info(f"Published prompt: {prompt}")
        except KeyboardInterrupt:
            self.get_logger().info("Prompt node interrupted by user.")

def main(args=None):
    rclpy.init(args=args)
    node = PromptNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
