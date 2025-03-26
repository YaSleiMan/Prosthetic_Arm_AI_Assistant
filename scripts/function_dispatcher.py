#!/usr/bin/env python3

import ast
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from franka_motion_commander import FrankaMotionCommander

class FunctionDispatcher(Node):
    def __init__(self):
        super().__init__('function_dispatcher')
        self.commander = FrankaMotionCommander()
        self.object_lookup_pub = self.create_publisher(String, "/target_object", 10)
        self.subscription = self.create_subscription(String, "/extracted_functions", self.callback, 10)
        self.get_logger().info("Function dispatcher ready.")

    def parse_args(self, arg_str):
        try:
            return ast.literal_eval(arg_str)
        except Exception as e:
            self.get_logger().error(f"Failed to parse arguments: {arg_str} — {e}")
            return None

    def dispatch_function_call(self, call_str):
        self.get_logger().info(f"Dispatching: {call_str}")

        if call_str.startswith("find_object"):
            args = self.parse_args(call_str[len("find_object"):])
            if isinstance(args, str):
                self.object_lookup_pub.publish(String(data=args))
            elif isinstance(args, tuple) and len(args) == 1:
                self.object_lookup_pub.publish(String(data=args[0]))
            else:
                self.get_logger().error("Invalid arguments for find_object")

        elif call_str.startswith("move_to_pose"):
            args = self.parse_args(call_str[len("move_to_pose"):])
            if isinstance(args, tuple) and len(args) == 2:
                position, rpy = args
                self.commander.move_to_pose(position=position, rpy=rpy, mode="absolute")
            elif isinstance(args, dict):
                position = args.get("position")
                rpy = args.get("rpy", (0, 0, 0))
                mode = args.get("mode", "absolute")
                self.commander.move_to_pose(position=position, rpy=rpy, mode=mode)
            else:
                self.get_logger().error("Invalid arguments for move_to_pose")

        elif call_str.startswith("set_gripper"):
            args = self.parse_args(call_str[len("set_gripper"):])
            self.get_logger().info(f"Would send gripper command: {args}")
            # TODO: Connect to actual gripper control

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

def main(args=None):
    rclpy.init(args=args)
    node = FunctionDispatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
