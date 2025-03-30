#!/usr/bin/env python3

import ast
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from franka_motion_commander import FrankaMotionCommander

class FunctionDispatcher(Node):
    def __init__(self):
        super().__init__('function_dispatcher')
        self.commander = FrankaMotionCommander(node=self)
        self.object_lookup_pub = self.create_publisher(String, "/target_object", 10)
        self.subscription = self.create_subscription(String, "/extracted_functions", self.callback, 10)
        self.get_logger().info("Function dispatcher ready.")

    def parse_kwargs(self, arg_str):
        """Parses keyword-style argument strings into a dictionary."""
        try:
            fake_func = f"f{arg_str}"
            parsed = ast.parse(fake_func, mode='eval')
            if not isinstance(parsed.body, ast.Call):
                raise ValueError("Not a function call")
            return {kw.arg: ast.literal_eval(kw.value) for kw in parsed.body.keywords}
        except Exception as e:
            self.get_logger().error(f"Failed to parse keyword arguments: {arg_str} — {e}")
            return None

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
            self.get_logger().error(f"Failed to parse single string argument: {arg_str} — {e}")
            return None

    def dispatch_function_call(self, call_str):
        self.get_logger().info(f"Dispatching: {call_str}")

        if call_str.startswith("find_object"):
            args = self.parse_single_string_arg(call_str[len("find_object"):])
            if isinstance(args, str):
                self.object_lookup_pub.publish(String(data=args))
            else:
                self.get_logger().error("Invalid arguments for find_object")

        elif call_str.startswith("move_to_pose"):
            args = self.parse_kwargs(call_str[len("move_to_pose"):])
            if isinstance(args, dict):
                position = args.get("position")
                rpy = args.get("rpy", (0, 0, 0))
                mode = args.get("mode", "absolute")
                self.commander.move_to_pose(position=position, rpy=rpy, mode=mode)
            else:
                self.get_logger().error("Invalid arguments for move_to_pose")

        elif call_str.startswith("set_gripper"):
            state = self.parse_single_string_arg(call_str[len("set_gripper"):])
            if state in ("open", "close"):
                self.commander.set_gripper(state)
            else:
                self.get_logger().error("Invalid value for set_gripper. Must be 'open' or 'close'.")

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
