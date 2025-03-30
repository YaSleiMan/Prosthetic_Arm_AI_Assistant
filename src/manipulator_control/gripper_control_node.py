#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import time
from std_msgs.msg import String
import ast

from gripper_interfaces import FrankaGripperActionClient

class GripperController:
    def __init__(self):
        rclpy.init()
        self.node = Node("gripper_control_node")
        self.gripper = FrankaGripperActionClient(self.node)

        self.subscription = self.node.create_subscription(String, "/extracted_functions", self.callback, 10)

        # Start background ROS spin thread
        self.spin_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.spin_thread.start()
        time.sleep(1.0)  # Give it time to initialize

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
        self.node.get_logger().info(f"Dispatching: {call_str}")

        if call_str.startswith("set_gripper"):
            state = self.parse_single_string_arg(call_str[len("set_gripper"):])
            if state in ("open", "close"):
                self.set_gripper(state)
            else:
                self.node.get_logger().error("Invalid value for set_gripper. Must be 'open' or 'close'.")

        else:
            self.node.get_logger().warn(f"Unknown function: {call_str}")
            time.sleep(2)

    def callback(self, msg):
        self.node.get_logger().info(f"Received function call string:\n{msg.data}")
        try:
            call_lines = [line.strip() for line in msg.data.strip().splitlines() if line.strip()]
            for call in call_lines:
                self.dispatch_function_call(call)
        except Exception as e:
            self.node.get_logger().error(f"Error processing function calls: {e}")
    
    def _spin_ros(self):
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def set_gripper(self, state):
        """
        Opens or closes the gripper.
        state: "open" or "close"
        """
        self.node.get_logger().info(f"Gripper command received: {state}")

        if state == "open":
            self.gripper.do_move_blocking(width=0.06, speed=0.2)
        elif state == "close":
            self.gripper.do_grasp_blocking(width=0.01, speed=0.2)
        else:
            self.node.get_logger().error(f"Invalid gripper state: {state}")

    def shutdown(self):
        rclpy.shutdown()
        self.spin_thread.join()

# Example usage
if __name__ == "__main__":
    controller = GripperController()

    # controller.set_gripper("close")
    # time.sleep(2)
    # controller.set_gripper("open")

    try:
        rclpy.spin(controller.node)
    except KeyboardInterrupt:
        pass
    controller.shutdown()
