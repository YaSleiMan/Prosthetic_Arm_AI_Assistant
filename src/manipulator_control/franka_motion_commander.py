import time
import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces import FrankaStateInterface
from gripper_interfaces import FrankaGripperActionClient
import threading


def get_object_coordinates(node, timeout_sec=5):
    """
    Waits for object coordinates published to /object_coordinates.

    Parameters:
    - node: rclpy node
    - timeout_sec: maximum time to wait in seconds

    Returns:
    - tuple (x, y, z) or None if timeout
    """
    object_coords = []

    def callback(msg):
        try:
            coords = tuple(map(float, msg.data.strip().split(",")))
            if len(coords) == 3:
                object_coords.append(coords)
        except:
            pass

    sub = node.create_subscription(String, "/object_coordinates", callback, 10)
    timeout = time.time() + timeout_sec
    while not object_coords and time.time() < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    return object_coords[0] if object_coords else None

class FrankaMotionCommander:
    def __init__(self, node, tool_offset=[0.0, 0.0, 0.145]):
        self.node = node
        self.fsi = FrankaStateInterface(self.node)
        self.gripper = FrankaGripperActionClient(self.node)

        self.panda = rtb.models.Panda()
        self.panda.tool = SE3()  # Define any actual tool offset here if different

        self.tool_offset = SE3(tool_offset)
        self.back_prop = SE3([-x for x in tool_offset])

        # Allow interface to warm up
        time.sleep(2)

    def move_to_pose(self, position=None, rpy=(0, 0, 0), mode="absolute", sleep_time=0.01):
        """
        Moves the robot end effector to the desired pose.

        Parameters:
        - position: (x, y, z) in meters or relative offset depending on mode
        - rpy: (roll, pitch, yaw) in radians
        - mode: "absolute" | "relative" | "object" — controls how position is interpreted
        - steps: Number of interpolated steps
        - sleep_time: Delay between steps in seconds
        """
        current_joint_positions = np.array(self.fsi.joint_positions)
        # print(current_joint_positions)
        current_pose = self.panda.fkine(current_joint_positions) * self.tool_offset

        if mode == "absolute":
            target_pose = SE3(*position) * SE3.RPY(rpy, order='xyz')

        elif mode == "relative":
            # Convert world-relative offset into absolute position
            base_to_tool = current_pose
            tool_to_base_rotation = base_to_tool.R.T  # Inverse rotation
            world_offset_local = tool_to_base_rotation @ np.array(position)

            delta_pose = SE3(*world_offset_local) * SE3.RPY(rpy, order='xyz')
            target_pose = current_pose * delta_pose

        elif mode == "object":
            position = get_object_coordinates(self.node)
            if position is None:
                self.node.get_logger().error("Timeout waiting for object coordinates")
                return
            target_pose = SE3(*position) * SE3.RPY(rpy, order='xyz')

        else:
            self.node.get_logger().error(f"Invalid movement mode: {mode}")
            return

        # target_pose_flange = target_pose * self.back_prop
        distance = np.linalg.norm(target_pose.t - current_pose.t)
        min_steps = 50
        resolution = 0.002
        steps = max(int(distance / resolution), min_steps)
        trajectory = rtb.tools.trajectory.ctraj(current_pose, target_pose, steps)

        for pose in trajectory:
            pose_flange = pose * self.back_prop
            sol = self.panda.ikine_LM(pose_flange, q0=current_joint_positions)
            if sol.success:
                # print(sol.q.tolist())
                self.fsi.publish_joints(sol.q.tolist())
                current_joint_positions = sol.q
                time.sleep(sleep_time)
            else:
                self.node.get_logger().warn("IK solution failed at one of the trajectory steps.")

    def set_gripper(self, state):
        """
        Opens or closes the gripper.
        state: "open" or "close"
        """
        if state == "open":
            self.node.get_logger().info("Opening gripper")
            self.gripper.do_move_blocking(width=0.06, speed=0.2)
        elif state == "close":
            self.node.get_logger().info("Closing gripper")
            self.gripper.do_grasp_blocking(width=0.01, speed=0.2)
        else:
            self.node.get_logger().error(f"Invalid gripper command: {state}")
        time.sleep(0.1)
    
    def shutdown(self):
        rclpy.shutdown()

# Example usage:
if __name__ == '__main__':
    commander = FrankaMotionCommander()

    spin_func = lambda _ : rclpy.spin(commander.node)
    spin_thread = threading.Thread(target=spin_func, args=(0,))
    spin_thread.start()
    time.sleep(2) # sleep to allow spin thread to get some messages

    # Example absolute move
    commander.move_to_pose(position=(0.3, 0.0, 0.55), rpy=(3.14, 3.14, 0), mode="absolute")

    # Example relative move
    # commander.move_to_pose(position=(0, 0, -1), rpy=(0, 0, 0), mode="relative")

    # Example object-based move
    # commander.move_to_pose(rpy=(0, 3.14, 0), mode="object")

    # Example gripper command
    commander.set_gripper("close")
    time.sleep(1)
    commander.set_gripper("open")

    commander.shutdown()
