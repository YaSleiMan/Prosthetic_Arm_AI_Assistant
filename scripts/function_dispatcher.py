import rospy
from std_msgs.msg import String
import ast

from franka_motion_commander import FrankaMotionCommander

commander = None

object_lookup_pub = None

def parse_args(arg_str):
    try:
        return ast.literal_eval(arg_str)
    except:
        rospy.logerr(f"Failed to parse arguments: {arg_str}")
        return None

def dispatch_function_call(call_str):
    global commander, object_lookup_pub
    rospy.loginfo(f"Dispatching: {call_str}")

    if call_str.startswith("find_object"):
        args = parse_args(call_str[len("find_object"):])
        if isinstance(args, str):
            object_lookup_pub.publish(args)
        elif isinstance(args, tuple) and len(args) == 1:
            object_lookup_pub.publish(args[0])
        else:
            rospy.logerr("Invalid arguments for find_object")

    elif call_str.startswith("move_to_pose"):
        args = parse_args(call_str[len("move_to_pose"):])
        if isinstance(args, tuple) and len(args) == 2:
            position, rpy = args
            commander.move_to_pose(position=position, rpy=rpy, mode="absolute")
        elif isinstance(args, dict):
            position = args.get("position")
            rpy = args.get("rpy", (0, 0, 0))
            mode = args.get("mode", "absolute")
            commander.move_to_pose(position=position, rpy=rpy, mode=mode)
        else:
            rospy.logerr("Invalid arguments for move_to_pose")

    elif call_str.startswith("set_gripper"):
        args = parse_args(call_str[len("set_gripper"):])
        rospy.loginfo(f"Would send gripper command: {args}")
        # TODO: Hook into your actual gripper control logic

    else:
        rospy.logwarn(f"Unknown function: {call_str}")

def callback(msg):
    rospy.loginfo(f"Received function call string: {msg.data}")
    try:
        call_lines = [line.strip() for line in msg.data.strip().splitlines() if line.strip()]
        for call in call_lines:
            dispatch_function_call(call)
    except Exception as e:
        rospy.logerr(f"Error while processing function calls: {e}")

def listener():
    global commander, object_lookup_pub

    rospy.init_node("function_dispatcher")
    commander = FrankaMotionCommander()
    object_lookup_pub = rospy.Publisher("/target_object", String, queue_size=10)
    rospy.Subscriber("/extracted_functions", String, callback)
    rospy.loginfo("Function dispatcher ready.")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
