import re
import rospy
from std_msgs.msg import String

def extract_function_calls(response):
    pattern = r"(\w+\(.*?\))"
    matches = re.findall(pattern, response)
    return matches

def callback(data):
    rospy.loginfo(f"Received response: {data.data}")
    function_calls = extract_function_calls(data.data)
    rospy.loginfo(f"Extracted function calls: {function_calls}")
    print(f"Function Calls: {function_calls}")

def listener():
    rospy.init_node("function_call_extractor_node")
    rospy.Subscriber("/chatgpt_response", String, callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
