#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def prompt_publisher():
    rospy.init_node("prompt_node")
    pub = rospy.Publisher("/prompt_input", String, queue_size=10)

    rospy.loginfo("Prompt node started. Type your command:")
    while not rospy.is_shutdown():
        prompt = input(">> ")
        if prompt.lower() == "exit":
            break
        pub.publish(prompt)
        rospy.loginfo(f"Published prompt: {prompt}")

if __name__ == "__main__":
    try:
        prompt_publisher()
    except rospy.ROSInterruptException:
        pass
