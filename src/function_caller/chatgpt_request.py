import openai
import rospy
from std_msgs.msg import String

# Load your OpenAI API key
openai.api_key = "your_openai_api_key"

def send_to_chatgpt(prompt):
    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}]
    )
    return response['choices'][0]['message']['content']

def callback(data):
    rospy.loginfo(f"Received prompt: {data.data}")
    response = send_to_chatgpt(data.data)
    rospy.loginfo(f"ChatGPT response: {response}")
    print(response)

    # Publish the response for the next processing step
    pub = rospy.Publisher("/chatgpt_response", String, queue_size=10)
    pub.publish(response)

def listener():
    rospy.init_node("chatgpt_request_node")
    rospy.Subscriber("/prompt_input", String, callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
