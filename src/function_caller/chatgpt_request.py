from openai import OpenAI
import rospy
from std_msgs.msg import String
from dotenv import load_dotenv
import os

# Get Directory
directory = "/home/yasleiman/catkin_ws/src/Prosthetic_Arm_AI_Assistant"

# Get API key from the environment variables
load_dotenv(directory+"/config/.env")
api_key=os.getenv("OPENAI_API_KEY")
# print(api_key)
client = OpenAI(api_key=api_key)

# Load Starting Prompt
with open(directory+"/config/starter_prompt.txt", "r") as file:
    starter_prompt = file.read().strip()

# Global variable to store the conversation history
conversation = [{"role": "system", "content": starter_prompt}]

def send_to_chatgpt(prompt):
    # Add the user input to the conversation
    conversation.append({"role": "user", "content": prompt})

    # Send the entire conversation to ChatGPT
    response = client.chat.completions.create(model="gpt-3.5-turbo",messages=conversation)

    # Get the response and add it to the conversation
    response_text = response.choices[0].message.content
    conversation.append({"role": "assistant", "content": response_text})

    return response_text

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
