import openai
import rospy
from std_msgs.msg import String
from dotenv import load_dotenv
import os

# Get API key from the environment variables
current_dir = os.path.dirname(os.path.abspath(__file__))
env_path = os.path.join(current_dir, "../../config/.env")
load_dotenv(env_path)
openai.api_key = os.getenv("OPENAI_API_KEY")

# Global variable to store the conversation history
conversation = [{"role": "system", "content": "You are a robotic assistant that decomposes tasks into function calls."}]

def send_to_chatgpt(prompt):
    # Add the user input to the conversation
    conversation.append({"role": "user", "content": prompt})
    
    # Send the entire conversation to ChatGPT
    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=conversation
    )
    
    # Get the response and add it to the conversation
    response_text = response['choices'][0]['message']['content']
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
