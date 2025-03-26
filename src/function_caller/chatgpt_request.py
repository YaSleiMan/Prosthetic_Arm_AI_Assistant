#!/usr/bin/env python3

import os
from dotenv import load_dotenv
from openai import OpenAI

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Path to your workspace directory
directory = "/home/yasleiman/catkin_ws/src/Prosthetic_Arm_AI_Assistant"

# Load API key and starter prompt
load_dotenv(directory + "/config/.env")
api_key = os.getenv("OPENAI_API_KEY")
client = OpenAI(api_key=api_key)

with open(directory + "/config/starter_prompt.txt", "r") as file:
    starter_prompt = file.read().strip()

# Initialize conversation with system prompt
conversation = [{"role": "system", "content": starter_prompt}]

def send_to_chatgpt(prompt):
    conversation.append({"role": "user", "content": prompt})

    response = client.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=conversation
    )

    response_text = response.choices[0].message.content
    conversation.append({"role": "assistant", "content": response_text})
    return response_text

class ChatGPTNode(Node):
    def __init__(self):
        super().__init__("chatgpt_request_node")

        self.publisher = self.create_publisher(String, "/chatgpt_response", 10)
        self.subscription = self.create_subscription(
            String,
            "/prompt_input",
            self.prompt_callback,
            10
        )

        self.get_logger().info("ChatGPT Request Node started.")

    def prompt_callback(self, msg):
        self.get_logger().info(f"Received prompt: {msg.data}")
        response = send_to_chatgpt(msg.data)
        self.get_logger().info(f"Response: {response}")
        self.publisher.publish(String(data=response))

def main(args=None):
    rclpy.init(args=args)
    node = ChatGPTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
