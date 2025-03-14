ML-based prosthetic arm control pipeline ROS implementation for BME1350 (based of design discussed in MIE1075)

Planned Features:
- LLM-based (ChatGPT) function caller for task breakdown
- Computer vision system including object recognition model (YOLO) for relevant object identification and location
- Franka Emika robot as proxy for arm prosthetic

Progress:
- Implementing the LLM section of the system that takes in requests for what the arm needs to do via keyboard input and returns a list of functions needed to perform it
