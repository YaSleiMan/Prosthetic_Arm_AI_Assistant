from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Prosthetic_Arm_AI_Assistant',
            executable='prompt_node',
            name='prompt_node',
            output='screen'
        ),
        Node(
            package='Prosthetic_Arm_AI_Assistant',
            executable='chatgpt_request',
            name='chatgpt_request_node',
            output='screen'
        ),
        Node(
            package='Prosthetic_Arm_AI_Assistant',
            executable='extract_function_calls',
            name='function_call_extractor_node',
            output='screen'
        ),
        Node(
            package='Prosthetic_Arm_AI_Assistant',
            executable='object_localization',
            name='object_localization_node',
            output='screen'
        ),
        Node(
            package='Prosthetic_Arm_AI_Assistant',
            executable='function_dispatcher',
            name='function_dispatcher',
            output='screen'
        ),
    ])
