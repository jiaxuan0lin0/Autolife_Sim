from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autolife_control',
            executable='joint_command_mux',
            name='joint_command_mux',
            output='screen',
        ),
        Node(
            package='autolife_control',
            executable='base_controller',
            name='base_controller',
            output='screen',
        ),
        Node(
            package='autolife_control',
            executable='torso_controller',
            name='torso_controller',
            output='screen',
        ),
        Node(
            package='autolife_control',
            executable='arm_controller',
            name='arm_controller',
            output='screen',
        ),
        Node(
            package='autolife_control',
            executable='gripper_controller',
            name='gripper_controller',
            output='screen',
        ),
        Node(
            package='autolife_control',
            executable='head_controller',
            name='head_controller',
            output='screen',
        ),
        Node(
            package='autolife_control',
            executable='whole_body_controller',
            name='whole_body_controller',
            output='screen',
        )
    ])
