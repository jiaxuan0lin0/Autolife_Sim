from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mux_publish_rate_hz', default_value='100.0'),
        DeclareLaunchArgument('whole_body_timeout', default_value='0.2'),
        DeclareLaunchArgument('whole_body_execution_mode', default_value='trajectory'),
        DeclareLaunchArgument('whole_body_publish_rate_hz', default_value='100.0'),
        DeclareLaunchArgument('whole_body_publish_velocity_commands', default_value='false'),
        DeclareLaunchArgument('whole_body_feedback_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('whole_body_goal_velocity_tolerance', default_value='0.0'),
        DeclareLaunchArgument('whole_body_goal_settle_time', default_value='0.25'),
        Node(
            package='autolife_control',
            executable='joint_command_mux',
            name='joint_command_mux',
            output='screen',
            parameters=[{
                'publish_rate_hz': ParameterValue(
                    LaunchConfiguration('mux_publish_rate_hz'), value_type=float
                ),
                'whole_body_timeout': ParameterValue(
                    LaunchConfiguration('whole_body_timeout'), value_type=float
                ),
            }],
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
            parameters=[{
                'execution_mode': LaunchConfiguration('whole_body_execution_mode'),
                'publish_rate_hz': ParameterValue(
                    LaunchConfiguration('whole_body_publish_rate_hz'), value_type=float
                ),
                'publish_velocity_commands': ParameterValue(
                    LaunchConfiguration('whole_body_publish_velocity_commands'), value_type=bool
                ),
                'feedback_rate_hz': ParameterValue(
                    LaunchConfiguration('whole_body_feedback_rate_hz'), value_type=float
                ),
                'goal_velocity_tolerance': ParameterValue(
                    LaunchConfiguration('whole_body_goal_velocity_tolerance'), value_type=float
                ),
                'goal_settle_time': ParameterValue(
                    LaunchConfiguration('whole_body_goal_settle_time'), value_type=float
                ),
            }],
        )
    ])
