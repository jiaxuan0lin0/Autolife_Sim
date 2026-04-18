from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=PathJoinSubstitution(
            [FindPackageShare("autolife_description"),"urdf","autolife.urdf.xacro"]
        ),
        description="Absolute path to robot xacro file",
    )

    rviz_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=PathJoinSubstitution(
            [FindPackageShare("autolife_description"),"rviz","view_autolife.rviz"]
        ),
        description="Absolute path to rviz config file",
    )

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Use joint_state_publisher_gui",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    prefix_arg = DeclareLaunchArgument(
        name="prefix",
        default_value="",
        description="Joint name prefix for multi-robot setup",
    )

    use_ros2_control_arg = DeclareLaunchArgument(
        "use_ros2_control",
        default_value="false",
        description="Enable ros2_control tags in xacro",
    )

    model = LaunchConfiguration("model")
    rvizconfig = LaunchConfiguration("rvizconfig")
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    prefix = LaunchConfiguration("prefix")
    use_ros2_control = LaunchConfiguration("use_ros2_control")

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    model,
                    " ",
                    "prefix:=",
                    prefix,
                    " ",
                    "use_ros2_control:=",
                    use_ros2_control,
                ]
            ),
            value_type=str,
        )
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(gui),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(gui),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizconfig],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    return LaunchDescription(
        [
            model_arg,
            rviz_arg,
            gui_arg,
            use_sim_time_arg,
            prefix_arg,
            use_ros2_control_arg,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            joint_state_publisher_node,
            rviz_node,
        ]
    )
