from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("autolife_planning_bridge"), "config", "planning.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=config_path),
            DeclareLaunchArgument(
                "planner_root",
                default_value="/data/jiaxuanLin/Autolife-Planning",
            ),
            DeclareLaunchArgument(
                "planner_python_site",
                default_value="/home/sutai/home/envs/autolife-planning/lib/python3.12/site-packages",
            ),
            DeclareLaunchArgument(
                "planner_python",
                default_value="/home/sutai/home/envs/autolife-planning/bin/python3",
            ),
            SetEnvironmentVariable(
                "AUTOLIFE_PLANNING_ROOT",
                LaunchConfiguration("planner_root"),
            ),
            SetEnvironmentVariable(
                "AUTOLIFE_PLANNING_PYTHON_SITE",
                LaunchConfiguration("planner_python_site"),
            ),
            SetEnvironmentVariable(
                "PYTHONPATH",
                [
                    LaunchConfiguration("planner_python_site"),
                    ":",
                    LaunchConfiguration("planner_root"),
                    ":",
                    EnvironmentVariable("PYTHONPATH", default_value=""),
                ],
            ),
            Node(
                package="autolife_planning_bridge",
                executable="planning_server",
                name="autolife_planning_server",
                output="screen",
                prefix=[LaunchConfiguration("planner_python"), " "],
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
