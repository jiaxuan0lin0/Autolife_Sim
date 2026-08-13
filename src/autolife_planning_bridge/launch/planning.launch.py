import os
import sys
import sysconfig
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    repository_root = Path(
        os.environ.get("AUTOLIFE_SIM_ROOT", Path(__file__).resolve().parents[3])
    ).resolve()
    planner_root = os.environ.get(
        "AUTOLIFE_PLANNING_ROOT",
        str(repository_root / ".deps/Autolife-Planning"),
    )
    active_prefix = Path(os.environ.get("CONDA_PREFIX", sys.prefix)).resolve()
    site_candidates = sorted(active_prefix.glob("lib/python*/site-packages"))
    discovered_site = (
        str(site_candidates[-1]) if site_candidates else sysconfig.get_paths()["purelib"]
    )
    discovered_python = active_prefix / "bin/python3"
    planner_python_site = os.environ.get(
        "AUTOLIFE_PLANNING_PYTHON_SITE",
        discovered_site,
    )
    planner_python = os.environ.get(
        "AUTOLIFE_PLANNING_PYTHON",
        str(discovered_python if discovered_python.is_file() else Path(sys.executable)),
    )
    config_path = PathJoinSubstitution(
        [FindPackageShare("autolife_planning_bridge"), "config", "planning.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=config_path),
            DeclareLaunchArgument(
                "planner_root",
                default_value=planner_root,
            ),
            DeclareLaunchArgument(
                "planner_python_site",
                default_value=planner_python_site,
            ),
            DeclareLaunchArgument(
                "planner_python",
                default_value=planner_python,
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
