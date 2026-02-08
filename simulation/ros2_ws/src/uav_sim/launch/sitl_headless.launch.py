"""Headless SITL launch — convenience wrapper for CI pipelines.

Identical to ``sitl.launch.py`` but defaults to ``headless:=true`` so that
no Gazebo GUI is started.  Useful in CI environments and Docker containers
where a display server is not available.

Usage:
  ros2 launch uav_sim sitl_headless.launch.py
  ros2 launch uav_sim sitl_headless.launch.py world:=outdoor_obstacles

Requirement refs: SIM003, SIM005, SIM012
"""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """Build and return the headless SITL launch description."""
    # ----------------------------------------------------------------- #
    # Paths
    # ----------------------------------------------------------------- #
    pkg_share = get_package_share_directory("uav_sim")

    _this_file = Path(__file__).resolve()
    _sim_dir = _this_file.parents[4]  # -> simulation/
    gazebo_dir = _sim_dir / "gazebo"
    worlds_dir = gazebo_dir / "worlds"
    models_dir = gazebo_dir / "models"

    # ----------------------------------------------------------------- #
    # Launch arguments
    # ----------------------------------------------------------------- #
    arg_world = DeclareLaunchArgument(
        "world",
        default_value="empty_flat",
        description="Gazebo world SDF name (without .sdf extension)",
    )
    arg_hover_height = DeclareLaunchArgument(
        "hover_height",
        default_value="1.0",
        description="Initial hover altitude in metres",
    )

    world = LaunchConfiguration("world")
    hover_height = LaunchConfiguration("hover_height")

    world_file = PathJoinSubstitution([str(worlds_dir), PythonExpression(["'", world, ".sdf'"])])

    # ----------------------------------------------------------------- #
    # Environment
    # ----------------------------------------------------------------- #
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join(
            filter(
                None,
                [
                    str(models_dir),
                    str(worlds_dir),
                    os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
                ],
            )
        ),
    )

    # ----------------------------------------------------------------- #
    # 1. Gazebo simulator — server only (no GUI)
    # ----------------------------------------------------------------- #
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", world_file, "-r", "-s"],
        output="screen",
    )

    # ----------------------------------------------------------------- #
    # 2. Spawn quadrotor (delayed 3 s for Gazebo startup)
    # ----------------------------------------------------------------- #
    spawn_model = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "ros_gz_sim",
                    "create",
                    "-name",
                    "quadrotor",
                    "-file",
                    str(models_dir / "quadrotor" / "model.sdf"),
                    "-z",
                    "0.2",
                ],
                output="screen",
            ),
        ],
    )

    # ----------------------------------------------------------------- #
    # 3. ros_gz_bridge
    # ----------------------------------------------------------------- #
    bridge_config = os.path.join(pkg_share, "config", "bridge_topics.yaml")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    # ----------------------------------------------------------------- #
    # 4. PID controller node
    # ----------------------------------------------------------------- #
    pid_node = Node(
        package="uav_sim",
        executable="pid_controller_node",
        name="pid_controller_node",
        parameters=[
            {"use_sim_time": True},
            {"hover_height": hover_height},
            {"control_rate": 250.0},
        ],
        output="screen",
    )

    # ----------------------------------------------------------------- #
    # Assemble
    # ----------------------------------------------------------------- #
    return LaunchDescription(
        [
            arg_world,
            arg_hover_height,
            gz_resource_path,
            gz_sim,
            spawn_model,
            bridge,
            pid_node,
        ]
    )
