"""SITL launch file: Gazebo Harmonic + ros_gz_bridge + PID controller.

Launches the full software-in-the-loop simulation stack:
  1. Gazebo Harmonic simulator with the specified world
  2. Spawn the quadrotor model into the running simulation
  3. ros_gz_bridge for topic translation (IMU, odometry, motors, clock)
  4. PID controller node with sim-time enabled

Launch arguments:
  world       — World SDF file name without extension (default: empty_flat)
  headless    — Run Gazebo in server-only mode, no GUI (default: false)
  hover_height — Initial hover altitude in metres (default: 1.0)

Usage:
  ros2 launch uav_sim sitl.launch.py
  ros2 launch uav_sim sitl.launch.py world:=outdoor_obstacles headless:=true

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
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """Build and return the SITL launch description."""
    # ----------------------------------------------------------------- #
    # Paths
    # ----------------------------------------------------------------- #
    # Package share directory (installed by colcon)
    pkg_share = get_package_share_directory("uav_sim")

    # Project-level Gazebo resources (worlds, models) — resolved relative
    # to this launch file's location in the source tree.
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
    arg_headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo without GUI (server-only mode)",
    )
    arg_hover_height = DeclareLaunchArgument(
        "hover_height",
        default_value="1.0",
        description="Initial hover altitude in metres",
    )

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    hover_height = LaunchConfiguration("hover_height")

    world_file = PathJoinSubstitution([str(worlds_dir), PythonExpression(["'", world, ".sdf'"])])

    # ----------------------------------------------------------------- #
    # Environment: tell Gazebo where to find our models
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
    # 1. Gazebo simulator
    # ----------------------------------------------------------------- #
    # With GUI (default)
    gz_sim_gui = ExecuteProcess(
        cmd=["gz", "sim", world_file, "-r"],
        output="screen",
        condition=UnlessCondition(headless),
    )
    # Headless (server only)
    gz_sim_headless = ExecuteProcess(
        cmd=["gz", "sim", world_file, "-r", "-s"],
        output="screen",
        condition=IfCondition(headless),
    )

    # ----------------------------------------------------------------- #
    # 2. Spawn the quadrotor model (with a short delay for Gazebo startup)
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
    # 3. ros_gz_bridge with YAML topic configuration
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
    # Assemble launch description
    # ----------------------------------------------------------------- #
    return LaunchDescription(
        [
            # Arguments
            arg_world,
            arg_headless,
            arg_hover_height,
            # Environment
            gz_resource_path,
            # Processes
            gz_sim_gui,
            gz_sim_headless,
            spawn_model,
            bridge,
            pid_node,
        ]
    )
