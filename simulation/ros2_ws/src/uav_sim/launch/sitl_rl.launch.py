"""SITL launch file: Gazebo GUI + ros_gz_bridge + RL policy node.

Identical to sitl.launch.py but replaces the PID controller with the
trained RL policy node for visual evaluation in Gazebo.

Launch arguments:
  world        — World SDF file name without extension (default: empty_flat)
  model_path   — Path to trained SB3 model .zip file (REQUIRED)
  hover_height — Goal hover altitude in metres (default: 1.0)

Usage:
  ros2 launch uav_sim sitl_rl.launch.py model_path:=/path/to/model.zip
  ros2 launch uav_sim sitl_rl.launch.py model_path:=/path/to/model.zip hover_height:=2.0
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
    """Build and return the RL SITL launch description."""
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
    arg_model_path = DeclareLaunchArgument(
        "model_path",
        description="Path to trained SB3 model (.zip file)",
    )
    arg_hover_height = DeclareLaunchArgument(
        "hover_height",
        default_value="1.0",
        description="Goal hover altitude in metres",
    )

    world = LaunchConfiguration("world")
    model_path = LaunchConfiguration("model_path")
    hover_height = LaunchConfiguration("hover_height")

    world_file = PathJoinSubstitution(
        [str(worlds_dir), PythonExpression(["'", world, ".sdf'"])]
    )

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
    # 1. Gazebo simulator (always with GUI for visual evaluation)
    # ----------------------------------------------------------------- #
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", world_file, "-r"],
        output="screen",
    )

    # ----------------------------------------------------------------- #
    # 2. Spawn the quadrotor model
    # ----------------------------------------------------------------- #
    spawn_model = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "ros_gz_sim", "create",
                    "-name", "quadrotor",
                    "-file", str(models_dir / "quadrotor" / "model.sdf"),
                    "-z", "0.2",
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
    # 4. RL policy node (replaces PID controller)
    #    Uses same node name "pid_controller_node" for bridge topic
    #    namespace compatibility (bridge routes ~/odometry etc.)
    # ----------------------------------------------------------------- #
    rl_node = Node(
        package="uav_sim",
        executable="rl_policy_node",
        name="pid_controller_node",  # Same name for topic namespace compat
        parameters=[
            {"use_sim_time": True},
            {"model_path": model_path},
            {"control_rate": 50.0},
            {"goal_x": 0.0},
            {"goal_y": 0.0},
            {"goal_z": hover_height},
            {"max_rot_velocity": 838.0},
            {"position_scale": 5.0},
            {"velocity_scale": 5.0},
            {"angular_velocity_scale": 10.0},
        ],
        output="screen",
    )

    # ----------------------------------------------------------------- #
    # Assemble
    # ----------------------------------------------------------------- #
    return LaunchDescription(
        [
            arg_world,
            arg_model_path,
            arg_hover_height,
            gz_resource_path,
            gz_sim,
            spawn_model,
            bridge,
            rl_node,
        ]
    )
