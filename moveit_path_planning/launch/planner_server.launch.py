#!/usr/bin/env python3
"""Launch file for the planner_server node.

Starts the MoveIt move_group and the planner_server with configuration.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # Build MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        "Arm_URDF_2025",
        package_name="arm_moveit_config"
    ).to_moveit_configs()

    # Get parameter file path
    planner_params = PathJoinSubstitution([
        FindPackageShare("moveit_path_planning"),
        "config",
        "planner_server.yaml"
    ])

    # Planner server node
    planner_server_node = Node(
        package="moveit_path_planning",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            planner_params,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return [planner_server_node]


def generate_launch_description():
    declared_arguments = []

    # Declare arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
