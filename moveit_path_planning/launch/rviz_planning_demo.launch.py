#!/usr/bin/env python3
"""Launch file for RViz planning demo.

Starts move_group, planner_server, and RViz with MoveIt plugin configured.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # RViz node
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("arm_moveit_config"),
        "config",
        "moveit.rviz"
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Include move_group launch
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("arm_moveit_config"),
                "launch",
                "move_group.launch.py"
            ])
        ]),
    )

    # Include planner_server launch
    planner_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("moveit_path_planning"),
                "launch",
                "planner_server.launch.py"
            ])
        ]),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items(),
    )

    # Static TF for virtual joint (if needed)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0",
                   "0.0", "0.0", "world", "base_link"],
    )

    # The node which pushes the outputs of the IK solver to RViz to update
    joint_converter_node = Node(
        package="moveit_path_planning",
        executable="rviz_sim",
        name="rviz_sim",
        output="screen",
        parameters=[
            #moveit_config.robot_description_kinematics,  # ADD THIS LINE
        ],
    )

    return [
        move_group_launch,
        planner_server_launch,
        static_tf_node,
        rviz_node,
        joint_converter_node,
    ]


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
