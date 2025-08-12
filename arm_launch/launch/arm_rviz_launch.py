#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # parameters
    sim_share = get_package_share_directory('simulation')

    urdf_path = os.path.join(sim_share, 'urdfs', 'Arm_URDF_2025.urdf')
    rviz_path = os.path.join(sim_share, 'urdfs', 'arm_urdf.rviz')

    # Read URDF
    try:
        with open(urdf_path, 'r') as f:
            robot_description = f.read()
    except (OSError, IOError) as e:
        print(f"Error opening URDF file at {urdf_path}: {e}")
        robot_description = ""

    # arguments
    gazebo_on_arg = DeclareLaunchArgument(
        'gazebo_on',
        default_value='false',
    )

    ik_on_arg = DeclareLaunchArgument(
        'ik_on',
        default_value='false',
    )

    rvizconfig_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=rviz_path,
    )

    # Nodes
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    viz_node = Node(
        package='visualization',
        executable='arm_viz',
        name='visualization',
        parameters=[{'gazebo_on': LaunchConfiguration('gazebo_on')}],
        output='screen',
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen',
    )

    # Only launch RViz after RSP starts
    rviz_after_rsp = RegisterEventHandler(
        OnProcessStart(
            target_action=rsp_node,
            on_start=[rviz_node]
        )
    )

    return LaunchDescription([
        gazebo_on_arg,
        ik_on_arg,
        rvizconfig_arg,
        rsp_node,
        viz_node,
        jsp_gui_node,
        rviz_after_rsp
    ])
