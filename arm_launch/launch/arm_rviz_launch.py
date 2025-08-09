#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # parameters
    sim_share = get_package_share_directory('simulation')
    viz_share = get_package_share_directory('visualization')

    urdf_path = os.path.join(sim_share, 'urdfs', 'arm_circ_2024.urdf')
    rviz_path = os.path.join(viz_share, 'rviz_config', 'arm_urdf.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

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

    return LaunchDescription([
        gazebo_on_arg,
        ik_on_arg,
        rvizconfig_arg,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rob_st_pub',
            parameters=[robot_description],
        ),

        Node(
            package='visualization',
            executable='arm_viz',
            name='visualization',
            parameters=[{'gazebo_on': True}],
            output='screen',
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            parameters=[robot_description],
            condition=UnlessCondition(LaunchConfiguration('ik_on')),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            output='screen',
        ),
    ])