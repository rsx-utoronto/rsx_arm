#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # arguments
    rvizconfig_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution(
            [FindPackageShare('rover'), 'rover', 'simulation', 'rviz_config', 'point_cloud_testing.rviz']
        ),
    )

    return LaunchDescription([
        rvizconfig_arg,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            on_exit=[Shutdown()], 
        ),
    ])