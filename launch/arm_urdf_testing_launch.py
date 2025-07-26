#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description = {
        'robot_description':Command([
            'xacro',
            '--inorder',
            PathJoinSubstitution([
                FindPackageShare('rover'),
                'rover',
                'simulation',
                'urdfs',
                'new_urdf',
                'urdf',
                'URDF.xacro',
            ])
        ])
    }

    rvizconfig_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution(
            [FindPackageShare('rover'), 'rover', 'simulation', 'rviz_config', 'arm_urdf.rviz']
        ),
    )

    return LaunchDescription([
        rvizconfig_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rob_st_pub',
            parameters=[robot_description],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            parameters=[robot_description],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            on_exit=[Shutdown()], 
        ),
    ])

