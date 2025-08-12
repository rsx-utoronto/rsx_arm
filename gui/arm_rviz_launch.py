#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # parameters
    robot_description = {
        'robot_description':Command([
            'xacro',
            '--inorder',
            PathJoinSubstitution([
                FindPackageShare('rover'),
                'rover',
                'simulation',
                'urdfs',
                'Arm_2023',
                'Arm_2023_gazebo.xacro',
            ])
        ])
    }

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
        default_value=PathJoinSubstitution(
            [FindPackageShare('rover'), 'rover', 'simulation', 'rviz_config', 'arm_urdf.rviz']
        ),
    )

    return LaunchDescription([
        # oink oink I am a pig oink oink
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
            package='rover',
            executable='arm_viz',
            name='arm_visualization',
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