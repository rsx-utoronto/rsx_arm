#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # Arguments
    # gazebo_on_arg = DeclareLaunchArgument(
    #     'gazebo_on',
    #     default_value='false',
    # )

    # ik_on_arg = DeclareLaunchArgument(
    #     'ik_on',
    #     default_value='false',
    # )

    # rvizconfig_arg = DeclareLaunchArgument(
    #     'rvizconfig',
    #     default_value=PathJoinSubstitution(
    #         [FindPackageShare('rover'), 'rover', 'simulation', 'rviz_config', 'arm_urdf.rviz']
    #     ),
    # )

    return LaunchDescription([
        # gazebo_on_arg,
        # ik_on_arg,
        # rvizconfig_arg,

        Node(
            # no param
            package='rover',
            executable='arm_controller',
            name='Arm_Controller',
            output='screen',
        ),
        # Node(
        #     # no param
        #     package='rover',
        #     executable='manual',
        #     name='Arm_Manual',
        #     output='screen',
        # ),
        Node(
            # no param
            package='rover',
            executable='safety',
            name='Arm_Safety',
            output='screen',
        ),
        # Node(
        #     # no param
        #     package='rover',
        #     executable='arm_sci',
        #     name='Arm_Sci',
        #     output='screen',
        # ),
        # Node(
        #     # no param
        #     package='rover',
        #     executable='repub_wrist_cam',
        #     name='repub_wrist_cam',
        #     output='screen',
        # ),
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='rob_st_pub',
        #     parameters=[robot_description],
        # ),
        # Node(
        #     package='rover',
        #     executable='arm_ik_and_viz',
        #     name='arm_ik_and_viz',
        #     output='screen',
        #     condtion=IfCondition(LaunchConfiguration('ik_on')),
        # ),
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher',
        #     parameters=[robot_description],
        # ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz',
        #     arguments=['-d', LaunchConfiguration('rvizconfig')],
        #     # on_exit=[Shutdown()], 
        # ),
        # Node(
        #     package='rover',
        #     executable='CAN_send',
        #     name='CAN_send',
        #     output="screen"
        # ),
        # Node(
        #     package='rover',
        #     executable='CAN_recv',
        #     name='CAN_recv',
        #     output="screen"
        # ),

    ])