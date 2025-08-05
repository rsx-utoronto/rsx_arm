#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # parameters
    controller_yaml = PathJoinSubstitution(
        [FindPackageShare('rover'), 'rover', 'simulation', 'config', 'arm_2023_joints.yaml']
    )

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
    rvizconfig_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution(
            [FindPackageShare('rover'), 'rover', 'simulation', 'rviz_config', 'arm_urdf.rviz']
        ),
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution(
            [FindPackageShare('rover'), 'rover', 'simulation', 'urdfs', 'Arm_2023', 'Arm_2023_gazebo.xacro']
        ),
    )

    paused_arg = DeclareLaunchArgument(
        'paused',
        default_value='false',
    )
    
    ik_on_arg = DeclareLaunchArgument(
        'ik_on',
        default_value='false',
    )

    gazebo_on_arg = DeclareLaunchArgument(
        'gazebo_on',
        default_value='true',
    )

    return LaunchDescription([
        rvizconfig_arg,
        model_arg,
        paused_arg,
        ik_on_arg,
        gazebo_on_arg,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'empty_world.launch.py'
                ]),
            ),
            launch_arguments={
                'paused': LaunchConfiguration('paused')
            }.items()
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_model',
            name='spawn_urdf',
            arguments=[
                '-file', LaunchConfiguration('model'),
                '-urdf',
                '-x', '0',
                '-y', '0',
                '-z', '0',
                '-model', 'arm'
            ],
            output='screen',
        ),

        # Launch ros2_control_node to load controllers
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_yaml],
            namespace='arm',
            output='screen',
        ),

        # Launch state broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_controller', '--controller-manager-timeout', '50'],
            namespace='arm',
            output='screen',
        ),

        # Spawn each individual position controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint1_position_controller'],
            namespace='arm',
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint2_position_controller'],
            namespace='arm',
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint3_position_controller'],
            namespace='arm',
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint4_position_controller'],
            namespace='arm',
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint5_position_controller'],
            namespace='arm',
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint6_position_controller'],
            namespace='arm',
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint7_position_controller'],
            namespace='arm',
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint8_position_controller'],
            namespace='arm',
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint9_position_controller'],
            namespace='arm',
            output='screen',
        ),

        Node(
            package='rover',
            executable='arm_inverse_kinematics',
            name='arm_inverse_kinematics',
            condition=IfCondition(LaunchConfiguration('ik_on')),
            output='screen',
        ),

        Node(
            package='rover',
            executable='fake_manual',
            name='fake_manual',
            condition=IfCondition(LaunchConfiguration('ik_on')),
            output='screen',
        ),

        Node(
            package='rover',
            executable='arm_viz',
            name='arm_visualization',
            parameters=[{'gazebo_on': True}],
            output='screen',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rob_st_pub',
            parameters=[robot_description],
            remappings=[('/joint_states', '/arm/joint_states')],
            output='screen',
        ),
    ])