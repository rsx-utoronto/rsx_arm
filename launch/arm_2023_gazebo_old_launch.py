from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define path shortcuts
    rover_pkg = FindPackageShare('rover')
    gazebo_pkg = FindPackageShare('gazebo_ros')

    # Declare rviz config arguments
    rviz_config_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution([
            rover_pkg, 'rover/simulation/rviz_config', 'arm_urdf.rviz'
        ]),
        description='Path to RViz config file'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([
            rover_pkg, 'rover/simulation/urdfs/Arm_2023', 'bruh_with_EE_gazebo.xacro'
        ]),
        description='Path to robot xacro file'
    )

    # xacro robot_description command
    robot_description = Command([
        'xacro ',
        LaunchConfiguration('model'),
        ' --inorder'
    ])

    # Load joint config YAML
    joint_config_yaml = PathJoinSubstitution([
        rover_pkg, 'rover/simulation/config', 'bruh_EE_joints.yaml'
    ])

    # Launch Gazebo empty world
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gazebo_pkg, 'launch', 'empty_world.launch.py'
        ]),
        launch_arguments={'paused': 'false'}.items()
    )

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rob_st_pub',
        output='screen',
        respawn=False,
        parameters=[
            {'gazebo_on': True},
            {'robot_description': robot_description}
        ],
        remappings=[
            ('/joint_states', '/arm/joint_states')
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )

    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_model',
        name='spawn_urdf',
        arguments=[
            '-file', LaunchConfiguration('model'),
            '-urdf', '-x', '0', '-y', '0', '-z', '0',
            '-model', 'arm'
        ],
        output='screen'
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='/arm',
        name='controller_spawner',
        arguments=[
            'joint1_position_controller',
            'joint2_position_controller',
            'joint3_position_controller',
            'joint4_position_controller',
            'joint5_position_controller',
            'joint6_position_controller',
            'joint7_position_controller',
            'joint8_position_controller',
            'joint_state_controller'
        ],
        output='screen',
        respawn=False
    )

    # Load parameters with ros2param alternative
    # Recommended alternative is to include this YAML in the Node’s `parameters=[...]` if compatible
    load_joints_yaml = Node(
        package='ros2param',
        executable='ros2param',
        name='load_arm_joints',
        arguments=['load', '/arm', joint_config_yaml],
        output='screen'
    )

    return LaunchDescription([
        rviz_config_arg,
        model_arg,
        gazebo_world,
        robot_state_publisher,
        rviz_node,
        spawn_model,
        load_joints_yaml,
        controller_spawner
    ])
