from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    # define launch arguments - rvizconfig
    rviz_config_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution([
            FindPackageShare('arm_ros2'),
            'rover/simulation/rviz_config',     # need to redefine the path
            'arm_urdf.rviz'
        ]),
        description='Full path to the rviz config file'
    ),



    # define text-based URDF
    urdf_path = PathJoinSubstitution([
        FindPackageShare('arm_ros2'),
        'rover/simulation/urdfs',       # need to redefine the path
        'Arm_2022_URDF.urdf'
    ]),

    
    # robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',

        # remappings=[('joint_states', 'arm sim_angles')]
    ),
    
    
    # joint state publisher node
    # joint_state_publisher_node = Node(
        # package='joint_state_publisher_gui',
        # executable='joint_state_publisher_gui',
        # name='joint_state_publisher',
    # ),

    # rviz node
    rviz_node = Node(
        package='rviz',
        executable='rviz',
        name='rviz',
        arguments=['d', LaunchConfiguration('rvizconfig')],
        output='screen'
    ),

    return LaunchDescription([
        rviz_config_arg,
        urdf_path,
        # robot_state_publisher_node,
        rviz_node

    ])
