"""
    Launches the planner server node, along with the move_group and transforms necessary to run it.
    Will additionally launch a second node that makes requests to the server based on the informaiton 
    in the arm_target_pose topic. It will then take the service's response and publish it to arm_goal_pos
    which can then be read by safety before being published to the arm.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # Build MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        "Arm_URDF_2025",
        package_name="arm_moveit_config"
    ).to_moveit_configs()

    # Get parameter file path
    planner_params = PathJoinSubstitution([
        FindPackageShare("moveit_path_planning"),
        "config",
        "planner_server.yaml"
    ])

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

    # Planner server node
    planner_server_node = Node(
        package="moveit_path_planning",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            planner_params,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    # Node that makes requests to server to send back to topics
    path_planner_publisher_node = Node(
        package="moveit_path_planning",
        executable="path_planning",
        name="path_planning",
        output="screen",
    )

    # Static TF for virtual joint (if needed)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )


    return [planner_server_node, 
            move_group_launch, 
            path_planner_publisher_node, 
            static_tf_node,
            robot_state_publisher]

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
