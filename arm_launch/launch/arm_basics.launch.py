from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, EmitEvent, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def check_args_and_launch(context, actions):
    use_feature = LaunchConfiguration('use_kb').perform(context).lower() == 'true'
    use_dependency = LaunchConfiguration('virtual').perform(context).lower() == 'true'

    if use_feature and not use_dependency:
        return [
            LogInfo(msg="ERROR: you cannot use the keyboard controller if you are not using virtual CAN!"),
            EmitEvent(event=Shutdown(reason='Invalid argument combination')),
        ]
    return actions

def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("arm_utilities"),
            "arm_configs",
            "arm_controller_default.yaml",
        ]),
        description="Base controller config YAML",
    )

    config_overrides_arg = DeclareLaunchArgument(
        "config_overrides",
        default_value="",
        description="Comma-separated list of controller config overrides",
    )

    virtual = LaunchConfiguration('virtual')
    virtual_arg = DeclareLaunchArgument(
        "virtual",
        default_value="false",
        description="Whether or not to use a virtual CAN bus",
    )

    ik_on = LaunchConfiguration('ik_on')
    ik_arg = DeclareLaunchArgument(
        "ik_on",
        default_value="true",
        description="Do we want to use ik stuff",
    )

    gui_on = LaunchConfiguration('gui_on')
    gui_arg = DeclareLaunchArgument(
        "gui_on",
        default_value="false",
        description="Do we need the gui running",
    )

    use_kb = LaunchConfiguration('use_kb')
    kb_arg = DeclareLaunchArgument(
        "use_kb",
        default_value="false",
        description="Do we want to control using a keyboard instead",
    )

    # Run unless we said we wanted the keyboard instead
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        remappings=[
            ('/joy', '/arm/joy')
        ],
        condition=UnlessCondition(use_kb)
    )

    # If we don't have a joystick, we can run this keyboard controller instead
    kb_node = Node(
        package='kb_joy_emu',
        executable='kb_joy_emulator',
        name='kb_joy_emulator',
        output='screen',
        remappings=[
            ('/joy', '/arm/joy')
        ],
        prefix=['gnome-terminal --'],
        condition=IfCondition(use_kb)
    )

    # Arm_Controller node
    virtual_arm_controller_node = Node(
        package='arm_controller',
        executable='virtual_controller',
        name='Arm_Controller',
        output='screen',
        parameters=[{
            "config_file": LaunchConfiguration("config_file"),
            "config_overrides": LaunchConfiguration("config_overrides"),
        }],
        condition=IfCondition(virtual)
    )

    arm_controller_node = Node(
        package='arm_controller',
        executable='main_controller',
        name='Arm_Controller',
        output='screen',
        parameters=[{
            "config_file": LaunchConfiguration("config_file"),
            "config_overrides": LaunchConfiguration("config_overrides"),
        }],
        condition=UnlessCondition(virtual)
    )

    gui_node = Node(
        package='gui',
        executable='arm_gui',
        name='Arm_GUI',
        output='screen',
        condition=IfCondition(gui_on)
    )

    ik_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('moveit_path_planning'),
                'launch',
                'planner_server_topic_publisher.launch.py'
            ])
        ),
        condition=IfCondition(ik_on)
    )

    # Rviz (if we are launching virtually we probably want it)
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('moveit_path_planning'),
                'launch',
                'rviz_planning_demo.launch.py'
            ])
        ),
        condition=IfCondition(virtual)
    )

    # The node that feeds the arm's joints to rviz in virtual mode
    rviz_tunnel = Node(
        package='moveit_path_planning',
        executable='rviz_sim',
        name='Rviz_Tunnel',
        output='screen',
        condition=IfCondition(virtual)
    )

    return LaunchDescription([
        config_file_arg,
        config_overrides_arg,
        virtual_arg,
        gui_arg,
        ik_arg,
        kb_arg,
        OpaqueFunction(
            function=check_args_and_launch,
            kwargs={'actions': [
                ik_controller_launch,
                joy_node,
                virtual_arm_controller_node,
                arm_controller_node,
                gui_node,
                rviz_launch,
                rviz_tunnel,
                kb_node
            ]}
        ), # don't use the keyboard controller if we are using real arm
    ])
\
"""Need to convert the following to Python (it was commented out in XML.

<!--node name="repub_wrist_cam" pkg="rover" type="repub_wrist_cam.py" output="screen"/-->

<!-- LIK+VIZ LAUNCH STEPS -->
<!-- Load the urdf into the parameter server. -->
<!-- <param name="gazebo_on" type="boolean" value="false"/>
<param name="robot_description" command="$(find xacro)/xacro inorder $(find rover)/rover/simulation/urdfs/Arm_2023/Arm_2023.xacro"/>

<arg name="ik_on" default="false"/>
<arg name="rvizconfig" default="$(find rover)/rover/simulation/rviz_config/arm_urdf.rviz"/>

<node pkg="robot_state_publisher" type="robot_state_publisher" name="rob_st_pub"/>

<node name="arm_ik_and_viz" pkg="rover" type="arm_ik_and_viz.py" if="$(arg ik_on)" output="screen"/>
<node name="joint_state_publisher" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" unless="$(arg ik_on)"/>

<node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" required="true"/> -->

<!-- STARTING CAN NODES-->

<!--node name="CAN_send" pkg="rover" type="CAN_send.py" output="screen"/>

<node name="CAN_recv" pkg="rover" type="CAN_send.py" output="screen"/-->):
"""
