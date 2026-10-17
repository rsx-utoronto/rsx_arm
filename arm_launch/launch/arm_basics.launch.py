from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, EmitEvent, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def running_on_wsl():
    try:
        with open('/proc/version') as f:
            return 'microsoft' in f.read().lower()
    except OSError:
        return False

def default_kb_terminal():
    # gnome-terminal needs a D-Bus session, which WSL doesn't run by default, so use xterm there
    return 'xterm -e' if running_on_wsl() else 'gnome-terminal --'

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

    kb_terminal = LaunchConfiguration('kb_terminal')
    kb_terminal_arg = DeclareLaunchArgument(
        "kb_terminal",
        default_value=default_kb_terminal(),
        description="Terminal command the keyboard emulator is opened in (it needs its own window to read keys)",
    )
    
    # auto: kb_joy_emu publishes the standard layout; a real joystick passed through to WSL
    # reports triggers/right stick on different axes (see JOY_AXIS_LAYOUTS in arm_control_utils)
    joy_layout = LaunchConfiguration('joy_layout')
    joy_layout_arg = DeclareLaunchArgument(
        "joy_layout",
        default_value="auto",
        choices=["auto", "standard", "wsl"],
        description="Joystick axis layout: standard (native Linux, rosbags, keyboard emulator) or wsl",
    )
    resolved_joy_layout = PythonExpression([
        "'", joy_layout, "' if '", joy_layout, "' != 'auto' else ('standard' if '", use_kb,
        "'.lower() == 'true' else '", 'wsl' if running_on_wsl() else 'standard', "')"
    ])

    joy_throttle_rate = LaunchConfiguration('joy_throttle_rate')
    joy_throttle_rate_arg = DeclareLaunchArgument(
        "joy_throttle_rate",
        default_value="10.0",
        description="Max /joy messages per second passed through to the controller",
    )

    throttle_on = LaunchConfiguration('throttle_on')
    throttle_on_arg = DeclareLaunchArgument(
        "throttle_on",
        default_value="false",
        description="Rate-limit joystick input via topic_tools before the controller sees it (useful on WSL)",
    )

    # joy_node only runs when we aren't using the keyboard emulator instead
    # throttle_on:=false -- joy_node publishes straight to arm_joy, no extra hop/latency
    joy_node_direct = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        remappings=[('joy', 'arm_joy')],
        condition=IfCondition(PythonExpression(
            ["'", use_kb, "'.lower() != 'true' and '", throttle_on, "'.lower() != 'true'"]))
    )

    # throttle_on:=true -- joy_node publishes to /joy as usual, throttle node downsamples it onto arm_joy
    joy_node_for_throttle = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", use_kb, "'.lower() != 'true' and '", throttle_on, "'.lower() == 'true'"]))
    )

    # If we don't have a joystick, we can run this keyboard controller instead.
    # Same routing as joy_node: /joy straight to arm_joy, or left on /joy for the throttle node
    kb_node = Node(
        package='kb_joy_emu',
        executable='kb_joy_emulator',
        name='kb_joy_emulator',
        output='screen',
        remappings=[
            ('/joy', PythonExpression(["'/joy' if '", throttle_on, "'.lower() == 'true' else 'arm_joy'"]))
        ],
        prefix=[kb_terminal],
        condition=IfCondition(use_kb)
    )

    joy_throttle_container = ComposableNodeContainer(
        name='joy_throttle_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='topic_tools',
                plugin='topic_tools::ThrottleNode',
                name='joy_throttle',
                parameters=[{
                    'input_topic': '/joy',
                    'output_topic': 'arm_joy',
                    'throttle_type': 'messages',
                    'msgs_per_sec': joy_throttle_rate,
                }]
            )
        ],
        condition=IfCondition(throttle_on)
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
            "joy_layout": resolved_joy_layout,
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
            "joy_layout": resolved_joy_layout,
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
        kb_terminal_arg,
        joy_layout_arg,
        joy_throttle_rate_arg,
        throttle_on_arg,
        OpaqueFunction(
            function=check_args_and_launch,
            kwargs={'actions': [
                ik_controller_launch,
                joy_node_direct,
                joy_node_for_throttle,
                joy_throttle_container,
                kb_node,
                virtual_arm_controller_node,
                arm_controller_node,
                gui_node,
                rviz_launch,
                rviz_tunnel
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
