from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Arm_Controller node
    arm_controller_node = Node(
        package='arm-ros2',
        executable='arm_controller.py',
        name='Arm_Controller',
        output='screen'
    ),

    # Arm_Manual node
    arm_manual_node = Node(
        package='arm-ros2',
        executable='manual.py',
        name='Arm_Manual',
        output='screen'
    ),

    # Arm_Safety node
    arm_safety_node = Node(
        package='arm-ros2',
        executable='safety.py',
        name='Arm_Safety',
        output='screen'
    )

    return LaunchDescription([
        arm_controller_node,
        arm_manual_node,
        arm_safety_node        
    ])

'''
Need to convert the following to Python (it was commented out in XML

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
'''