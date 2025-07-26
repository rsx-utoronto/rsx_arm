from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    arm_assemble_pc_node = Node(
        package='arm-ros2',
        executable='arm_point_cloud_assembler',
        name='arm_pc_assembler',
        output='screen',

        remappings=[('cloud_input', '/camera/dpeth/points')]
    )
    
    return LaunchDescription([
        arm_assemble_pc_node      
    ])