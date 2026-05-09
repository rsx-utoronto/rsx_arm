# Change state to path planning
ros2 topic pub --once /arm_state std_msgs/msg/String "{data: 'PATH_PLANNING'}"

# Send a pose to the arm
#ros2 topic pub --once /arm_target_pose geometry_msgs/msg/Pose "{position: {x: 0.4, y: 0.4, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
ros2 topic pub arm_target_pose  geometry_msgs/msg/Pose "{position: {x: 0, y: 0.5, z: 0.7}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}" --once
# Send joints to the rviz topic to move
# ros2 topic pub /joint_states sensor_msgs/msg/JointState "{name: ['joint_1','joint_2','joint_3', 'joint_4','joint_5','joint_6', 'finger_joint_1', 'finger_joint_2'], position: [0.0, 1.0, -0.5, 1.0, 1.0, 1.0, 1.0, 1.0]}"
