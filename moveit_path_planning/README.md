## MoveIt package build
```bash
colcon build --packages-select moveit_path_planning
```

### Testing the publisher:

Launch MoveIt demo:
```bash
ros2 launch arm_moveit_config demo.launch.py
```
Run path_planning from moveit_path_planning package:
```bash
ros2 run moveit_path_planning path_planning
```

Send target position to arm_target_pose topic:
```bash
ros2 topic pub arm_target_pose  geometry_msgs/msg/Pose "{position: {x: 0, y: 0.5, z: 0.7}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}" --once 
```

Subscribe to arm_target_joints for joint targets:
```bash
ros2 topic echo /arm_target_joints
```

### Rviz testing

Push target joint values to Rviz:
```bash
ros2 run moveit_path_planning rviz_sim
```