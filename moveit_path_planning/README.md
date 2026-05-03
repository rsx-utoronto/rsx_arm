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
### Set arm_state
```bash
ros2 topic pub arm_state std_msgs/msg/String  "{'data': 'PATH_PLANNING'}" --once
```

### Test positions

```bash
ros2 topic pub arm_target_pose  geometry_msgs/msg/Pose "{position: {x: 0.12699, y: 0.85208, z: 0.34692}, orientation: {x: 4.186e-05, y: -1.6316e-05, z: 0.7132, w: 0.70096}}" --once 
```

```bash
ros2 topic pub arm_target_pose  geometry_msgs/msg/Pose "{position: {x: 0.034679, y: 0.3559, z: 0.85175}, orientation: {x: 0.21512, y: -0.21142, z: 0.67998, w: 0.66832}}" --once 
```

```bash
ros2 topic pub arm_target_pose  geometry_msgs/msg/Pose "{position: {x: 0.85396, y: 0.021767, z: 0.19615}, orientation: {x: 0.2152, y: -0.21142, z: 0.67992, w: 0.66835}}" --once 
```
original:
0.054901; 0.36896; 0.58728
0; 0; 0.70683; 0.70739

0.12699; 0.85208; 0.34692
4.186e-05; -1.6316e-05; 0.7132; 0.70096

ros2 topic pub arm_target_pose  geometry_msgs/msg/Pose "{position: {x: 0.12699, y: 0.85208, z: 0.34692}, orientation: {x: 4.186e-05, y: -1.6316e-05, z: 0.7132, w: 0.70096}}" --once 

0.034679; 0.3559; 0.85175
0.21512; -0.21142; 0.67998; 0.66832

ros2 topic pub arm_target_pose  geometry_msgs/msg/Pose "{position: {x: 0.034679, y: 0.3559, z: 0.85175}, orientation: {x: 0.21512, y: -0.21142, z: 0.67998, w: 0.66832}}" --once 

0.85396; 0.021767; 0.19615
0.2152; -0.21142; 0.67992; 0.66835

ros2 topic pub arm_target_pose  geometry_msgs/msg/Pose "{position: {x: 0.85396, y: 0.021767, z: 0.19615}, orientation: {x: 0.2152, y: -0.21142, z: 0.67992, w: 0.66835}}" --once 