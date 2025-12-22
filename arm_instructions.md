# How to run the arm
## Starting up
1. Connect arm to power supply, ensure daisy chaining is done correctly and all Sparkmaxes/motors have power
2. Connect arm CAN network to computer via CAN USB, ensure all connections are secure
3. run setup-can.sh in the rsx-arm package to initialize CAN network
4. Turn on the power supply output 
5. Open a terminal and run:
```bash
ros2 run joy joy_node
```
6. Run:
```bash
ros2 run arm_controller main_controller
```
7. If running IK, run (DO NOT RUN WITH IK RIGHT NOW):
```bash
ros2 launch moveit_path_planning planner_server_topic_publisher.py
```

8. Controls:
States (D-Pad): IDLE: down, MANUAL: up, IK: left, PATH PLANNING: right (disabled)
Joints: 
Left joystick horizontal: base joint
Left joystick vertical: shoulder
Right joystick vertical: elbow pitch
Right joystick horizontal: elbow roll
Left/right triggers: wrist roll
Left/right bumpers: wrist pitch

## Things to look out for
- Joint state retention post program crash is not fully tested - be cautious when restarting the controller script after it has been shutdown and the arm was moved 
- Always turn off the controller script prior to cutting power to the CAN network if possible - otherwise CAN network buffer will fill up and the CAN network will need to be restarted (power cycle)