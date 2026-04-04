# How to run the arm
## Starting up
1. Connect arm to power supply, ensure daisy chaining is done correctly and all Sparkmaxes/motors have power
2. Connect arm CAN network to computer via CAN USB, ensure all connections are secure
3. run 
```bash setup-can.sh``` in the rsx-arm package to initialize CAN network
4. Turn on the power supply output 
5. Open a terminal and run:
```bash ros2 launch arm_launch arm_basics_launch.py```
This can be run with the following arguments:
- ik_on (default true)
- virtual (default false)
5. If you want the GUI, open a second terminal and run
```bash ros2 run gui arm_gui```
6. Controls:
States (D-Pad): IDLE: down, MANUAL: up, IK: left, PATH PLANNING: right (disabled)
Joints in manual: 
Left joystick horizontal: base joint
Left joystick vertical: shoulder
Right joystick vertical: elbow pitch
Right joystick horizontal: elbow roll
Left/right triggers: wrist roll
Left/right bumpers: wrist pitch

Controls in IK:
Press X, O, triangle, and square simultaneously to override homing
Left joystick horizontal: X-axis translation (left-right)
Left joystick vertical: Y-axis translation (forward-backward)
Right joystick horizontal: X-axis rotation (pitch)
Right joystick vertical: Y-axis rotation (roll)
Left/right triggers: Z-axis translation (up-down)
Left/right bumpers: Z-axis rotation (yaw)

## Things to look out for
- Joint state retention post program crash is not fully tested - be cautious when restarting the controller script after it has been shutdown and the arm was moved 
- Always turn off the controller script prior to cutting power to the CAN network if possible - otherwise CAN network buffer will fill up and the CAN network will need to be restarted (power cycle)