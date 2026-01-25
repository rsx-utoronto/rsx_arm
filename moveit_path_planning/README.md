## Remember to source and build!
In every terminal you start, run
```bash
workon_arm
```

In one of them, run:
```bash
build_arm
```

## How this branch simulates the arm
The JointsToRviz node intercepts the joint angles sent from the main_controller node and publishes them, in the correct format, to the /joint_states topic, which RViz reads and uses to update the arm simulation. 

In order to properly update joints in simulation (without motor controllers), the current_pose variable in main_controller no longer reads from CAN, but simply assumes the target was reached. For that reason this branch **SHOULD NOT** be directly run on the arm with main_controller!

## What works and what doesn't
In simulation, this branch can:
* Find IK solutions with MoveIt
* Avoid solutions with collisions
* Run every motorized joint smoothly

What it can't do properly:
* Bug with the wrist (two solutions exist which are both valid; camera flips)
* Run manual
* Does not save to last valid position (the current_pose will continue to update further out of range but can be brought back)
* Be homed (this is currently hard-coded in main_controller)

## Demo
<video src="./ik_demo.mp4" controls width="600"></video>

## Using RViz to test IK:
Launch RViz with the ability to intercept joint targets:
```bash
ros2 launch moveit_path_planning rviz_planning_demo.launch.py
```
In a separate terminal, run the path_planning node, along with the helpers through a launch file:
```bash
ros2 launch moveit_path_planning planner_server_topic_publisher.py
```
In two different terminals, start the joy node as well as the **virtual** controller:

Terminal 1:
```bash
ros2 run joy joy_node
```

Terminal 2:
```bash
ros2 run arm_controller virtual_controller
```

# Using the controller (for now)

To start IK, first press the right d-pad button, then all four of the right buttons (x, o, triangle, square) as according to the controller, to override homing.
* Left joystick: +x and -x, +y and -y
* Triggers: +z and -z