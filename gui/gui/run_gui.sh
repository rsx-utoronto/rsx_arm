#!/bin/bash

ros2 run gui arm_gui &
ros2 launch arm_launch arm_rviz_launch.py ik_on:=true