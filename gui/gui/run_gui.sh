#!/bin/bash

ros2 run rover qt5_gui.py &
ros2 launch arm_rviz.launch.py ik_on:=true