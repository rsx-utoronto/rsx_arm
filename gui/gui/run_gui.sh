#!/bin/bash

rosrun rover qt5_gui.py &
roslaunch rover arm_rviz.launch ik_on:=true