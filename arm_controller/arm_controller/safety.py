#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Float32MultiArray, String, UInt8
from arm_utilities.arm_enum_utils import ArmState, SafetyErrors
from arm_utilities.arm_control_utils import clamp
import time
import os
import math


class SafetyChecker():
    def __init__(self):

        # TODO: max change in theta at a given step, need to test and put in a config file
        self.max_d_theta = [10, 10, 10, 10, 120, 120, 80000]
        # TODO: currently set arbitrarily, needs to correspond correctly with the actual arm
        self.joint_limits = [(-180, 180), (-180, 180), (-180, 180),
                             (-180, 180), (-180, 180), (-180, 180), (-math.inf, math.inf)]

        self.goal_pos = [0., 0., 0., 0., 0., 0., 0.]
        self.curr_pos = [0., 0., 0., 0., 0., 0., 0.]
        self.motor_curr = [0., 0., 0., 0., 0., 0., 0.]
        # TODO may need to make this with direction
        self.limit_switch = [False, False, False, False, False, False]
        self.STATE = ArmState.IDLE

        # Arm position errors
        self.safe_goal_pos = Float32MultiArray()
        self.safe_goal_pos.data = [0., 0., 0., 0., 0., 0., 0.]

        self.joint_safety_status = [SafetyErrors.NONE]*len(self.goal_pos)

    def update_safe_goal_pos(self, goal_pos: list, curr_pos: list) -> None:
        if self.STATE == ArmState.MANUAL or self.STATE == ArmState.IK:

            # Check if the goal position is safe
            self.goal_pos, pos_safety_status = self.constrain_safe_pos(
                goal_pos)
            curr_safety_status = self.current_check(self.goal_pos)

        else:
            return goal_pos, [0] * len(self.goal_pos)
        safety_status = [0] * len(self.goal_pos)
        for i in range(len(self.goal_pos)):
            # TODO: the typing here is hardcoded, it shouldn't be
            safety_status[i] = int(pos_safety_status[i]) + \
                int(curr_safety_status[i])
        print(goal_pos, safety_status)
        return goal_pos, safety_status

    def constrain_safe_pos(self, pos: list = None) -> None:

        joint_pos_safety_status = [SafetyErrors.NONE.value]*len(self.goal_pos)
        if not pos:
            pos = self.goal_pos
        safe_goal_pos = pos
        # Going through each element of GOAL_POS
        for i in range(len(self.goal_pos)):
            # Doing position comparisons for safety
            safe_goal_pos[i] = clamp(
                pos[i], self.curr_pos[i]-self.max_d_theta[i], self.curr_pos[i]+self.max_d_theta[i])
            if safe_goal_pos[i] != pos[i]:

                self.logger().info("Constrained joint %d due to excessive change in angle" % i)
                joint_pos_safety_status[i] = SafetyErrors.EXCEEDING_POS.value

            pos[i] = safe_goal_pos[i]

            safe_goal_pos[i] = clamp(
                pos[i], self.joint_limits[i][0], self.joint_limits[i][1])
            if safe_goal_pos[i] != pos[i]:

                self.logger().info("Constrained joint %d due to movement outside joint limits" % i)
                joint_pos_safety_status[i] = SafetyErrors.EXCEEDING_POS.value
                continue
        return safe_goal_pos, joint_pos_safety_status

    def current_check(self, pos: list = None) -> None:
        '''
        (None) -> (None)

        Checks the maximum current being consumed by a motor. If the current is higher than
        the expected max, sets the error for the particular motor as Errors.ERROR_EXCEEDING_CURRENT.
        Helps us to know when too much torque is being applied

        @parameters

        pos (list(int)) (optional): POS values to be checked. Only fill it for IK mode, otherwise keep it None
        '''
        # TODO: move to config file
        # Max current values for each motor
        max_current = [40, 40, 40, 20, 20, 20, 20]
        current_error = [SafetyErrors.NONE.value]*len(self.goal_pos)
        # Checking maximum currents
        for i in range(len(self.motor_curr)):
            if self.motor_curr[i] > max_current[i]:
                current_error[i] = SafetyErrors.EXCEEDING_CURR.value

            # Checking if the current has been under the max limit for more than 10 ms
            else:
                current_error[i] = SafetyErrors.NONE.value
        return current_error
