#!/usr/bin/env python3

from std_msgs.msg import Float32MultiArray
from arm_utilities.arm_enum_utils import SafetyErrors
import math


class SafetyChecker():
    def __init__(self):

        # TODO: max change in theta at a given step, need to test and put in a config file
        self.max_d_theta = [15, 15, 15, 15, 15, 15, 80000]
        # TODO: currently set arbitrarily, needs to correspond correctly with the actual arm
        self.joint_limits = [(-180, 180), (-180, 180), (-180, 180),
                             (-180, 180), (-math.inf, math.inf), (-math.inf, math.inf), (-math.inf, math.inf)]

        self.goal_pos = [0., 0., 0., 0., 0., 0., 0.]
        self.curr_pos = [0., 0., 0., 0., 0., 0., 0.]
        self.motor_curr = [0., 0., 0., 0., 0., 0., 0.]
        # TODO may need to make this with direction
        self.limit_switch = [False, False, False, False, False, False]

        # Arm position errors
        self.safe_goal_pos = Float32MultiArray()
        self.safe_goal_pos.data = [0., 0., 0., 0., 0., 0., 0.]

        self.joint_safety_status = [SafetyErrors.NONE]*len(self.goal_pos)

    def update_safe_goal_pos(self, goal_pos: list, curr_pos: list) -> None:

        # Check if the goal position is safe
        self.curr_pos = curr_pos
        self.goal_pos, pos_safety_status = self.constrain_safe_pos(
            goal_pos)
        curr_safety_status = self.current_check(self.goal_pos)

        safety_status = [0] * len(self.goal_pos)
        for i in range(len(self.goal_pos)):
            # TODO: the typing here is hardcoded, it shouldn't be
            safety_status[i] = int(pos_safety_status[i]) +\
                int(curr_safety_status[i])
        return self.goal_pos, safety_status

    def constrain_safe_pos(self, pos: list = None) -> None:

        joint_pos_safety_status = [SafetyErrors.NONE.value]*len(self.goal_pos)
        if not pos:
            pos = self.goal_pos
        safe_goal_pos = self.curr_pos.copy()

        # Doing position comparisons for safety
        # To properly update, we will take the ratio between the highest requested change and max allowable change
        # If it is > 1 we are exceeding max; scale all values down by that amount to keep on the same trajectory
        # If we just clamped a single joint we'd have undesired/awkward movements when only specific joints
        # are constrained in joint space rather than via a pose
        joint_deltas = [pos[i] - self.curr_pos[i] for i in range(len(pos))]
        max_requested_delta_ratios = [abs(joint_deltas[i] / self.max_d_theta[i]) for i in range(len(pos))]
        highest_delta_ratio = max(max_requested_delta_ratios)

        if highest_delta_ratio < 1:
            highest_delta_ratio = 1

        safe_goal_pos = [(joint_deltas[i] / highest_delta_ratio) + self.curr_pos[i] for i in range(len(pos))]

        # Update flags and limits (legacy code just kept for now in case it breaks something to remove it)
        for i in range(len(pos)):
            if max_requested_delta_ratios[i] > 1:
                # safe_goal_pos = self.curr_pos.copy()
                joint_pos_safety_status[i] = SafetyErrors.EXCEEDING_POS.value
                print("Exceeded max position change for joint ", i, flush=True)
                print("Requested: ", pos[i], " Current: ", self.curr_pos[i],
                      " Max Change: ", self.max_d_theta[i], flush=True)

            elif pos[i] <= self.joint_limits[i][0] or pos[i] >= self.joint_limits[i][1]:
                safe_goal_pos[i] = self.curr_pos[i]
                joint_pos_safety_status[i] = SafetyErrors.EXCEEDING_LIMS.value
                print("Exceeded joint limits for joint ", i)
            else:
                safe_goal_pos[i] = pos[i]

        return safe_goal_pos, joint_pos_safety_status

    def current_check(self, pos: list = None) -> None:
        """Checks the maximum current being consumed by a motor.

        If the current is higher than the expected max, sets the error for the
        particular motor as SafetyErrors.EXCEEDING_CURR. Helps us to know when
        too much torque is being applied
        """
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
