from arm_utilities.arm_control_utils import *
import time
import rclpy
import pytest
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
from pynput import keyboard

import sys
sys.path.insert(0, "../arm_utilities")


# === Tests for handle_joy_input ===
def test_handle_joy_input_dpad():

    # D-Pad left
    joy_msg = Joy(axes=[0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, -1.0, 0.0], buttons=[0]*13)
    arm_inputs = handle_joy_input(joy_msg)
    assert arm_inputs.dpad_left == 1.0
    assert arm_inputs.dpad_right == 0.0
    assert arm_inputs.dpad_up == 0.0
    assert arm_inputs.dpad_down == 0.0

    # D-Pad right
    joy_msg.axes[6] = 1.0
    arm_inputs = handle_joy_input(joy_msg)
    assert arm_inputs.dpad_right == 1.0

    # D-Pad up
    joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    arm_inputs = handle_joy_input(joy_msg)
    assert arm_inputs.dpad_up == 1.0

    # D-Pad down
    joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]
    arm_inputs = handle_joy_input(joy_msg)
    assert arm_inputs.dpad_down == 1.0


# === Tests for handle_keyboard_input ===
def test_handle_keyboard_input_char():
    arm_inputs = handle_keyboard_input(type("FakeKey", (), {"char": "w"})())
    assert arm_inputs.l_vertical == 1.0


def test_handle_keyboard_input_special():
    arm_inputs = handle_keyboard_input(keyboard.Key.up)
    assert arm_inputs.dpad_up == 1.0


# === Tests for map_inputs_to_manual ===
def test_map_inputs_to_manual():
    arm_inputs = ArmInputs()
    arm_inputs.l_horizontal = 1.0
    arm_inputs.l_vertical = 0.5
    arm_inputs.r_horizontal = -0.5
    arm_inputs.r_vertical = 1.0
    arm_inputs.r1 = 1
    arm_inputs.l1 = 0
    arm_inputs.r_trigger = 1.0
    arm_inputs.l_trigger = 0.0

    speed_limits = [1, 1, 1, 1, 1, 1, 1]
    current_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    manual_cmds = map_inputs_to_manual(
        arm_inputs, speed_limits, current_joints)
    manual_cmds = manual_cmds.data  # convert dict_values to list
    assert manual_cmds[0] == 1.0    # base_rotation
    assert manual_cmds[1] == 0.5    # shoulder
    assert manual_cmds[4] == 1      # elbow_roll (R1 pressed)


# === Tests for map_inputs_to_ik ===
def test_map_inputs_to_ik():
    arm_inputs = ArmInputs()
    arm_inputs.l_horizontal = 1.0
    arm_inputs.l_vertical = -1.0
    arm_inputs.r_trigger = 1.0
    arm_inputs.l_trigger = 0.0
    arm_inputs.r_horizontal = 1.0

    curr_pose = Pose()
    curr_pose.position.x = 0.0
    curr_pose.position.y = 0.0
    curr_pose.position.z = 0.0
    curr_pose.orientation.z = 0.0

    new_pose = map_inputs_to_ik(arm_inputs, curr_pose)
    assert new_pose.position.x > 0.0     # moved in x
    assert new_pose.position.y < 0.0    # moved in -y
    assert new_pose.position.z > 0.0     # moved up
    assert new_pose.orientation.z > 0.0  # rotated yaw


# === Tests for clamp ===
def test_clamp():
    assert clamp(5, 0, 10) == 5
    assert clamp(-1, 0, 10) == 0
    assert clamp(11, 0, 10) == 10
