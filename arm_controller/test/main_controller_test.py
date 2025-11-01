from rclpy.logging import LoggingSeverity
from arm_utilities.arm_enum_utils import ArmState
from arm_utilities.arm_test_utils import test_node, MESSAGE_WAIT
import arm_controller.main_controller as main_controller
from std_msgs.msg import Float32MultiArray
import numpy as np
from arm_msgs.msg import ArmInputs
import time
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Joy

from rclpy.node import Node
import rclpy
import sys
sys.path.insert(0, "..")


def test_main_controller_init():
    args = None
    rclpy.init(args=args)
    controller_node = main_controller.Controller()

    assert rclpy.ok(), "rclpy did not initialize correctly"
    assert controller_node.state == ArmState.IDLE, "Main Controller did not initialize to Idle"
    assert controller_node.speed_limits == [0.1, 0.09, 0.15, 0.75,
                                            0.12, 0.12, 20], "speed limits does not match the values!"
    rclpy.shutdown()


def test_main_controller_state():
    args = None
    rclpy.init(args=args)
    controller_node = main_controller.Controller()
    test = test_node([("joy_node", Joy, "/joy")], [])

    # D-Pad Left -> ArmState.IK
    test.test_publishers["joy_node"].publish(
        Joy(axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0], buttons=[0]*13))
    rclpy.spin_once(controller_node, timeout_sec=1)
    time.sleep(MESSAGE_WAIT)
    assert controller_node.state == ArmState.IK, "Node did not update status to IK"

    # D-Pad Right -> ArmState.PATH_PLANNING
    test.test_publishers["joy_node"].publish(
        Joy(axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0], buttons=[0]*13))
    rclpy.spin_once(controller_node, timeout_sec=1)
    time.sleep(MESSAGE_WAIT)
    assert controller_node.state == ArmState.PATH_PLANNING, "Node did not update status to PATH_PLANNING"

    # D-Pad Up -> ArmState.MANUAL
    test.test_publishers["joy_node"].publish(
        Joy(axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], buttons=[0]*13))
    rclpy.spin_once(controller_node, timeout_sec=1)
    time.sleep(MESSAGE_WAIT)
    assert controller_node.state == ArmState.MANUAL, "Node did not update status to MANUAL"

    # D-Pad Down -> ArmState.IDLE
    test.test_publishers["joy_node"].publish(
        Joy(axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0], buttons=[0]*13))
    rclpy.spin_once(controller_node, timeout_sec=1)
    time.sleep(MESSAGE_WAIT)
    assert controller_node.state == ArmState.IDLE, "Node did not update status to IDLE"

    rclpy.shutdown()


def test_arm_goal_pos():
    args = None
    rclpy.init(args=args)
    controller_node = main_controller.Controller()
    test = test_node(
        [], [("arm_target_joints", Float32MultiArray, "arm_target_joints")])

    assert rclpy.ok(), "rclpy did not initialize correctly"

    tests = [[1., 0., 0., 0., 0., 0., 0.],
             [1.78, 94.3, -9.3, 0.1, 3.14, 5.5],
             [1.325, -34.5, 434.5, -32.14, 64.1, 12.3]
             ]

    for arr in tests:
        controller_node.target_joint_pub.publish(Float32MultiArray(data=arr))
        rclpy.spin_once(test, timeout_sec=1)
        time.sleep(0.2)
        assert test.subscriber_data["arm_target_joints"] == Float32MultiArray(
            data=arr).data

    rclpy.shutdown()


def test_arm_input_sub():
    args = None
    rclpy.init(args=args)
    controller_node = main_controller.Controller()
    # Prevent stderr from logging
    controller_node.get_logger().set_level(LoggingSeverity.FATAL)
    test = test_node([("joy_node", Joy, "/joy")],
                     [("arm_target_joints", Float32MultiArray, "arm_target_joints")])

    assert rclpy.ok(), "rclpy did not initialize correctly"
    assert controller_node.state == ArmState.IDLE, "Manual node did not initialize to Idle"

    test.test_publishers["joy_node"].publish(
        Joy(axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], buttons=[0]*13))
    rclpy.spin_once(controller_node, timeout_sec=MESSAGE_WAIT)
    time.sleep(MESSAGE_WAIT)
    assert controller_node.state == ArmState.MANUAL, "Node did not update status to MANUAL"

    rclpy.spin_once(controller_node, timeout_sec=MESSAGE_WAIT)
    time.sleep(MESSAGE_WAIT)

    rclpy.spin_once(test, timeout_sec=MESSAGE_WAIT)
    time.sleep(MESSAGE_WAIT)
    joy_msg = Joy()
    joy_msg.axes = [0.0]*8  # all axes neutral
    joy_msg.buttons = [0]*13  # all buttons unpressed

    # Publish and spin
    test.test_publishers["joy_node"].publish(joy_msg)
    rclpy.spin_once(controller_node, timeout_sec=MESSAGE_WAIT)
    time.sleep(MESSAGE_WAIT)
    rclpy.spin_once(test, timeout_sec=MESSAGE_WAIT)
    time.sleep(MESSAGE_WAIT)

    # Expected target joints: everything zero
    expected = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    received = test.subscriber_data["arm_target_joints"]
    assert len(received) == 7
    for n, item in enumerate(list(received)):
        assert item - \
            expected[n] < 1e-3, "Difference %f was greater than 1e-3" % item-expected[n]

    joy_msg = Joy()
    joy_msg.axes = [
        -10.0,   # l_horizontal
        -5.0,    # l_vertical
        -1.0,   # l_trigger raw
        5.0,    # r_horizontal
        4.0,    # r_vertical
        -1.0,   # r_trigger raw
        0.0,    # dpad horizontal
        0.0     # dpad vertical
    ]
    buttons = [0]*13
    joy_msg.buttons = buttons

    # Publish Joy
    test.test_publishers["joy_node"].publish(joy_msg)
    rclpy.spin_once(controller_node, timeout_sec=MESSAGE_WAIT)
    time.sleep(MESSAGE_WAIT)

    rclpy.spin_once(test, timeout_sec=MESSAGE_WAIT)
    time.sleep(MESSAGE_WAIT)
    # Expected based on speed limits
    expected = [-1.0, -0.45, 0.6, 3.75, 1.2, 0.6, 0.0]

    # Verify received arm_target_joints matches expected
    assert "arm_target_joints" in test.subscriber_data, "No data received on arm_target_joints"
    received = test.subscriber_data["arm_target_joints"]
    assert len(received) == 7
    for n, item in enumerate(list(received)):
        assert item - \
            expected[n] < 1e-3, "Difference %f was greater than 1e-3" % item-expected[n]

    rclpy.shutdown()
