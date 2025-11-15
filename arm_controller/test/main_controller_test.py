from rclpy.logging import LoggingSeverity
from arm_utilities.arm_enum_utils import ArmState, CANAPI
from arm_utilities.arm_test_utils import test_node, MESSAGE_WAIT, spin_n
import arm_controller.main_controller as main_controller
from std_msgs.msg import Float32MultiArray, Bool, Int16
from array import array
import numpy as np
from arm_msgs.msg import ArmInputs
import time
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Joy

from arm_utilities.arm_enum_utils import ControlMode, ArmState, HomingStatus

from rclpy.node import Node
import rclpy
import sys
sys.path.insert(0, "..")


def test_main_controller_init():
    args = None
    rclpy.init(args=args)
    controller_node = main_controller.Controller(virtual = True)

    assert rclpy.ok(), "rclpy did not initialize correctly"
    assert controller_node.state == ArmState.IDLE, "Main Controller did not initialize to Idle"
    assert controller_node.speed_limits == [0.1, 0.09, 0.15, 0.75,
                                            0.12, 0.12, 5], "speed limits does not match the values!"
    controller_node.shutdown_node()
    rclpy.shutdown()


def test_main_controller_state():
    args = None
    rclpy.init(args=args)
    controller_node = main_controller.Controller(virtual = True)
    test = test_node([("joy_node", Joy, "/joy")], [])

    # D-Pad Left -> ArmState.IK
    test.test_publishers["joy_node"].publish(
        Joy(axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0], buttons=[0]*13))
    rclpy.spin_once(controller_node)
    time.sleep(MESSAGE_WAIT)
    assert controller_node.state == ArmState.IK, "Node did not update status to IK"

    # D-Pad Right -> ArmState.PATH_PLANNING
    test.test_publishers["joy_node"].publish(
        Joy(axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0], buttons=[0]*13))
    rclpy.spin_once(controller_node)
    time.sleep(MESSAGE_WAIT)
    assert controller_node.state == ArmState.PATH_PLANNING, "Node did not update status to PATH_PLANNING"

    # D-Pad Up -> ArmState.MANUAL
    test.test_publishers["joy_node"].publish(
        Joy(axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], buttons=[0]*13))
    rclpy.spin_once(controller_node)

    time.sleep(MESSAGE_WAIT)
    assert controller_node.state == ArmState.MANUAL, "Node did not update status to MANUAL"

    # D-Pad Down -> ArmState.IDLE
    test.test_publishers["joy_node"].publish(
        Joy(axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0], buttons=[0]*13))
    rclpy.spin_once(controller_node)

    time.sleep(MESSAGE_WAIT)
    assert controller_node.state == ArmState.IDLE, "Node did not update status to IDLE"

    controller_node.shutdown_node()
    rclpy.shutdown()

def test_arm_goal_pos():
    args = None
    rclpy.init(args=args)
    controller_node = main_controller.Controller(virtual = True)
    test = test_node(
        [], [("safe_arm_target_joints", Float32MultiArray, "safe_arm_target_joints")])

    assert rclpy.ok(), "rclpy did not initialize correctly"

    tests = [[1., 0., 0., 0., 0., 0., 0.],
             [1.78, 94.3, -9.3, 0.1, 3.14, 5.5],
             [1.325, -34.5, 434.5, -32.14, 64.1, 12.3]
             ]

    for arr in tests:
        controller_node.safe_target_joints_pub.publish(Float32MultiArray(data=arr))
        rclpy.spin_once(test)

        time.sleep(0.2)
        assert test.subscriber_data["safe_arm_target_joints"] == Float32MultiArray(
            data=arr).data
    controller_node.shutdown_node()
    rclpy.shutdown()

def test_arm_input_sub():
    args = None
    rclpy.init(args=args)
    controller_node = main_controller.Controller(virtual = True)
    # Prevent stderr from logging
    controller_node.get_logger().set_level(LoggingSeverity.FATAL)
    test = test_node([("joy_node", Joy, "/joy")],
                     [("safe_arm_target_joints", Float32MultiArray, "safe_arm_target_joints")])

    assert rclpy.ok(), "rclpy did not initialize correctly"
    assert controller_node.state == ArmState.IDLE, "Manual node did not initialize to Idle"

    test.test_publishers["joy_node"].publish(
        Joy(axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], buttons=[0]*13))
    rclpy.spin_once(controller_node)

    time.sleep(MESSAGE_WAIT)
    assert controller_node.state == ArmState.MANUAL, "Node did not update status to MANUAL"

    time.sleep(MESSAGE_WAIT)
    joy_msg = Joy()
    joy_msg.axes = [0.0]*8  # all axes neutral
    joy_msg.buttons = [0]*13  # all buttons unpressed

    # Publish and spin
    test.test_publishers["joy_node"].publish(joy_msg)
    rclpy.spin_once(controller_node)
    time.sleep(MESSAGE_WAIT)
    rclpy.spin_once(test)
    time.sleep(MESSAGE_WAIT)

    # # Expected target joints: everything zero
    expected = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    received = test.subscriber_data["safe_arm_target_joints"]
    assert len(received) == 7
    for n, item in enumerate(list(received)):
        assert item - \
            expected[n] < 1e-3, "Difference %f was greater than 1e-3" % item-expected[n]

    joy_msg = Joy()
    joy_msg.axes = [
        -1.0,   # l_horizontal
        -1.0,    # l_vertical
        -1.0,   # l_trigger raw
        1.0,    # r_horizontal
        1.0,    # r_vertical
        0.0,   # r_trigger raw
        0.0,    # dpad horizontal
        0.0     # dpad vertical
    ]
    buttons = [0]*13
    # l1
    buttons[4] = -1
    joy_msg.buttons = buttons

    # Publish Joy
    assert controller_node.state == ArmState.MANUAL
    test.test_publishers["joy_node"].publish(joy_msg)
    rclpy.spin_once(controller_node)
    time.sleep(MESSAGE_WAIT)
    spin_n(3, test)
    # rclpy.spin_once(test)
    time.sleep(MESSAGE_WAIT)
    # Expected based on speed limits
    expected = [-0.1, -0.09, 0.15, 0.75, 0.12, 0.12, 0.0]

    # # Verify received arm_target_joints matches expected
    assert "safe_arm_target_joints" in test.subscriber_data, "No data received on safe_arm_target_joints"
    received = test.subscriber_data["safe_arm_target_joints"]
    assert len(received) == 7

    for n, item in enumerate(list(received)):
        assert type(item) == float
        assert type(expected[n]) == float
        assert item - expected[n] < 1e-3, "Difference %f was greater than 1e-3" % (item-expected[n])

    controller_node.shutdown_node()
    rclpy.shutdown()


def test_homing():
   ''' test homing algorithm, test joint by joint first, then full, then cancelling the homing process'''

def test_can_init():
    '''initialize main controller, confirm that heartbeat is received. 
    The virtual CAN network receives its own messages, so you should be 
    able to receive the heartbeat message and isolate it from the others. 
    You can set up the virtual CAN network on your computer by searching up
    "set up vcan0 ubuntu" on Google and the AI overview answer should be sufficient.
    Call read_message from the CAN Connection object inside main_controller directly
    multiple times and you should be able to detect the heartbeat message.'''

def test_can_comm():
    '''Confirm that you can both send and receive CAN messages manually. This should be
    possible entirely isolated from ROS and just using the can_connection class'''

def test_can_joints_comm():
    '''Confirm that the target joints are being sent and received correctly via
    vcan0 by introducing some non-zero target joint, which should update via CAN automatically 
    in main_controller code at a given interval.'''

def test_safety():
    '''Confirm that safety constrains values as expected. Be careful with this.'''