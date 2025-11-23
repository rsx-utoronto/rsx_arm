from rclpy.logging import LoggingSeverity
from arm_utilities.arm_enum_utils import CANAPI
from arm_utilities.arm_test_utils import test_node, MESSAGE_WAIT, spin_n
from arm_utilities.arm_can_utils import generate_can_id
import arm_controller.main_controller as main_controller
from std_msgs.msg import Float32MultiArray, Bool, Int16
from array import array
import numpy as np
from arm_msgs.msg import ArmInputs
import time
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Joy

from arm_utilities.arm_enum_utils import ControlMode, ArmState, HomingStatus, SafetyErrors

from arm_controller.can_connection import CAN_connection
import can
import time
from arm_controller.safety import SafetyChecker

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
    #init ros
    args = None
    rclpy.init(args=args)
    #start controller in virtual mode
    controller_node = main_controller.Controller(virtual = True)
    #access CAN bus
    bus = controller_node.can_con.bus
    #expected heartbeat_can_id
    heartbeat_id = generate_can_id(
        dev_id = 0x0,
        api = CANAPI.CMD_API_NONRIO_HB.value)
    seen_heartbeat = False
    #try read few message from the bus
    for _ in range(50):
        msg=bus.recv(timeout=0.1)
        if msg is None:
            continue
        #check ID and extended flag
        if msg.arbitration_id == heartbeat_id and msg.is_extended_id:
            seen_heartbeat = True
            break
    #clean up ROS
    controller_node.shutdown_node()
    rclpy.shutdown()
    assert seen_heartbeat, "Heart message not detected on CAN bus"
            
    
    
def test_can_comm():
    '''Confirm that you can both send and receive CAN messages manually. This should be
    possible entirely isolated from ROS and just using the can_connection class'''
    can_con = CAN_connection(channel = "vcan0", interface = "virtual", receive_own_messages = True)
    
    # Create a test CAN message
    test_message = can.Message(
        arbitration_id=0x123,
        data=[1, 2, 3, 4, 5],
        is_extended_id=False
    )
    
    # Send the message
    can_con.send_message(test_message)
    
    received_message = None
    
    # Loop and skip unrelated messages (e.g., periodic heartbeat frames)
    for _ in range(50):
        msg = can_con.bus.recv(timeout=0.1)
        if msg is None:
            continue
        
        # Only accept our test frame:
        # - standard ID (not extended)
        # - correct arbitration_id
        if (not msg.is_extended_id) and (msg.arbitration_id == test_message.arbitration_id):
            received_message = msg
            break
    
    assert received_message is not None, "No looped-back test message received on vcan0"
    assert received_message.arbitration_id == test_message.arbitration_id, "Arbitration ID mismatch"
    assert list(received_message.data[:5]) == list(test_message.data), "Message data mismatch"


def test_can_joints_comm():
    '''Confirm that the target joints are being sent and received correctly via
    vcan0 by introducing some non-zero target joint, which should update via CAN automatically 
    in main_controller code at a given interval.'''
    # Init ROS
    args = None
    rclpy.init(args=args) 
    controller_node = main_controller.Controller(virtual = True)
    #mute logging so test output is not spammed
    controller_node.get_logger().set_level(LoggingSeverity.FATAL)
    #access the can bus
    bus = controller_node.can_con.bus
    #create non-zero target joints
    target_joints = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
    #Publish these targets to the controller's "safe" target joints topic.
    #The controller should read this and then send CAN commands based on it.
    controller_node.safe_target_joints_pub.publish(
        Float32MultiArray(data = target_joints)
    )
    
    time.sleep(0.3)
    expected_joint_cmd_id = generate_can_id(
        dev_id = 0x0,
        api = CANAPI.CMD_API_NONRIO_HB.value
    )
    #store the frame we find here
    joint_frame = None
    
    # Loop and read messages from the CAN bus, trying to find our joint-target frame
    for _ in range(100):
        msg = bus.recv(timeout=0.05)  # wait up to 0.05s for each message

        # If no message arrived in this time slice, continue to next loop iteration
        if msg is None:
            continue
        
        if msg.is_extended_id and msg.arbitration_id == expected_joint_cmd_id:
            joint_frame = msg
            break
    
    controller_node.shutdown_node()
    rclpy.shutdown()
    
    assert joint_frame is not None, "No joint-target CAN frame detected on vcan0"

    # Convert payload bytes to a list so we can inspect it
    data_list = list(joint_frame.data)

    # Sanity check: payload should not be all zeros, since we sent non-zero joints
    assert any(b != 0 for b in data_list), \
        f"Joint-target CAN payload appears to be all zeros: {data_list}"
    
def test_safety():
    '''Confirm that safety constrains values as expected. Be careful with this.'''

    safety = SafetyChecker()

    # ----------------------- Test constrain_safe_pos -----------------------
    safety.curr_pos = [0.0] * 7
    unsafe_pos = [100.0] + [0.0] * 6
    safe_pos, pos_status = safety.constrain_safe_pos(unsafe_pos.copy())
    assert safe_pos[0] == safety.max_d_theta[0]
    assert pos_status[0] == SafetyErrors.EXCEEDING_POS.value

    # ----------------------- Test current_check ----------------------------
    safety.motor_curr = [0.0] * 7
    safety.motor_curr[2] = 999.0
    curr_status = safety.current_check()
    assert curr_status[2] == SafetyErrors.EXCEEDING_CURR.value

    # ----------------------- Test update_safe_goal_pos (MANUAL) ------------
    safety.STATE = ArmState.MANUAL
    safety.curr_pos = [0.0] * 7
    safety.motor_curr = [0.0] * 7

    unsafe_goal = [safety.max_d_theta[0] * 3] + [0.0] * 6
    safe_goal, status = safety.update_safe_goal_pos(unsafe_goal.copy(), safety.curr_pos)

    assert abs(safe_goal[0] - safety.max_d_theta[0]) < 1e-6
    assert status[0] == int(SafetyErrors.EXCEEDING_POS.value)

    # ----------------------- Test update_safe_goal_pos (IK) ------------
    safety.STATE = ArmState.IK   # <----- now we also test IK mode!

    unsafe_goal = [safety.max_d_theta[0] * 3] + [0.0] * 6
    safe_goal, status = safety.update_safe_goal_pos(unsafe_goal.copy(), safety.curr_pos)

    assert abs(safe_goal[0] - safety.max_d_theta[0]) < 1e-6
    assert status[0] == int(SafetyErrors.EXCEEDING_POS.value)