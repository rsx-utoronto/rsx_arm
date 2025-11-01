from rclpy.logging import LoggingSeverity
from arm_utilities.arm_enum_utils import ArmState
from arm_utilities.arm_test_utils import test_node, MESSAGE_WAIT
import arm_controller.main_controller as main_controller
from arm_controller.safety import Safety_Node

import can
from arm_can.CAN_recv import CAN_Recv
from arm_can.CAN_send import CAN_Send
from arm_can import CAN_utilities as can_utils
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


def test_heartbeat():
    can_utils.initialize_bus("vcan0", "virtual")
    can_utils.BUS.receive_own_messages = True
    hb = can.Message(
    arbitration_id= can_utils.generate_can_id(
        dev_id= 0x0,
        api= can_utils.CMD_API_NONRIO_HB),
    data= bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]),
    is_extended_id= True,
    is_remote_frame = False,
    is_error_frame = False)
    hb_send = can_utils.BUS.send(hb, 0.1)
    time.sleep(MESSAGE_WAIT)
    msg = can_utils.BUS.recv(timeout = 1.0)
    assert msg.data == hb.data

def test_can_message_send():
    can_utils.initialize_bus("vcan0", "virtual")
    can_utils.BUS.receive_own_messages = True
    hb = can.Message(
    arbitration_id= can_utils.generate_can_id(
        dev_id= 0x0,
        api= can_utils.CMD_API_NONRIO_HB),
    data= bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]),
    is_extended_id= True,
    is_remote_frame = False,
    is_error_frame = False)
    hb_send = can_utils.BUS.send(hb, 0.1)
    time.sleep(MESSAGE_WAIT)
    msg = can_utils.BUS.recv(timeout = 1.0)
    assert msg.data == hb.data

    id= can_utils.generate_can_id(
        dev_id= 0x0,
        api= can_utils.CMD_API_NONRIO_HB)

    can_utils.send_can_message(id, data= [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
    time.sleep(MESSAGE_WAIT)
    msg = can_utils.BUS.recv(timeout = 1.0)
    data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
    assert msg.data == data

def test_full_can():

    can_utils.initialize_bus("vcan0", "virtual")
    can_utils.BUS.receive_own_messages = True
    hb = can.Message(
    arbitration_id= can_utils.generate_can_id(
        dev_id= 0x0,
        api= can_utils.CMD_API_NONRIO_HB),
    data= bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]),
    is_extended_id= True,
    is_remote_frame = False,
    is_error_frame = False)
    hb_send = can_utils.BUS.send(hb, 0.1)
    time.sleep(MESSAGE_WAIT)
    msg = can_utils.BUS.recv(timeout = 1.0)
    assert msg.data == hb.data



    rclpy.init()
    can_send = CAN_Send()
    can_recv = CAN_Recv(can_utils.BUS)
    controller_node = main_controller.Controller()
    safety_node = Safety_Node()
    test = test_node([("joy_node", Joy, "/joy")], [])

    assert rclpy.ok(), "rclpy did not initialize correctly"
    assert controller_node.state == ArmState.IDLE, "Main Controller did not initialize to Idle"

    test.test_publishers["joy_node"].publish(
        Joy(axes=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], buttons=[0]*13))
    rclpy.spin_once(controller_node, timeout_sec=1)
    time.sleep(MESSAGE_WAIT)
    assert controller_node.state == ArmState.MANUAL, "Node did not update status to MANUAL"

    init_joints = can_utils.generate_data_packet([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    for i in range(7):
        id = can_utils.generate_can_id(dev_id=11, api=can_utils.CMD_API_STAT2)

        default_joint_pos = can.Message(
        arbitration_id= id,
        data= init_joints[i],
        is_extended_id= True,
        is_remote_frame = False,
        is_error_frame = False)
        can_utils.BUS.send(default_joint_pos, 0.1)
        time.sleep(MESSAGE_WAIT)
        rclpy.spin_once(can_recv, timeout_sec = MESSAGE_WAIT)
        time.sleep(MESSAGE_WAIT)
        rclpy.spin_once(can_send, timeout_sec = MESSAGE_WAIT)
    assert len(can_send.CURR_POS) == 7
        
    
    joy_msg = Joy()
    joy_msg.axes = [
        1.0,   # l_horizontal
        0.0,    # l_vertical
        0.0,   # l_trigger raw
        0.0,    # r_horizontal
        0.0,    # r_vertical
        0.0,   # r_trigger raw
        0.0,    # dpad horizontal
        0.0     # dpad vertical
    ]
    buttons = [0]*13
    joy_msg.buttons = buttons

    # safety node should initialize with 0 goal positions
    assert len(safety_node.goal_pos) == 7
    for item in safety_node.goal_pos:
        assert item == 0.0
    for item in can_send.CURR_POS:
        assert item == 0.0
    assert can_send.triggered == 1

    test.test_publishers["joy_node"].publish(joy_msg)
    rclpy.spin_once(controller_node, timeout_sec=MESSAGE_WAIT)
    time.sleep(MESSAGE_WAIT)
    expected_raw_targets = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    rclpy.spin_once(safety_node, timeout_sec=MESSAGE_WAIT)
    time.sleep(MESSAGE_WAIT)

    # this is a safe goal position, they should be equivalent
    assert len(safety_node.goal_pos) == len(expected_raw_targets)
    for n, item in enumerate(safety_node.goal_pos):
        assert item - \
            expected_raw_targets[n] < 1e-3, "Difference %f was greater than 1e-3" % item-expected_raw_targets[n]

    rclpy.spin_once(can_send, timeout_sec=MESSAGE_WAIT)
    time.sleep(MESSAGE_WAIT)
    assert len(can_send.SAFE_GOAL_POS) == len(expected_raw_targets)
    for n, item in enumerate(can_send.SAFE_GOAL_POS[0:6]):
        assert item - \
            expected_raw_targets[n] < 1e-3, "Difference %f was greater than 1e-3" % item-expected_raw_targets[n]

    spark_input = can_utils.generate_data_packet(
            can_send.SAFE_GOAL_POS)

    # Send data packets
    expected_msgs = []
    for i in range(1, len(spark_input)+1):

        # Motor number corresponds with device ID of the SparkMAX
        motor_num = 10 + i
        
        if motor_num > 10 and motor_num < 18:
            # API WILL BE CHANGED WHEN USING THE POWER (DC) SETTING
            id = can_utils.generate_can_id(dev_id=motor_num, api=can_utils.CMD_API_POS_SET)
            expected_msgs.append(can.Message(
            arbitration_id= id,
            data= spark_input[i-1],
            is_extended_id= True,
            is_remote_frame = False,
            is_error_frame = False))
        else:
            break
    time.sleep(1.0)
    

    for i in range(len(spark_input)):
        msg = can_utils.BUS.recv(timeout = 1.0)
        assert msg.data == expected_msgs[i].data
    # assert msg != None
    rclpy.shutdown()
