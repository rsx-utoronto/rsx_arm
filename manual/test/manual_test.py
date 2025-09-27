import sys
sys.path.insert(0, "../manual")
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import manual.manual as manual
from std_msgs.msg import String
import time
from arm_msgs.msg import ArmInputs
import numpy as np
from std_msgs.msg import Float32MultiArray

# TODO: move to utilities folder (can just copy past into test folers for now)
class test_node(Node):
    def __init__(self, publishers, subscribers):
        # input publishers and subscribers as (name, type, topic)

        super().__init__('test_node')
        self.test_publishers = {}
        for publisher in publishers:
            self.test_publishers[publisher[0]] = self.create_publisher(
                publisher[1], publisher[2], 10)

        self.test_subscribers = {}
        for subscriber in subscribers:
            self.test_subscribers[subscriber[0]] = self.create_subscription(
                subscriber[1], subscriber[2], self.listener_callback, 10)


def test_sample():
    assert 2+2 == 4


def test_manual_init():
    args = None
    rclpy.init(args=args)
    manual_node = manual.Manual()
    test = test_node([("arm_state", String, "arm_state")], [])

    assert rclpy.ok(), "rclpy did not initialize correctly"
    assert manual_node.status == "Idle", "Manual node did not initialize to Idle"
    test.test_publishers["arm_state"].publish(String(data="Manual"))
    rclpy.spin_once(manual_node, timeout_sec=1)
    time.sleep(0.2)
    assert manual_node.status == "Manual", "Manual node did not update status to Manual"
    rclpy.shutdown()


def test_arm_input_sub():
    args = None
    rclpy.init(args=args)
    manual_node = manual.Manual()
    test = test_node([("arm_inputs", ArmInputs, "arm_inputs")], [])

    assert rclpy.ok(), "rclpy did not initialize correctly"
    assert manual_node.status == "Idle", "Manual node did not initialize to Idle"
    assert manual_node.controller_input == [0., 0., 0., 0., 0., 0, 0]
    assert list(manual_node.goal_pos.data) == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    l_horizontal = 10.0
    l_vertical = 5.0
    r_horizontal = 5.0
    r_vertical = 4.0
    l1 = 0
    l2 = 0.0
    r1 = 10
    r2 = 5.0
    x = 5
    o = 4
    
    arm_input = ArmInputs(l_horizontal = l_horizontal,
                          l_vertical = l_vertical,
                          r_horizontal = r_horizontal,
                          r_vertical = r_vertical,
                          l1 = l1,
                          l2 = l2,
                          r1 = r1,
                          r2 = r2,
                          x = x,
                          o = o)

    test.test_publishers["arm_inputs"].publish(arm_input)
    rclpy.spin_once(manual_node, timeout_sec=2)
    time.sleep(0.2)
    assert manual_node.controller_input == [l_horizontal, l_vertical, r_vertical, r_horizontal, l1 - r1, l2 - r2, x - o]

    rclpy.shutdown()

def test_arm_input_sub_manual():
    args = None
    rclpy.init(args=args)
    manual_node = manual.Manual()
    test = test_node([("arm_inputs", ArmInputs, "arm_inputs"), ("arm_state", String, "arm_state")], [])

    assert rclpy.ok(), "rclpy did not initialize correctly"
    assert manual_node.status == "Idle", "Manual node did not initialize to Idle"
    assert manual_node.controller_input == [0., 0., 0., 0., 0., 0, 0]
    assert list(manual_node.goal_pos.data) == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    test.test_publishers["arm_state"].publish(String(data="Manual"))
    rclpy.spin_once(manual_node, timeout_sec=2)
    time.sleep(0.2)
    assert manual_node.status == "Manual", "Manual node did not update status to Manual"
    initial_goal_pos = manual_node.goal_pos.data
    
    l_horizontal = 10.0
    l_vertical = 5.0
    r_horizontal = 5.0
    r_vertical = 4.0
    l1 = 0
    l2 = 0.0
    r1 = 10
    r2 = 5.0
    x = 5
    o = 4
    
    arm_input = ArmInputs(l_horizontal = l_horizontal,
                          l_vertical = l_vertical,
                          r_horizontal = r_horizontal,
                          r_vertical = r_vertical,
                          l1 = l1,
                          l2 = l2,
                          r1 = r1,
                          r2 = r2,
                          x = x,
                          o = o)

    test.test_publishers["arm_inputs"].publish(arm_input)
    rclpy.spin_once(manual_node, timeout_sec=2)
    time.sleep(0.2)

    expected_input = [l_horizontal, l_vertical, r_vertical, r_horizontal, l1 - r1, l2 - r2, x - o]
    assert manual_node.controller_input == expected_input
    
    expected_goal_pos = Float32MultiArray()
    expected_goal_pos.data = list(np.array(expected_input) * np.array(manual_node.SPEED_LIMIT) + np.array(initial_goal_pos))
    assert manual_node.goal_pos.data == expected_goal_pos.data

    rclpy.shutdown()
    