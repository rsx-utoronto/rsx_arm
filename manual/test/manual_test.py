import sys
sys.path.insert(0, "../manual")
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import manual.manual as manual
# import manual 
from std_msgs.msg import String, Float32MultiArray

import time

# TODO: move to utilities folder (can just copy past into test folers for now)
class test_node(Node):
    def __init__(self, publishers, subscribers):
        #input publishers and subscribers as (name, type, topic)

        super().__init__('test_node')
        self.test_publishers = {}
        for publisher in publishers:
            self.test_publishers[publisher[0]] = self.create_publisher(publisher[1], publisher[2], 10)

        self.test_subscribers = {}
        for subscriber in subscribers:
            self.test_subscribers[subscriber[0]] = self.create_subscription(subscriber[1], subscriber[2], self.listener_callback, 10)
    def listener_callback(self,msg):
        self.arm_goal_pos = msg.data
      

def test_sample():
    assert 2+2==4
"""
def test_manual_init():
    args = None
    rclpy.init(args=args)
    manual_node = manual.Manual()
    test = test_node([("arm_state", String, "arm_state")], [])

    assert rclpy.ok(), "rclpy did not initialize correctly"
    assert manual_node.status == "Idle", "Manual node did not initialize to Idle"
    assert manual_node.controller_input == [0., 0., 0., 0., 0., 0, 0] and [type(value) for value in manual_node.controller_input] == [type(value) for value in [0., 0., 0., 0., 0., 0, 0]], "controller_input is not zero!"
    goal_pos_data = Float32MultiArray()
    goal_pos_data.data = [0., 0., 0., 0., 0., 0., 0.]
    assert manual_node.goal_pos.data == goal_pos_data.data and [type(value) for value in manual_node.goal_pos.data] == [type(value) for value in goal_pos_data.data], "goal_pos is not zero!"
    assert manual_node.error_messages == [0, 0, 0, 0, 0, 0, 0], "error_messages is not zero!"
    assert manual_node.error_offsets == [0, 0, 0, 0, 0, 0, 0], "error_offsets is not zero!"
    assert manual_node.SPEED_LIMIT == [-0.1, 0.09, 0.15, 0.75, 0.12, 0.12, 20], "SPEED_LIMIT does not match the values!"

    test.test_publishers["arm_state"].publish(String(data="Manual"))
    rclpy.spin_once(manual_node, timeout_sec=1)
    time.sleep(0.2)
    assert manual_node.status == "Manual", "Manual node did not update status to Manual"
    rclpy.shutdown()
"""


def test_arm_goal_pos():
    args = None
    rclpy.init(args=args)
    manual_node = manual.Manual()
    test = test_node([],[("arm_goal_pos",Float32MultiArray,"arm_goal_pos")])

    assert rclpy.ok(), "rclpy did not initialize correctly"
    #assert manual_node.status == "Idle", "Manual node did not initialize to Idle"
    
    tests = [[1., 0., 0., 0., 0., 0., 0.],
           [1.78, 94.3, -9.3, 0.1, 3.14, 5.5],
           [1.325, -34.5, 434.5, -32.14, 64.1, 12.3]
           ]

    for arr in tests:
        manual_node.goal.publish(Float32MultiArray(data=arr))
        rclpy.spin_once(test, timeout_sec=1)
        time.sleep(0.2)
        assert test.arm_goal_pos == Float32MultiArray(data=arr).data
    
    rclpy.shutdown()
    