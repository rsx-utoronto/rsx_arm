import sys
sys.path.insert(0, "../controller")
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import controller.arm_controller as controller
# import manual 
from std_msgs.msg import String
from sensor_msgs.msg import Joy

import time

# TODO: move to utilities folder (can just copy past into test folers for now)
class test_node(Node):
    def __init__(controller_node, publishers, subscribers):
        #input publishers and subscribers as (name, type, topic)

        super().__init__('test_node')
        controller_node.test_publishers = {}
        for publisher in publishers:
            controller_node.test_publishers[publisher[0]] = controller_node.create_publisher(publisher[1], publisher[2], 10)

        controller_node.test_subscribers = {}
        for subscriber in subscribers:
            controller_node.test_subscribers[subscriber[0]] = controller_node.create_subscription(subscriber[1], subscriber[2], controller_node.listener_callback, 10)
    def listener_callback(controller_node,msg):
        controller_node.arm_goal_pos = msg.data
      

def test_sample():
    assert 2+2==4

def test_joy():
    args = None
    rclpy.init(args=args)
    controller_node = controller.Controller()
    test = test_node([("Joy", Joy, "/joy")], [])

    assert rclpy.ok(), "rclpy did not initialize correctly"

    dummy = Joy()
    dummy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    dummy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    test.test_publishers["Joy"].publish(dummy)

    rclpy.spin_once(controller_node, timeout_sec=1)
    time.sleep(0.2)
    
    assert controller_node.values.l_horizontal == dummy.axes[0]
 
    assert controller_node.values.l_vertical == dummy.axes[1]
    assert controller_node.values.r_horizontal == dummy.axes[3]
    assert controller_node.values.r_vertical == dummy.axes[4]
    assert controller_node.values.l1 == dummy.buttons[4]
    assert controller_node.values.r1 == dummy.buttons[5]
    assert controller_node.values.l2 == -0.5 * dummy.axes[2] + 0.5
    assert controller_node.values.r2 == -0.5 * dummy.axes[5] + 0.5
    assert controller_node.values.x == dummy.buttons[0]
    assert controller_node.values.o == dummy.buttons[1]
    assert controller_node.values.triangle == dummy.buttons[2]
    assert controller_node.values.square == dummy.buttons[3]
    assert controller_node.values.share == dummy.buttons[8]
    assert controller_node.values.options == dummy.buttons[9]
    assert controller_node.values.l3 == dummy.buttons[11]
    assert controller_node.values.r3 == dummy.buttons[12]


    rclpy.shutdown()

