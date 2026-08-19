import sys
sys.path.insert(0, "../controller")
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import controller.arm_controller as controller
from std_msgs.msg import String
import time

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

def test_controller_init():
    args = None
    rclpy.init(args=args)
    controller_node = controller.Controller()
    controller_node.state = " "
    controller_node.values.l_horizontal = 1.0
    controller_node.publish_loop()

    assert controller_node.publishInputs == True, "Publish Loop Error (should not be run)"

    controller_node.values.l_horizontal = 0.0
    controller_node.publish_loop()

    assert controller_node.publishInputs == False, "Publish Loop Error (should be run)"

    test = test_node([("arm_state", String, "arm_state")], [])

    assert rclpy.ok(), "rclpy did not initialize correctly"
    test.test_publishers["arm_state"].publish(String(data="Manual"))
    rclpy.spin_once(controller_node, timeout_sec=1)
    time.sleep(0.2)
    rclpy.shutdown()