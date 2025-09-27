import sys
sys.path.insert(0, "../manual")
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import manual.manual as manual
# import manual 
from std_msgs.msg import String
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


def test_sample():
    assert 2+2==4

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