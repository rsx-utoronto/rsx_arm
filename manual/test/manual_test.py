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
    
def test_update_pos_general():
    joy = [0.5, -1.0]
    goal = [10.0, 20.0]
    speed = [4.0, 2.0]
    
    args = None
    rclpy.init(args=args)
    manual_node = manual.Manual()

    out = manual_node.update_pos(joy, goal, speed)

    assert out == [12.0, 18.0], "update_pos did not compute correctly"

def test_update_pos_zero_input():
    joy = [0.0, 0.0]
    goal = [5.0, 15.0]
    speed = [10.0, 10.0]
    manual_node = manual.Manual()
    out = manual_node.update_pos(joy, goal, speed)
    assert out == goal, "Zero joystick should not change goal position" #checking correctness

def test_update_pos_negative_input():
    joy = [-0.5, -1.0]
    goal = [100.0, 50.0]
    speed = [20.0, 10.0]
    manual_node = manual.Manual()
    out = manual_node.update_pos(joy, goal, speed)
    assert out == [90.0, 40.0], "Negative inputs should decrease position"

def test_update_pos_length_mismatch():
    joy = [0.1, 0.2, 0.3]
    goal = [1.0, 2.0]
    speed = [3.0, 4.0, 5.0]
    manual_node = manual.Manual()
    try:
        manual_node.update_pos(joy, goal, speed)
        assert False, "Mismatched list lengths error"
    except Exception:
        pass  
    
def test_update_pos_zero_speed():
    m = manual.Manual()
    joy, goal, speed = [1.0], [10.0], [0.0]
    out = m.update_pos(joy, goal, speed)
    assert out == [10.0], "Zero speed should block movement"

def test_update_pos_negative_speed_current_behavior():
    m = manual.Manual()
    joy, goal, speed = [1.0], [10.0], [-5.0]
    out = m.update_pos(joy, goal, speed)
    assert out == [5.0], "Negative speed reverses direction"

