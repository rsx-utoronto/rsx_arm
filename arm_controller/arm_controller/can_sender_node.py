#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

# Reuse same helpers to encode/decode frames (copy for safety)
def decode_can_frame(msg:UInt8MultiArray):
    d = list(msg.data)
    if len(d) < 3:
        return None
    can_id = d[0] | (d[1] << 8)
    dlc = d[2]
    data = d[3:3+dlc]
    return can_id, dlc, data

class CanSenderNode(Node):
    def __init__(self):
        super().__init__('can_sender')
        self.get_logger().info('CAN Sender Node started')

        # parameters
        self.declare_parameter('use_physical_can', False)
        self.use_physical_can = bool(self.get_parameter('use_physical_can').get_parameter_value().bool_value)

        self.declare_parameter('can_interface', 'can0')
        self.can_interface = self.get_parameter('can_interface').get_parameter_value().string_value

        self.can_tx_sub = self.create_subscription(UInt8MultiArray, 'can_tx', self.on_can_tx, 10)
        # If simulation mode, we will publish to can_rx to simulate reception/loopback
        self.can_rx_pub = self.create_publisher(UInt8MultiArray, 'can_rx', 10)

        self.get_logger().info(f'can_sender: use_physical_can={self.use_physical_can} interface={self.can_interface}')

        # if physical, try to import python-can and open a bus; don't crash if not available
        self._can_bus = None
        if self.use_physical_can:
            try:
                import can
                # one-line send is below; keep a bus instance for efficiency
                self._can_bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')
                self._can_lib = can
                self.get_logger().info('Opened python-can socketcan bus successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to open python-can on {self.can_interface}: {e}')
                self.get_logger().error('Falling back to simulation mode (set use_physical_can=false or fix python-can)')
                self.use_physical_can = False

    def on_can_tx(self, msg:UInt8MultiArray):
        decoded = decode_can_frame(msg)
        if decoded is None:
            self.get_logger().warning('Received malformed CAN frame on can_tx')
            return
        can_id, dlc, data = decoded

        if self.use_physical_can and self._can_bus is not None:
            try:
                # single-line send (the "one line of ROS boilerplate + send" you described)
                can_msg = self._can_lib.Message(arbitration_id=can_id, data=bytes(data), is_extended_id=False)
                self._can_bus.send(can_msg)   # <-- the actual physical send line
                self.get_logger().info(f'PHYSICAL SEND id=0x{can_id:X} dlc={dlc} data={data}')
            except Exception as e:
                self.get_logger().error(f'Error sending on CAN interface: {e}')
        else:
            # Simulation/loopback: just republish to can_rx so logic node can receive it
            self.can_rx_pub.publish(msg)
            self.get_logger().info(f'SIMULATED SEND (loopback) id=0x{can_id:X} dlc={dlc} data={data}')

def main(args=None):
    rclpy.init(args=args)
    node = CanSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

