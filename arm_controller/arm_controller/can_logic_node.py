#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import struct
import random

# CAN message IDs (matching your C++ profile)
CANID_FLOAT = 3

def MAKE_CAN_ID(device, message):
    return (device << 4) | (message & 0xF)

def GET_MESSAGE_ID(canid):
    return canid & 0xF

def GET_DEVICE_ID(canid):
    return canid >> 4

def pack_float_to_bytes(f):
    # little-endian IEEE754 (matches typical C float packing)
    return list(struct.pack('<f', float(f)))

def unpack_float_from_bytes(b):
    return struct.unpack('<f', bytes(b))[0]

def encode_can_frame(can_id:int, data_bytes:bytes):
    """
    Encode to UInt8MultiArray layout:
    [ id_low, id_high, dlc, data0, data1, ... ]
    id is 11-bit value stored little-endian in two bytes
    """
    id_low = can_id & 0xFF
    id_high = (can_id >> 8) & 0xFF
    dlc = len(data_bytes)
    arr = [id_low, id_high, dlc] + list(data_bytes)
    msg = UInt8MultiArray()
    msg.data = arr
    return msg

def decode_can_frame(msg:UInt8MultiArray):
    d = list(msg.data)
    if len(d) < 3:
        return None
    can_id = d[0] | (d[1] << 8)
    dlc = d[2]
    data = d[3:3+dlc]
    return can_id, dlc, data

class CanLogicNode(Node):
    def __init__(self):
        super().__init__('can_logic')
        self.get_logger().info('CAN Logic Node started')

        # publishers & subscribers
        self.can_tx_pub = self.create_publisher(UInt8MultiArray, 'can_tx', 10)
        self.can_rx_sub = self.create_subscription(UInt8MultiArray, 'can_rx', self.handle_can_rx, 10)

        # parameters
        self.declare_parameter('device_id', 1)
        self.device_id = self.get_parameter('device_id').get_parameter_value().integer_value

        self.declare_parameter('publish_rate', 1.0)
        self.publish_rate = float(self.get_parameter('publish_rate').get_parameter_value().double_value)

        # timer to generate example messages
        self.timer = self.create_timer(1.0 / max(self.publish_rate, 1e-6), self.timer_callback)

        self.get_logger().info(f'can_logic: device_id={self.device_id}, rate={self.publish_rate} Hz')

    def timer_callback(self):
        # Example: produce a Float message to be sent over CAN
        # Replace this part with your calculation logic
        value = random.uniform(-10.0, 10.0)
        can_id = MAKE_CAN_ID(self.device_id, CANID_FLOAT)
        data_bytes = pack_float_to_bytes(value)
        can_msg = encode_can_frame(can_id, data_bytes)
        self.can_tx_pub.publish(can_msg)
        self.get_logger().info(f'Published CAN frame: id=0x{can_id:X} float={value:.3f}')

    def handle_can_rx(self, msg:UInt8MultiArray):
        decoded = decode_can_frame(msg)
        if decoded is None:
            self.get_logger().warning('Received malformed CAN frame')
            return
        can_id, dlc, data = decoded
        mid = GET_MESSAGE_ID(can_id)
        device = GET_DEVICE_ID(can_id)
        if mid == CANID_FLOAT and dlc >= 4:
            val = unpack_float_from_bytes(data[:4])
            self.get_logger().info(f'Received FLOAT from device {device}: {val:.6f}')
        else:
            self.get_logger().info(f'Received CAN id=0x{can_id:X} mid={mid} dlc={dlc} data={data}')
        
def main(args=None):
    rclpy.init(args=args)
    node = CanLogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

