#!/usr/bin/env python3

import can
import struct
from arm_utilities.arm_enum_utils import CANAPI

########## GLOBAL VARIABLES ##########


# REDUCTION Ratios
# 7th Motor is a DC motor without a gearbox
REDUCTION = [160, 100, 100, 100, 100, 100, 1]
WRIST_RATIO = 5/4

########## SHARED FUNCTIONS ##########


def generate_can_id(dev_id: int, api: int,
                    man_id=0x05, dev_type=0x2) -> int:
    """
    (int, int, int, int, int) -> (int)

    Generates a valid extended CAN ID for a Rev object (SparkMAX by default) 
    with id "dev_id" and API call "api"

    @parameters

    dev_id (int) = The unique devide ID of the object in the CAN network (6 bits)
    api (int) = The API for SparkMAX that is required to be run (10 bits)
    man_id (int) (optional) = Manufactured ID, default value is 5 (8 bits)
    dev_type (int) (optional) = Device Type, default value is 2 (5 bits)

    Note: The complete CAN ID should never be larger than 29 bits
    """

    # Forming the CAN ID
    can_id = 0 << 5
    can_id = (can_id | dev_type) << 8
    can_id = (can_id | man_id) << 10
    can_id = (can_id | api) << 6
    can_id = (can_id | dev_id)
    return can_id


def pos_to_sparkdata(f: float) -> list:
    """
    float -> list(int)

    Takes in a float position value (number of rotations) and returns the data packet in the form that
    SparkMAX requires

    @parameters:

    f (float): Float value that needs to be converted
    """
    input_hex = hex(struct.unpack('<I', struct.pack('<f', f))[0])
    if len(input_hex) != 10:
        input_hex = input_hex[:2] + input_hex[2:] + (10-len(input_hex))*'0'

    return [eval('0x'+input_hex[-2:]), eval('0x'+input_hex[-4:-2]),
            eval('0x'+input_hex[-6:-4]), eval('0x'+input_hex[-8:-6]),
            0x00, 0x00, 0x00, 0x00]


def power_to_sparkdata(f: float) -> list:
    """
    float -> list(int)

    Takes in a float duty cycle value (power percentage) and returns the data packet in the form that
    SparkMAX requires: 8-byte CAN payload consists of the converted value in a 4-byte IEEE 754 float 
    and 4 zero bytes appended at the end.

    @parameters:

    f (float): Float value that needs to be converted
    """
    f = max(-1.0, min(1.0, f)
            )  # Clamping the value for power to make sure it's a percentage

    input_hex = hex(struct.unpack('<I', struct.pack('<f', f))[0])
    if len(input_hex) != 10:
        input_hex = input_hex[:2] + input_hex[2:] + (10-len(input_hex))*'0'

    return [eval('0x'+input_hex[-2:]), eval('0x'+input_hex[-4:-2]),
            eval('0x'+input_hex[-6:-4]), eval('0x'+input_hex[-8:-6]),
            0x00, 0x00, 0x00, 0x00]


def sparkfixed_to_float(fixed: int, frac: int = 5) -> float:
    """
    (int, int) -> (float)
    Returns floating point representation of the fixed point represenation of 
    data received from SparkMAX

    @parameters:
    fixed (int): Input fixed point number from SparkMAX
    frac (int) (optional): Number of fractional bits
    """
    # Divide the received value by 2^(number of fractional bits)
    f = fixed / (2 ** frac)
    return f


def initialize_bus(channel='can0', interface='socketcan', receive_own_messages=False):
    """
    (str, str) -> (None)

    Creates an instance of the CAN bus for sending and receiving
    CAN messages. By defaul, the network it searches for is named 'can0'
    and the interfaces it uses is 'socketcan'. For a detailed list of 
    available interfaces, check out the python-can documentation

    @parameters

    channel (str) (optional) = Name of the CAN network you are using
    interface (str) (optional) = Name of the interface you are using

    """
    # Getting global BUS

    # Initializing the global BUS
    # BUS = can.ThreadSafeBus(channel= channel, interface= interface, receive_own_messages= False)
    bus = can.Bus(channel=channel, interface=interface,
                  receive_own_messages=receive_own_messages)
    print('BUS initialized')
    return bus


def send_can_message(bus, can_id: int, data=None, ext=True, err=False, rtr=False) -> None:
    """
    (int, list(float), bool, bool, bool) -> (None)

    Forms and sends the complete CAN packet with the given data and can_id

    @parameters

    can_id (int) = The complete 29 bit CAN address, can be generated from generate_can_id
    data (list(float)) = An iterable object of ints or bytes, data can be max 64 bit
    ext (bool) (optional) = True if can_id is extended 29 bits, False if it is 11 bits.
        True by default
    err (bool) (optional) = True if it is an error frame, False otherwise.
        False by default
    rtr (bool) (optional) = True if it is remote frame, False otherwise.
        False by default
    """

    # Converting list data to byte
    if data:
        data = bytes(data)

    # Creating the message packet
    msg = can.Message(
        arbitration_id=can_id,
        data=data,
        is_extended_id=ext,
        is_remote_frame=rtr,
        is_error_frame=err
    )

    # Sending the created message
    # try:
    bus.send(msg)
    # print(f"Message sent on {bus.channel_info}")

    # except can.CanError:
    #     print("Message NOT sent", can.CanError)
    #     pass
    return


def read_can_message(data, api, motor_num: int = 0) -> float:
    """
    (bytearray, int, int) -> (float)

    Converts CAN data packets from hex to float decimal values based on which API is 
    called

    @parameters

    data (bytearray) = The CAN data packet containing just the payload. 
    api (int) = The API you want the payload of
    motor_num (int) (optional) = The motor number that can be used for indexing lists
    """
    if api:
        # API: Status Message 1 - Gives us information on limit switches
        if api == CANAPI.CMD_API_STAT0:
            # 132 for forward and 68 for reverse
            limitswitch_pressed = data[3] & 0xC0

            # Return 1 if limit switches are pressed
            if limitswitch_pressed > 0:
                return 1

            # Return 0 if no limit switches are pressed
            else:
                return 0

        # API: Status Message 1 - Gives us motor velocity, motor voltage and
        # motor current every 20ms. We only need current
        elif api == CANAPI.CMD_API_STAT1:

            # Getting the motor current value stored as hexadecimals
            currVal_fixed = (data[-1] << 4) | ((data[-2] & 0xF0) >> 4)

            # SparkMAX stores motor current value in fixed point format,
            # need to convert it to float for reading
            currVal_float = sparkfixed_to_float(currVal_fixed)

            return currVal_float

        # API: Status Message 2 - Gives us current position and comes every 20ms
        elif api == CANAPI.CMD_API_STAT2:
            # Checking if all the bits are 0 or not, if yes return 0
            if not (data[3] or data[2] or data[1] or data[0]):
                return 0

            # Extracting the position value bytes
            pos_float = data[3] << 8
            pos_float = (pos_float | data[2]) << 8
            pos_float = (pos_float | data[1]) << 8
            pos_float = (pos_float | data[0])

            # Converting the extracted bytes to hex
            pos_hex = str(hex(pos_float))

            # Check if we have 8 hex characters in pos_hex, if not then pad it with zeros
            if len(pos_hex) != 10:
                pos_hex = format(pos_float, '#010x')
            
            # Converting the hex representation to floating point decimal value
            pos_float = struct.unpack('!f', bytes.fromhex(pos_hex[2:]))[0]
            # Returning the shaft angle in degrees
            return pos_float * 360 / REDUCTION[motor_num]

    else:
        # Error Message, invalid API
        print("Invalid API ID")
        return -1


# NEEDS CALIBRATION
def calc_differential(roll: float, pitch: float) -> tuple:
    """
    (float, (float) --> tuple(float)

    Calculates the motor angle (in degrees) for wrist motors based on the provided roll and
    pitch angles (in degrees)

    @parameters

    roll (float): roll angle for the gripper (in degrees)
    pitch (float): pitch angle for the gripper (in degrees)
    """

    # Motor movement required to produce roll
    wrist_motor1 = roll * WRIST_RATIO + pitch
    wrist_motor2 = roll * WRIST_RATIO - pitch

    # correction through the gripper motor to stop the gripper from opening and closing
    # gripper_correction = roll # rotate the nut in the opposite direction same amount (no gear ratios)

    # print(roll_motor1, roll_motor2)
    return wrist_motor1, wrist_motor2  # gripper_correction


def generate_data_packet(data_list: list) -> list:
    """
    list(float) -> list(list(int))

    Takes in the goal position angles for each motor and converts them to bytearrays specific for
    each motor

    @parameters

    data_list (list(float)): List containing the anglular positions (in degrees) for each motor in the order
    """

    # Angle conversion for differential system
    # Assuming the last two angles specify the angle of the differential system,
    # convert those two values to the required angles for motors 5 and 6
    # wrist_motor1, wrist_motor2, gripper_correction = calc_differential(data_list[-3], data_list[-2])
    wrist_motor1, wrist_motor2 = calc_differential(
        data_list[-3], data_list[-2])

    # Sparkmax Data list
    spark_data = []

    for i in range(len(data_list)):
        # Specific angle for wrist motor 1
        if i + 1 == 5:
            angle = wrist_motor1

        # Specific angle for wrist motor 2
        elif i + 1 == 6:
            angle = wrist_motor2

        # CHANGES: changes angle for gripper control
        elif i + 1 == 7:
            # gripper_correction needs to be added according to previous
            angle = data_list[i]
            # the gripper correction is in gripper ccchannelcchannelcchannelcchannelcchannelchannel = 'vcan0', interface = 'virtual'channel = 'vcan0', interface = 'virtual'channel = 'vcan0', interface = 'virtual'ontroller - check this out further

        # For any other motor
        else:
            angle = data_list[i]

        # Converting the angle to spark data
        angle = angle/360 * REDUCTION[i]
        spark_data.append(pos_to_sparkdata(angle))

    return spark_data

# odrive variants

def generate_odrive_can_id(cmd_id: int, motor_id: int):
    """
    (int, int) -> (int)

    Generates a valid CAN ID for an ODrive CAN Command with a function with ID cmd_id executed on a motor with ID motor_id

    @parameters

    cmd_id (int) = The unique 5-bit number ID of the object in the CAN network (6 bits)
    motor_id (int) = The unique 6-bit number ID of the motor the command will execute on (10 bits)

    Note: The complete CAN ID should never be larger than 11 bits
    """

    cmd_id_reduced = cmd_id&0b11111 #reduce cmd_id to the first 5 bits
    motor_id_reduced = motor_id&0b111111 #reduce node_id (called motor_id) to the first 6 bits
    motor_id_shifted = motor_id<<5 #make the first 5 bits of node_id 0 by bit shifting it 5 bits to the left
    cmdmotor_CAN_id = motor_id_shifted|cmd_id_reduced #combine them into 1 full CAN ID

    return cmdmotor_CAN_id

def pos_to_odrive_data(f: float) -> list:
    """
    float -> list(int)

    Takes in a float position value (number of rotations) and returns the data packet in the form that
    SparkMAX requires

    @parameters:

    f (float): Float value that needs to be converted
    """
    input_hex = hex(struct.unpack('<I', struct.pack('<f', f))[0])
    if len(input_hex) != 10:
        input_hex = input_hex[:2] + input_hex[2:] + (10-len(input_hex))*'0'

    #check the last 2 int16 inputs as they might be too low
    return [eval('0x'+input_hex[-2:]), eval('0x'+input_hex[-4:-2]),
            eval('0x'+input_hex[-6:-4]), eval('0x'+input_hex[-8:-6]),
            0x00, 0x00, 0x00, 0x00] 

def generate_odrive_data_packet(data_list: list) -> list:
    """
    list(float) -> list(list(int))

    Takes in the goal position angles for each motor and converts them to bytearrays specific for
    each motor

    @parameters

    data_list (list(float)): List containing the anglular positions (in degrees) for each motor in the order
    """

    # ODrive Data list
    odrive_data = []

    for i in range(len(data_list)):
        odrive_data.append(pos_to_odrive_data(data_list[i]))

    return odrive_data
