from arm_utilities.arm_can_utils import *
from arm_utilities.arm_enum_utils import CANAPI, ODRIVE_CANAPI

class CAN_connection():
    def __init__(self, channel="can0", interface='socketcan', receive_own_messages=False, num_joints=7, send_rate=1000, read_rate=1000):
        self.bus = initialize_bus(channel, interface, receive_own_messages)
        hb = can.Message(
            arbitration_id=generate_can_id(
                dev_id=0x0,
                api=CANAPI.CMD_API_NONRIO_HB.value),
            data=bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]),
            is_extended_id=True,
            is_remote_frame=False,
            is_error_frame=False
        )
        task = self.bus.send_periodic(hb, 0.01, store_task=False)
        self.send_rate = send_rate
        self.read_rate = read_rate
        self.num_joints = num_joints

    def read_message(self):
        """
        (can.Message) -> (None)

        Function that reads status message regarding position, limit switch and current 
        from all motors and updates the global variable CURR_POS, LIMIT_SWITCH and MOTOR_CURR
        to store the values
        """

        msg = self.bus.recv(timeout=0.0001)
        
        if msg == None:
            return None
        # Checking if SparkMAXes are powered on and sending status messages
        can_id = msg.arbitration_id
        dev_id = get_odrive_dev_id(can_id)
        cmd = get_odrive_cmd_id(can_id)
        # man_id  = (can_id) & 0b00000111111110000000000000000

        # Getting the list index value based on motor device id
        index = dev_id
        
        if dev_id >= 0 and dev_id < self.num_joints:
            # # API for reading limit switch
            # if cmd == CANAPI.CMD_API_STAT0.value:

            #     # Update the LIMIT_SWITCH data
            #     lim_switch[index] = read_can_message(
            #         msg.data, CANAPI.CMD_API_STAT0) # Should we append a ".value"?

            # cmd for reading motor current
            if cmd == ODRIVE_CANAPI.CMD_API_GET_BUS_VOLTAGE_CURRENT.value:

                # Update the MOTOR_CURR data
                curr_val = read_can_message(
                    msg.data, ODRIVE_CANAPI.CMD_API_GET_BUS_VOLTAGE_CURRENT.value)
                return (index, cmd, curr_val)

            # API for reading current position of motor
            elif cmd == ODRIVE_CANAPI.CMD_GET_ENCODER_ESTIMATES.value:

                # Update the CURR_POS data
                joint_val = read_can_message(
                    msg.data, ODRIVE_CANAPI.CMD_GET_ENCODER_ESTIMATES.value, index)
                return (index, cmd, joint_val)
        return None

    def send_target_message(self, goal_position):
        """
        Timer callback function for sending CAN messages at regular intervals
        """

        # Convert SparkMAX angles to SparkMAX data packets
        odrive_input = generate_odrive_data_packet(goal_position)  # assuming data is safe

        # Send data packets
        for i in range(1, len(spark_input)+1):

            # Motor number corresponds with device ID of the SparkMAX
            motor_num = i

            # print(spark_input)
            if motor_num >= 0 and motor_num < 8:
                # API WILL BE CHANGED WHEN USING THE POWER (DC) SETTING
                id = generate_odrive_can_id(motor_id=motor_num, cmd_id=ODRIVE_CANAPI.CMD_API_SET_INPUT_POS)
                send_can_message(self.bus, can_id=id, data=odrive_input[i - 1])

            else:
                break

    def send_message(self, message):
        try:
            self.bus.send(message)
        except:
            print("Error encountered while sending CAN message!")
            pass
