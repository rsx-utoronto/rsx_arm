from arm_utilities.arm_can_utils import *
from arm_utilities.arm_enum_utils import CANAPI


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

        # Variable to hold the boolean whether each motor is read once or not
        # For each motor connected, the corresponding motor_list element should be set to 0
        # motor_read = [1, 1, 1, 1, 1, 0, 1]
        msg = self.bus.recv(timeout=0.0001)
        
        if msg == None:
            return None
        # Checking if SparkMAXes are powered on and sending status messages
        can_id = msg.arbitration_id
        dev_id = can_id & 0b00000000000000000000000111111
        api = (can_id >> 6) & 0b00000000000001111111111
        # man_id  = (can_id) & 0b00000111111110000000000000000

        # If every element in motor_list is True, it means this was for initialization
        # and we should break out of the loop
        # if not False in motor_read:
        # 	break

        # Getting the list index value based on motor device id
        index = dev_id - 11
        # If this is for initialization, set the correseponding element in motor_list to be True
        # if init:
        # 	motor_read[index] = True
        # print(dev_id)

        if dev_id > 10:
            # API for reading limit switch
            if api == CANAPI.CMD_API_STAT0.value:

                # Update the LIMIT_SWITCH data
                lim_value = read_can_message(
                    msg.data, CANAPI.CMD_API_STAT0)
                return (index, api, lim_value)

            # API for reading motor current
            elif api == CANAPI.CMD_API_STAT1.value:

                # Update the MOTOR_CURR data
                curr_value = read_can_message(
                    msg.data, CANAPI.CMD_API_STAT1)
                return (index, api, curr_value)

            # API for reading current position of motor
            elif api == CANAPI.CMD_API_STAT2.value:

                # Update the CURR_POS data
                joint_val = read_can_message(
                    msg.data, CANAPI.CMD_API_STAT2, index)
                return (index, api, joint_val)
                


    def send_target_message(self, goal_position):
        """
        Timer callback function for sending CAN messages at regular intervals
        """

        # Convert SparkMAX angles to SparkMAX data packets
        spark_input = generate_data_packet(
            goal_position)  # assuming data is safe

        # Send data packets
        for i in range(1, len(spark_input)+1):

            # Motor number corresponds with device ID of the SparkMAX
            motor_num = 10 + i

            # print(spark_input)
            if motor_num > 10 and motor_num < 18:
                # API WILL BE CHANGED WHEN USING THE POWER (DC) SETTING
                id = generate_can_id(
                    dev_id=motor_num, api=CANAPI.CMD_API_POS_SET.value)
                send_can_message(self.bus, can_id=id, data=spark_input[i - 1])

            else:
                break

    def send_message(self, message):
        try:
            self.bus.send(message)
        except:
            print("Error encountered while sending CAN message!")
            pass
