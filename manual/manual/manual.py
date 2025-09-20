#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np

from std_msgs.msg import Float32MultiArray, String, UInt8MultiArray
from arm_msgs.msg import ArmInputs

######################## CLASSES ##########################


class Manual(Node):
    """
    (None)

    Initializes the manual node and connects it to 
    different topics for publishing and subscribing
    """

    def __init__(self):
        super().__init__('arm_manual')
        # Buffer for controller input
        # Idx - Associated Controller input:
        # 0 - Left analog horizonal
        # 1 - Left analog vertical
        # 2 - Right analog horizonal
        # 3 - Right analog vertical
        # 4 - L1 / R1
        # 5 - L2 / R2 (analog)
        # 6 - X / O (gripper open/close)
        self.controller_input = [0., 0., 0., 0., 0., 0, 0]

        # Buffer for storing and publishing goal position
        # Idx - Associated Motor:
        # 0 - Motor 11 (Base)
        # 1 - Motor 12 (Shoulder)
        # 2 - Motor 13 (Elbow)
        # 3 - Motor 14 (Elbow Roll)
        # 4 - Motor 15 (Wrist 1)
        # 5 - Motor 16 (Wrist 2)
        # 6 - Motor 17 (End Effector Open/Close)
        self.goal_pos = Float32MultiArray()
        self.goal_pos.data = [0., 0., 0., 0., 0., 0., 0.]

        # Buffer to hold error messages and offsets
        # Message Descriptions:
        # 0 -> No Error
        # 1 -> Faraway position
        # 2 -> Exceeding Max Current
        # 3 -> Hitting Limit Switch
        self.error_messages = [0, 0, 0, 0, 0, 0, 0]
        self.error_offsets = [0, 0, 0, 0, 0, 0, 0]

        # Constant Speed limits, values are chosen by trial and error #TODO Find value for 7th motor
        self.SPEED_LIMIT = [-0.1, 0.09, 0.15, 0.75, 0.12, 0.12, 20]

        # Variable for the status, start at idle
        self.status = "Idle"

        # Subscribers
        self.create_subscription(
            UInt8MultiArray, 'arm_error_msg', self.CallbackError, 10)
        self.create_subscription(String, 'arm_state', self.CallbackState, 10)
        self.create_subscription(
            ArmInputs, 'arm_inputs', self.CallbackInput, 10)
        self.create_subscription(
            Float32MultiArray, 'arm_error_offset', self.CallbackErrOffset, 10)

        # Publisher
        self.goal = self.create_publisher(
            Float32MultiArray, 'arm_goal_pos', 10)
        # self.SafePos_pub = self.create_publisher(Float32MultiArray, 'arm_safe_goal_pos', 10)

    def CallbackError(self, errors):
        """
        (UInt8MultiArray) -> (None)

        Recieves and stores error messages from safety node

        @parameters

        errors (UInt8MultiArray): stores error messages received from ROS topic
        """

        # Updates errors
        self.error_messages = errors.data

    def CallbackErrOffset(self, offsets: Float32MultiArray) -> None:
        """
        (Float32MultiArray) -> (None)

        Recieves and stores error offsets from safety node. Uses them for correcting controller input

        @parameters

        offsets (Float32MultiArray): stores error offsets received from ROS topic
        """

        # Update offsets
        self.error_offsets = list(offsets.data)
        # print("before and offset" ,self.goal_pos.data[6], self.error_offset[6])
        self.goal_pos.data = list(
            np.array(self.goal_pos.data) - np.array(self.error_offsets))
        # print("after" ,self.goal_pos.data[6])

    def CallbackState(self, status: String) -> None:
        """
        (String) -> (None)

        Recieves and stores state (idle, manual, ik or setup) from controller node

        @parameters

        status (String): stores the status received from the ROS topic
        """

        # Updates state
        self.status = status.data

    def CallbackInput(self, inputs: ArmInputs) -> None:
        """
        (ArmInputs) -> (None)

        Recieves inputs from controller and if state is "Manual",
        updates the goal positions and publishes them

        Also has option of kill switch

        @parameters

        inputs (ArmInputs): Stores the received inputs from controller node
        """

        # Get inputs from controller
        self.controller_input[0] = inputs.l_horizontal
        self.controller_input[1] = inputs.l_vertical
        self.controller_input[2] = inputs.r_vertical
        self.controller_input[3] = inputs.r_horizontal
        self.controller_input[4] = inputs.l1 - inputs.r1
        self.controller_input[5] = inputs.l2 - inputs.r2
        # Added Gripper open/close, check if correct (x open and o close)
        self.controller_input[6] = inputs.x - inputs.o

        # Print Statement for console view
        # print("State:", self.status)

        # Apply amy offsets as necessary
        # self.goal_pos.data      = list(np.array(self.goal_pos.data) - np.array(self.error_offsets))

        # Checking the state, only proceed if in manual
        if self.status == "Manual":

            # Update goal positions and print/publish them
            self.goal_pos.data = self.update_pos(self.controller_input, self.goal_pos.data,
                                                 self.SPEED_LIMIT)
            self.get_logger().info(str(self.goal_pos.data))
            self.goal.publish(self.goal_pos)

    def update_pos(self, joy_input: list, curr_goal_pos: list, speed_limit: list) -> list:
        """
        (list(float), list(float), list(float)) -> list(float)

        Takes in controller inputs, current goal positions, speed limits and time passed
        to update goal positions

        @parameters

        joy_input (list(float)): Buffer holding the input from controller

        curr_goal_pos (list(float)): List containing the current goal positions (in degrees) of the motors

        speed_limit (list(float)): List containing tested out manually controlled speeds for each motor
        """
        # # Apply gripper correction due to wrist roll
        # curr_goal_pos[6] -= (joy_input[4] * speed_limit[4]) * 0.5

        # Set controller pos based on joint speed calculations/joypos
        return list(np.array(joy_input) * np.array(speed_limit) + np.array(curr_goal_pos))

############################## MAIN ############################


def main(args=None):
    print("Manual runs")
    try:
        rclpy.init(args=args)
        Manual_Node = Manual()
        rclpy.spin(Manual_Node)
        Manual_Node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':

    main()
