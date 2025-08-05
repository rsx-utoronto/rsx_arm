#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import *
from enum import Enum
import time
import numpy as np
import os

############ ENUMERATIONS #############

# Enum for all the errors
class Errors(Enum):
    ERROR_NONE              = 0
    ERROR_EXCEEDING_POS     = 1
    ERROR_EXCEEDING_CURRENT = 2
    ERROR_LIMIT_SWITCH      = 3

############### CLASSES ###############

class Safety_Node(Node):
    """
    (None)

    This class represents an instance of the safety node and connects the ROS node to
    multiple topics by making it a subscriber or a publisher
    """

    def __init__(self):
        super().__init__('Safety')
        # Attributes to hold data from subscribed topics
        self.GOAL_POS             = [0, 0, 0, 0, 0, 0, 0]
        self.CURR_POS             = [0, 0, 0, 0, 0, 0, 0]
        self.MOTOR_CURR           = [0, 0, 0, 0, 0, 0, 0]
        self.LIMIT_SWITCH         = [False, False, False, False, False, False, False]
        self.STATE                = 'Idle'

        # Attributes to hold data for publishing to topics
        self.ERRORS               = UInt8MultiArray()
        self.ERRORS.data          = [0, 0, 0, 0, 0, 0, 0]
        self.SAFE_GOAL_POS        = Float32MultiArray()
        self.SAFE_GOAL_POS.data   = [0, 0, 0, 0, 0, 0, 0]
        self.ERROR_OFFSET         = Float32MultiArray()
        self.ERROR_OFFSET.data    = [0, 0, 0, 0, 0, 0, 0]

        # Attributes needed for detecting whether motor current is exceeding
        self.TIME                 = [0, 0, 0, 0, 0, 0, 0]
        self.FIRST                = [True, True, True, True, True, True, True]

        # Variables for ROS publishers and subscribers
        self.Goal_sub             = self.create_subscription(Float32MultiArray, "arm_goal_pos", self.callback_Goal, 10)
        self.MotorCurr_sub        = self.create_subscription(Float32MultiArray, "arm_motor_curr", self.callback_MotorCurr, 10)
        self.CurrPos_sub          = self.create_subscription(Float32MultiArray, "arm_curr_pos", self.callback_CurrPos, 10)
        self.LimitSwitch_sub      = self.create_subscription(UInt8MultiArray, "arm_limit_switch", self.callback_LimitSwitch, 10)
        self.KillSwitch_sub       = self.create_subscription(UInt8, "arm_killswitch", self.callback_KillSwitch, 10)
        self.State_sub            = self.create_subscription(String, "arm_state", self.CallbackState, 10)
        self.SafePos_pub          = self.create_publisher(Float32MultiArray, "arm_safe_goal_pos", 10)
        self.Error_pub            = self.create_publisher(UInt8MultiArray, "arm_error_msg", 10)
        self.Offset_pub           = self.create_publisher(Float32MultiArray, "arm_error_offset", 10)

    def callback_KillSwitch(self, data : UInt8):
        """
        (UInt8) -> (None)
        
        Receives and stores killswitch input from the controller node. If pressed, we shut down CAN_Send node
        after sending the curr_pos as the latest position.

        @parameters

        data (UInt8): Input indicating whether killswitch is pressed or not
        """
        
        # Storing the boolean value
        killswitch = data.data

        if killswitch:
            self.SAFE_GOAL_POS.data = self.CURR_POS
            self.SafePos_pub.publish(self.SAFE_GOAL_POS)
            rclpy.sleep(0.001)
            os.system("rosnode kill "+ "CAN_Send")
        
        else:
            os.system("rosrun rover "+ "CAN_send.py")
            self.SAFE_GOAL_POS.data = self.CURR_POS

    def CallbackState (self, status: String) -> None:
        """
        (String) -> (None)

        Recieves and stores state (idle, manual, ik or setup) from controller node

        @parameters

        status (String): stores the status received from the ROS topic
        """

        # Update the state
        self.STATE = status.data
        
        # # If state is setup, do setup
        # if self.STATE == 'Setup':
        #     self.setup()

    def callback_LimitSwitch(self, limitSwitch_data : UInt8MultiArray) -> None:
        """
        (UInt8MultiArray) -> (None)
        
        Receives and stores limit switch inputs from SparkMax through the LIMIT_SWITCH topic
        into LIMIT_SWITCH attribute. Also updates the ERRORS attribute if needed and publishes it on 
        ERROR_MSG topic

        @parameters

        limitSwitch_data (UInt8MultiArray): List containing limit switch inputs from SparkMax
        """

        # Store the received limit switch data
        self.LIMIT_SWITCH = limitSwitch_data.data
        #print(self.LIMIT_SWITCH)

        # # Updating the ERRORS attribute if needed and publishing it
        # self.limitSwitch_check()
        # self.Error_pub.publish(self.ERRORS)

    def callback_Goal(self, goal_data : Float32MultiArray) -> None:
        """
        (Float32MultiArray) -> (None)

        Receives and stores data from GOAL_POS topic into GOAL_POS attribute, checks
        if the value is safe and publishes it to the SAFE_GOAL_POS topic if it is safe

        After the safety check, also publishes the errors to ERROR_MSG topic

        @parameters

        goal_data (Float32MultiArray): List containing input angles (degrees) of each motor
        """

        # Store the received goal postion
        self.GOAL_POS = list(goal_data.data)

        # self.GOAL_POS      = list(np.array(self.GOAL_POS) - np.array(self.ERROR_OFFSET.data))
        #print("Received inputs:", self.GOAL_POS[0])

        # Update the SAFE_GOAL_POS and publish if the received position is safe
        self.update_safe_goal_pos(self.GOAL_POS)

    
    def callback_MotorCurr(self, MotorCurr_data : Float32MultiArray) -> None:
        """
        (Float32MultiArray) -> (None)

        Receives and stores data from MOTOR_CURR topic into MOTOR_CURR attribute and
        checks whether any current value is exceeding limits. Publishes the errors on 
        ERROR_MSG topic after the checks

        @parameters

        MotorCurr_data (Float32MultiArray): List containing current values (amps, I think) of each motor
        """
        
        # Store the received motor current value
        self.MOTOR_CURR = MotorCurr_data.data

        # # Check if motor current is exceeding and publish the errors
        # self.current_check()
        # self.Error_pub.publish(self.ERRORS)
    
    def callback_CurrPos(self, CurrPos_data : Float32MultiArray) -> None:
        """
        (Float32MultiArray) -> (None)

        Receives and stores data from CURR_POS topic into CURR_POS attribute

        @parameters

        CurrPos_data (Float32MultiArray): List containing current angles (degrees) of each motor
        """

        # Store the received current position values
        self.CURR_POS = CurrPos_data.data

    def update_safe_goal_pos(self, final_pos : list) -> None:
        """
        (list(float)) -> (None)

        Does checks on the received data and publishes them to arm_safe_goal_pos topic if the checks pass.
        Becomes a recursive function for IK while a single loop function for manual

        @parameters

        final_pos (list(float)): POS values to be checked and published if safe
        """

        # # Check if we are in IK state
        # if self.STATE == "IK":

        #     # Speed Limits
        #     SPEED_LIMIT = [0.005, 0.003, 0.01, 0.075, 
        #                             0.075, 0.075, 0.075]

        #     # Update the final pos according to direction we are going in
        #     direction = [1, 1, 1, 1, 1, 1, 1]

        #     for i in range(len(self.GOAL_POS)):
        #         direction[i] = sign(self.GOAL_POS[i] - self.CURR_POS[i])
            
        #     final_pos = list(np.array(self.CURR_POS) + np.array(direction) * np.array(SPEED_LIMIT))
            
        #     # Do position check on this final_pos as well as all the other checks
        #     self.postion_check(pos= final_pos)
        #     self.current_check(pos= final_pos)
        #     self.limitSwitch_check(pos= final_pos)
        
        if self.STATE == "Manual" or self.STATE == "IK":
            
            # Check if the goal position is safe 
            #self.GOAL_POS      = list(np.array(self.GOAL_POS) - np.array(self.ERROR_OFFSET.data))
            self.postion_check()
            # self.current_check()
            # self.limitSwitch_check()
            #self.GOAL_POS      = list(np.array(self.GOAL_POS) - np.array(self.ERROR_OFFSET.data))

        else:
            return

        # Publish the errors
        self.Error_pub.publish(self.ERRORS)

        #print("goal: {}, error: {}".format(final_pos, self.ERRORS.data))

        # Check if there are any errors
        if self.ERRORS.data.count(Errors.ERROR_EXCEEDING_POS.value) == 0 or self.STATE == 'IK':

            # Print/Publish the position to SAFE_GOAL_POS topic
            #print(self.GOAL_POS)
            self.SAFE_GOAL_POS.data = self.GOAL_POS
            self.SafePos_pub.publish(self.SAFE_GOAL_POS)

            # Publish any errors that we need to
            self.Offset_pub.publish(self.ERROR_OFFSET)
            #pass
            self.ERROR_OFFSET.data  = [0, 0, 0, 0, 0, 0, 0]
        
        # If there are any errors, publish the error offsets and reset them
        elif Errors.ERROR_EXCEEDING_POS.value in self.ERRORS.data:

            print("publishing offsets:", self.ERROR_OFFSET.data)
            self.Offset_pub.publish(self.ERROR_OFFSET)
            #pass
            self.ERROR_OFFSET.data  = [0, 0, 0, 0, 0, 0, 0]
        
        # # Repeat the function if needed in 'IK' mode
        # if self.STATE == "IK" and max(abs(np.array(self.GOAL_POS) - np.array(self.CURR_POS))):
        #     self.update_safe_goal_pos(self.GOAL_POS)
    
    def postion_check(self, pos : list = None) -> None:
        '''
        (lsit(float)) -> (None)

        Compares the goal position with current position to see that the difference 
        between them is not gigantic. If gigantic sets the error as for that motor as
        Errors.ERROR_EXCEEDING_POS. Makes sure the motor doesn't jerk back to 0 
        position in case computer loses power

        @parameters

        pos (list(int)) (optional): POS values to be checked. Only fill it for IK mode, otherwise keep it None
        '''
        # TODO
        # Limits for position safety (Need to test these values)
        #limit = [1.25, 1.25, 1.25, 20, 1.25, 1.25, 1.25]
        limit = [10, 10, 10, 30, 120, 120, 80000] # 40 was an old value for this limit, no clue if it is correct for now

        if not pos:
            pos = self.GOAL_POS

        # Going through each element of GOAL_POS
        #for i in range(len(self.GOAL_POS)):
        for i in [0, 1, 2, 3, 4, 5, 6]:    
            # Doing position comparisons for safety
            if (abs(pos[i] - self.CURR_POS[i]) > limit[i]):

                # Calculating the offset, applying it to goal position and storing it
                offset                     = pos[i] - self.CURR_POS[i]
                self.ERROR_OFFSET.data[i]  = offset #- sign(offset) * limit[i]
                #spark_input[i] = arm_can.pos_to_sparkdata(CURR_POS[i])

                # # Update the error if no other error is set
                # if (self.ERRORS.data[i] == Errors.ERROR_NONE.value or 
                #     self.ERRORS.data[i] == Errors.ERROR_EXCEEDING_POS.value):

                # Set the error for the motor
                self.ERRORS.data[i] = Errors.ERROR_EXCEEDING_POS.value

            # NOTE
            # Remove the error if it is no longer there
            # By design, the EXCEEDING_POS error should not be there after one 
            # cycle of the loop as the offset to controller input will bring
            # the input back to a safe value.
            else:
                ## Call other checks just to confirm there are no errors
                # self.current_check()
                # self.limitSwitch_check()
                # if (self.ERRORS.data[i] == Errors.ERROR_NONE.value or 
                #     self.ERRORS.data[i] == Errors.ERROR_EXCEEDING_POS.value):
                self.ERRORS.data[i] = Errors.ERROR_NONE.value


    def current_check(self, pos : list = None) -> None:
        '''
        (None) -> (None)

        Checks the maximum current being consumed by a motor. If the current is higher than
        the expected max, sets the error for the particular motor as Errors.ERROR_EXCEEDING_CURRENT.
        Helps us to know when too much torque is being applied

        @parameters

        pos (list(int)) (optional): POS values to be checked. Only fill it for IK mode, otherwise keep it None
        '''

        # Max current values for each motor
        max_current = [40, 40, 40, 20, 20, 20, 20]

        if not pos:
            pos = self.GOAL_POS

        # Checking maximum currents
        for i in range(len(self.MOTOR_CURR)):
            if self.MOTOR_CURR[i] > max_current[i]:

                # Check if the motor has an error associated with it already
                if (self.ERRORS.data[i] == Errors.ERROR_NONE.value or 
                    self.ERRORS.data[i] == Errors.ERROR_EXCEEDING_CURRENT.value):
                    
                    # Checking if the input from controller is making it go further into 
                    # the direction that increases current
                    #if (sign(pos[i] - self.CURR_POS[i]) == sign(pos[i])):

                        # Change the error associated with the motor to EXCEEDING_CURRENT
                        self.ERRORS.data[i] = Errors.ERROR_EXCEEDING_CURRENT.value

                        # # Calculating the offset, applying it to goal position and storing it
                        # offset                  = list(np.array(self.GOAL_POS) - np.array(self.CURR_POS))
                        # self.ERROR_OFFSET.data  = offset

                        # Check if this is the first time this error is set 
                        if self.FIRST[i]:

                            # Collect time stamp since the error was set
                            self.TIME[i]    = time.time()
                            self.FIRST[i]   = False
                
                    # If the controller input is in the direction that reduces the current
                    # else:

                    #     # Remove any error and set TIME and FIRST back to original form
                    #     self.ERRORS.data[i] = Errors.ERROR_NONE.value
                    #     self.FIRST[i]       = True
                    #     self.TIME[i]        = 0
            
            # Checking if the current has been under the max limit for more than 10 ms
            elif self.MOTOR_CURR[i] < max_current[i] and time.time() - self.TIME[i] > 0.01:

                # Check if the motor has an error associated with it already
                if (self.ERRORS.data[i] == Errors.ERROR_EXCEEDING_CURRENT.value or
                    self.ERRORS.data[i] == Errors.ERROR_NONE.value):

                    # Remove the error and set TIME and FIRST back to original form
                    self.ERRORS.data[i] = Errors.ERROR_NONE.value
                    self.FIRST[i]  = True
                    self.TIME[i]   = 0

    def limitSwitch_check(self, pos : list = None) -> None:
        '''
        (None) -> (None)

        Checks whether any motor of the arm is hitting the limit switch attached to it. 
        Updates the error status of the motor if needed

        @parameters

        pos (list(int)) (optional): POS values to be checked. Only fill it for IK mode, otherwise keep it None
        '''

        if not pos:
            pos = self.GOAL_POS

        # Going through each limit switch input
        for i in range(len(self.LIMIT_SWITCH)):
            
            # Checking if any other error is set already
            if self.LIMIT_SWITCH[i]:
            
                #print(i)

                # Checking if the input from controller is making it go further into 
                # the direction that presses the limit switch
                #if (sign(pos[i] - self.CURR_POS[i]) == sign(pos[i])):
                    
                    #print(self.LIMIT_SWITCH[6])

                    # Checking if limit switch is being pressed, 
                    # if yes then change the error status for the motor
                    #if i == 6:
                        #print(self.LIMIT_SWITCH[i])
                        if (self.ERRORS.data[i] == Errors.ERROR_NONE.value or 
                            self.ERRORS.data[i] == Errors.ERROR_LIMIT_SWITCH.value):
                            #print("error in", i)
                            self.ERRORS.data[i]     = Errors.ERROR_LIMIT_SWITCH.value

                    # # Calculating the offset, applying it to goal position and storing it
                    # offset                  = list(np.array(self.GOAL_POS) - np.array(self.CURR_POS))
                    # print(self.GOAL_POS[2], self.CURR_POS[2], offset[2])
                    # self.ERROR_OFFSET.data  = offset
                
                #else:
                    # if (self.ERRORS.data[i] == Errors.ERROR_NONE.value or 
                    #     self.ERRORS.data[i] == Errors.ERROR_LIMIT_SWITCH.value):
                    #     self.ERRORS.data[i] = Errors.ERROR_NONE.value

            else:
                if (self.ERRORS.data[i] == Errors.ERROR_NONE.value or 
                    self.ERRORS.data[i] == Errors.ERROR_LIMIT_SWITCH.value):
                    self.ERRORS.data[i] = Errors.ERROR_NONE.value

    def setup(self):
        # For each motor...
        #for i in range(len(self.SAFE_GOAL_POS) - 1):
        for i in [0]:
            # Store current angles
            initial_pos = self.CURR_POS[i]
            # Send in a large value to hit limit switch
            #self.SAFE_GOAL_POS.data[i] = 10000000
            # When an error occurs, check again and reverse angle change
            while Errors.ERROR_LIMIT_SWITCH.value != self.ERRORS.data[i]:
                self.SAFE_GOAL_POS.data[i] += 1
                self.SafePos_pub.publish(self.SAFE_GOAL_POS)
                print(initial_pos)
            final_pos = self.CURR_POS[i]
            print(final_pos)

            self.SAFE_GOAL_POS.data[i] = final_pos- (final_pos - initial_pos)
            self.SafePos_pub.publish(self.SAFE_GOAL_POS)



############## FUNCTIONS ##############

def sign(x : float) -> int:
	'''
	(float) -> (int)

	Returns the sign of the given number

	@parameters

	x (float) = The number for which sign is needed
	'''

	return (x > 0) - (x < 0)

###################### MAIN #########################

def main() -> None:
    '''
    (None) -> (None)
    
    Main function that initializes the safety node
    '''

    try:
        # Initialize a ROS node
        rclpy.init()

        # Set all the publishers and subscribers of the node
        Safety = Safety_Node()

        # ROS spin to keep the node alive
        rclpy.spin(Safety)

    except KeyboardInterrupt:
        pass

if __name__ == "__main__":

    main()