#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import String, UInt8
from rover.msg import ArmInputs

# import arm_serial_connector as arm_serial
#from rover.srv import Corrections

'''
publishes: arm_state, arm_inputs (joystick value)
subscribes: Joy
rosservice: correction
'''

class Controller(Node):
    """
    (None)

    This class represents an instance of controller node and connects the node to 
    its publishing and subscribing topics
    """

    def __init__(self):
        super().__init__("Arm_Controller")

        ## Attributes to hold data for publishing to topics
        # Attribute to publish state
        self.state               = "Idle"

        # Attribute to store/publish killswitch value
        self.killswitch          = 0

        ## Attribute to store servo state
        # state -> angle:
        # 0 -> 63 degrees
        # 1 -> 84 degrees
        self.servo               = 0

        ## Atrribute to store on/off state of gripper motor
        # state -> angle:
        # 0 -> off
        # 1 -> on
        self.gripper             = 0


        # Attribute to publish arm inputs (with initialized values)
        self.values              = ArmInputs()
        self.values.l_horizontal = 0
        self.values.l_vertical   = 0
        self.values.r_horizontal = 0
        self.values.r_vertical   = 0
        self.values.l1           = 0
        self.values.r1           = 0
        self.values.l2           = 0
        self.values.r2           = 0
        self.values.x            = 0
        self.values.o            = 0


        # Publishers
        self.state_pub = self.create_publisher(String, "arm_state", 10)
        self.input_pub = self.create_publisher(ArmInputs, "arm_inputs", 10)
        self.killswitch_pub = self.create_publisher(UInt8, "arm_killswitch", 10)

        # Subscriber
        self.joy_sub = self.create_subscription(Joy, "arm/joy", self.getROSJoy, 10)



        ## Variables for serial connections
        # Device Names
        # arduino_name = "1a86_USB2.0-Ser_"
        gripper_name = "STMicroelectronics_STM32_STLink_066DFF485671664867172828"

        # Device Connections
        # arduino_connection = arm_serial.Serial_Port(device_name= arduino_name)
        # self.gripper_connection = arm_serial.Serial_Port(device_name= gripper_name)
        # self.gripper_connection.open_device_port(baudrate= 4800)

        # # Printing state on the console and publishing it
        # print("State:", self.state)
        # self.state_pub.publish(self.state)

        # # If square is pressed, flip the servo configuration
        # if rawButtons[3] == 1:
        #     self.servo = not self.servo
            
        #     if self.servo:
        #         arm_servo.write_servo_high_angle()
        #         print("Servo going to 84 degrees configuration")
            
        #     else:
        #         arm_servo.write_servo_low_angle()
        #         print("Servo going to 63 degrees configuration")
        
        # Timer loop at 30 Hz (ros2)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_loop)
        # ROS 2 will automatically call publish_loop() every 1/30 seconds (i.e. 30 Hz)
        # repeating background task — a timer callback that keeps firing automatically on the interval you specify.
        
        self.publishInputs = False


    def publish_loop(self):
        # Print/Publish the inputs if state is neither Idle or Setup

        if self.state not in ["Idle", "Setup"]:
            if self.publishInputs:
                self.input_pub.publish(self.values)
                self.get_logger().info(str(self.values))

            if (abs(self.values.l_horizontal) >= 0.05 or abs(self.values.l_vertical) >= 0.05 or 
                abs(self.values.r_horizontal) >= 0.05 or abs(self.values.r_vertical) >= 0.05 or 
                self.values.l1 or self.values.r1 or self.values.l2 or self.values.r2 or
                self.values.x or self.values.o or self.values.triangle or self.values.share or
                self.values.options or self.values.r3):
                self.publishInputs = True
            else:
                self.publishInputs = False
            
                # # ROS 1 code  
                # Check if the gripper motor is on
                # if self.gripper:
                
                # # Check if the gripper is off, and if so, do we want it on
                # elif not self.gripper:
                    
                    # # Check if we need to turn it on
                    # if self.values.x == 1:
                    
                    #     gripper_connection.open_device_port(baudrate= 4800)

                    #     # If port opened successfully, send the data, close the port and then flip gripper_triggered
                    #     if gripper_connection.device_port:

                    #         gripper_connection.send_bytes(data= '2') # Message '2' is for turning the motor on/off on the ST Link chip
                    #         gripper_connection.close_device_port()
                    #         gripper_triggered = not gripper_triggered
                    
                    # # Check if we are switching directions
                    # if self.values.o == 1:

                    #     gripper_connection.open_device_port(baudrate= 4800)

                    #     # If port opened successfully, send the data and then close the port
                    #     if gripper_connection.device_port:

                    #         gripper_connection.send_bytes(data= '1') # Message '1' is for switching the motor spin direction on the ST Link chip
                    #         gripper_connection.close_device_port()

                    
                # # If square is pressed, flip the servo configuration
                # if self.servo and not servo_triggered:

                #     # Open the Arduino port
                #     arduino_connection.open_device_port()

                #     # If Arduino port is properly opened, flip servo configurations
                #     if arduino_connection.device_port:

                #         arduino_connection.send_bytes(angle_high)
                #         print("Servo going to "+ angle_high + "degrees configuration")
                #         servo_triggered = 1

                #         # Close the Arduino port since it was properly opened                    
                #         arduino_connection.close_device_port()
                    
                #     # Make sure self.servo and servo_triggered are reversed
                #     else:
                #         self.servo = not self.servo

                # elif not self.servo and servo_triggered:

                #     # Open the Arduino port
                #     arduino_connection.open_device_port()

                #     # If Arduino port is properly opened, flip servo configurations
                #     if arduino_connection.device_port:

                #         arduino_connection.send_bytes(angle_low)
                #         print("Servo going to "+ angle_low + "degrees configuration")
                #         servo_triggered = 0
                
                #         # Close the Arduino port since it was properly opened                    
                #         arduino_connection.close_device_port()
                    
                #     # Make sure self.servo and servo_triggered are reversed
                #     else:
                #         self.servo = not self.servo
                
                # Control rate sleep
                # self.rate.sleep()

    def getROSJoy(self, joy_input : Joy) -> None:    
        ''' 
        (Joy) -> (None)

        Gets joystick values from "Joy" Topic

        Assuming using this package:
            http://wiki.ros.org/joy
        
        
        @paramters
    
        joy_input (Joy): Stores the received joystick inputs from "Joy" topic 
        '''

        ## Getting the joystick inputs
        # Axes Mapping (note: max absolute value = 1)
        # Idx : Button on PS4 (direction)
        # 0 : Left Analog Stick Horizontal (left positive, right negative)
        # 1 : Left Analog Stick vertical (Up positive, down negative)
        # 2 : L2 (1 when unpressed, -1 when full pressed)
        # 3 : Right Analog Stick Horizontal (left positive, right negative)
        # 4 : Right Analog Stick vertical (Up positive, down negative)
        # 5 : R2 (1 when unpressed, -1 when full pressed)
        # 6 : D-Pad Horizontal (left Arrow = -1, Right Arrow = 1, 0 when neither are pressed)
        # 7 : D-Pad Vertical (Up Arrow = 1, Down Arrow = -1, 0 when neither are pressed)
        rawAxes                     = joy_input.axes

        # Button Mapping (0 when unpressed, 1 when pressed)
        # Idc : Button
        # 0 : X
        # 1 : O (circle)
        # 2 : triangle
        # 3 : square
        # 4 : L1
        # 5 : R1
        # 6 : L2 (even when lightly pressed, it becomes 1)
        # 7 : R2 (even when lightly pressed, it becomes 1)
        # 8 : share
        # 9 : options
        # 10: PS
        # 11: L3
        # 12: R3
        rawButtons                  = joy_input.buttons

        # Setting the ArmInputs variable to be published later
        self.values.l_horizontal    = rawAxes[0]
        self.values.l_vertical      = rawAxes[1]
        self.values.r_horizontal    = rawAxes[3]
        self.values.r_vertical      = rawAxes[4]
        self.values.l1              = rawButtons[4]
        self.values.r1              = rawButtons[5]
        self.values.l2              = -0.5 * rawAxes[2] + 0.5
        self.values.r2              = -0.5 * rawAxes[5] + 0.5
        self.values.x               = rawButtons[0]
        self.values.o               = rawButtons[1]
        self.values.triangle        = rawButtons[2]
        self.values.square          = rawButtons[3]
        self.values.share           = rawButtons[8]
        self.values.options         = rawButtons[9]
        self.values.l3              = rawButtons[11]
        self.values.r3              = rawButtons[12]

        # Check if analog sticks are not moving and triggers are not pressed 
        # and any other buttons are not pressed. If any of them is false, then do not
        # change the state
        if ((not (rawAxes[0] or rawAxes[1] or rawAxes[3] or rawAxes[4])) 
            and (rawAxes[2] == 1 and rawAxes[5] == 1) and (1 not in rawButtons)
            and self.killswitch == 0):
            
            if rawAxes[7] == -1:
                self.state = "Idle"
            
            elif rawAxes[6] == 1:
                self.state = "Manual"

            elif rawAxes[7] == 1:
                self.state = "Setup"

            elif rawAxes[6] == -1:
                self.state = "IK"
        
        # if self.state != 'Idle':

        #     # Check if we need to turn it on/off
        #     if self.values.x == 1:

        #         # If port opened successfully, send the data, close the port and then flip gripper_triggered
        #         if self.gripper_connection.device_port.is_open:
                    
        #             self.gripper_connection.send_bytes(data= '2') # Message '2' is for turning the motor on/off on the ST Link chip
        #             # gripper_connection.close_device_port()
            
        #     # Check if we are switching directions
        #     if self.values.o == 1:

        #         # If port opened successfully, send the data and then close the port
        #         if self.gripper_connection.device_port.is_open:

        #             self.gripper_connection.send_bytes(data= '1') # Message '1' is for switching the motor spin direction on the ST Link chip
        #             # gripper_connection.close_device_port()

        # # If square is pressed, flip the servo configuration
        # if self.values.square == 1:
        #     self.servo = not self.servo
        
        # # If X is pressed, flip the gripper state
        # if rawButtons[0] == 1:
        #     self.gripper = not self.gripper

        # If triangle is pressed, change IK state if in overall state is IK
        if self.values.triangle == 1 and self.state == "IK":
            self.values.triangle = 1
        else:
            self.values.triangle = 0

        # If PS button is pressed and killswitch was not activated, activate killswitch
        if rawButtons[10] == 1 and self.killswitch == 0:
            self.killswitch = 1
            self.killswitch_pub.publish(self.killswitch)

            # Set the state to "Idle"
            self.state      = "Idle"

        # If PS button is pressed and killswitch was already activated, deactivate it
        elif rawButtons [10] == 1 and self.killswitch == 1:
            self.killswitch = 0
            self.killswitch_pub.publish(self.killswitch)
        
        # Printing state on the console and publishing it
        self.get_logger().info(f"State: {self.state} \t killswitch: {bool(self.killswitch)}")
        self.state_pub.publish(String(data=self.state))
            
        # #     if self.servo:
        # #         arm_servo.write_servo_high_angle()
        # #         print("Servo going to 84 degrees configuration")
            
        # #     else:
        # #         arm_servo.write_servo_low_angle()
        # #         print("Servo going to 63 degrees configuration")

        # # Print/Publish the inputs if state is neither Idle or Setup
        # if self.state != "Idle" and self.state != "Setup":

        #     while ((rawAxes[0] or rawAxes[1] or rawAxes[3] or rawAxes[4]) 
        #            or (self.values.l2r2 != 0) or (1 in rawButtons)):
        #         print(self.values)
        #         self.input_pub.publish(self.values)



    # def updateCorrections():
    #     ''' Update the Correction values the IK node is using

    #     Paramters
    #     ---------

    #     Returns
    #     -------
    #     '''

    #     try:
    #         correctionService = rospy.ServiceProxy('update_ik_corrections', Corrections)
    #         serviceResponse = correctionService() # input data into function to send to service, response contains boolean if succesfully updated or not
    #     except rospy.ServiceException as ex:
    #         print("Service call failed: %s"%ex)


def main(args=None):
    rclpy.init(args=args)
    controller_node = Controller()
    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
