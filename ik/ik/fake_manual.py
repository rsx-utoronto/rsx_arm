#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

from arm_msgs.msg import ArmInputs  # Replace with your actual msg definition

'''
This code is just to help debug arm control code when
the real arm isn't plugged in an manual is needed
'''

class FakeManualNode(Node):
    def __init__(self):
        super().__init__('fake_manual')

        self.arm_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.state = "Idle"
        self.scale = 0.0001

        # Publishers
        self.joint_pub = self.create_publisher(Float32MultiArray, 'arm_goal_pos', 10)
        self.real_joint_pub = self.create_publisher(Float32MultiArray, 'arm_curr_pos', 10)

        # Subscribers
        self.create_subscription(String, 'arm_state', self.updateStates, 10)
        self.create_subscription(Float32MultiArray, 'arm_goal_pos', self.updateRealAngles, 10)
        self.create_subscription(ArmInputs, 'arm_inputs', self.updateController, 10)


    def updateStates(self, msg):
        ''' Callback function for the /arm_states topic'''
        self.state = msg.data
        angles_msg = Float32MultiArray()
        angles_msg.data = self.arm_angles
        self.joint_pub.publish(angles_msg)
        self.real_joint_pub.publish(angles_msg)

    def updateController(self, msg):
        ''' Callback function for /arm_inputs 

            Recieves the ArmInput ros message and uses the values to change
            the arm angles that fake manual thinks it's at.
        '''

        if self.state != "Manual":
            return

        self.arm_angles[0] += msg.l_horizontal * self.scale
        self.arm_angles[1] += msg.l_vertical * self.scale
        self.arm_angles[2] += msg.r_horizontal * self.scale
        self.arm_angles[3] += msg.r_vertical * self.scale
        self.arm_angles[4] += (msg.l1 - msg.r1) * self.scale
        self.arm_angles[5] += (msg.l2 - msg.r2) * self.scale

        angles_msg = Float32MultiArray()
        angles_msg.data = self.arm_angles
        self.joint_pub.publish(angles_msg)
        self.real_joint_pub.publish(angles_msg)
            #print(armAngles)

    def updateRealAngles(self, msg):
        ''' Callback function for /arm_goal_pos topic

        Idealized version of manual with no saftey
        '''
        if self.state == "Manual":
            return

        self.arm_angles = list(msg.data)
        angles_msg = Float32MultiArray()
        angles_msg.data = self.arm_angles
        self.real_joint_pub.publish(angles_msg)

def main():
    rclpy.init()
    node = FakeManualNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()