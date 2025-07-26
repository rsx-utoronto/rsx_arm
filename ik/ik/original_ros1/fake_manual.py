#! /usr/bin/env python3
import rospy
from std_msgs.msg import String, Float32MultiArray
from rover.msg import ArmInputs

'''
This code is just to help debug arm control code when
the real arm isn't plugged in an manual is needed
'''

class FakeManualNode():
    def __init__(self) -> None:
        rospy.init_node("fake_manual")
        self.armAngles = [0, 0, 0, 0, 0, 0]

        self.jointPublisher = rospy.Publisher("arm_goal_pos", Float32MultiArray, queue_size=10)
        self.realJointPublisher = rospy.Publisher("arm_curr_pos", Float32MultiArray, queue_size=10)
        self.curState = "Idle"
        self.scale = 0.0001

        rospy.Subscriber("arm_state", String, self.updateStates)
        rospy.Subscriber("arm_goal_pos", Float32MultiArray, self.updateRealAngles)
        rospy.Subscriber("arm_inputs", ArmInputs, self.updateController)


    def updateStates(self, data):
        ''' Callback function for the /arm_states topic'''
        self.curState = data.data
        anglesToPublish = Float32MultiArray()
        anglesToPublish.data = self.armAngles
        self.jointPublisher.publish(anglesToPublish)
        self.realJointPublisher.publish(anglesToPublish)

    def updateController(self, data):
        ''' Callback function for /arm_inputs 

            Recieves the ArmInput ros message and uses the values to change
            the arm angles that fake manual thinks it's at.
        '''

        if self.curState == "Manual":
            self.armAngles[0] += data.l_horizontal*self.scale
            self.armAngles[1] += data.l_vertical*self.scale
            self.armAngles[2] += data.r_horizontal*self.scale
            self.armAngles[3] += data.r_vertical*self.scale
            self.armAngles[4] += (data.l1 - data.r1)*self.scale
            self.armAngles[5] += (data.l2 - data.r2)*self.scale

            anglesToPublish = Float32MultiArray()
            anglesToPublish.data = self.armAngles
            self.jointPublisher.publish(anglesToPublish)
            self.realJointPublisher.publish(anglesToPublish)
            #print(armAngles)

    def updateRealAngles(self, data):
        ''' Callback function for /arm_goal_pos topic

        Idealized version of manual with no saftey
        '''
        if self.curState != "Manual":

            tempList = list(data.data)
            # tempList[0] = tempList[0]
            # tempList[1] = -tempList[1]
            # tempList[4] = -tempList[4]
            self.armAngles = tempList
            # print(tempList)
            anglesToPublish = Float32MultiArray()
            anglesToPublish.data = self.armAngles
            self.realJointPublisher.publish(anglesToPublish)

if __name__ == "__main__":
    try:
        fakeManual = FakeManualNode()
        rospy.spin()
    except Exception as ex:
        print(ex)