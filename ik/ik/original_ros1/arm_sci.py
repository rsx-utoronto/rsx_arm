#!/usr/bin/env python3

import rospy
from math import pi, atan2, sin, cos, sqrt, acos
from arm_science_ik import SciArm
from queue import Queue
from std_msgs.msg import Float32MultiArray, String
from numpy import deg2rad, rad2deg
from rover.msg import ArmInputs

class ArmSciNode():
    def __init__(self):
        self.armState = "Idle"
        self.joyInputQueue = Queue(maxsize=5)
        self.curAngleQueue = Queue(maxsize=5)
        self.joyInput = []

        dhTable = [[79.7, 0, 0, pi/2],
                   [0,    0, 367, 0],
                   [0,    0, 195, 0],
                   [0,    0, 67, pi/2],
                   [92,   0, 0, 0]]
        offsets = [0,
                    -atan2(112.99, 348.08), 
                    atan2(161, 110.7) + atan2(112.99, 348.08),
                    (atan2(92, 67) + atan2(110.75, 161)),
                    0]
        angleOrientation = [1, 1, 1, -1, 1]
        startingAngles = [0, pi-atan2(348.08, 112.99), 
                           (-pi/2)+atan2(112.99, 348.08), -pi/2, 0]

        self.BUTTON_NAMES = ["X", "CIRCLE", "TRIANGLE", "SQUARE", "L1", "R1", "L2", "R2", "SHARE", "OPTIONS", "PLAY_STATION", "L3", 
                             "R3", "UP", "DOWN", "LEFT", "RIGHT"]
        self.buttonStatus= {"X": False, "CIRCLE": False, "TRIANGLE": False, "SQUARE": False, "L1": False, "R1": False, "L2": False, 
                               "R2": False, "SHARE": False, "OPTIONS": False, "PLAY_STATION": False, "L3": False, "R3": False,"UP": False, 
                               "DOWN": False, "LEFT": False, "RIGHT": False} 

        self.arm = SciArm(5, dhTable, offsets, angleOrientation, startingAngles)
        self.sparkMaxOfssets = [0]*len(angleOrientation)

        rospy.init_node("arm_sci")
        self.rate = rospy.Rate(30)

        self.goalPub = rospy.Publisher("arm_goal_pos", Float32MultiArray, queue_size=10)
        self.vizPub = rospy.Publisher("arm_viz_pos", Float32MultiArray, queue_size=10)

        rospy.Subscriber("arm_state", String, self.onArmStateUpdate)
        rospy.Subscriber("arm_inputs", ArmInputs, self.onJoystickUpdate)
        rospy.Subscriber("arm_curr_pos", Float32MultiArray, self.onCurrPosUpdate)


    def handle_sampling_sequence(self):
        if self.armState != "IK":
            rospy.logwarn("Arm must be in IK mode for sampling sequence")
            return

        self.arm.setMode("Cyl")
        success, message = self.arm.execute_sampling_sequence()

        if success:
            rospy.loginfo(message)
        else: 
            rospy.logerr(message)

    def handle_custom_sampling(self):

        # Define custom checkpoints for your specific task
        checkpoints = [
            # [theta, r, z, alpha]
            [0, 300, 200, 0],        # Move to ready position
            [0, 400, 0, -pi/2],      # Approach sample
            [0, 400, -50, -pi/2],    # Collect sample
            [0, 300, 200, -pi/4],    # Lift sample
            [pi/2, 300, 300, 0],     # Rotate to deposit
            [pi/2, 400, 200, -pi/4]  # Deposit sample
        ]
        
        # Execute the sequence with slower speed for precision
        success, message = self.arm.execute_custom_sequence(checkpoints, speed_factor=0.5)
        
        if success:
            rospy.loginfo(message)
        else:
            rospy.logerr(message)

    def getJoystickButtonStatus(self, tempButton:list) -> dict: # setting up the buttons
        ''' Gets the Status of the Pressed Buttons on Joystick

        If the value in the returned dictionary for a button is 
            2: button was just pressed
            1: button is pressed
            0: button is not pressed
            -1: button was just released 

        Returns
        -------
        dictionary
            dictionary with keys for buttons from BUTTON_NAMES, value is either 2, 1, 0, or -1
        
        '''
        buttons = {"X": 0, "CIRCLE": 0, "TRIANGLE": 0, "SQUARE": 0 , "L1": 0, "R1": 0, "L2": 0, "R2": 0, "SHARE": 0, "OPTIONS": 0, "PLAY_STATION": 0, 
            "L3": 0, "R3": 0,"UP": 0, "DOWN": 0, "LEFT": 0, "RIGHT": 0 } # 1 is pressed, 0 is not pressed, -1 is just released
    
        for i in range(0, tempButton.__len__()):
            button = tempButton[i]
            if self.buttonStatus[self.BUTTON_NAMES[i]] == True and button == 0: # button just released
                button = -1
            if self.buttonStatus[self.BUTTON_NAMES[i]] == False and button == 1: # button just pressed
                button = 2

            if button < 1:
                self.buttonStatus[self.BUTTON_NAMES[i]] = False
            else:
                self.buttonStatus[self.BUTTON_NAMES[i]] = True
        
            buttons[self.BUTTON_NAMES[i]] = button
            
        return buttons

    def publishVizAngles(self, anglesToViz):
        vizAngles = Float32MultiArray()
        anglesToViz[1] += pi/2 
        # anglesToViz[3] += pi/2 
        vizAngles.data = rad2deg(anglesToViz)
        self.vizPub.publish(vizAngles)

    def publishAngles(self, anglesToPub):
        goalTopicData = Float32MultiArray()
        correctedDirection = self.arm.correctAngleDirection(self.arm.getPublishingAngles())
        offsetAngles = rad2deg(self.arm.addSparkMaxOffsets(correctedDirection))
        goalTopicData.data = [offsetAngles[0], offsetAngles[1], offsetAngles[2],
                              0, 0, offsetAngles[3], offsetAngles[4]] 
        
        # testTopicData = Float32MultiArray()
        # testTopicData.data = self.arm.getPublishingAngles()
        self.goalPub.publish(goalTopicData)
        self.publishVizAngles(correctedDirection)

    # ROS Topic Subscriptions
    def onCurrPosUpdate(self, data:Float32MultiArray):
        # don't forgoet to
        rawCurAngles = data.data
        correctedOrientation = [rawCurAngles[0], rawCurAngles[1], rawCurAngles[2],
                                rawCurAngles[3], rawCurAngles[4]]
        offsetsRemovedAngles = self.arm.removeSparkMaxOffsets(deg2rad(correctedOrientation))
        self.curAngleQueue.put(offsetsRemovedAngles)

    def onArmStateUpdate(self, data):
        self.armState = data.data
        if self.armState == "Idle" or self.armState == "Manual":
            self.arm.setMode("Forwad") 
            with self.joyInputQueue.mutex:
                self.joyInputQueue.queue.clear()
        if self.armState == "IK":
            self.arm.setMode("IK")

    def onJoystickUpdate(self, data):
        ''' Callback function for the arm_inputs topic'''

        global movementSpeed
        global isMovementNormalized
        global curArmAngles
        global newTargetValues

        buttonsPressed = [data.x, data.o, data.triangle, data.square, data.l1, data.r1, data.l2, data.r2, data.share, data.options, 0, 0, 0, 0, 0, 0, 0]
        # isButtonPressed = {"X": data.x, "CIRCLE": data.o, "TRIANGLE": data.triagle, 
        #                         "SQUARE": 0, "L1": data.l1, "R1": data.r1, "L2": data.l2, 
        #                         "R2": data.r2, "SHARE": data.share, "OPTIONS": data.options, 
        #                         "PLAY_STATION": 0, "L3": 0, "R3": 0, "UP": 0, "DOWN": 0, 
        #                         "LEFT": 0, "RIGHT": 0} 
        isButtonPressed = self.getJoystickButtonStatus(buttonsPressed)
        # print(isButtonPressed)
        joystickAxesStatus = {"L-Right": -data.l_horizontal, "L-Down": -data.l_vertical, "L2": data.l2, 
                            "R-Right": -data.r_horizontal, "R-Down": -data.r_vertical, "R2": data.r2}
        
        self.joyInputQueue.put([isButtonPressed, joystickAxesStatus])
        self.joyInput = [isButtonPressed, joystickAxesStatus]

    # Main Loop Code

    def updateTarget(self, joystickData):
        pass

    def main(self):
        print(f'------ {self.arm.curMode} ------')
        while not rospy.is_shutdown():
            # deal with controller input queue 
            # do IK
            # publish target if state allows it

            if not self.curAngleQueue.empty():
                self.arm.setCurAngles(self.curAngleQueue.get())

            
            if not self.joyInputQueue.empty():
                [buttonPressed, joystickStatus] = self.joyInputQueue.get()
                [buttonPressed, joystickStatus] = self.joyInput

                                # Add new button combination for sampling sequence
                # if buttonPressed["L1"] and buttonPressed["R1"]:
                #     self.handle_sampling_sequence()

                if buttonPressed["TRIANGLE"] == 2:
                    self.arm.iterateMode()
                    print(f'------ {self.arm.curMode} ------')
                if buttonPressed["CIRCLE"] == 2:
                    # self.arm.storeSparkMaxOffsets(self.arm.curAngles)
                    print('------ Homed------')
                    self.arm.homeArm()

                if self.armState == "IK" and self.arm.getCurMode() == "Cyl":
                    self.arm.controlTarget(buttonPressed, joystickStatus)
                    status = self.arm.inverseKinematics()
                    goalAngles = self.arm.getOffsetGoalAngles()
                    self.publishAngles(goalAngles)
                elif self.armState == "IK" and self.arm.getCurMode() == "Forward":
                    self.arm.activeForwardKinematics(buttonPressed, joystickStatus)
                    # angles offsets?
                    goalAngles = self.arm.getOffsetGoalAngles()
                    self.publishAngles(goalAngles)

                print(self.arm.getOffsetGoalAngles())
                # print(buttonPressed)
            
            if self.armState != "IK":
                # self.arm.passiveForwardKinematics()
                pass

            # rospy.loginfo(self.arm.cylTarget)
            self.rate.sleep()

        

if __name__ == "__main__":
    node = ArmSciNode()
    node.main()
