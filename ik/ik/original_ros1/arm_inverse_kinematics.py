#!/usr/bin/env python3

import rospy
import numpy as np
import ik_library as ik
import geometry_msgs.msg
import tf_conversions
import tf2_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float32MultiArray, String
from rover.msg import ArmInputs
from rover.srv import Corrections, CorrectionsResponse, GoToArmPos, GoToArmPosResponse, SaveArmPos, SaveArmPosResponse
import json
import copy
from math import pi

# Global Constants 
NODE_RATE = 1000
VECTOR_SMOOTHING = 0.5

# Global Variables
global curArmAngles # angles IK thinks the arm is it relative to the home position 
global liveArmAngles # angles coming in from sensors
global prevTargetTransform # transform matrix of previous transform
global prevTargetValues # [roll, pitch, yaw, [x, y, z]]
global newTargetValues

global isMovementNormalized
global isIKEntered

global goToPosValues # [isSrvCalled, [posAngles]]
global angleCorrections
global savedCanAngles

global nRevTheta 
global prevArmAngles

class ScriptState():
    def __init__(self) -> None:
        pass

    def main(self, ikNode) -> None:
        ''' Function that runs the program
    
        Combines all code to takes Joystick values, chooses script mode, changes arm movement speed,
        changes EE transform, solves IK, publishes joints, and updates RViz.
        '''

    def onJoystickUpdate(self, isButtonPressed, joystickAxesStatus) -> None:
        pass

    def controlEEPosition(self) -> None:
        ''' Change the end effector target position'''
        pass
   
    def createDesiredEETarget(self, newTargetValues):
        ''' Create the End Effector target to be displayed in RViz

            Use in a inherited mode if you want to overwrite the
            updateDesiredEETransformation function.

            Paramters
            ---------
            newTargetValues:
                [roll, pitch, yaw, [x, y, z]] of the position
        '''
        # the following is specific the URDF, you will have to change these values and maybe swap coordiantes if you change URDFs
        tempTarget = copy.deepcopy(newTargetValues) # target transform scaled to rviz
        tempX = tempTarget[3][0] # swap x and y coords
        tempTarget[3][0] = tempTarget[3][1]
        tempTarget[3][1] = tempX

        # scale target transform
        if tempTarget[3][0] >= 0:
            tempTarget[3][0] = -0.00101165*tempTarget[3][0] + 0.0594
        else:   
            tempTarget[3][0] = -0.00101165*tempTarget[3][0] + 0.0494
        if tempTarget[3][1] >= 0:
            tempTarget[3][1] = 0.001025*tempTarget[3][1] + 0.03
        else:
            tempTarget[3][1] = 0.001025*tempTarget[3][1] + 0.04
        tempTarget[3][2] = (0.47/450)*tempTarget[3][2] # scale z-axis

        tempRoll = tempTarget[0] # swap roll and pitch
        tempTarget[0] = -tempTarget[1]
        tempTarget[1] = tempRoll # should be 0 for default mode
        return tempTarget

    def updateDesiredEETransformation(self, newTargetValues):
        ''' Updates tf2 transformation of desired end effector position.

            Displays a tf2 transform of the end effector position.
            
            Paramters
            ---------
            newTargetValues:
                [roll, pitch, yaw, [x, y, z]] of the position
        '''
        tempTarget = self.createDesiredEETarget(newTargetValues)
        referenceLink = "base_link" # link to base end effector transform off of
        self.displayEndEffectorTransform(tempTarget, referenceLink) # display target transform

    def displayEndEffectorTransform(self, endEffectorPosition, referenceLink="base_link", quaternionAngles=None):
        '''
        Publishes a tf2 transform at the End Effector Position 

        Parameters
        ----------
        endEffectorPosition
            The [roll, pitch, yaw, [x, y, z]] position of the end effector
        referenceLink
            The link you want to display the end effector transform relative to.
            By default is the "base_link" of the arm.
        quaternionAngles
            A list of 4 numbers that represent a frame in space using quaternions. 
            Use to ignore endEffectorPosition paramter. By default is None.
        '''
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = referenceLink
        t.child_frame_id = "target_position"

        posArray = endEffectorPosition
        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = posArray[3][0]
        t.transform.translation.y = posArray[3][1]
        t.transform.translation.z = posArray[3][2]
        
        q = tf_conversions.transformations.quaternion_from_euler(posArray[0], posArray[1], posArray[2], 'sxyz')
        if quaternionAngles != None:
            q = quaternionAngles
            t.transform.rotation.x = q.x
            t.transform.rotation.y = q.y
            t.transform.rotation.z = q.z
            t.transform.rotation.w = q.w
        else:
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
        br.sendTransform(t)

class ForwardKin(ScriptState):
    ''' The Forward Kinematics Mode

        As the arm_ik node does not do forward kinematics, this node
        simply follows what the arm is doing based on the real angles
        given.
    '''
    def main(self, ikNode) -> None:
        global prevTargetValues
        global curArmAngles
        global armPos

        curArmAngles = copy.deepcopy(liveArmAngles)
        adjustedCurAngles = curArmAngles
        adjustedCurAngles[1] += pi/2

        dhTable = ik.createDHTable(adjustedCurAngles)
        prevTargetTransform = ik.calculateTransformToLink(dhTable, 6)

        [newRoll, newPitch, newYaw] = ik.calculateRotationAngles(prevTargetTransform)
        newTargetValues = [newRoll, newPitch, newYaw, [prevTargetTransform[0][3], prevTargetTransform[1][3], prevTargetTransform[2][3]]]
        prevTargetValues = newTargetValues

        temp = [newRoll, newPitch, newYaw, prevTargetTransform[0][3], prevTargetTransform[1][3], prevTargetTransform[2][3]]
        
        temp2 = Float32MultiArray()
        temp2.data = temp

        armPos.publish(temp2)

        print(temp)
        self.updateDesiredEETransformation(newTargetValues)
        # publishNewAngles(liveArmAngles)

class IKMode(ScriptState):
    '''The General Description of the various IK modes'''
    def main(self, ikNode) -> None:
        global curArmAngles
        global prevArmAngles
        global prevTargetValues
        global prevTargetTransform
        global goToPosValues
        
        ikNode.publishNewAngles(curArmAngles)
        

        self.updateDesiredEETransformation(prevTargetValues)

        if goToPosValues[0]: # if the service flag to go to a saved position has been called
            goToPosValues[0] = False
            curArmAngles = goToPosValues[1]
            prevArmAngles = curArmAngles

            dhTable = ik.createDHTable(curArmAngles)
            prevTargetTransform = ik.calculateTransformToLink(dhTable, 6)

            [newRoll, newPitch, newYaw] = ik.calculateRotationAngles(prevTargetTransform)
            newTargetValues = [newRoll, newPitch, newYaw, [prevTargetTransform[0][3], prevTargetTransform[1][3], prevTargetTransform[2][3]]]
            prevTargetValues = newTargetValues
            
            self.updateDesiredEETransformation(newTargetValues)

    def shouldLimitSpeed(self, newArmAngles, curArmAngles):
        ''' Limits the rate at which the end effector targert changes

        Helps arm joints move in sync (ish)

        '''

        jointSpeeds = [0.005, 0.003, 0.01, 0.075, 0.075, 0.0375, 0.04]
        angleDelta = list(np.subtract(np.array(newArmAngles), np.array(curArmAngles)))
        # slowedDownAngles = copy.deepcopy(newArmAngles)

        for i in range(len(jointSpeeds)):
            if abs(angleDelta[i]) > jointSpeeds[i]:
                # exit, which tells ik to half distance of delta vector and try again until all joints can move there within the time frame
                return False

        return True

    def controlEEPosition(self, prevTargetValues):
        global movementSpeed

        self.positionScale = 10*movementSpeed
        self.angleScale = 0.1*movementSpeed
       
        self.newTarget = copy.deepcopy(prevTargetValues)

        self.newRoll = self.newTarget[0]
        self.newPitch = self.newTarget[1]
        self.newYaw = self.newTarget[2]
    
        self.newX = self.newTarget[3][0]
        self.newY = self.newTarget[3][1]
        self.newZ = self.newTarget[3][2]
    
    def controlGripperAngle(self, isButtonPressed, curArmAngles):
        ''' Adjust the angle at which the gripper is open

        Moves all fingers by an equal amount. Focuses on L1 and L2.

        Paramters
        ---------
        isButtonPressed
            a dictionary containing buttons pressed. Values can be retrived via isButtonPressed["Button Name"].
        
        Returns
        -------
        float
            a float containing the angle at which the gripper is open.
        '''
        global movementSpeed

        gripperAngle = curArmAngles[6]
        # Tune amount that angle is changed with each cycle
        angleIncrement = 0.1*movementSpeed

        # If both buttons are pressed at the same time, angle will not change
        if isButtonPressed["CIRCLE"] == 1 and isButtonPressed["X"] != 1:
            gripperAngle -= angleIncrement

        if isButtonPressed["X"] == 1 and isButtonPressed["CIRCLE"] != 1:
            gripperAngle += angleIncrement

        return gripperAngle

    def onJoystickUpdate(self, isButtonPressed:dict, joystickAxesStatus:dict) -> None:
        global curArmAngles
        global prevArmAngles
        global prevTargetTransform
        global prevTargetValues
        global movementSpeed
        global isMovementNormalized
        

        try:
            dhTable = ik.createDHTable(curArmAngles)

            targetEEPos = self.controlEEPosition(isButtonPressed, joystickAxesStatus, prevTargetValues, 
                                        prevTargetTransform)
            
            targetAngles = ik.inverseKinematics(dhTable, targetEEPos) 
            # targetAngles = self.continousInvTan(targetAngles, curArmAngles)
            targetAngles[1] = targetAngles[1] - pi/2 # raises zero position of shoulder (shoulder is pointing up)

            maxIterations = 10
            iteration = 0
            originalSpeed = movementSpeed

            while isMovementNormalized and (iteration < maxIterations):
                if not self.shouldLimitSpeed(targetAngles, curArmAngles):
                    break
                iteration += 1
                movementSpeed = movementSpeed*0.5

                targetEEPos = self.controlEEPosition(isButtonPressed, joystickAxesStatus, prevTargetValues, 
                                        prevTargetTransform)
                targetAngles = ik.inverseKinematics(dhTable, targetEEPos)
                targetAngles = self.continousInvTan(targetAngles, curArmAngles)

            movementSpeed = originalSpeed

            targetAngles.append(self.controlGripperAngle(isButtonPressed, curArmAngles))

            prevArmAngles = curArmAngles
            curArmAngles = targetAngles
            prevTargetTransform = targetEEPos
            prevTargetValues = newTargetValues
        except ik.CannotReachTransform:
            print("Cannot reach outside of arm workspace")
        # except Exception as ex:
        #     print(ex)

    def continousInvTan(self, armAngles, prevArmAngles):
        ''' Makes output of atan continous instead of fliping 180 degrees
            
            The range of atan(x) is (-pi/2, pi/2) which causes joints in the arm
            to flip suddenly. This function changes the angle to the its equivlanet 
            value with the opposite sign the prevent flipping.
            
            Paramaters
            ----------
            armAngles
                an array of angles that correspond to the arms joint
            prevArmAngles
                an array of angles that correspond the values of the arm joints in the
                previous iteration
            
            Returns
            -------
            array
                an array for the adjusted arm angles
        '''
        global nRevTheta
        
        adjustedPrevAngles = copy.deepcopy(prevArmAngles)
        adjustedPrevAngles[3] -= 2*pi*nRevTheta[3]
        adjustedPrevAngles[5] -= 2*pi*nRevTheta[5]

        deltaTheta4 = armAngles[3] - adjustedPrevAngles[3] 
        deltaTheta6 = armAngles[5] - adjustedPrevAngles[5] 
        deltaAngles = [0, 0, 0, deltaTheta4, 0, deltaTheta6]

        for i in range(len(deltaAngles)):
            if adjustedPrevAngles[i] >= 0: 
                if round(deltaAngles[i], 5) < 0 and abs(deltaAngles[i]) > pi/1.5: # round to 5 decimals to not worry about floating point run off
                    nRevTheta[i] += 1 # increase current revolution if switchted to negative from positive
                    # print("p -> n, link: ", i + 1, ", at time: ", rospy.get_time())
                    # print("link: ", i+1, " angles are: ", armAngles[i], ", were: ", prevArmAngles[i])
                    # if i == 5:
                    #     print()
            else:
                if round(deltaAngles[i], 5) > 0 and abs(deltaAngles[i]) > pi/1.5:
                    nRevTheta[i] -= 1 # decrease current revolution if switched to positive from negative
                    # print("n -> p, link: ", i + 1, ", at time: ", rospy.get_time())
                    # print("link: ", i+1, " angles are: ", armAngles[i], ", were: ", prevArmAngles[i])
                    # if i == 5:
                    #     print()

        for i in range(len(armAngles)):
            if nRevTheta[i] != 0:
                armAngles[i] += 2*pi*nRevTheta[i]

        return armAngles

class DefaultIK(IKMode):
    ''' The Default mode for IK

        In this mode you specify the x, y, z coordinates and
        the roll, pitch, yaw variables which is then used to make
        the target matrix.
    '''
    def controlEEPosition(self, isButtonPressed, joystickAxis, prevTargetValues, prevTargetTransform):
        super().controlEEPosition(prevTargetValues)
        global newTargetValues

        # control x, y, z
        if joystickAxis["L-Down"] != 0:
            self.newX -= joystickAxis["L-Down"]*self.positionScale
        if joystickAxis["L-Right"] != 0:
            self.newY -= joystickAxis["L-Right"]*self.positionScale
        if joystickAxis["L2"] != 0:
            self.newZ -= joystickAxis["L2"]*self.positionScale
        if joystickAxis["R2"] != 0:
            self.newZ += joystickAxis["R2"]*self.positionScale
        
        # control roll, pitch, yaw
        if joystickAxis["R-Right"] != 0:
            self.newYaw -= joystickAxis["R-Right"]*self.angleScale
        if joystickAxis["R-Down"] != 0:
            self.newPitch -= joystickAxis["R-Down"]*self.angleScale
        if isButtonPressed["DOWN"]:
            self.newPitch -= self.angleScale
        elif isButtonPressed["UP"]:
            self.newPitch += self.angleScale
        if isButtonPressed["L1"]:
            self.newRoll -= self.angleScale
        elif isButtonPressed["R1"]:
            self.newRoll += self.angleScale

        self.newTarget = [self.newRoll, self.newPitch, self.newYaw, [self.newX, self.newY, self.newZ]]
        # print("x: ", self.newX, " y: ", newY, " z: ", newZ, " roll: ", newRoll, " pitch: ", newPitch, " yaw: ", newYaw)
        newTargetValues = self.newTarget
        
        return ik.createEndEffectorTransform(self.newRoll, self.newPitch, self.newYaw, self.newTarget[3])

    def updateDesiredEETransformation(self, newTargetValues):
        tempTarget = self.createDesiredEETarget(newTargetValues)
        tempTarget[1] = 0
        self.displayEndEffectorTransform(tempTarget, "base_link") # display target transform

class GlobalIK(IKMode):
    ''' The Global IK mode

        This mode manipulates the target matrix such that all 
        changes in coordinates (x, y, z, roll, pitch, yaw) occur
        relative to the global coordinate frame.
    '''
    def controlEEPosition(self, isButtonPressed, joystickAxis, prevTargetValues, prevTargetTransform):
        super().controlEEPosition(prevTargetValues)
        global newTargetValues

        self.newRoll = 0
        self.newPitch = 0
        self.newYaw = 0

        if joystickAxis["L-Down"] != 0:
            self.newX -= joystickAxis["L-Down"]*self.positionScale
        if joystickAxis["L-Right"] != 0:
            self.newY -= joystickAxis["L-Right"]*self.positionScale
        if joystickAxis["L2"] != 0:
            self.newZ -= joystickAxis["L2"]*self.positionScale
        if joystickAxis["R2"] != 0:
            self.newZ += joystickAxis["R2"]*self.positionScale
        # control roll, pitch, yaw
        if joystickAxis["R-Right"] != 0:
            self.newRoll += joystickAxis["R-Right"]*self.angleScale
        if joystickAxis["R-Down"] != 0:
            self.newPitch += joystickAxis["R-Down"]*self.angleScale
        # if isButtonPressed["UP"]:
        #     self.newPitch = -self.angleScale
        # elif isButtonPressed["DOWN"]:
        #     self.newPitch = self.angleScale
        if isButtonPressed["L1"]:
            self.newYaw = -self.angleScale
        elif isButtonPressed["R1"]:
            self.newYaw = self.angleScale
        
        prevRotation = prevTargetTransform[:3, :3]
        
        newRotation = np.dot(ik.createRotationMatrix(self.newRoll, self.newPitch, self.newYaw, "ypr"), prevRotation)
        newTransformation = np.block([
                                        [newRotation, np.array([[self.newX, self.newY, self.newZ]]).transpose()],
                                        [0, 0, 0, 1]
                                        ])
        [correctedRoll, correctedPitch, correctedYaw] = ik.calculateRotationAngles(newTransformation)

        newTargetValues = [correctedRoll, correctedPitch, correctedYaw, [self.newX, self.newY, self.newZ]]
        # print("x: ", newX, " y: ", newY, " z: ", newZ, " roll: ", correctedRoll, " pitch: ", correctedPitch, " yaw: ", correctedYaw)
        
        return newTransformation
    
    def updateDesiredEETransformation(self, newTargetValues):
        tempTarget = self.createDesiredEETarget(newTargetValues) 
        tempTarget[:3] = [0, 0, 0] # don't rotate global axis
        self.displayEndEffectorTransform(tempTarget, "base_link") # display target transform

class RotRelativeIK(IKMode):
    ''' The Relative Rotation IK mode

        This mode moves the x, y, z coordinates relative to the
        global coordinate system but rotates it relative to the
        end effectors.
    '''
    def controlEEPosition(self, isButtonPressed, joystickAxis, prevTargetValues, prevTargetTransform):
        super().controlEEPosition(prevTargetValues)
        global newTargetValues

        self.newRoll = 0
        self.newPitch = 0
        self.newYaw = 0

        if joystickAxis["L-Down"] != 0:
            self.newX -= joystickAxis["L-Down"]*self.positionScale
        if joystickAxis["L-Right"] != 0:
            self.newY -= joystickAxis["L-Right"]*self.positionScale
        if joystickAxis["L2"] != 0:
            self.newZ -= joystickAxis["L2"]*self.positionScale
        if joystickAxis["R2"] != 0:
            self.newZ += joystickAxis["R2"]*self.positionScale
        # control roll, pitch, yaw
        if joystickAxis["R-Right"] != 0:
            self.newRoll = -joystickAxis["R-Right"]*self.angleScale
        if joystickAxis["R-Down"] != 0:
            self.newPitch = joystickAxis["R-Down"]*self.angleScale
        if isButtonPressed["L1"]:
            self.newYaw = -self.angleScale
        elif isButtonPressed["R1"]:
            self.newYaw = self.angleScale
        
        prevRotation = prevTargetTransform[:3, :3]
        
        newRotation = np.matmul(prevRotation, ik.createRotationMatrix(self.newRoll, self.newPitch, self.newYaw, "ypr"))
        newTransformation = np.block([
                                        [newRotation, np.array([[self.newX, self.newY, self.newZ]]).transpose()],
                                        [0, 0, 0, 1]
                                        ])
        [correctedRoll, correctedPitch, correctedYaw] = ik.calculateRotationAngles(newTransformation)

        newTargetValues = [correctedRoll, correctedPitch, correctedYaw, [self.newX, self.newY, self.newZ]]

        return newTransformation

    def updateDesiredEETransformation(self, newTargetValues):
        tempTarget = [0, 0 , pi, [0, 0, 198*0.47/450]] # Display frame relative to tip of of end effector
        self.displayEndEffectorTransform(tempTarget, "Link_6")
        
class RelativeIK(RotRelativeIK):
    ''' The Relative IK mode
    
        This mode moves the target matrix relative to the
        end effector frame. Class inherits from RotRelativeIK
        because they share the same end effector visualization.
    '''
    def controlEEPosition(self, isButtonPressed, joystickAxis, prevTargetValues, prevTargetTransform):
        IKMode.controlEEPosition(self, prevTargetValues)
        global curArmAngles
        global newTargetValues
        [newRoll, newPitch, newYaw, newX, newY, newZ] = [0, 0, 0, 0, 0, 0]
        
        if joystickAxis["L-Down"] != 0:
            newX = joystickAxis["L-Down"]*self.positionScale
        if joystickAxis["L-Right"] != 0:
            newY = -joystickAxis["L-Right"]*self.positionScale
        if joystickAxis["L2"] != 0:
            newZ = -joystickAxis["L2"]*self.positionScale
        if joystickAxis["R2"] != 0:
            newZ = joystickAxis["R2"]*self.positionScale
        # control roll, pitch, yaw
        if joystickAxis["R-Right"] != 0:
            newRoll = joystickAxis["R-Right"]*self.angleScale
        if joystickAxis["R-Down"] != 0:
            newPitch = joystickAxis["R-Down"]*self.angleScale
        if isButtonPressed["L1"]:
            newYaw = -self.angleScale
        elif isButtonPressed["R1"]:
            newYaw = self.angleScale
        
        t06 = ik.calculateTransformToLink(ik.createDHTable(curArmAngles), 6) # transform to link 5
        r06 = t06[:3, :3] # rotation matrix of third link
        relativeXYZ = np.matmul(r06, np.transpose([np.array([newX, newY, newZ ])]))
        targetXYZ = relativeXYZ+np.transpose([prevTargetTransform[:3, 3]])

        prevRotation = prevTargetTransform[:3, :3]
        newRotation = np.matmul(prevRotation, ik.createRotationMatrix(newRoll, newPitch, newYaw, "ypr"))

        newTransformation = np.block([
                                        [newRotation, targetXYZ],
                                        [0, 0, 0, 1]
                                        ])

        [correctedRoll, correctedPitch, correctedYaw] = ik.calculateRotationAngles(newTransformation)
        [correctedX, correctedY, correctedZ] = [newTransformation[0][3], newTransformation[1][3], newTransformation[2][3]]
        newTargetValues = [correctedRoll, correctedPitch, correctedYaw, [correctedX, correctedY, correctedZ]]

        return newTransformation

class CamRelativeIK(RotRelativeIK):
    ''' The Camera Relative IK mode

        This mode changes the target matrix relative to the 
        frame of the camera. Ideal for POV control of the arm.
    '''
    def controlEEPosition(self, isButtonPressed, joystickAxis, prevTargetValues, prevTargetTransform):
        IKMode.controlEEPosition(self, prevTargetValues)
        global curArmAngles
        global newTargetValues
        [newRoll, newPitch, newYaw, newX, newY, newZ] = [0, 0, 0, 0, 0, 0]
        
        if joystickAxis["L-Down"] != 0:
            newX = joystickAxis["L-Down"]*self.positionScale
        if joystickAxis["L-Right"] != 0:
            newY = -joystickAxis["L-Right"]*self.positionScale
        if joystickAxis["L2"] != 0:
            newZ = -joystickAxis["L2"]*self.positionScale
        if joystickAxis["R2"] != 0:
            newZ = joystickAxis["R2"]*self.positionScale
        # control roll, pitch, yaw
        if joystickAxis["R-Right"] != 0:
            newRoll = joystickAxis["R-Right"]*self.angleScale
        if joystickAxis["R-Down"] != 0:
            newPitch = joystickAxis["R-Down"]*self.angleScale
        if isButtonPressed["L1"]:
            newYaw = -self.angleScale
        elif isButtonPressed["R1"]:
            newYaw = self.angleScale
        
        t05 = ik.calculateTransformToLink(ik.createDHTable(curArmAngles), 5) # transform to link 5
        r05 = t05[:3, :3] # rotation matrix of third link
        relativeXYZ = np.matmul(r05, np.transpose([np.array([newX, newY, newZ ])]))
        targetXYZ = relativeXYZ+np.transpose([prevTargetTransform[:3, 3]])

        prevRotation = prevTargetTransform[:3, :3]
        newRotation = np.matmul(prevRotation, ik.createRotationMatrix(newRoll, newPitch, newYaw, "ypr"))

        newTransformation = np.block([
                                        [newRotation, targetXYZ],
                                        [0, 0, 0, 1]
                                        ])

        [correctedRoll, correctedPitch, correctedYaw] = ik.calculateRotationAngles(newTransformation)
        [correctedX, correctedY, correctedZ] = [newTransformation[0][3], newTransformation[1][3], newTransformation[2][3]]
        newTargetValues = [correctedRoll, correctedPitch, correctedYaw, [correctedX, correctedY, correctedZ]]

        return newTransformation

class InverseKinematicsNode():
    
    def __init__(self) -> None:
        global armAngles
        global armPos
        global correctionsService
        global goToArmPosService
        global saveArmPosService
        
        rospy.init_node("arm_ik")
        self.rate = rospy.Rate(NODE_RATE) # run at {NODE_RATE}Hz 

        armAngles = rospy.Publisher("arm_goal_pos", Float32MultiArray, queue_size=10)

        armPos = rospy.Publisher("arm_end_effector_pos", Float32MultiArray, queue_size=10)

        rospy.Subscriber("arm_curr_pos", Float32MultiArray, self.updateLiveArmAngles)
        rospy.Subscriber("arm_state", String, self.onStateUpdate)
        rospy.Subscriber("arm_inputs", ArmInputs, self.onControllerUpdate)
        rospy.Subscriber("ik_targets", Float32MultiArray, self.onIKTargetUpdate)

        self.SCRIPT_MODES = [ForwardKin(), DefaultIK(), GlobalIK(), RotRelativeIK(), RelativeIK(), CamRelativeIK()]
        self.curModeListIndex = 0
        self.scriptMode = self.SCRIPT_MODES[0]

        self.BUTTON_NAMES = ["X", "CIRCLE", "TRIANGLE", "SQUARE", "L1", "R1", "L2", "R2", "SHARE", "OPTIONS", "PLAY_STATION", "L3", 
                             "R3", "UP", "DOWN", "LEFT", "RIGHT"]
        self.buttonStatus= {"X": False, "CIRCLE": False, "TRIANGLE": False, "SQUARE": False, "L1": False, "R1": False, "L2": False, 
                               "R2": False, "SHARE": False, "OPTIONS": False, "PLAY_STATION": False, "L3": False, "R3": False,"UP": False, 
                               "DOWN": False, "LEFT": False, "RIGHT": False} 

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

    # ROS Stuff

    def savePosition(self, posName):
        ''' Service function that saves the current angles the arm is in

        Uses global variable curArmAngles and saves angles to file. Asks for name
        of position before saving (preferable a GUI).

        Paramters
        ---------
        posName
            the input message of SaveArmPos.srv, or the name to give the position
        
        Returns
        '''

        # (Placeholders for angles 1-5)

        # positionName = input("Enter Position Name: ")
        try:
            global curArmAngles

            positionName = posName.positionName
            print(positionName)

            newAngles = {
            "title":positionName,
            "angle0":curArmAngles[0],
            "angle1":curArmAngles[1],
            "angle2":curArmAngles[2],
            "angle3":curArmAngles[3],
            "angle4":curArmAngles[4],
            "angle5":curArmAngles[5],
            "gripperAngle":curArmAngles[6]
            }

            with open('arm_positions.json','r+') as file:
                breakLoop = False
                savedPos = json.load(file)
                # Checks if any existing positions have same positionName
                for x in range(len(savedPos["position_names"])):
                    if breakLoop:
                        break
                    if savedPos["position_names"][x]["title"] == positionName: # if found, deletes existing entry
                        savedPos["position_names"].pop(x)
                        breakLoop = True
                # Appends new position to end of file
                savedPos["position_names"].append(newAngles)
                file.seek(0)
                json.dump(savedPos,file,indent = 0)
                file.close()
            
            return SaveArmPosResponse(True)
        except Exception as ex:
            print("An error has occured")
            print(ex)
            return SaveArmPosResponse(False)

    def goToPosition(self, posName):
        ''' Pulls up GUI with options of saved joint angles.

        Paramters
        ---------
        posName
            the input data message from GoToArmPos.srv, or the name of the saved position

        Returns
        -------
        service response
            has the service been completed sucessfully
        '''
        try:
            global goToPosValues

            # retrievePos = input("Enter Position Name to Retrieve: ")
            retrievePos = posName.positionName
            tempAngles = [0, 0, 0, 0, 0, 0]

            with open('arm_positions.json','r') as file:
                found = False
                savedPos = json.load(file)
                # Checks to see if position names match request
                for x in range(len(savedPos["position_names"])):
                    if savedPos["position_names"][x]["title"] == retrievePos: # if found, changes current angles to requested position
                        found = True
                        tempAngles[0] = savedPos["position_names"][x]["angle0"]
                        tempAngles[1] = savedPos["position_names"][x]["angle1"]
                        tempAngles[2] = savedPos["position_names"][x]["angle2"]
                        tempAngles[3] = savedPos["position_names"][x]["angle3"]
                        tempAngles[4] = savedPos["position_names"][x]["angle4"]
                        tempAngles[5] = savedPos["position_names"][x]["angle5"]
                        tempAngles[6] = savedPos["position_names"][x]["gripperAngle"]

                if not found:
                    print("Position name '"+retrievePos+"' not found") # if not found, prints message
                    return GoToArmPosResponse(False)

                goToPosValues[0] = True # sets the goTO arm position value true a
                goToPosValues[1] = tempAngles
                return GoToArmPosResponse(True)
        except Exception as ex:
            print("The following error has occured: ")
            print(ex)
            return GoToArmPosResponse(False)

    def updateAngleCorrections(self, corrections):
        ''' Service function that updates the angle corrections for published IK values
        
        Paramters
        ---------
        corrections
            the input data message from Corrections.srv

        Returns
        -------
        service response
            has the service been completed sucessfully
        '''
        try:
            global angleCorrections

            angleCorrections = [corrections.correction1, corrections.correction2, corrections.correction3, corrections.correction4,
                                corrections.correction5, corrections.correction6, corrections.correction7]
            return CorrectionsResponse(True)
        except Exception as ex:
            print("The following error has occured")
            print(ex)
            return CorrectionsResponse(False)

    def publishNewAngles(self, newJointAngles):
        ''' Publishes the new angles to /arm_target_angles

        Publishes a list 32 bit floats.

        Parameters
        ---------- 
        newJointAngles
            an array of the new angles (7 elements)

        '''
        global angleCorrections
        global savedCanAngles

        ikAngles = Float32MultiArray()
        adjustedAngles = [0, 0, 0, 0, 0, 0, 0]
        for i in range(adjustedAngles.__len__()):
            adjustedAngles[i] = newJointAngles[i] - angleCorrections[i] + savedCanAngles[i]
            adjustedAngles[i] = np.rad2deg(adjustedAngles[i])

        adjustedAngles[0] = -adjustedAngles[0]
        adjustedAngles[1] = adjustedAngles[1] #+ 73.6
        adjustedAngles[2] = -adjustedAngles[2] 
        adjustedAngles[3] = adjustedAngles[3]
        adjustedAngles[4] = -adjustedAngles[4]
        adjustedAngles[5] = adjustedAngles[5]

        # print(adjustedAngles[3])
        temp = adjustedAngles[5]
        adjustedAngles[5] = adjustedAngles[4]
        adjustedAngles[4] = temp

        ikAngles.data = adjustedAngles
        armAngles.publish(ikAngles)

    def updateLiveArmAngles(self, data):
        ''' Updates the real angle that IK knows

        Should be used as the callback function of the real angle subscriber.

        Paramters
        ---------
        data
            data.data contains a list 32 bit floats that correspond to joint angles
        '''
        global liveArmAngles
        global savedCanAngles

        tempList = list(data.data)

        for i in range(len(tempList)):
            tempList[i] = np.deg2rad(tempList[i])

        tempList[0] = -tempList[0]
        # tempList[1] = -tempList[1]
        tempList[2] = -tempList[2]
        temp = tempList[5]
        tempList[5] = -tempList[4]
        tempList[4] = temp
        # tempList[5] = -tempList[5]
        # tempList[6] = tempList[6]
        tempAngles = copy.deepcopy(list(np.subtract(np.array(tempList),np.array(savedCanAngles))))
        liveArmAngles = tempAngles

    def onStateUpdate(self, data):
        ''' Callback function for arm_state rostopic

        If a state has been changed enter into forward kinematics mode
        to copy the current arm angles.
        '''
        global isIKEntered
        global savedCanAngles
        global curArmAngles
        global goToPosValues
        global prevTargetTransform
        global prevTargetValues
        global liveArmAngles

        if data.data != "IK":
            self.scriptMode = self.SCRIPT_MODES[0]
            self.curModeListIndex = 0
        else:
            if not isIKEntered:
                self.curModeListIndex = 1
                self.scriptMode = self.SCRIPT_MODES[self.curModeListIndex] # set to defaultIK mode
                print(type(self.scriptMode).__name__)

                savedCanAngles = copy.deepcopy(liveArmAngles) 
                liveArmAngles = [0, 0, 0, 0, 0, 0, 0]
                curArmAngles = [0, 0, 0, 0, 0, 0, 0]

                dhTable = ik.createDHTable([0, pi/2, 0, 0, 0, 0, 0])
                prevTargetTransform = ik.calculateTransformToLink(dhTable, 6)

                [newRoll, newPitch, newYaw] = ik.calculateRotationAngles(prevTargetTransform)
                newTargetValues = [newRoll, newPitch, newYaw, [prevTargetTransform[0][3], prevTargetTransform[1][3], prevTargetTransform[2][3]]]
                prevTargetValues = newTargetValues
                
                self.scriptMode.updateDesiredEETransformation(newTargetValues)
                #scriptMode.updateDesiredArmvizulation(curArmAngles)

                isIKEntered = True

    def onControllerUpdate(self, data):
        ''' Callback function for the arm_inputs topic'''

        global movementSpeed
        global isMovementNormalized
        global curArmAngles
        global newTargetValues

        buttonsPressed = [0, 0, data.triangle, 0, data.l1, data.r1, data.l2, data.r2, data.share, data.options, 0, 0, 0, 0, 0, 0, 0]
        # isButtonPressed = {"X": data.x, "CIRCLE": data.o, "TRIANGLE": data.triagle, 
        #                         "SQUARE": 0, "L1": data.l1, "R1": data.r1, "L2": data.l2, 
        #                         "R2": data.r2, "SHARE": data.share, "OPTIONS": data.options, 
        #                         "PLAY_STATION": 0, "L3": 0, "R3": 0, "UP": 0, "DOWN": 0, 
        #                         "LEFT": 0, "RIGHT": 0} 
        isButtonPressed = self.getJoystickButtonStatus(buttonsPressed)
        # print(isButtonPressed)
        joystickAxesStatus = {"L-Right": -data.l_horizontal, "L-Down": -data.l_vertical, "L2": data.l2, 
                            "R-Right": -data.r_horizontal, "R-Down": -data.r_vertical, "R2": data.r2}

        if isButtonPressed["TRIANGLE"] == 2: # changes mode when button is released
            if (self.curModeListIndex + 1) >= len(self.SCRIPT_MODES):
                self.curModeListIndex = 1
                self.scriptMode = self.SCRIPT_MODES[self.curModeListIndex]
            else:
                self.curModeListIndex = self.curModeListIndex + 1
                print(self.curModeListIndex)
                self.scriptMode = self.SCRIPT_MODES[self.curModeListIndex]
            print(type(self.scriptMode).__name__)

        if isButtonPressed["OPTIONS"] == 2:
            movementSpeed *= 2 
        if isButtonPressed["SHARE"] == 2:
            movementSpeed /= 2 
            if movementSpeed < 0: # don't let movement speed go into negatives
                movementSpeed = 0

        if isButtonPressed["R3"] == 2:
            isMovementNormalized = not isMovementNormalized
            print("Normalized Movement: ", isMovementNormalized)

        # print(newTargetValues)
        self.scriptMode.onJoystickUpdate(isButtonPressed, joystickAxesStatus)

    def onIKTargetUpdate(self, data):
        ''' Callback function for the /ik_target topic

            Sets the [x, y, z, roll, pitch, yaw, gripperPos] target for
            the arm and then ik publishes the angle values.
            Assumes you want default ik mode (will make changeable 
            later).

            Paramters
            ---------
            data
                A float32 array of the [x, y, z, roll, ptich, yaw, gripperPos] target
                for ik
        '''
        global curArmAngles
        global prevArmAngles
        global newTargetValues
        global prevTargetTransform
        global prevTargetValues

        if self.curModeListIndex == 1: # in default IK mode
            try:
                tempList = list(data.data)

                dhTable = ik.createDHTable(curArmAngles)
                newTargetValues = [tempList[5], tempList[4], tempList[3], 
                                [tempList[0], tempList[1], tempList[2]]]
                targetEEPos = ik.createEndEffectorTransform(tempList[5], tempList[4], tempList[3], newTargetValues[3])
                targetAngles = ik.inverseKinematics(dhTable, targetEEPos)
                targetAngles[1] -= pi/2
                # targetAngles.append(tempList[6])

                curArmAngles = targetAngles
                prevArmAngles = curArmAngles
                prevTargetTransform = targetEEPos
                prevTargetValues = newTargetValues
            except ik.CannotReachTransform:
                print("Cannot reach outside of arm workspace")
            except Exception as ex:
                print(ex)

# Main Area

if __name__ == "__main__":
    angleCorrections = [0, 0, 0, 0, 0, 0, 0]
    curArmAngles = [0, 0, 0, 0, 0, 0, 0]
    prevArmAngles = [0, 0, 0, 0, 0, 0, 0]
    liveArmAngles = [0, 0, 0, 0, 0, 0, 0]
    prevTargetValues = [0, 0, 0, [250, 0, 450]] # start position for ik
    newTargetValues = prevTargetValues
    prevTargetTransform = ik.createEndEffectorTransform(prevTargetValues[0], prevTargetValues[1],
                                                        prevTargetValues[2], prevTargetValues[3])
    goToPosValues = [False, [0, 0, 0, 0, 0, 0, 0]]
    savedCanAngles = [0, 0, 0, 0, 0, 0, 0]

    isIKEntered = False
    movementSpeed = 10/NODE_RATE
    isMovementNormalized = False

    nRevTheta = [0, 0, 0, 0, 0, 0, 0]
        
    ikNode = InverseKinematicsNode()

    # services don't like being in the ikNode object for some reason
    correctionsService = rospy.Service('update_arm_corrections', Corrections, ikNode.updateAngleCorrections)
    goToArmPosService = rospy.Service('move_arm_to', GoToArmPos, ikNode.goToPosition)
    saveArmPosService = rospy.Service('save_arm_pos_as', SaveArmPos, ikNode.savePosition)

    while not rospy.is_shutdown():
        #try:  
        ikNode.scriptMode.main(ikNode)
        #except Exception as ex:
        #    print(ex)
        ikNode.rate.sleep()
