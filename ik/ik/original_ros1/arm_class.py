import numpy as np
from math import pi, sin, cos, atan2, sqrt
from copy import deepcopy

class Arm():
    def __init__(self, numJoints:int, dhTable, offsets, angleOrientation, startingAngles):
        ''' Object That represents a robot arm config with common functions

        Parameters
        ----------
        numJoints
            number of joints in the arm
        dhTable
            the arms dh-table

        '''
        self.isHomed = False
        self.numJoints:int = numJoints
        self.curAngles = [0]*numJoints
        self.goalAngles = [0]*numJoints 
        self.prevGoalAngles = [0]*numJoints
        self.sparkMaxOffsets = [0]*self.numJoints
        self.trueStartingAngles = [0]*self.numJoints
        self.publishingAngles = [0]*self.numJoints

        if len(startingAngles) != numJoints:
            print("The number of joints in the starting angles liss")
        self.startingAngles = startingAngles
        self.goalAngles = deepcopy(startingAngles)

        if len(dhTable) != numJoints:
            print("The number of joints in the DH table does not match the nuber specified")
            raise TypeError
        self.dhTable = np.array(dhTable) # pass [d, Theta, r, alpha]

        if len(offsets) != numJoints:
            print("The number of joints in the offsets list does not match the nuber specified")
            raise TypeError
        self.offsets = offsets

        if len(angleOrientation) != numJoints:
            print("The number of joints in the angle orientation does not match the nuber specified")
            raise TypeError
        self.angleOrientation = angleOrientation

        self.target = [0]*6 # [x, y, z, roll , pitch, yaw]
        self.prevTarget = [0]*6 # [x, y, z, roll , pitch, yaw]
        self.modes = ["Forward"]
        self.curMode = self.modes[0] # makes this a data structure function callback
        self.numModes = len(self.modes)
        self.CONTROL_SPEED = 0.003 
    
    def updateDHTable(self, newAngles):
        ''' Updates to the DH Table to have the current joint angles
        
        Update the theta values of the DH table. Will only succesfully run if the
        length of the jointAngles list equals self.numJoints.

        Paramters
        ---------
        jointAngles
            list of angles for each joint

        '''
        self.dhTable[:, 1] = newAngles 

    def getDHTable(self):
        return self.dhTable

    def iterateMode(self):
        ''' Iterate the Kinematics Mode of the Arm
        '''
        if not self.isHomed:
            return

        nextIndex = self.modes.index(self.curMode) + 1
        if nextIndex > self.numModes:
            nextIndex = 0

        self.curMode = self.modes[nextIndex]

    def setMode(self, newMode: str):
        '''Set the kinematics mode of the arm

        Parameters
        ----------
        newMode
            the mode to set the arms kinematic mode to

        Return
        ------
        bool
            True - mode succesfully updated
            False - mode not succesfully updated
        '''
        if newMode in self.modes:
            self.curMode = newMode
            return True 
        return False

    def getCurMode(self) -> str:
        return self.curMode

    def setTarget(self, goalPos):
        ''' Update the target of IK


        '''
        self.updateTarget = goalPos

    def controlTarget(self, isButtonPressed, joystickStatus):
        pass

    def controlEndEffector(self, goalPos):
        pass

    def setCurAngles(self, angles:np.ndarray):
        self.curAngles = angles

    def getGoalAngles(self):
        return deepcopy(self.goalAngles)

    def getOffsetGoalAngles(self):
        angles = self.getGoalAngles()
        offsetAngles = [0]*self.numJoints
        for i in range(len(angles)): 
            offsetAngles[i] = angles[i] + self.offsets[i] 
        return deepcopy(offsetAngles) 

    def removeOffsets(self, angles):
        correctedAngles = [0]*self.numJoints
        for i in range(len(angles)):
            correctedAngles[i] = angles[i] - self.offsets[i] 
        return deepcopy(correctedAngles)

    def getPublishingAngles(self):
        angles = self.getOffsetGoalAngles()
        for i in range(self.numJoints):
            self.publishingAngles[i] = angles[i] - self.trueStartingAngles[i]
        return deepcopy(self.publishingAngles)

    def correctAngleDirection(self, angles):
        correctedDirection = [0]*self.numJoints
        for i in range(len(angles)):
            correctedDirection[i] = angles[i]*self.angleOrientation[i]
        return deepcopy(correctedDirection)

    def removeSparkMaxOffsets(self, angles):
        offsetAngles = deepcopy(angles)
        if len(angles) != self.numJoints:
            return angles
        for i in range(self.numJoints):
            offsetAngles[i] -= self.sparkMaxOffsets[i]
        return offsetAngles

    def addSparkMaxOffsets(self, angles):
        offsetAngles = deepcopy(angles)
        # offsetAngles[1] += pi/2
        if len(angles) != self.numJoints:
            return angles
        for i in range(self.numJoints):
            offsetAngles[i] += self.sparkMaxOffsets[i] #- self.trueStartingAngles[i] 
            # offsetAngles[i] +=  - self.trueStartingAngles[i] 
        return offsetAngles

    def storeSparkMaxOffsets(self, sparkMaxAngles):
        if len(sparkMaxAngles) != self.numJoints:
            return False
        sparkMaxAngles[3] = -sparkMaxAngles[3]
        self.sparkMaxOffsets = sparkMaxAngles
        self.goalAngles = self.removeSparkMaxOffsets(self.goalAngles)
        return True

    def homeArm(self):
        homeAngles = self.getOffsetGoalAngles()
        homeAngles[1] -= pi/2
        self.storeSparkMaxOffsets(homeAngles)
        self.goalAngles = deepcopy(self.startingAngles)
        self.isHomed = True
        print(f'---- {self.startingAngles} -----')
        print(f'---- {self.goalAngles} -----')

    def calculateRotationAngles(self, transformationMatrix):
        ''' Returns the roll, pitch, and yaw angles of a transformation matrix

        The function will also work with a rotation matrix since it is embeded within 
        a transformation matrix:
            | [ Rotation ] X |
            | [          ] Y |
            | [  Matrix  ] Z |
            | 0   0   0    1 |

        I recommend looking at ZYX matrix in variable form to understand how the formulas 
        for the angles were derived.

        Parmaters
        ---------
        transformationMatrix
            a 4x4 transformation matrix or a 3x3 rotation m atrix

        Returns
        -------
        list
            a list of floats containing the angles in the form [roll, pitch, yaw]
        '''

        roll = atan2(transformationMatrix[2,1], transformationMatrix[2,2])
        pitch = atan2(-transformationMatrix[2,0], sqrt(transformationMatrix[2,1]**2 + transformationMatrix[2,2]**2))
        yaw = atan2(transformationMatrix[1,0], transformationMatrix[0,0])

        return [roll, pitch, yaw]

    def createTransformationMatrix(self, d, theta, r, alpha):
        ''' Creates a transform matrix based on dh-table paramters.

        Input is assumed to be relative to user defined origin (base_link, link_1, etc). Uses
        matrix 3.10 (p.g 79) from ECE470 textbook. 

        Paramters
        ---------
        d
            d value from dh-table
        theta
            theta value from dh-table
        r
            r value from dh-table
        alpha
            alpha value from dh-table

        Return
        ------
        numpy matrix
            the transfromation matrix based on the given paramters of the dh-table
        '''
        # convert theta and alpha to radians

        #DELETED SOME STUFF

        # define cos and sin for theta and alpha

        cosTheta = cos(theta)
        sinTheta = sin(theta)

        cosAlpha = cos(alpha)
        sinAlpha = sin(alpha)

        DHTransformMatrix = np.array([
            [cosTheta, (-sinTheta * cosAlpha), (sinTheta * sinAlpha), (r * cosTheta)],
            [sinTheta, (cosTheta * cosAlpha), (-cosTheta * sinAlpha), (r * sinTheta)],
            [0, sinAlpha, cosAlpha, d],
            [0, 0, 0, 1]
            ])
        # print(np.round(DHTransformMatrix, 2))
        return DHTransformMatrix

    def calculateTransformToLink(self, linkNumber):
        ''' Find the transform matrix to specified location
        
        Basically just multiplies all element transforms from 0 to linkNumber. Uses createTransformMatrix().

        Paramters
        ---------
        dhTable
            dhTable of arm

        linkNumber
            the number of the link you want the transform of (0 is base)

        Returns
        ------
        numpy matrix
            the multiplied matrix
        '''
        dhTable = self.getDHTable() 
        # dhTable[3, 1] += atan2(92,67) 
        # print(dhTable)
        transformToLink = np.eye(4)

        for i in range(0, linkNumber):
            ithTransform = self.createTransformationMatrix(dhTable[i][0], dhTable[i][1], dhTable[i][2], dhTable[i][3])
            transformToLink = np.matmul(transformToLink, ithTransform)

        return transformToLink

    def forwardKinematics(self):
        self.goalAngles = self.curAngles
        # self.updateDHTable(self.curAngles)
        # print(self.goalAngles)

    def activeForwardKinematics(self):
        pass

    def passiveForwardKinematics(self):
        pass

    def inverseKinematics(self, dhTable, target):
        ''' Defined in each arm class
        '''
        pass

