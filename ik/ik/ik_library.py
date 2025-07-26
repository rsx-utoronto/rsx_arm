#!/usr/bin/env python3
import numpy as np
import math

# Custom Exceptions

class CannotReachTransform(Exception):
    """Exception Raised when transformation cannot be reached by arm
    """

    def __init__(self, message="Transformation outside of robot workspace"):
        self.message = message
        super().__init__(self.message)


# IK Code

def createRotationMatrix(roll: float, pitch: float, yaw: float, order: str = "ypr"):
    ''' Creates a rotation matrix based on XYZ euler angles (Roll, Pitch, Yaw)

    Paramaters (euler angles)
    ----------
    roll (alpha)
        angle x-axis rotated (float)
    pitch (beta)
        angle y-axis rotates (float)
    yaw (gamma)
        angle z-axis rotates (float)
    order
        the order in which the rotations are applied, defaults to ypr

    Returns
    -------
    numpy matrix
        rotation matrix using XYZ euler angles
    '''

    # Convert input angles to radians

    # roll *= math.pi / 180
    # pitch *= math.pi / 180
    # yaw *= math.pi / 180

    # Define sine and cosine of each angle to reduce # of calculations

    cRoll = math.cos(roll)
    sRoll = math.sin(roll) # 

    cPitch = math.cos(pitch)
    sPitch = math.sin(pitch) # beta

    cYaw = math.cos(yaw) 
    sYaw = math.sin(yaw) # alpha

    # Assemble 3x3 rotation matrices
    # Rotations are applied in the same order each time:
    # Yaw --> Pitch --> Roll

    yawRotation = np.array([
                            [cYaw, -sYaw, 0],
                            [sYaw, cYaw, 0],
                            [0, 0, 1]
                            ])
    
    pitchRotation = np.array([
                            [cPitch, 0, sPitch],
                            [0, 1, 0],
                            [-sPitch, 0, cPitch]
                            ])
    
    rollRotation = np.array([
                            [1, 0, 0],
                            [0, cRoll, -sRoll],
                            [0, sRoll, cRoll]
                            ])

    rotMatrices = {"r": rollRotation, "p": pitchRotation, "y": yawRotation}

    rotationMatrix = np.matmul(np.matmul(rotMatrices[order[0]], rotMatrices[order[1]]), rotMatrices[order[2]])
    
    # Returns matrix

    return rotationMatrix

def calculateRotationAngles(transformationMatrix):
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

    roll = math.atan2(transformationMatrix[2,1], transformationMatrix[2,2])
    pitch = math.atan2(-transformationMatrix[2,0], math.sqrt(transformationMatrix[2,1]**2 + transformationMatrix[2,2]**2))
    yaw = math.atan2(transformationMatrix[1,0], transformationMatrix[0,0])

    return [roll, pitch, yaw]

def createEndEffectorTransform(roll: float, pitch: float, yaw: float, position):
    ''' Creates the matrix that defines the transform of the end effector based on target

    Uses createRotationMatrix() to define rotation portion of matrix.

    Paramaters
    ----------
    position
        list in format [x, y, z] that contains x, y, and z location matrix
    roll
        angle x-axis rotates
    pitch
        angle y-axis roates
    yaw
        angle z-axis rotates

    Returns
    -------
    numpy matrix
        transfromation matrix of target end effector position

    '''
    # define exception - outOfWorkspace

    # Raised when end effector coordinates are out of arm's range   

    # checks for out of workspace

    positionArray = np.array([position])
    
    # create 4x4 block matrix
    # | (rotation matrx) (position) |
    # |   0      0       0       1  |

    endEffectorTransform = np.block([
                                    [createRotationMatrix(roll, pitch, yaw), np.transpose(positionArray)],
                                    [0, 0, 0, 1]
                                    ])

    return endEffectorTransform

def createTransformationMatrix(r, alpha, d, theta):
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

    cosTheta = math.cos(theta)
    sinTheta = math.sin(theta)

    cosAlpha = math.cos(alpha)
    sinAlpha = math.sin(alpha)

    DHTransformMatrix = np.array([
        [cosTheta, (-sinTheta * cosAlpha), (sinTheta * sinAlpha), (r * cosTheta)],
        [sinTheta, (cosTheta * cosAlpha), (-cosTheta * sinAlpha), (r * sinTheta)],
        [0, sinAlpha, cosAlpha, d],
        [0, 0, 0, 1]
        ])

    return DHTransformMatrix

def calculateTransformToLink(dhTable, linkNumber):
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
    transformToLink = np.eye(4)

    for i in range(0, linkNumber):
        ithTransform = createTransformationMatrix(dhTable[i, 0], dhTable[i, 1], dhTable[i, 2], dhTable[i, 3])
        transformToLink = np.matmul(transformToLink, ithTransform)

    return transformToLink

def createDHTable(jointAngles):
    ''' Create DH Table for arm based on current position 

    *include detailed description here based on what you made the function do*
    Change return type - the " -> list" to match numpy matrix

    Paramaters
    ---------
    jointAngles
        array for changing angles (theta star of dh-table)

    Returns
    -------
    numpy matrix
        a matrix containing the dh table in [r, alpha, d, theta*] order
    '''

    # units in mm [r, al]
    DHTable = np.array([[0, math.pi/2, 79, jointAngles[0]],
                        [400, 0, 0, jointAngles[1]],
                        [115, math.pi/2, 0, jointAngles[2]],
                        [0, -math.pi/2, 389, jointAngles[3]],
                        [0, math.pi/2, 0, jointAngles[4]],
                        [0, 0, 219, jointAngles[5]]]) #253.125
    
    # DHTable = np.array([[0, math.pi/2, 75, jointAngles[0]],
    #                     [375, 0, 0, jointAngles[1]],
    #                     [69.55, math.pi/2, 0, jointAngles[2]],
    #                     [0, -math.pi/2, 448.125, jointAngles[3]],
    #                     [0, math.pi/2, 0, jointAngles[4]],
    #                     [0, 0, 155, jointAngles[5]]]) #253.125
    return DHTable

def inverseKinematics(dhTable, targetPos):
    ''' Calculates joint angles based on desired EE position and DH-Table Paramters

    Step by step solution can be found pages 148 - 151 and 153-154 of ECE470 textbook.
    However, textbook is missing adjustment for offset in third link so look at repo 
    wiki to learn more.


    Parameters
    ----------
    dhTable
        the DH-Table for the Arm

    targetPos
        a numpy matrix that defines the desired EE positions

    Exceptions
    ----------
    CannotReachTransform
        when the target transformation is outside of the arms reach

    Returns
    -------
    array
        list of joint angles
    '''

    # useful values from DH table
    d1 = dhTable[0][2]
    r2 = dhTable[1][0]
    r3 = dhTable[2][0]
    d4 = dhTable[3][2]
    d6 = dhTable[5][2]

    desiredWristRotation = np.array([targetPos[0][:3], targetPos[1][:3], targetPos[2][:3]])
    desiredWristOrigin = np.transpose([np.array([targetPos[0][3], targetPos[1][3], targetPos[2][3]])])

    # calculate wrist center
    rotationAdjustment = d6 * np.matmul(desiredWristRotation, np.transpose(np.array([0, 0, 1])))
    
    wristCenter = desiredWristOrigin - np.transpose([rotationAdjustment])
    
    wristCenterX = wristCenter[0][0]
    wristCenterY = wristCenter[1][0]
    wristCenterZ = wristCenter[2][0]

    theta1 = math.atan2(wristCenterY, wristCenterX)

    link3Hypotenuse = math.sqrt(r3**2 + d4**2) # adjust for offset of arm at link 3
    cosTheta3Numerator = wristCenterX**2 + wristCenterY**2 + (wristCenterZ-d1)**2 - r2**2 - link3Hypotenuse**2
    cosTheta3 = cosTheta3Numerator/(2*r2*link3Hypotenuse)

    if abs(cosTheta3) > 1:  # cos(x) cannot be greater than 1, so throw error
        raise CannotReachTransform

    # positive in front of square root assumes elbow up
    theta3 = math.atan2(cosTheta3, math.sqrt(1 - cosTheta3**2))
    theta3Adjusted = theta3 - math.atan2(r3, d4) # assumes that zero is right angle down

    innerAngle = math.atan2(link3Hypotenuse*math.sin(theta3 - math.pi/2), r2 + link3Hypotenuse*math.cos(theta3 - math.pi/2))
    theta2 = math.atan2(wristCenterZ - d1, math.sqrt(wristCenterX**2 + wristCenterY**2)) - innerAngle

    # transform matrix from base to third link
    t03 = calculateTransformToLink(createDHTable([theta1, theta2, theta3Adjusted, 0, 0, 0]), 3)

    
    r03 = np.array([t03[0][:3], t03[1][:3], t03[2][:3]]) # rotation matrix of third link
    
    r36 = np.matmul(np.linalg.inv(r03), desiredWristRotation) # rotation matrix from third link to EE

    theta5 = math.atan2(math.sqrt(1-r36[2][2]**2), r36[2][2])

    theta4 = math.atan2(r36[1][2], r36[0][2])

    theta6 = math.atan2(r36[2][1], -r36[2][0])

    return [theta1, theta2, theta3Adjusted, theta4, theta5, theta6]

    