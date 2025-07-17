#!/usr/bin/env python3

import rospy
import random
import numpy as np
from std_msgs.msg import Float32MultiArray

'''path[] saves incoming positions from path planning. path[0] is the first position the arm moves to.
when the rover starts moving to path[0], path[0] is deleted and replaced by the next in queue.

pathHistory[] saves all positions from path planning without deleting anything.'''

path = []
pathHistory = []

# used in initializeTrajectory to check if the arm is already trying to change positions
armMoving = False

# maximum average speed in deg/sec
speedMax = 15.0

simAngles = [0,-90,0,0,0,0,0]
actualAngles = [0,-90,0,0,0,0,0]
staticAngles = [0,-90,0,0,0,0,0]
staticPID = [None] * 7

KP_STATIC = 0
KI_STATIC = 0
KD_STATIC = 0

class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prevError = 0.0
        self.integral = 0.0
    
    def calculate(self, error, direction):
        if (direction == 0):
            error *= -1

        p = self.Kp * error

        self.integral = self.integral + error
        i = self.Ki * self.integral

        d = self.Kd * (error - self.prevError)
        self.prevError = error

        return (p + i + d)

    def reset(self):
        self.integral = 0
        self.prevError = 0
        return

# create staticPID controller
for i in range(7):
    staticPID[i] = PIDcontroller(KP_STATIC, KI_STATIC, KD_STATIC)

class quinticPolynomial:
    ''' Creates a quintic polynomial that determines the path of a joint angle.
    initialized with the following parameters:
    qI / qF - initial / final joint angle
    vI / vF - initial / final joint angular velocity
    aI / aF - initial / final joint angular acceleration
    tF - final time (time it takes to reach qF)'''

    def __init__(self, qI, qF, vI, vF, aI, aF, tF):
        if (qF - qI > 0):
            self.direction = 1
        else:
            self.direction = 0
            
        self.compFactor = 1

        if (tF == 0):
            self.a0 = qF
            self.a1 = 0
            self.a2 = 0
            self.a3 = 0
            self.a4 = 0
            self.a5 = 0
        else:
            self.a0 = qI
            self.a1 = vI
            self.a2 = aI / 2

            A = np.array([[tF**3, tF**4, tF**5],
                        [3*tF**2, 4*tF**3, 5*tF**4],
                        [6*tF, 12*tF**2, 20*tF**3]])
            b = np.array([[qF - qI - vI*tF - aI*tF**2],
                        [vF - vI - aI*tF],
                        [aF - aI]])
        
            x = np.linalg.solve(A, b)

            self.a3 = x[0]
            self.a4 = x[1]
            self.a5 = x[2]
 

    def calculatePoint(self, time):
        ''' Uses polynomial to calculate desired angle at specified time'''
        q = self.a0 + self.compFactor*self.a1*time + (self.compFactor**2)*self.a2*time**2 + \
            (self.compFactor**3)*self.a3*time**3 + (self.compFactor**4)*self.a4*time**4 + (self.compFactor**5)*self.a5*time**5
        return q

    def changeEndTime(self, compFactor):
        self.compFactor = compFactor
        return

# takes current arm position and goal position as arguments, and the time it should take to complete the movement
# publishes angles using a quintic trajectory model
# uses a PID controller to see if the arm is moving too slowly, and adjusts the curve accordingly
def moveToPosition(posI, posF, endTime):
    ''' Uses class quinticPolynomial to calculate quintic trajectory for each joint angle.
    Updates arm_goal_pos at 10hz (which can be changed with the frequency variable)'''
    global pubArmAngles
    global rate
    global simAngles
    global actualAngles

    testTimeStart = rospy.get_time()

    frequency = 10
    period = 1 / frequency

    # weight of each pid output
    Kp = 0
    Ki = 0
    Kd = 0

    publishPos = Float32MultiArray() # array of angles to publish

    time = 0 # time elapsed
    compFactor = 1 # factor that quintic polynomials are compressed by

    polyAngles = [None] * 7 # array of angles calculated from polynomials
    pid = [None] * 7 # array of pid output for each angle

    for angleNum in range(7):
        # by default, initial/final velocities/accelerations are 0
        polyAngles[angleNum] = quinticPolynomial(posI[angleNum], posF[angleNum], 0, 0, 0, 0, endTime)
        # Kp Ki Kd can be adjusted for each individual angle, but for now they are uniform for all 7 angles
        pid[angleNum] = PIDcontroller(Kp, Ki, Kd)
        pid[angleNum].reset()

    while (time <= endTime):
        calcStartTime = rospy.get_time()

        pidOutLargest = 0
        
        for angleNum in range(7):
            pidOut = pid[angleNum].calculate(simAngles[angleNum] - actualAngles[angleNum], polyAngles[angleNum].direction)
            if (pidOut > pidOutLargest):
                pidOutLargest = pidOut

        currCompFactor = endTime / (endTime + pidOutLargest)
        compFactor *= currCompFactor
        endTime = endTime + pidOutLargest

        time = time / currCompFactor

        for angleNum in range(7):
            polyAngles[angleNum].changeEndTime(compFactor)
            posI[angleNum] = polyAngles[angleNum].calculatePoint(time)

        publishPos.data = posI
        pubArmAngles.publish(publishPos)

        calcEndTime = rospy.get_time()
        rospy.sleep(period - (calcStartTime - calcEndTime))
        time = time + period

    testTimeEnd = rospy.get_time()

    print("Time elapsed: ", testTimeEnd - testTimeStart)
    return

# adjusts angles using PID controller to maintain static position
# repeatedly called by controlArm when there are no goal positions stored in path
def holdStaticPosition():
    global staticPID
    publishPos = Float32MultiArray()

    for angleNum in range(7):
        simAngles[angleNum] -= staticPID[angleNum].calculate(staticAngles[angleNum]
                                                            - actualAngles[angleNum], 1)

    publishPos.data = simAngles
    pubArmAngles.publish(publishPos)
    rospy.sleep(0.1)
    return

# decides whether to hold the arm at a static position or move to a new position
# it then calls the appropriate functions to do so
def controlArm():
    global armMoving
    global speedMax
    global path
    global simAngles
    global staticAngles
    global pubArmAngles
    global staticPID

    # checks if the arm is already moving to a new position
    if armMoving:
        return
    
    while len(path) == 0:
        holdStaticPosition()

    armMoving = True
    
    for angle in staticPID:
        angle.reset()

    posI = simAngles
    posF = path.pop(0)
    
    # when the arm reaches posF, we want it to stay there
    # save that position for later in staticAngles
    staticAngles = posF
    distanceMax = 0

    # find the largest angle difference, then use that difference to calculate speed
    for angleNum in range(7):
        if (abs(posF[angleNum] - posI[angleNum]) > distanceMax):
            distanceMax = abs(posF[angleNum] - posI[angleNum])

    time = distanceMax / speedMax

    if (time != 0):
        moveToPosition(posI, posF, time)

    armMoving = False
    controlArm()
    return

def savePathPlanning(angles):
    global path
    global pathHistory
    path.append(list(angles.data))
    pathHistory.append(list(angles.data))
    return

def updateSimAngles(simAngles_):
    global simAngles
    global pubActualAngles #placeholder

    simAngles = list(simAngles_.data)
    print("curr angles: ", simAngles)

    # -----------------------------------------------------
    fakeAngles = [] #placeholder begins

    for i in range(7):
        fakeAngles.append(simAngles[i] + random.randint(0,6) - 3)

    fakeAnglesPosition = Float32MultiArray()
    fakeAnglesPosition.data = fakeAngles

    pubActualAngles.publish(fakeAnglesPosition) #placeholder ends
    # -----------------------------------------------------
    return

def updateActualAngles(actualAngles_):
    global actualAngles
    actualAngles = list(actualAngles_.data)
    return


def getAngles():
    global pubArmAngles
    global pubActualAngles #placeholder publisher
    global rate
    
    rospy.init_node('trajectoryPlanning')
    pubArmAngles = rospy.Publisher("/arm_goal_pos", Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)

    # -----------------------------------------------------
    pubActualAngles = rospy.Publisher("/ARM_ACTUAL_POSITION", Float32MultiArray, queue_size=10)
    # -----------------------------------------------------

    rospy.Subscriber("/PATH_PLANNING", Float32MultiArray, savePathPlanning, queue_size=10)
    rospy.Subscriber("/arm_goal_pos", Float32MultiArray, updateSimAngles, queue_size=1000000000)
    
    # -----------------------------------------------------
    rospy.Subscriber("/ARM_ACTUAL_POSITION", Float32MultiArray, updateActualAngles, queue_size=10)
    # -----------------------------------------------------
    
    # ARM_ACTUAL_POSITION is also a placeholder for sensor data
    controlArm()
    rospy.spin()

if __name__ == '__main__':
    try:
        getAngles()
    except rospy.ROSInterruptException:
        pass