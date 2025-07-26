from arm_science_ik import SciArm
from math import pi, atan2, sin, cos, sqrt, acos
from numpy import deg2rad, rad2deg


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
angleOrientation = [1, 1, 1, 1, 1]


arm = SciArm(5, dhTable, offsets, angleOrientation)

buttons = {"X": 0, "CIRCLE": 0, "TRIANGLE": 0, "SQUARE": 0 , "L1": 0, "R1": 0, "L2": 0, "R2": 0, "SHARE": 0, "OPTIONS": 0, "PLAY_STATION": 0, 
            "L3": 0, "R3": 0,"UP": 0, "DOWN": 0, "LEFT": 0, "RIGHT": 0 } # 1 is pressed, 0 is not pressed, -1 is just released

joystickAxesStatus = {"L-Right": 0, "L-Down": 0, "L2": 0, 
                        "R-Right": 0, "R-Down": 0, "R2": 0}


print(f'Before IK: {arm.cylTarget}')
status = arm.inverseKinematics()

arm.forwardKinematics()
print(f'After Forward: {arm.cylTarget}')
