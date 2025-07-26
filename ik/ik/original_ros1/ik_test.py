from arm_science_ik import SciArm
from math import pi, atan2
import numpy as np

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

arm = SciArm(5, dhTable, offsets, angleOrientation)
# arm.setTarget([0 , 629, -12.3, 0.941354, 0])
# print(f'IK Target {arm.cylTarget}')
# status = arm.inverseKinematics()
# print(arm.getGoalAngles())

arm.setCurAngles([0, (pi/2)-atan2(112.99, 348.08), 0, 0, 0])
arm.passiveForwardKinematics()

print(f'After forward Kin {arm.cylTarget}')
print()

arm.storeSparkMaxOffsets([0, pi/2, 0, 0, 0])
# print(arm.addSparkMaxOffsets([0, 0, 0, 0, 0]))
print(arm.getGoalAngles())
