from enum import Enum


class ArmState(Enum):
    IDLE = 0
    MANUAL = 1
    IK = 2
    PATH_PLANNING = 3
    SCIENCE = 4


class ControlMode(Enum):
    CONTROLLER = 0
    KEYBOARD = 1
    GUI = 2


class HomingStatus(Enum):
    IDLE = 0
    ACTIVE = 1
    COMPLETE = 2


class SafetyErrors(Enum):
    NONE = 0
    EXCEEDING_POS = 1
    EXCEEDING_CURRENT = 2
    EXCEEDING_POS_CURRENT = 3
    LIMIT_SWITCH = 4  # May switch out of an error as this is not only used as an error
