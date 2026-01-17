from dataclasses import dataclass


@dataclass(frozen=True)
class ArmControllerConfig:
    # Node Name
    node_name: str

    # Speed Limits
    speed_limits: list[float]

    # Depth Queues
    publisher_depth_queue: int
    subscriber_depth_queue: int

    # Topic Names
    arm_state_topic: str
    target_joint_topic: str
    target_pose_topic: str
    killswitch_topic: str
    joy_topic: str
    arm_curr_pos_topic: str
    joint_safety_state_topic: str

    # Buttons
    homing_combo_buttons: list[str]

    # IK Homing Config
    allow_ik_without_homing: bool


@dataclass(frozen=True)
class ArmConfig:
    arm_controller_config: ArmControllerConfig
