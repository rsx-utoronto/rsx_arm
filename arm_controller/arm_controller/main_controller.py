#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import numpy as np

from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, String, UInt8, Float32MultiArray, UInt8MultiArray, Bool
from arm_msgs.msg import ArmInputs
from geometry_msgs.msg import Pose

from arm_utilities.arm_enum_utils import ControlMode, ArmState, HomingStatus
from arm_utilities.arm_control_utils import handle_joy_input, handle_keyboard_input, map_inputs_to_manual, map_inputs_to_ik

import copy
import functools
from pynput import keyboard


class Controller(Node):
    """
    (None)

    This class represents an instance of controller node and connects the node to 
    its publishing and subscribing topics
    """

    def __init__(self):
        super().__init__("Arm_Controller")

        # Attributes to hold data for publishing to topics
        # Attribute to publish state
        self.state = ArmState.IDLE

        # By default, control through controller
        self.control_mode = ControlMode.CONTROLLER

        # Attribute to store/publish killswitch value
        self.killswitch = 0

        # TODO load from config
        self.speed_limits = [0.1, 0.09, 0.15, 0.75, 0.12, 0.12, 20]

        self.current_pose = Pose()
        self.current_joints = [0.0]*6

        self.gripper_on = False

        # Publishers
        self.state_pub = self.create_publisher(String, "arm_state", 10)
        self.target_joint_pub = self.create_publisher(
            Float32MultiArray, "arm_target_joints", 10)
        self.target_pose_pub = self.create_publisher(
            Pose, "arm_target_pose", 10)
        self.killswitch_pub = self.create_publisher(
            UInt8, "arm_killswitch", 10)

        # Joynode subscriber
        self.joy_sub = self.create_subscription(
            Joy, "/joy", self.handle_joy, 10)

        # CAN feedback subscriber
        self.feedback_sub = self.create_subscription(
            Float32MultiArray, "arm_curr_pos", self.update_internal_joint_state, 10)

        # Safety subscribers
        self.joint_safety_sub = self.create_subscription(
            UInt8MultiArray, "joint_safety_state", self.process_safety, 10)

        # Nonblocking keyboard listener
        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.keyboard_listener.start()

        # Right switch subscriber
        self.right_switch_sub = [0] * 6
        for _joint_index in range(6):
            self.right_switch_sub[_joint_index] = self.create_subscription(
                Int16, f"right_switch_{str(_joint_index)}_state", functools.partial(self.switch_homing_stage, _joint_index), 10)

        


      
        # Homing state/params
        """joint index is numbered 0 to 5 in order of base rotation, shoulder, elbow, wrist_pitch, wrist_roll, gripper"""
        self.homed = [False] * 6
        self.homing = HomingStatus.IDLE

        self.has_reached_endpoint = [False] * 6
        self.relative_endpoint_pos = [0.0] * 6

        # Tunables for base homing motion
        self.rotation_step = [1.0] * 6
        self.rotation_span = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]

        # Threading controls
        self._homing_thread = None
        self._homing_stop = threading.Event()
        self._homing_lock = threading.Lock()
        self._shutdown = False # set true before shutdown to stop loops


    def update_arm(self, update):

        match self.control_mode:
            case ControlMode.CONTROLLER:
                inputs = handle_joy_input(update)
            case ControlMode.KEYBOARD:
                inputs = handle_keyboard_input(update)

            case ControlMode.GUI:
                # Update arm state based on GUI input
                pass

        if inputs.dpad_up:
            self.state = ArmState.MANUAL
            self.get_logger().info("Switching to MANUAL mode")
        elif inputs.dpad_down:
            self.state = ArmState.IDLE
            self.get_logger().info("Switching to IDLE mode")
        elif inputs.dpad_left:
            self.state = ArmState.IK
            if not self.homed:
                self.get_logger().info("Arm not homed, home arm before moving in IK mode")
            else:
                self.get_logger().info("Switching to IK mode, homed and ready to move")
        elif inputs.dpad_right:
            self.state = ArmState.PATH_PLANNING

        match self.state:
            case ArmState.IDLE:
                # In idle state, only allow killswitch and state changes
                # self.logger().info("Currently in IDLE: No control change")
                pass
            case ArmState.MANUAL:
                target_joints = map_inputs_to_manual(
                    inputs, self.speed_limits, self.current_joints)
                self.target_joint_pub.publish(target_joints)
            case ArmState.IK:
                if self.homed:
                    # Join homing thread if just finished homing
                    if self.homing_thread.is_alive():
                        self.homing_thread.join()

                    target_pose = map_inputs_to_ik(inputs, self.current_pose)
                    self.target_pose_pub.publish(target_pose)
                else:
                    if inputs.x and inputs.o:
                        self.get_logger().info("Homing arm, press X to cancel")
                        self.homing = True
                        self.homing_thread.start()
                    else:
                        self.get_logger().info(
                            "Arm not homed, cannot move in IK mode, press X and O simultaneously to home")
                        target_pose = self.current_pose
                        self.target_pose_pub.publish(target_pose)

    def handle_joy(self, msg):
        self.update_arm(msg)

    def on_press(self, key):
        self.update_arm(key)

    def on_release(self, key):
        # On key release, send zeroed inputs to stop movement
        zero_inputs = ArmInputs()
        self.update_arm(zero_inputs)

    def switch_homing_stage(self, joint_index, msg):
        if msg.data == 1:
            self.has_reached_endpoint[joint_index] = True
            self.relative_endpoint_pos[joint_index] = self.current_joints[joint_index]

  
    def home_arm(self, joint_index):
        #endpoint refers to positive-direction endpoint
        """joint index is numbered 0 to 5 in order of base rotation, shoulder, elbow, wrist_pitch, wrist_roll, gripper"""
    
        target_joints = copy.deepcopy(self.current_joints)
        
        if self.homed[joint_index] == False:
            if not self.has_reached_endpoint[joint_index]:
                target_joints[joint_index] += self.rotation_step[joint_index]
            else:
                dist = abs(self.relative_endpoint_pos[joint_index] - self.current_joints[joint_index]) 
                if dist >= self.rotation_span[joint_index]/2:
                   # self.current_joints[0] = 0
                    self.homed[joint_index] = True
                else:
                    target_joints[joint_index] -= self.rotation_step[joint_index]
     
            target_joints_msg = Float32MultiArray()
            target_joints_msg.data = target_joints
            self.target_joint_pub.publish(target_joints_msg)

            # not needed, only for convenience of test
            self.current_joints = target_joints
    
    def start_homing(self):
        """Start the homing loop in a background thread (idempotent)."""
        with self._homing_lock:
            if self._homing_thread and self._homing_thread.is_alive():
                return  # already running
            self.get_logger().info("Homing arm... press X to cancel.")
            self.homing = HomingStatus.ACTIVE
            self._homing_stop.clear()
            self._homing_thread = threading.Thread(target=self._homing_loop, daemon=True)
            self._homing_thread.start()

    def cancel_homing(self):
        """Request to cancel homing loop."""
        with self._homing_lock:
            if self.homing == HomingStatus.ACTIVE:
                self.get_logger().info("Homing canceled.")
            self.homing = HomingStatus.IDLE
            self._homing_stop.set()

    def _homing_loop(self, hz: float = 50.0):
        """Call home_arm at a fixed rate until COMPLETE, CANCELED, or shutdown."""
        period = 1.0 / hz
        joint_index = 0

        while not self._homing_stop.is_set() and not self._shutdown:
            # If homing completed elsewhere, bail
            if self.homing != HomingStatus.ACTIVE:
                break

            try:
                self.home_arm(joint_index)
            except Exception as e:
                self.get_logger().error(f"Homing loop error: {e}")
                # fail-safe: stop homing
                self.cancel_homing()
                break

            if self.homed[joint_index]:
                if(joint_index == 0): #change to 5
                    self.homing = HomingStatus.COMPLETE
                else:
                    joint_index += 1
            
             # Stop when complete
            if self.homing == HomingStatus.COMPLETE:
                self.get_logger().info("Homing complete.")
                self._homing_stop.set()
                break

            threading.Event().wait(period)

    def update_internal_joint_state(self, joints):
        self.current_joints = joints.data
        self.update_internal_pose_state(joints.data)
        # Function to update internal joint state from feedback
        pass

    def update_internal_pose_state(self):
        """
        Function to update internal pose state through forward kinematics.
        - Get pose through IK solver instance in MoveIt.
        - Might have to ping MoveIt node with joints
        and handle on C++ before publishing pose back.
        """
        pass

    def process_safety(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    arm_controller = Controller()

    rclpy.spin(arm_controller)

    arm_controller.destroy_node()
    rclpy.shutdown()
