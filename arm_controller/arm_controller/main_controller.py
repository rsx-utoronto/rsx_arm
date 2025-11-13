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
from arm_controller.can_connection import CAN_connection
from arm_controller.safety import SafetyChecker
import copy
import functools
from pynput import keyboard
import time

class Controller(Node):
    """
    (None)

    This class represents an instance of controller node and connects the node to 
    its publishing and subscribing topics
    """

    def __init__(self, can_update_rate = 1000, n_joints = 7, virtual = False):
        super().__init__("Arm_Controller")

        self.n_joints = n_joints
        
        if virtual:
            self.can_con = CAN_connection(channel = "vcan0", interface = "virtual", receive_own_messages = True, num_joints = n_joints)
        else:
            self.can_con = CAN_connection(num_joints = n_joints)

        self.safety_checker = SafetyChecker()
        # Attributes to hold data for publishing to topics
        # Attribute to publish state
        self.state = ArmState.IDLE

        # By default, control through controller
        self.control_mode = ControlMode.CONTROLLER

        # Attribute to store/publish killswitch value
        self.killswitch = 0

        # TODO load from config
        self.speed_limits = [0.1, 0.09, 0.15, 0.75, 0.12, 0.12, 5]

        self.current_pose = Pose()
        self.current_joints = [0.0] * self.n_joints
        # relative offsets from relative encoders determined during homing
        self.joint_offsets = [0.0] * self.n_joints
        self.target_joints = self.current_joints
        self.safe_target_joints = self.target_joints
        self.lim_switches = [0] * self.n_joints
        self.motor_curr = [0.0] * self.n_joints
        self.safety_flags = [0] * self.n_joints

        self.gripper_on = False

        # Publishers
        self.state_pub = self.create_publisher(String, "arm_state", 10)
        self.safe_target_joints_pub = self.create_publisher(
            Float32MultiArray, "safe_arm_target_joints", 10)
        self.target_pose_pub = self.create_publisher(
            Pose, "arm_target_pose", 10)
        self.killswitch_pub = self.create_publisher(
            UInt8, "arm_killswitch", 10)

        # Joynode subscriber
        self.joy_sub = self.create_subscription(Joy, "/joy", self.handle_joy, 10)

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

        self.at_limit = [False] * 6
      
        # Homing state/params
        """joint index is numbered 0 to 5 in order of base rotation, shoulder, elbow, wrist_pitch, wrist_roll, gripper"""
        self.homed = [False] * 6
        self.homing = HomingStatus.IDLE
        self.homing_pid = {"P": 0.2, "I": 0.1, "D": 0}
        self.homing_threshold = 1e-3

        self.has_reached_endpoint = [False] * 6
        self.relative_endpoint_pos = [0.0] * 6

        # Tunables for base homing motion
        self.rotation_step = [1.0] * 6
        self.rotation_span = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]

        # Threading controls
        self.homing_thread = threading.Thread(target=self.home_arm, daemon=True)
        self.homing_stop = threading.Event()
        self.homing_lock = threading.Lock()
        self.shutdown = False # set true before shutdown to stop loops

        self.can_rate = 1000
        self.can_send_thread = threading.Thread(target = self.send_can_callback, daemon=True)
        self.can_read_thread = threading.Thread(target = self.read_can_callback, daemon=True)
        self.can_send_thread.start()
        self.can_read_thread.start()

        self.arm_update_lock = threading.Lock()

        self.threads = [self.homing_thread, self.can_send_thread, self.can_read_thread]

    def read_can_callback(self):
        while self.shutdown == False:
            self.current_joints, self.lim_switches, self.motor_curr = self.can_con.read_message()
            for n, lim_switch in enumerate(self.lim_switches):
                if lim_switch == 1:
                    self.get_logger().warn("Joint %d hit limit!" % n)
                    self.at_limit[n] = True
                else:
                    self.at_limit[n] = False

            # add joint offsets for absolute joint values
            self.current_joints = self.current_joints + self.joint_offsets
            threading.Event().wait(1/self.can_rate)
    def send_can_callback(self):
        while self.shutdown == False:
            self.can_con.send_message(self.safe_target_joints)
            threading.Event().wait(1/self.can_rate)

    def update_arm(self, update):
        # lock to prevent local variables from being modified by CAN threads during execution
        with self.arm_update_lock:
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
            state_string = String()
            state_string.data = self.state.name
            self.state_pub.publish(state_string)

            match self.state:
                case ArmState.IDLE:
                    # In idle state, only allow killswitch and state changes
                    # self.logger().info("Currently in IDLE: No control change")
                    pass
                case ArmState.MANUAL:
                    target_joints = map_inputs_to_manual(inputs, self.speed_limits, self.target_joints)
                    self.target_joints = target_joints.data
                    self.safe_target_joints, self.safety_flags = self.safety_checker.update_safe_goal_pos(self.target_joints, self.current_joints)
                    msg = Float32MultiArray()
                    msg.data = self.safe_target_joints
                    self.safe_target_joints_pub.publish(msg)
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
        time.sleep(0.01)
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
    

    def home_arm(self, joint_indices = [0, 1, 2, 3, 4, 5, 6],  hz: float = 50.0):
        #endpoint refers to positive-direction endpoint
        """joint index is numbered 0 to 6 in order of base rotation, shoulder, elbow, elbow roll, wrist_pitch, wrist_roll, gripper"""
        period = 1.0 / hz
        target_joints = copy.deepcopy(self.current_joints)
        joint_offsets = [0.0] * self.n_joints
        while self.homing == HomingStatus.ACTIVE and self.shutdown == False:
            for joint in joint_indices():
                # make sure current_joints isn't modified by CAN thread while updating step
                with self.homing_lock():
                    if self.homed[joint] == False:
                        if self.at_limit:
                            self.has_reached_endpoint = True

                            # relative endpoint posiiton measures the distance between the initial joint position and the relative position of the limit switch
                            self.relative_endpoint_pos[joint_index] = self.current_joints[joint_index]

                            # set temporary joint offset (relative endpoint position is where the arm thinks it is and half the rotation span is where it actually is)
                            # wait until homing for this joint is done to assign the actual joint offset
                            joint_offsets[joint_index] = self.relative_endpoint_pos[joint_index] - self.rotation_span[joint_index]/2

                        if not self.has_reached_endpoint[joint]:
                            # move towards forward limit switch
                            target_joints[joint] += self.rotation_step[joint]

                        else:
                            # calculate error between target zero position and current absolute position
                            error = (self.relative_endpoint_pos[joint_index] - self.current_joints[joint_index]) - self.rotation_span[joint_index]/2
                            if abs(error) < self.homing_threshold:
                                self.homed[joint_index] = True
                            else:
                                target_joints[joint_index] -= self.rotation_step[joint_index]

                    self.safe_target_joints, self.safety_flags = self.safety_checker.update_safe_goal_pos(target_joints, self.current_joints)
                    msg = Float32MultiArray()
                    msg.data = self.safe_target_joints
                    self.safe_target_joints_pub.publish(msg)

                if self.homed[index] == True for index in joint_indices():
                    self.homing = HomingStatus.COMPLETE

                threading.Event().wait(period)
    
    def start_homing(self):
        """Start the homing loop in a background thread (idempotent)."""
        with self.homing_lock:
            if self.homing_thread and self.homing_thread.is_alive():
                return  # already running
            self.get_logger().info("Homing arm... press X to cancel.")
            self.homing = HomingStatus.ACTIVE
            self.homing_stop.clear()
            self.homing_thread.start()

    def cancel_homing(self):
        """Request to cancel homing loop."""
        with self.homing_lock:
            if self.homing == HomingStatus.ACTIVE:
                self.get_logger().info("Homing canceled.")
            self.homing = HomingStatus.IDLE
            self.homing_stop.set()

    def update_internal_joint_state(self, joints):
        self.current_joints = joints.data
        self.update_internal_pose_state(joints.data)
        # Function to update internal joint state from feedback
        pass

    def update_internal_pose_state(self, joints):
        """
        Function to update internal pose state through forward kinematics.
        - Get pose through IK solver instance in MoveIt.
        - Might have to ping MoveIt node with joints
        and handle on C++ before publishing pose back.
        """
        pass

    def shutdown_node(self):
        self.get_logger().info("Shutting down node")
        self.shutdown = True
        for thread in self.threads:
            thread.join()
        

  
def real_controller(args=None):
    rclpy.init(args=args)

    arm_controller = Controller()

    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        arm_controller.get_logger().info("Keyboard interrupt received")
    finally:
        arm_controller.shutdown_node()
        arm_controller.destroy_node()
        rclpy.shutdown()


def virtual_controller(args=None):
    rclpy.init(args=args)

    arm_controller = Controller(virtual = True)

    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        arm_controller.get_logger().info("Keyboard interrupt received")
    finally:
        arm_controller.shutdown_node()
        arm_controller.destroy_node()
        rclpy.shutdown()