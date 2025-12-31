#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import numpy as np

from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, String, UInt8, Float32MultiArray, UInt8MultiArray, Bool
from arm_msgs.msg import ArmInputs, KeyboardCoords
from geometry_msgs.msg import Pose
from arm_utilities.arm_enum_utils import ControlMode, ArmState, HomingStatus, CANAPI
from arm_utilities.arm_control_utils import handle_joy_input, handle_keyboard_input, map_inputs_to_manual, map_inputs_to_ik
from arm_controller.can_connection import CAN_connection
from arm_controller.safety import SafetyChecker
import copy
import functools
from pynput import keyboard
import time
import math


class Controller(Node):
    """
    (None)

    Controller class for the robotic arm. Handles input from joystick/keyboard/GUI,
    manages arm state, safety checks, and communicates with the CAN bus to control the arm motors
    """

    def __init__(self, can_update_rate=1000, n_joints=7, virtual=False):
        super().__init__("Arm_Controller")

        self.n_joints = n_joints
        # with open("initial_positions.yaml", "r") as f:
        #     init_config = yaml.safe_load(f)
        # self.init_joints = [init_config["joint_1"], init_config["joint_2"], init_config["joint_3"], init_config["joint_4"], init_config["joint_5"], init_config["joint_6", 0]]
        self.initial_positions = [0] * n_joints
        self.initial_positions[2] = 90
        # TODO: LOAD THE YAML CORRECTLY
        # Initialize CAN connection
        if virtual:
            self.can_con = CAN_connection(
                channel="vcan0", interface="virtual", receive_own_messages=True, num_joints=n_joints)
        else:
            self.can_con = CAN_connection(num_joints=n_joints)

        # intialize safety checker
        self.safety_checker = SafetyChecker()
        # Attributes to hold data for publishing to topics
        # Attribute to publish state
        self.state = ArmState.IDLE

        # By default, control through controller
        self.control_mode = ControlMode.CONTROLLER

        # Attribute to store/publish killswitch value
        self.killswitch = 0

        # TODO load from config
        self.speed_limits = [1.5, 2, 1, 1, 0.3, 0.3, 5]

        self.current_pose = Pose()
        self.current_joints = [0.0] * self.n_joints
        # relative offsets from relative encoders determined during homing
        self.joint_offsets = [0.0] * self.n_joints

        self.target_joints = self.current_joints
        self.safe_target_joints = self.target_joints
        self.lim_switches = [0] * self.n_joints
        self.motor_curr = [0.0] * self.n_joints
        self.safety_flags = [0] * self.n_joints

        # track target internal joints to prevent slippage
        self.arm_internal_current_joints = [0.0] * self.n_joints

        self.gripper_on = False

        # Publishers
        self.state_pub = self.create_publisher(String, "arm_state", 10)

        # Publisher for safe target joints after safety check, used for debugging and data logging
        self.safe_target_joints_pub = self.create_publisher(
            Float32MultiArray, "safe_arm_target_joints", 10)

        # Used for publishing current joint angles, used in forward kinematics calculations
        self.arm_curr_joints_pub = self.create_publisher(
            Float32MultiArray, "arm_curr_angles", 10)
        
        # Publisher for target end effector pose in IK mode
        self.target_pose_pub = self.create_publisher(
            Pose, "arm_target_pose", 10)
        self.killswitch_pub = self.create_publisher(
            UInt8, "arm_killswitch", 10)

        # Joynode subscriber
        self.joy_sub = self.create_subscription(
            Joy, "/joy", self.handle_joy, 10)
        
        # FK pose subscriber, updates from calculations in path planner node
        self.fk_sub = self.create_subscription(
            Pose, "arm_fk_pose", self.update_fk_pose_callback, 10)
        self.ik_target_sub = self.create_subscription(Float32MultiArray, "arm_ik_target_joints", self.update_ik_target, 10)

        self.keyboard_coord_sub = self.create_subscription(KeyboardCoords, "keyboard_corners", self.handle_keyboard_coords, 10)  
        self.path_planner_target_pub = self.create_publisher(Float32MultiArray, "arm_path_planner_target_joints", 10)
        self.path_planner_joint_sub = self.create_subscription(Float32MultiArray, "arm_path_joints", self.update_path_planner_joints, 10)
        self.path_executor_thread = threading.Thread(
            target=self.path_executor_loop, daemon=True)
        # will fill when a path is given
        self.current_path = []
        self.joint_target_threshold = 1 # maximum allowed error in degrees to consider joint target reached

        # Nonblocking keyboard listener
        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.keyboard_listener.start()

        # Joint limit tracking
        self.at_limit = [False] * self.n_joints

        # Homing state/params
        """joint index is numbered 0 to 5 in order of base rotation, shoulder, elbow, wrist_pitch, wrist_roll, gripper"""
        self.homed = [False] * self.n_joints
        self.homing = HomingStatus.IDLE
        self.homing_pid = {"P": 0.2, "I": 0.1, "D": 0}
        self.homing_threshold = 1e-3

        self.has_reached_endpoint = [False] * self.n_joints
        self.relative_endpoint_pos = [0.0] * self.n_joints

        # Tunables for base homing motion
        # TODO: need real values for these
        self.rotation_step = [0.2] * self.n_joints

        # Threading controls
        self.homing_thread = threading.Thread(
            target=self.home_arm, daemon=True)
        self.homing_stop = threading.Event()
        self.homing_lock = threading.Lock()
        self.shutdown = False  # set true before shutdown to stop loops

        self.can_rate = 2500
        # Update thread for reading CAN messages
        self.can_read_thread = threading.Thread(
            target=self.read_can_callback, daemon=True)
        self.can_read_thread.start()

        self.arm_update_lock = threading.Lock()

        self.threads = [self.homing_thread, self.can_read_thread, self.path_executor_thread]
        
        # Initialization flags for CAN readings (for setting initial joint values)
        self.init = [False]*7

        # Keyboard targets to be hard coded at time of challenge
        # TODO: parse from config file
        self.keyboard_targets = []
        # TODO: set up all keys
        self.keyboard_positions = {}
        self.last_quadrant = [0, 0] # for tracking which side of the joint base and wrist are in
        try:
            with open("limit_log.txt") as f:
                data = f.read().split('\n')
                for line in data:
                    if line.startswith("Joint 0"):
                        self.last_quadrant[0] = int(line[-2:])
                    else:
                        self.last_quadrant[1] = int(line[-2:])
        except:
            self.get_logger().warn("error logging limits!")


    def read_can_callback(self):
        while self.shutdown == False:
            read_msg = self.can_con.read_message()
            if read_msg == None:
                continue
            index = read_msg[0]
            api = read_msg[1]
            value = read_msg[2]
            # Limit switch
            if api == CANAPI.CMD_API_STAT0.value:
                if read_msg[2] == 1:
                    self.get_logger().warn("Joint %d hit limit!" % n)
                    self.at_limit[index] = True
                    if index == 0 or index == 6:
                        # TODO: this should be cleaner, log if joint is on positive or negative side of limit
                        with open("limit_log.txt", "w") as f:
                            f.write("Joint %d hit limit going: %f\n" % (index, math.copysign(self.current_joints[index])))
                            if index == 0:
                                f.write("Joint 6 hit limit going %d\n" % self.current_joints[1])
                            else:
                                f.write("Joint 0 hit limit going %d\n" % self.current_joints[0])
                    else:
                        pass
            
            # Motor current value
            elif api == CANAPI.CMD_API_STAT1.value:
                self.motor_curr[index] = value
            
            # Joint angle value
            elif api == CANAPI.CMD_API_STAT2.value:
                # Check if we updated wrist motors and apply the conversions
                if index == 4:
                    wrist1_angle = value
                    wrist2_angle = self.current_joints[5]
                    self.current_joints[4] = float(
                        (wrist1_angle + wrist2_angle) / 2)
                    self.current_joints[5] = float(
                        (wrist1_angle - wrist2_angle) / 2)
                elif index == 5:
                    wrist1_angle = self.current_joints[4]
                    wrist2_angle = value
                    self.current_joints[4] = float(
                        (wrist1_angle + wrist2_angle) / 2)
                    self.current_joints[5] = float(
                        (wrist1_angle - wrist2_angle) / 2)

                else:
                    self.current_joints[index] = value + self.joint_offsets[index]
                
                # Check if angle has ben clibrated to encoder measured initial angles
                if self.init[index] == False:
                    self.arm_internal_current_joints[index] = self.current_joints[index]
                    self.init[index] = True
                
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

            if inputs.dpad_up and inputs.dpad_down and inputs.dpad_left and inputs.dpad_right:
                if self.state != ArmState.AUTO_KEYBOARD:
                    self.state = ArmState.AUTO_KEYBOARD
                    self.get_logger().info("ACTIVATING AUTO KEYBOARD MODE")
                else:
                    self.state = ArmState.IDLE
                    self.get_logger().info("DEACTIVATING AUTO KEYBOARD MODE")
                time.sleep(2)

            if inputs.dpad_up:
                self.state = ArmState.MANUAL
                self.get_logger().info("Switching to MANUAL mode")

            elif inputs.dpad_down:
                self.state = ArmState.IDLE
                self.get_logger().info("Switching to IDLE mode")

            elif inputs.dpad_left:
                self.state = ArmState.IK
                if False in self.homed:
                    self.get_logger().info("Arm not homed, home arm before moving in IK mode")
                else:
                    self.get_logger().info("Switching to IK mode, homed and ready to move")

            elif inputs.dpad_right:
                self.state = ArmState.PATH_PLANNING
                self.get_logger().info("Switching to PATH PLANNING mode")

            state_string = String()
            state_string.data = self.state.name
            self.state_pub.publish(state_string)

            match self.state:
                case ArmState.IDLE:
                    self.safe_target_joints = self.current_joints
                    self.can_con.send_target_message(self.safe_target_joints)
                    # In idle state, only allow killswitch and state changes
                    # self.logger().info("Currently in IDLE: No control change")
                    pass
                case ArmState.MANUAL:
                    # use internal current joints to prevent joint slippage when no input is given
                    self.target_joints = map_inputs_to_manual(
                        inputs, self.speed_limits, self.arm_internal_current_joints)
                    self.safe_target_joints, self.safety_flags = self.safety_checker.update_safe_goal_pos(
                        self.target_joints, self.arm_internal_current_joints)

                    # update internal state to reflect target, when input is zero, internal and real current joints should be equivalent given the assumption
                    # that the target is reachable
                    self.arm_internal_current_joints = self.safe_target_joints

                    msg = Float32MultiArray()
                    msg.data = self.safe_target_joints
                    self.safe_target_joints_pub.publish(msg)

                    self.can_con.send_target_message(self.safe_target_joints)
                case ArmState.IK:
                    if not False in self.homed:
                        # Join homing thread if just finished homing
                        if self.homing_thread.is_alive():
                            self.homing_thread.join()

                        target_pose = map_inputs_to_ik(
                            inputs, self.current_pose)
                        self.target_pose_pub.publish(target_pose)
                    else:
                        if inputs.x and inputs.circle and inputs.triangle and inputs.square:
                            self.get_logger().info("OVERRIDING HOMING")
                            for i in range(self.n_joints - 1):
                                self.joint_offsets[i] = self.initial_positions[i] - self.current_joints[i]
                            self.homed = [True]*self.n_joints
                        elif inputs.x and inputs.circle:
                            self.get_logger().info("Homing arm, press X to cancel")
                            self.homing = True
                            self.homing_thread.start()
                        else:
                            self.get_logger().info(
                                "Arm not homed, cannot move in IK mode, press X and O simultaneously to home")
                            target_pose = self.current_pose
                            self.target_pose_pub.publish(target_pose)
                            time.sleep(0.2)
                case ArmState.PATH_PLANNING:
                    if self.path_executor_thread.is_alive:
                        if not self.executing_path:
                            self.path_executor_thread.join()
                        else:
                            pass
                    else:
                        self.path_executor_thread.start()
                        self.executing_path = True
                case ArmState.AUTO_KEYBOARD:
                
                    for target in self.keyboard_targets:
                        if self.executing_path:
                            pass
                        else:
                            if self.path_executor_thread.is_alive():
                                self.path_executor_thread.join()
                                
                            target_pose = self.keyboard_positions[target]
                            self.get_logger().info("Moving to key: %s" % target)
                            # TODO: path plan should publish to an intermediate pose offset from the keyboard prior to publishing the actual key press
                            self.target_pose_pub.publish(target_pose)
                            self.executing_path = True
                            self.path_executor_thread.start()
                    pass
        self.update_internal_pose_state(self.current_joints)
        time.sleep(0.05)

    def handle_joy(self, msg):
        self.update_arm(msg)

    def on_press(self, key):
        self.update_arm(key)

    def on_release(self, key):
        # On key release, send zeroed inputs to stop movement
        zero_inputs = ArmInputs()
        self.update_arm(zero_inputs)

    def home_arm(self, joint_indices=[0, 1, 2, 3, 4, 5, 6],  hz: float = 50.0):
        # endpoint refers to positive-direction endpoint
        """joint index is numbered 0 to 6 in order of base rotation, shoulder, elbow, elbow roll, wrist_pitch, wrist_roll, gripper"""
        period = 1.0 / hz
        target_joints = copy.deepcopy(self.current_joints)
        joint_offsets = [0.0] * self.n_joints
        base_joint_status = 0
        wrist_roll_status = 0
        while self.homing == HomingStatus.ACTIVE and self.shutdown == False:
            for joint_index in joint_indices():
                # make sure current_joints isn't modified by CAN thread while updating step
                with self.homing_lock():
                    if self.homed[joint_index] == False:

                        # Logic for handling when a limit is hit on joint index
                        if self.at_limit:

                            # relative endpoint position measures the distance between the initial joint position and the relative position of the limit switch
                            self.relative_endpoint_pos[joint_index] = self.current_joints[joint_index]

                            #TODO: make this cleaner
                            if joint_index == 0 or joint_index == 6:
                                if joint_index == 0:
                                    joint_offsets[joint_index] = self.relative_endpoint_pos[joint_index] - (
                                    self.safety_checker.joint_limits[joint_index][1]-self.safety_checker.joint_limits[joint_index][0])/2 + \
                                    self.initial_positions[joint_index]
                                    if self.last_quadrant[0] == 1:
                                        joint_offsets[joint_index] += 360

                                elif joint_index == 6:
                                    joint_offsets[joint_index] = self.relative_endpoint_pos[joint_index] - (
                                    self.safety_checker.joint_limits[joint_index][1]-self.safety_checker.joint_limits[joint_index][0])/2 + \
                                    self.initial_positions[joint_index]
                                    if self.last_quadrant[0] == 1:
                                        joint_offsets[joint_index] += 360
                            else:
                                joint_offsets[joint_index] = self.relative_endpoint_pos[joint_index] - (
                                    self.safety_checker.joint_limits[joint_index][1]-self.safety_checker.joint_limits[joint_index][0])/2 + \
                                    self.initial_positions[joint_index]
                            self.has_reached_endpoint[joint_index] = True
                        
                        # if a limit has not been hit and the we have not reached the limit prior to this, go towards the endpoint (positive direction)
                        elif not self.has_reached_endpoint[joint_index]:
                            # move towards forward limit switch
                            target_joints[joint_index] += self.rotation_step[joint_index]

                        # if a limit has not been hit, but we previously reached the limit, and we have not reached the absolute zero,
                        # move back towards the absolute zero position
                        elif not self.homed[joint_index]:
                            # calculate error between target zero position and current absolute position
                            error = (self.relative_endpoint_pos[joint_index] - self.current_joints[joint_index]) - (
                                self.safety_checker.joint_limits[joint_index][1]-self.safety_checker.joint_limits[joint_index][0])/2
                            if abs(error) < self.homing_threshold:
                                self.homed[joint_index] = True
                            else:
                                # TODO: factor of 1/30 is arbitrary, can be changed later
                                target_joints[joint_index] = min(error*self.rotation_step/30, math.copysign(error)*self.rotation_step[joint_index])

                        # if we are not actively homing for this joint anymore, maintain the current joint position
                        else:
                            target_joints[joint_index] = self.internal_current_joints[joint_index]
            # double check that we are not finished yet
            if all(homed == True for homed in self.homed):
                self.homing = HomingStatus.COMPLETE
            
            # once a step has been calculated for each joint, publish the targets and then wait
            self.safe_target_joints, self.safety_flags = self.safety_checker.update_safe_goal_pos(
                target_joints, self.current_joints)
            msg = Float32MultiArray()
            msg.data = self.safe_target_joints
            self.safe_target_joints_pub.publish(msg)
            
            # update internal state
            self.internal_current_joints = self.safe_target_joints

            threading.Event().wait(period)
        self.joint_offsets = joint_offsets

    def start_homing(self):

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

    def update_internal_pose_state(self, joints):
        angles = Float32MultiArray(data=joints[0:6])
        self.arm_curr_joints_pub.publish(angles)
        # Pose will be returned in a subscription, should update elsewhere

    def update_fk_pose_callback(self, msg):
        self.current_pose = msg

    def shutdown_node(self):
        self.get_logger().info("Shutting down node")
        self.shutdown = True
        for thread in self.threads:
            if thread.is_alive():
                thread.join()

    def update_ik_target(self, msg):
        self.target_joints = list(msg.data)
        # append the end effector current rotation because IK solution does not have this
        self.target_joints.append(self.current_joints[-1])
        self.safe_target_joints, self.safety_flags = self.safety_checker.update_safe_goal_pos(
                        self.target_joints, self.current_joints)
        msg = Float32MultiArray()
        msg.data = self.safe_target_joints
        self.safe_target_joints_pub.publish(msg)
        self.arm_internal_current_joints = self.safe_target_joints
        # TODO: temporarily disabled, safety issues
        # self.can_con.send_target_message(self.safe_target_joints)

    def handle_keyboard_coords(self, msg):
        self.get_logger().info("Received keyboard coordinates from camera node")
        corners = msg.corners  # assuming corners is a list of 4 (x, y) tuples
        # Process corners to determine key positions
        self.interpolate_key_positions(corners)

    def interpolate_key_positions(self, corners):
        # TODO: implement interpolation of key positions based on keyboard corners
        pass

    def update_path_planner_joints(self, msg):
        self.current_path.append(list(msg.data))

    def path_executor_loop(self):
        while len(self.current_path) > 0 and self.shutdown == False:
            for step in self.current_path:
                error = [abs(step[i] - self.current_joints[i]) for i in range(self.n_joints)]
                if all(e < self.joint_target_threshold for e in error):
                    self.current_path.pop(0)
                    continue
                    
                self.target_joints[0:6] = step
                self.safe_target_joints, self.safety_flags = self.safety_checker.update_safe_goal_pos(
                    self.target_joints, self.current_joints)
                msg = Float32MultiArray()
                msg.data = self.safe_target_joints
                self.safe_target_joints_pub.publish(msg)

                self.internal_current_joints = self.safe_target_joints

                # DISABLED TEMPORARILY FOR SAFETY
                # self.can_con.send_target_message(self.safe_target_joints)
                time.sleep(0.1)  # wait for some time before next step
            self.executing_path = False
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

    arm_controller = Controller(virtual=True)

    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        arm_controller.get_logger().info("Keyboard interrupt received")
    finally:
        arm_controller.shutdown_node()
        arm_controller.destroy_node()
        rclpy.shutdown()
