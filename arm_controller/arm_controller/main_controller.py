#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import numpy as np

from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, String, UInt8, Float32MultiArray, UInt8MultiArray, Bool
from arm_msgs.msg import ArmInputs, KeyboardCoords, TargetPosition, ArmStatuses
from geometry_msgs.msg import Pose, Point, Quaternion
from arm_utilities.arm_enum_utils import ControlMode, ArmState, HomingStatus, CANAPI
from arm_utilities.arm_control_utils import handle_joy_input, map_inputs_to_manual, map_inputs_to_ik
from arm_configs.loader import load_arm_controller_config_from_node, load_keyboard_config_from_node
from arm_controller.can_connection import CAN_connection
from arm_controller.safety import SafetyChecker
import copy
import functools
from pynput import keyboard
import time
import math
import time
import math


class Controller(Node):
    """
    (None)

    This class represents an instance of controller node and connects the node to 
    its publishing and subscribing topics
    """

    def __init__(self, can_update_rate=1000, n_joints=7, virtual=False):
        super().__init__("Arm_Controller")
        self.cfg = load_arm_controller_config_from_node(self)

        self.n_joints = n_joints
        self.initial_positions = [0., 0., 0., 0., 0., 0., 0.]

        # Initialize CAN connection
        if virtual:
            self.can_con = CAN_connection(
                channel="vcan0", interface="virtual", receive_own_messages=True, num_joints=n_joints)
        else:
            self.can_con = CAN_connection(num_joints=n_joints)

        self.virtual = virtual
        # intialize safety checker
        self.safety_checker = SafetyChecker()
        # Attributes to hold data for publishing to topics
        # Attribute to publish state
        self.state = ArmState.IDLE

        # By default, control through controller
        self.control_mode = ControlMode.CONTROLLER

        # Attribute to store/publish killswitch value
        self.killswitch = 0

        self.speed_limits = self.cfg["speed_limits"]

        # TODO: having some issues with updating self.current_pose in the callback, need to investigate
        self.current_pose = Pose()
        self.current_joints = self.initial_positions.copy()
        # relative offsets from relative encoders determined during homing
        self.joint_offsets = [0.0] * self.n_joints

        self.target_joints = self.current_joints
        self.safe_target_joints = self.target_joints
        self.lim_switches = [0] * self.n_joints
        self.motor_curr = [0.0] * self.n_joints
        self.safety_flags = [0] * self.n_joints

        # track target internal joints to prevent slippage
        self.arm_internal_current_joints = self.initial_positions.copy()

        self.gripper_on = False

        # Publishers
        # Publisher for safe target joints after safety check, used for debugging and data logging
        self.safe_target_joints_pub = self.create_publisher(
            Float32MultiArray, "safe_arm_target_joints", 10)

        # Used for publishing current joint angles, used in forward kinematics calculations
        self.arm_curr_joints_pub = self.create_publisher(
            Float32MultiArray, "arm_curr_angles", 10)
        self.state_pub = self.create_publisher(
            String, self.cfg["arm_state_topic"], self.cfg["publisher_depth_queue"])
        self.target_joint_pub = self.create_publisher(
            Float32MultiArray, self.cfg["target_joint_topic"], self.cfg["publisher_depth_queue"])
        self.target_pose_pub = self.create_publisher(
            Pose, self.cfg["target_pose_topic"], self.cfg["publisher_depth_queue"])
        # self.killswitch_pub = self.create_publisher(
        #     UInt8, self.cfg["killswitch_topic"], self.cfg["publisher_depth_queue"])
        self.limit_switch_pub = self.create_publisher(
            ArmStatuses, "arm_lim_switches", self.cfg["publisher_depth_queue"])
        self.safety_status_pub = self.create_publisher(
            ArmStatuses, "arm_safety_status", self.cfg["publisher_depth_queue"])
        self.homing_status_pub = self.create_publisher(
            String, "arm_homing_status", self.cfg["publisher_depth_queue"])
        self.path_planning_status_pub = self.create_publisher(
            String, "arm_path_planning_status", self.cfg["publisher_depth_queue"])
        self.keyboard_target_pub = self.create_publisher(
            TargetPosition, "key_targets", self.cfg["publisher_depth_queue"])

        # Joynode subscriber
        self.joy_sub = self.create_subscription(
            Joy, self.cfg["joy_topic"], self.handle_joy, self.cfg["subscriber_depth_queue"])
        
        # FK pose subscriber, updates from calculations in path planner node
        self.fk_sub = self.create_subscription(
            Pose, "arm_fk_pose", self.update_fk_pose_callback, 10)
        self.ik_target_sub = self.create_subscription(Float32MultiArray, "arm_ik_target_joints", self.update_ik_target,
            self.cfg["subscriber_depth_queue"])

        # Safety subscribers
        self.joint_safety_sub = self.create_subscription(
            UInt8MultiArray, self.cfg["joint_safety_state_topic"], self.process_safety,
            self.cfg["subscriber_depth_queue"])

        self.homing_thread = threading.Thread(target=self.home_arm)
        self.keyboard_coord_sub = self.create_subscription(KeyboardCoords, "keyboard_corners", self.handle_keyboard_coords, 10)  
        # self.path_planner_target_pub = self.create_publisher(Float32MultiArray, "arm_path_planner_target_joints", 10)
        self.path_planner_joint_sub = self.create_subscription(Float32MultiArray, "arm_path_joints", self.update_path_planner_joints, 10)
        self.path_executor_thread = threading.Thread(
            target=self.path_executor_loop, daemon=True)
        # will fill when a path is given
        self.current_path = []
        self.joint_target_threshold = 1 # maximum allowed error in degrees to consider joint target reached

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
        self.executing_path = False
        
        # Initialization flags for CAN readings (for setting initial joint values)
        self.init = [False]*7

        # Keyboard targets to be hard coded at time of challenge
        self.keyboard_targets = []

        self.keyboard = load_keyboard_config_from_node(self)
        self.key_positions = self.keyboard["keyboard"]["keys"]
        self.keys = self.key_positions.keys()
        for key in self.keyboard_targets:
            if key not in self.keys:
                self.logger.warn(f"Key {key} not found in config")
        self.relative_key_positions = self.key_positions.copy()
        self.current_key = None

        # TODO: potentially remove
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
            self.get_logger().warn ("error logging limits!")


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
                    self.get_logger().warn("Joint %d hit limit!" % index)
                    self.at_limit[index] = True
                    self.publish_limit_switch_status()
                    if index == 0 or index == 6:
                        # TODO: this should be cleaner, log if joint is on positive or negative side of limit
                        with open("limit_log.txt", "w") as f:
                            f.write("Joint %d hit limit going: %f\n" % (index, math.copysign(1, self.current_joints[index])))
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
                
                # Check if angle has ben calibrated to encoder measured initial angles
                if self.init[index] == False:
                    self.arm_internal_current_joints[index] = self.current_joints[index]
                    self.init[index] = True
                
            threading.Event().wait(1/self.can_rate)
    def process_safety(self, msg):
        pass

    def update_arm(self, update):
        # TODO: need to update state tracking to consistently unify real world angles with internal state
        # lock to prevent local variables from being modified by CAN threads during execution
        with self.arm_update_lock:
            match self.control_mode:
                case ControlMode.CONTROLLER:
                    inputs = handle_joy_input(update)
                case ControlMode.GUI:
                    # Update arm state based on GUI input
                    pass
            if inputs.share:
                self.safe_target_joints = self.current_joints
                self.get_logger().error("KILLSWITCH PRESSED, LOCKING ARM AND EXITING")
                for i in range(10):
                    self.safe_target_joints_pub(self.safe_target_joints)
                    time.sleep(0.05)
                    
                self.shutdown_node()
                sys.exit()

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
                self.state = ArmState.AUTO_KEYBOARD
                self.get_logger().info("Switching to PATH PLANNING mode")

            state_string = String()
            state_string.data = self.state.name
            self.state_pub.publish(state_string)

            match self.state:
                # TODO: add killswitch
                case ArmState.IDLE:
                    self.safe_target_joints = self.arm_internal_current_joints
                    self.can_con.send_target_message(self.safe_target_joints)
                    # In idle state, only allow killswitch and state changes
                    self.logger().info("Currently in IDLE: No control change")

                case ArmState.MANUAL:
                    # use internal current joints to prevent joint slippage when no input is given
                    self.target_joints = map_inputs_to_manual(
                        inputs, self.speed_limits, self.arm_internal_current_joints)
                    self.safe_target_joints, self.safety_flags = self.safety_checker.update_safe_goal_pos(
                        self.target_joints, self.arm_internal_current_joints)
                    self.publish_safety_status()

                    # update internal state to reflect target, when input is zero, internal and real current joints should be equivalent given the assumption
                    # that the target is reachable
                    self.arm_internal_current_joints = self.safe_target_joints
                    for i in range(self.n_joints):
                        if type(self.safe_target_joints[i]) == np.float32:
                            self.safe_target_joints[i] = self.safe_target_joints[i].item()
                    msg = Float32MultiArray()
                    msg.data = self.safe_target_joints
                    self.safe_target_joints_pub.publish(msg)

                    self.can_con.send_target_message(self.safe_target_joints)
                case ArmState.IK:
                    if not False in self.homed or self.cfg["allow_ik_without_homing"]:
                        # Join homing thread if just finished homing
                        if self.homing_thread.is_alive():
                            self.homing_thread.join()

                        target_pose = map_inputs_to_ik(
                            inputs, self.current_pose)
                        self.target_pose_pub.publish(target_pose)
                        self.current_pose = target_pose
                    else:
                        if inputs.x and inputs.circle and inputs.triangle and inputs.square or self.virtual:
                            self.get_logger().info("OVERRIDING HOMING")
                            # TODO: include offsets in override
                            # for i in range(self.n_joints - 1):
                            #     self.joint_offsets[i] = self.initial_positions[i] - self.current_joints[i]
                            self.homed = [True]*self.n_joints
                        elif inputs.x and inputs.circle:
                            self.get_logger().info("Homing arm, press X to cancel")
                            self.homing = HomingStatus.ACTIVE
                            self.publish_homing_status()
                            self.homing_thread.start()
                        else:
                            self.get_logger().info(
                                "Arm not homed, cannot move in IK mode, press X and O simultaneously to home")
                            target_pose = self.current_pose
                            # self.target_pose_pub.publish(target_pose)
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
                        self.publish_path_planning_status()
                case ArmState.AUTO_KEYBOARD:
                    # TODO: actively update the relative location of keyboard targets based on detected keyboard corners
                    for target in self.keyboard_targets:
                        if self.executing_path:
                            if inputs.circle:
                                self.executing_path = False
                                self.publish_path_planning_status()
                                if self.path_executor_thread.is_alive():
                                    self.path_executor_thread.join()
                            else:
                                pass
                        else:
                            
                            if self.path_executor_thread.is_alive():
                                self.path_executor_thread.join()
                            self.get_logger().info("Moving to key: %s" % target)
                            self.current_key = target
                            
                            # TODO: path plan should publish to an intermediate pose offset from the keyboard prior to publishing the actual key press
                            # self.target_pose_pub.publish(target_pose)
                            self.executing_path = True
                            self.publish_path_planning_status()
                            self.path_executor_thread.start()
                    pass
        # TODO: arm pose should update with the arm's movement, this can be done with internal state that's checked with against the real state but requires testing for jitter
        if self.state != ArmState.IK:
            # TODO: need to be updating internal pose state using FK pose updates, need to resolve discrepancies from IK solutions
            self.update_internal_pose_state(self.current_joints)
        time.sleep(0.05)

    def handle_joy(self, msg):
        self.update_arm(msg)
    def on_press(self, key):
        pass
    def on_release(self, key):
        # On key release, send zeroed inputs to stop movement
        zero_inputs = ArmInputs()
        self.update_arm(zero_inputs)

    def home_arm(self, joint_indices=[0, 1, 2, 3, 4, 5, 6],  hz: float = 50.0):
        # endpoint refers to positive-direction endpoint
        # TODO: handle homing in parallel either using the multithreaded executor provided by ROS2 or moving it to a different node altogether
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
                self.publish_homing_status()
            
            # once a step has been calculated for each joint, publish the targets and then wait
            self.safe_target_joints, self.safety_flags = self.safety_checker.update_safe_goal_pos(
                target_joints, self.current_joints)
            self.publish_safety_status()
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
        self.publish_homing_status()
        self.homing_stop.clear()
        self.homing_thread.start()

    def cancel_homing(self):
        """Request to cancel homing loop."""
        with self.homing_lock:
            if self.homing == HomingStatus.ACTIVE:
                self.get_logger().info("Homing canceled.")
            self.homing = HomingStatus.IDLE
            self.publish_homing_status()
            self.homing_stop.set()

    def publish_safety_status(self):
        msg = ArmStatuses()  #creates an ArmStatues msg
        statuses = [str(flag) for flag in self.safety_flags]
        msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5, msg.joint6, msg.joint7 = statuses
        self.safety_status_pub.publish(msg)

    def publish_limit_switch_status(self):
        msg = ArmStatuses()
        statuses = ["TRIGGERED" if value else "CLEAR" for value in self.at_limit]
        msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5, msg.joint6, msg.joint7 = statuses
        self.limit_switch_pub.publish(msg)

    def publish_path_planning_status(self):
        msg = String()
        msg.data = "ACTIVE" if self.executing_path else "IDLE"
        self.path_planning_status_pub.publish(msg)

    def publish_homing_status(self):
        msg = String()
        msg.data = self.homing.name
        self.homing_status_pub.publish(msg)

    def update_internal_pose_state(self, joints):
        for i in range(len(joints)):
            joints[i] = np.pi * joints[i] / 180.0
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
        self.target_joints = list(np.array(msg.data, dtype=float)*180/math.pi)
        # append the end effector current rotation because IK solution does not have this
        self.target_joints.append(self.current_joints[-1])
        self.safe_target_joints, self.safety_flags = self.safety_checker.update_safe_goal_pos(
                        self.target_joints, self.arm_internal_current_joints) 
        self.publish_safety_status()
        msg = Float32MultiArray()
        msg.data = self.safe_target_joints
        self.safe_target_joints_pub.publish(msg)
        self.arm_internal_current_joints = self.safe_target_joints
        self.can_con.send_target_message(self.safe_target_joints)

        time.sleep(0.05)  # wait for some time before next update

    def handle_keyboard_coords(self, msg):
        corners = [msg.tl, msg.tr, msg.bl, msg.br]
        # Process corners to determine key positions
        self.interpolate_key_positions(corners)

    def interpolate_key_positions(self, corners):
        # zero_r = Rotation.identity()
        # tl_corner_tf = RigidTransform.from_components(np.array([corners[0].x, corners[0].y, corners[0].z]), zero_r)
        for key in self.keyboard_targets:
            self.key_positions[key] = self.key_positions[key] + np.array([corners[0].x, corners[0].y, corners[0].z], dtype = float)

    def update_path_planner_joints(self, msg):
        self.current_path.append(list(msg.data))

    def path_executor_loop(self):
        # while len(self.current_path) > 0 and self.shutdown == False:
        while self.executing_path:
            target_pose = self.key_positions[self.current_key]
            key_target = TargetPosition()
            key_target.name = self.current_key
            key_target.position.x, key_target.position.y, key_target.position.z = target_pose[0], target_pose[1], target_pose[2]
            key_target.distance = 0.0
            
            self.keyboard_target_pub.publish(key_target)
            # for step in self.current_path:
            #     error = [abs(step[i] - self.current_joints[i]) for i in range(self.n_joints)]
            #     if all(e < self.joint_target_threshold for e in error):
            #         self.current_path.pop(0)
            #         continue
                    
            #     self.target_joints[0:6] = step
            #     self.safe_target_joints, self.safety_flags = self.safety_checker.update_safe_goal_pos(
            #         self.target_joints, self.current_joints)
            #     msg = Float32MultiArray()
            #     msg.data = self.safe_target_joints
            #     self.safe_target_joints_pub.publish(msg)

            #     self.internal_current_joints = self.safe_target_joints

            #     # DISABLED TEMPORARILY FOR SAFETY
            #     # self.can_con.send_target_message(self.safe_target_joints)
            #     time.sleep(0.1)  # wait for some time before next step
            # self.executing_path = False
        self.executing_path = False
        self.publish_path_planning_status()
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