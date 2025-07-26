#!/usr/bin/env python3

import rospy
import numpy as np
from math import pi
from std_msgs.msg import Float32MultiArray, Int32, Bool, String
from geometry_msgs.msg import Point
from arm_science_ik import SciArm

class ArraySamplingNode:
    def __init__(self):
        rospy.init_node('array_sampling_node')
        
        # Initialize parameters
        self.rate = rospy.Rate(30)  # 30Hz update rate
        self.current_sequence = []
        self.is_executing = False
        self.current_position = None
        
        # Get ROS parameters
        self.dh_table = [[79.7, 0, 0, 0],
                        [0, 0, 367, 0],
                        [0, 0, 195, 0],
                        [0, 0, 67, 0],
                        [92, 0, 0, 0]]
        
        # Initialize arm
        self.arm = SciArm(5, self.dh_table)
        
        # Define array sampling positions
        self.setup_array_positions()
        
        # Publishers
        self.goal_pub = rospy.Publisher(
            'arm_goal_pos', 
            Float32MultiArray, 
            queue_size=10
        )
        self.status_pub = rospy.Publisher(
            'array_sampling_status', 
            String, 
            queue_size=10
        )
        self.position_reached_pub = rospy.Publisher(
            'position_reached', 
            Bool, 
            queue_size=10
        )
        
        # Subscribers
        rospy.Subscriber(
            'start_array_sampling', 
            Int32, 
            self.start_sampling_callback
        )
        rospy.Subscriber(
            'arm_curr_pos', 
            Float32MultiArray, 
            self.current_position_callback
        )
        rospy.Subscriber(
            'abort_sampling', 
            Bool, 
            self.abort_callback
        )
        
        # Service to get available positions
        self.available_positions = {}
        self.setup_array_positions()
        
        rospy.loginfo("Array sampling node initialized")

    def setup_array_positions(self):
        """
        Define all sampling positions and sequences
        """
        # Base positions (in cylindrical coordinates [theta, r, z, alpha])
        self.available_positions = {
            'home': [0, 200, 300, 0],
            'safe_transport': [0, 250, 400, 0],
            'pre_sample': [0, 350, 200, -pi/4],
            'deposit': [pi/2, 400, 300, -pi/4],
        }
        
        # Array sampling positions
        array_positions = [
            [0, 350, 0, -pi/2],      # Position 1
            [pi/4, 350, 0, -pi/2],   # Position 2
            [pi/2, 350, 0, -pi/2],   # Position 3
            [3*pi/4, 350, 0, -pi/2]  # Position 4
        ]
        
        # Add array positions to available positions
        for i, pos in enumerate(array_positions):
            self.available_positions[f'array_{i+1}'] = pos

    def generate_sampling_sequence(self, array_index):
        """
        Generate sequence for sampling from specific array position
        """
        if f'array_{array_index}' not in self.available_positions:
            return None
            
        sequence = [
            self.available_positions['home'],
            self.available_positions['safe_transport'],
            self.available_positions['pre_sample'],
            self.available_positions[f'array_{array_index}'],
            self.available_positions['safe_transport'],
            self.available_positions['deposit'],
            self.available_positions['home']
        ]
        
        return sequence

    def publish_status(self, message):
        """
        Publish status message
        """
        status_msg = String()
        status_msg.data = message
        self.status_pub.publish(status_msg)

    def publish_goal_position(self, position):
        """
        Publish goal position
        """
        msg = Float32MultiArray()
        self.arm.cylTarget = position
        if self.arm.inverseKinematics():
            msg.data = self.arm.getGoalAngles()
            self.goal_pub.publish(msg)
            return True
        return False

    def current_position_callback(self, msg):
        """
        Handle current position updates
        """
        self.current_position = msg.data
        
    def position_reached(self, target_position, tolerance=0.05):
        """
        Check if target position is reached
        """
        if self.current_position is None:
            return False
            
        self.arm.cylTarget = target_position
        if not self.arm.inverseKinematics():
            return False
            
        target_angles = self.arm.getGoalAngles()
        
        # Check if all joint angles are within tolerance
        for curr, target in zip(self.current_position, target_angles):
            if abs(curr - target) > tolerance:
                return False
        return True

    def start_sampling_callback(self, msg):
        """
        Handle start sampling request
        """
        if self.is_executing:
            self.publish_status("Sampling sequence already in progress")
            return
            
        array_index = msg.data
        sequence = self.generate_sampling_sequence(array_index)
        
        if sequence is None:
            self.publish_status(f"Invalid array index: {array_index}")
            return
            
        self.current_sequence = sequence
        self.is_executing = True
        self.publish_status(f"Starting sampling sequence for array position {array_index}")

    def abort_callback(self, msg):
        """
        Handle abort request
        """
        if msg.data:
            self.is_executing = False
            self.current_sequence = []
            self.publish_status("Sampling sequence aborted")
            # Move to safe position
            self.publish_goal_position(self.available_positions['home'])

    def execute_sequence(self):
        """
        Execute current sequence
        """
        if not self.current_sequence or not self.is_executing:
            return
            
        current_target = self.current_sequence[0]
        
        # Check if current position is reached
        if self.position_reached(current_target):
            self.position_reached_pub.publish(True)
            self.current_sequence.pop(0)
            if not self.current_sequence:
                self.is_executing = False
                self.publish_status("Sampling sequence completed")
                return
                
        # Publish current target
        if not self.publish_goal_position(current_target):
            self.is_executing = False
            self.current_sequence = []
            self.publish_status("Failed to reach target position")

    def run(self):
        """
        Main run loop
        """
        while not rospy.is_shutdown():
            if self.is_executing:
                self.execute_sequence()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ArraySamplingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass