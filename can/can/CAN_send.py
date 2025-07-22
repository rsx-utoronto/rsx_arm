#!/usr/bin/env python3

from CAN_utilities import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class CAN_Send(Node):
	"""
	(None)

	This class represents an instance of the CAN_Send node that connects to Safety node
	by arm_safe_goal_pos topic and sends the received positions to the CAN bus
	"""

	def __init__(self):
		super().__init__('CAN_Send')

		# Variables to store publishing rate and trigger for node initiation
		self.pub_rate = 2000
		self.triggered = 0

		# Subscriber buffers
		self.CURR_POS			= [0, 0, 0, 0, 0, 0, 0] # ADDED BACK 7TH MOTOR
		self.SAFE_GOAL_POS	 	= [0, 0, 0, 0, 0, 0, 0]	# ADDED BACK 7TH MOTOR

		# Variables for ROS publishers and subscribers
		self.SafePos_sub 		= self.create_subscription(
			Float32MultiArray, 
			"arm_safe_goal_pos", 
			self.callback_SafePos, 
			10
		)
		self.CurrPos_sub		= self.create_subscription(
			Float32MultiArray, 
			"arm_curr_pos", 
			self.callback_CurrPos, 
			10
		)

		# Create timer for sending messages
		self.timer = self.create_timer(1.0/self.pub_rate, self.send_msgs_callback)
	
	def callback_CurrPos(self, data : Float32MultiArray):
		"""
		(Float32MultiArray) -> (None)

		Callback function for receiving CURR_POS data and updating the CURR_POS variable

		@parameters

		data (Float32MultiArray) : The data from the topic is stored in this parameter
		"""
		
		# Save the data from the topic into our buffer and set the trigger to 1
		self.CURR_POS = list(data.data)
		self.triggered = 1

	def callback_SafePos(self, data : Float32MultiArray):
		"""
		(Float32MultiArray) -> (None)

		Callback function for receiving SAFE_GOAL_POS data and updating the SAFE_GOAL_POS variable

		@parameters

		data (Float32MultiArray) : The data from the topic is stored in this parameter
		"""

		# Save the data from the topic into our buffer
		self.SAFE_GOAL_POS = list(data.data)

		# # Add correction for gripper (due to mechanical design)
		# self.SAFE_GOAL_POS[6] -= (self.SAFE_GOAL_POS[4] - self.CURR_POS[4])
		# print(self.SAFE_GOAL_POS[6])
	
	def send_msgs_callback(self):
		"""
		Timer callback function for sending CAN messages at regular intervals
		"""
		if not self.triggered:
			return

		# Convert SparkMAX angles to SparkMAX data packets
		spark_input = generate_data_packet(self.SAFE_GOAL_POS) # assuming data is safe
		print(self.SAFE_GOAL_POS)
		
		# Send data packets
		for i in range(1, len(spark_input)+1):
					
			# Motor number corresponds with device ID of the SparkMAX
			motor_num = 10 + i

			#print(spark_input)
			if motor_num > 10 and motor_num < 18:
				id = generate_can_id(dev_id= motor_num, api= CMD_API_POS_SET) # API WILL BE CHANGED WHEN USING THE POWER (DC) SETTING
				send_can_message(can_id= id, data= spark_input[i - 1])
			
			else:
				break



if __name__=="__main__":
        
	# # Instantiate CAN bus
	# initialize_bus()

	# Broadcast heartbeat
	hb = can.Message(
		arbitration_id= generate_can_id(
			dev_id= 0x0, 
			api= CMD_API_NONRIO_HB), 
		data= bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]), 
		is_extended_id= True,
		is_remote_frame = False, 
		is_error_frame = False
	)
	task = BUS.send_periodic(hb, 0.01, store_task= False)
	print("Heartbeat initiated")
	
	# Initialize ROS2
	rclpy.init()

	# Setup and run node
	CAN_Send_Node = CAN_Send()

	# Spin to keep node alive
	try:
		rclpy.spin(CAN_Send_Node)
	except KeyboardInterrupt:
		pass
	finally:
		# Stop the heartbeat when shutting down
		task.stop()
		CAN_Send_Node.destroy_node()
		rclpy.shutdown()