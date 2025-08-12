#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import math
import geometry_msgs.msg
import tf2_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32MultiArray, Header, Float64
from numpy import deg2rad, subtract, array
from math import pi
from copy import deepcopy


def anglePosition():
    '''
    Prompts the user to enter 6 angles in degrees and returns an array of size 6. 
    '''
    angle1, angle2, angle3, angle4, angle5, angle6 = input(
        "Enter 6 angles (seperated by spaces) in degrees: ").split()  # Angles in degrees
    angleArray = list(map(float, [angle1, angle2, angle3, angle4, angle5, angle6]))
    for i in range(len(angleArray)):
        angleArray[i] = angleArray[i] * math.pi /180  # Conversion from degrees to radians
    return angleArray


def framePosition():
    '''
    Prompts the user to enter 3 distances and returns an array of size 3. 
    '''
    roll, pitch, yaw, x, y, z = input("Enter roll, pitch, yaw, x, y, z (seperated by spaces): ").split(
    )  # Distances from base_link in unit squares values
    tempArray = list(map(float, [roll, pitch, yaw, x, y, z]))
    posArray = [tempArray[0], tempArray[1], tempArray[2], [tempArray[3], tempArray[4], tempArray[5]]]
    return posArray

class ArmVisualizationNode(Node):
    ''' The node that visualizes the arm.

    The arm can be visalized both in RViz and gazebo. 
    When using with the real arm make sure to set the ros paramter
    /gazebo_on to false. If using the node with gazebo, set /gazebo_on
    to true so that node publishes joint messages to gazebo.
    
    '''

    def __init__(self):
        super().__init__('arm_viz')
        self.liveArmAngles = [0, 0, 0, 0, 0, 0, 0]
        self.savedCanAngles = [0, 0, 0, 0, 0, 0, 0]
        self.savedSciCanAngles = [0, 0, 0, 0, 0]

        self.curMode = "Idle"

        self.declare_parameter('gazebo_on', False)
        self.gazebo_on = self.get_parameter('gazebo_on').get_parameter_value().bool_value
        self.ikEntered = False
    
        # ROS Topic Setup
        if self.gazebo_on:
            self.gazeboPublisher = self.startGazeboJointControllers(9)
        else:
            self.jointPublisher = self.create_publisher(JointState, "joint_states", 10)

        self.create_subscription(String, "arm_state", self.updateState, 10)
        self.create_subscription(Float32MultiArray, "arm_viz_pos", self.displayArmGoalPos, 10)
        # self.create_subscription(Float32MultiArray, "arm_curr_pos", self.displayArmLivePos, 10)

        # Create TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


    def startGazeboJointControllers(self, numJoints):
        ''' Starts to publishers for the joints in gazebo

        Parameters
        ----------
        numJoints
            the number of joints in the gazebo model
        '''

        gazeboPublisher = list()

        for i in range(numJoints):
            topic_name = f"/arm/joint{i+1}_position_controller/command"
            publisher = self.create_publisher(Float64, topic_name, 10)
            gazeboPublisher.append(publisher)
        
        return gazeboPublisher


    def displayEndEffectorTransform(self, endEffectorPosition, referenceLink="base_link", quaternionAngles=None):
        '''
        Publishes a tf2 transform at the End Effector Position 

        Parameters
        ----------
        endEffectorPosition
            The [roll, pitch, yaw, [x, y, z]] position of the end effector
        referenceLink
            The link you want to display the end effector transform relative to.
            By default is the "base_link" of the arm.
        quaternionAngles
            A list of 4 numbers that represent a frame in space using quaternions. 
            Use to ignore endEffectorPosition paramter. By default is None.
        '''
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = referenceLink
        t.child_frame_id = "target_position"

        posArray = endEffectorPosition
        t.header.stamp = self.get_clock().now().to_msg()
        t.transform.translation.x = posArray[3][0]
        t.transform.translation.y = posArray[3][1]
        t.transform.translation.z = posArray[3][2]
        
        r = sp.spatial.transform.Rotation.from_euler('xyz', [posArray[0], posArray[1], posArray[2]])
        q = r.as_quat()
        if quaternionAngles != None:
            q = quaternionAngles
            t.transform.rotation.x = q.x
            t.transform.rotation.y = q.y
            t.transform.rotation.z = q.z
            t.transform.rotation.w = q.w
        else:
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def getFrameTransform(self, referenceFrame: str, targetFrame: str):
        ''' Gets the transform of a tf frame to another

        Gets the transform from referenceFrame to targetFrame. In other words,
        the transformation of targetFrame to referenceFrame.

        Paramters
        ---------
        referenceFrame
            the name of the frame you are basing the transformation on (the targets transform is given relative to this frame)
        targetFrame
            the name of target

        Returns
        -------
        targetTransform
            a transform message with a .translation and a .rotation component, is none if target doesn't exist
        '''
        
        try:
            if self.tf_buffer.can_transform(referenceFrame, targetFrame, Time()):
                trans = self.tf_buffer.lookup_transform(referenceFrame, targetFrame, Time())
                return trans.transform
        except Exception as ex:
            self.get_logger().warn(f"Transform lookup failed: {ex}")
            return None

    def runNewDesiredJointState(self, angles):
        '''
        Publishes the newJointState header, stamp, name, and position in a continuous while loop. 
        '''

        # data to be published
        newJointState = JointState()
        newJointState.header = Header()
        newJointState.header.stamp = self.get_clock().now().to_msg()
        newJointState.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "finger_joint_1", "finger_joint_2", "camera_joint"]
        # Angles in radians [Joint_1, Joint_2, ....], re-run this script and change the values to see it work.
        newJointState.position = angles
        self.jointPublisher.publish(newJointState)  # send data to be published
        # positionArray = framePosition() # Display End Effector position with a transform
        # displayEndEffectorTransform(positionArray)

    def runNewRealJointState(self, angles):
        '''
        Publishes the newJointState header, stamp, and name for the real arm values 
        '''

        # data to be published
        newJointState = JointState()
        newJointState.header = Header()
        newJointState.header.stamp = self.get_clock().now().to_msg()
        newJointState.name = ["real_joint_1", "real_joint_2", "real_joint_3", "real_joint_4", 
                            "real_joint_5", "real_joint_6", "real_finger_joint_1", "real_finger_joint_2", 
                            "real_camera_joint"]
        # Angles in radians [Joint_1, Joint_2, ....], re-run this script and change the values to see it work.
        newJointState.position = angles
        self.jointPublisher.publish(newJointState)  # send data to be published

    def runNewJointState4(self, angles):
        '''
        Publishes the newJointState header, stamp, and name for the real arm values 
        '''

        # data to be published
        newJointState = JointState()
        newJointState.header = Header()
        newJointState.header.stamp = self.get_clock().now().to_msg()
        newJointState.name = ["Real_Joint_1", "Real_Joint_2", "Real_Joint_3", "Real_Joint_4", "Real_Joint_5", 
                            "Real_Joint_6", "Real_Joint_7", "Real_Joint_8", "Real_Joint_9", "Joint_1", 
                            "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6", "Joint_7", "Joint_8", "Joint_9"]
        # Angles in radians [Joint_1, Joint_2, ....], re-run this script and change the values to see it work.
        newJointState.position = angles
        self.jointPublisher.publish(newJointState)  # send data to be published

    def runNewSciJointState(self, angles):
        '''
        Publishes the newJointState header, stamp, and name for the real arm values 
        '''

        # data to be published
        newJointState = JointState()
        newJointState.header = Header()
        newJointState.header.stamp = self.get_clock().now().to_msg()
        newJointState.name = ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"]
        # Angles in radians [Joint_1, Joint_2, ....], re-run this script and change the values to see it work.
        newJointState.position = angles
        self.jointPublisher.publish(newJointState)  # send data to be published


    def moveInGazebo(self, angles):
        ''' Moves arm in Gazebo based on IK angles
        
        Paramters
        ---------
        jointControllerPublishers
            list of publishers for each joint controller
        angles
            list of joint angle in same order as publishers
        '''
        
        for i in range(len(self.gazeboPublisher)):
            self.gazeboPublisher[i].publish(Float64(data=angles[i]))

    
    def updateState(self, data):
        ''' Callback function for the /arm_state topic

        Paramters
        ---------
        data
            data is a string message type. Use data.data to get the
            string that contains the name of the current mode.
        '''
        self.curMode = data.data

        if data.data == "IK":
            if not self.ikEntered:
                self.savedCanAngles = deepcopy(self.liveArmAngles)
            self.ikEntered = True

    def displayArmGoalPos(self, data):
        ''' Callback function for /arm_goal_pos
        
        Uses the goal position angles from /arm_goal_pos to display 
        the arm target

        Paramters
        ---------
        data
            A Float32MultiArray that contains the arm angles in degrees.
        '''
        tempAngles = list(deg2rad(list(data.data)))
        print(tempAngles)
        # tempAngles = list(subtract(array(tempAngles), -1*array(self.savedCanAngles)))
        tempAngles = list(subtract(array(tempAngles), -1*array(self.savedSciCanAngles)))
        # tempAngles.append(tempAngles[6]) # make gripper angles equal
        # tempAngles.append(tempAngles[6]) # make gripper angles equal

        # unswap angles to match with joint order
        # if False:
        #     temp = tempAngles[5]
        #     tempAngles[5] = tempAngles[4]
        #     tempAngles[4] = temp

        #     # undo sign inversion for motors 
        #     tempAngles[0] = -tempAngles[0]
        #     tempAngles[1] = tempAngles[1]
        #     tempAngles[5] = tempAngles[5]
            
        if self.gazebo_on:
            tempAngles.append(0)
            tempAngles.append(0)
            tempAngles.append(0)
            self.moveInGazebo(tempAngles)
        else:
            self.runNewSciJointState(tempAngles)

    def displayArmLivePos(self, data):
        ''' Callback function for the /arm_curr_pos topic

        Use this callback function to update visualization of the 
        arms current position in RViz.

        Paramters
        ---------
        data
            data is a Float32MultiArray for the current arm positions.
            data.data contains 7 floats that describe the arms current
            position.
        '''
        if not self.gazebo_on: 
            tempList = list(deg2rad(list(data.data)))

            tempList[0] = -tempList[0]
            tempList[1] = tempList[1]
            temp = tempList[5]
            tempList[5] = tempList[4]
            tempList[4] = temp
            # tempList[5] = -tempList[5]
            # tempList[6] = tempList[6]
            tempAngles = list(subtract(array(tempList), array(self.savedCanAngles)))

            self.liveArmAngles = deepcopy(tempAngles)

            tempAngles.append(tempAngles[6])
            tempAngles.append(0)

            self.runNewRealJointState(tempAngles)


def main(args=None):
    rclpy.init(args=args)

    try:
        arm_viz_node = ArmVisualizationNode()
        rclpy.spin(arm_viz_node)
    except Exception as ex:
        print(f"Some error has occurred: {ex}")
    finally:
        rclpy.shutdown()



if __name__ == "__main__":  # if used rosrun on this script then ...
    main()