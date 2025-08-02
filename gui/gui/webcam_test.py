#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
import cv_bridge

webcam = cv2.VideoCapture(0)
bridge = cv_bridge.CvBridge()

rospy.init_node("webcam_stream")
rate = rospy.Rate(30)

image_publisher = rospy.Publisher("camera/color/image_king", Image, queue_size=10)

while not rospy.is_shutdown():
    ret, frame = webcam.read()
    img_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    image_publisher.publish(img_message)
    rate.sleep()