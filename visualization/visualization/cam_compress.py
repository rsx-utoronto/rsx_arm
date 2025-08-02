#!/usr/bin/env python3
import cv2
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# # argvs=sys.argv
# # if (len(argvs) != 2):
# #     print('Usage: # python %s imagefilename' % argvs[0])
# #     quit()
 
# # imagefilename = argvs[1]


# try:
#      vid=cv2.VideoCapture("/dev/video6")
# except:
#      print('faild to load pic')
#      quit()

# #encode to jpeg format
# #encode param image quality 0 to 100. default:95
# #if you want to shrink data size, choose low image quality.
# while(True):
#     ret, img = vid.read()  
#     # cv2.imshow('Source Image',img)
#     # cv2.waitKey(0)      
#     encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),10]
#     result,encimg=cv2.imencode('.jpg',img,encode_param)
#     if False==result:
#         print('could not encode image!')
#         quit()

#     # #decode from jpeg format
#     decimg=cv2.imdecode(encimg,1)
#     cv2.imshow('Source Image',img)
#     cv2.imshow('Decoded image',decimg)
#     cv2.waitKey(0)
#     # cv2.destroyAllWindows()
#     # publish(image data) 

# import cv2
# import sys
# import numpy
# import rospy
# from sensor_msgs.msg import Image

# argvs=sys.argv
# if (len(argvs) != 2):
#     print 'Usage: # python %s imagefilename' % argvs[0]
#     quit()


class CameraCompressNode(Node):
    def __init__(self):
        super().__init__('image_encoder')
        self.pub = self.create_publisher(Image, 'encoded_image_topic', 10)
        try:
            self.vid = cv2.VideoCapture(0)
        except:
            print('faild to load pic')
            quit()
        self.bridge = CvBridge()
        # Timer replaces while(True)
        self.timer = self.create_timer(0.1, self.capture_and_publish)

    def capture_and_publish(self):
        ret, img = self.vid.read()
        # cv2.imshow('Source Image',img)
        # cv2.waitKey(0)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 10]
        result, encimg = cv2.imencode('.jpg', img, encode_param)
        if not result:
            print('could not encode image!')
            # quit()
        # #decode from jpeg format
        # decimg = cv2.imdecode(encimg, 1)
        # cv2.imshow('Source Image', img)
        # cv2.imshow('Decoded image', decimg)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        try:
            self.pub.publish(self.bridge.cv2_to_imgmsg(encimg, 'mono8')) #change to 'bgr' encoding if need to publish original image
        except CvBridgeError as e:
            print(f"Error publishing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraCompressNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
                    
