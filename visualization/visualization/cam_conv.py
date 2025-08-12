
# import roslib
# roslib.load_manifest('my_package')
import sys
import rclpy
from rclpy.node import Node
import cv2
# from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import numpy

class ImageConverterNode(Node):

    def __init__(self):
        super().__init__('image_converter')
        self.image_pub = self.create_publisher(Image, "image_topic_2", 10)

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, "/encoded_image_topic", self.callback, 10)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
            # newImg = cv_image
            # # newImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # # newImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # encodeParam = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            # result, encImg = cv2.imencode('.jpg', newImg, encodeParam)
            # if result == False:
            #     print('did not encode')
            # print("hello")
            decImg = cv2.imdecode(cv_image, 1)
            # print(decImg)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(decImg, "bgr8"))
        except CvBridgeError as e:
            print(e)

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "grey"))
    # except CvBridgeError as e:
    #   print(e)

def main(args=None):
    rclpy.init(args=args)
    node = ImageConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()