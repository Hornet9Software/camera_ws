import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge

class CameraOpticalNode(Node):

    def __init__(self):
        super().__init__("cam_oflow")
        self.hsv_img = self.create_publisher(Image, "cam/oflow", 10)
        self.sub_image_feed = self.create_subscription(
            Image,
            "cam/processed",
            self.image_feed_callback,
            10)
        self.bridge = CvBridge()

    def image_feed_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv_msg = self.bridge.cv2_to_imgmsg(hsv, encoding="bgr8") 
        self.hsv_img.publish(hsv_msg)

        
def main(args=None):
    rclpy.init(args=args)
    optical_node = CameraOpticalNode()
    rclpy.spin(optical_node)

    # Below lines are not strictly necessary
    optical_node.destroy_node()
    rclpy.shutdown()
        
if __name__=='__main__':
    main()
