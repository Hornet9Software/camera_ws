import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge

class CameraSubscriberNode(Node):

    def __init__(self):
        super().__init__("process")
        self.pub_debug_img = self.create_publisher(Image, "cam/processed", 10)
        # self.hsv_img = self.create_publisher(Image, "hsv", 10)
        self.sub_image_feed = self.create_subscription(
            CompressedImage,
            "cam/compressed",
            self.image_feed_callback,
            10)
        self.bridge = CvBridge()

    def image_feed_callback(self, msg):

        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # hsv_msg = self.bridge.cv2_to_imgmsg(hsv, encoding="bgr8") 
        # self.hsv_img.publish(hsv_msg)

        final_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8") 
        self.pub_debug_img.publish(final_msg)


        
def main(args=None):
    rclpy.init(args=args)
    subscriber = CameraSubscriberNode()
    rclpy.spin(subscriber)

    # Below lines are not strictly necessary
    subscriber.destroy_node()
    rclpy.shutdown()
        
if __name__=='__main__':
    main()
