import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge

class Detector(Node):

    def __init__(self):
        super().__init__("detector")
        self.pub_debug_img = self.create_publisher(Image, "detected_image", 10)
        self.sub_image_feed = self.create_subscription(
            CompressedImage,
            "image_raw/compressed",
            self.image_feed_callback,
            10)
        self.bridge = CvBridge()

    def image_feed_callback(self, msg):

        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        final_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8") 
        self.pub_debug_img.publish(final_msg)


        
def main(args=None):
    rclpy.init(args=args)
    detector = Detector()
    rclpy.spin(detector)

    # Below lines are not strictly necessary
    detector.destroy_node()
    rclpy.shutdown()
        
if __name__=='__main__':
    main()
