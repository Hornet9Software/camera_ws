import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge

# node reads image input from serial and publishes to topic

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_compress')
        self.publisher = self.create_publisher(CompressedImage, 'cam/compressed', 10)
        self.cv_bridge = CvBridge()
        timer_period = 0.5
        
        self.timer = self.create_timer(timer_period,self.publish_frame)  # Adjust the frame rate as needed


    def publish_frame(self):
        # Open the USB camera (adjust the index as needed, e.g., 0 for the first camera)
        fps = 30
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                # image_msg = self.cv_bridge.cv2_to_compressed_imgmsg(frame)
                image_msg = self.cv_bridge.cv2_to_compressed_imgmsg(frame)
                self.publisher.publish(image_msg)
                

        cap.release()
        
def main(args=None):
    rclpy.init(args=args)
    publisher = CameraPublisherNode()
    rclpy.spin(publisher)

    # Below lines are not strictly necessary
    publisher.destroy_node()
    rclpy.shutdown()
        
if __name__=='__main__':
    main()
