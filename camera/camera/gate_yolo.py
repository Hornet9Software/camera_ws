import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Detect Task 1 Gate
# Use YOLO to detect the gate

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.left_camera_sub = self.create_subscription(Image, 'left_camera_topic', self.left_camera_callback, 10)
        self.cv_bridge = CvBridge()

    def left_camera_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        # Perform object detection using yolov8 on the left camera image
        # ...
        # ...



def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
