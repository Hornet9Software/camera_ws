import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

####################
# This node is run in conjunction with the enhance node..
# This node detects for qualification gate (orange/yellow poles) and publishes the bearing and distance to the gate.
# ML method of detection, uses detecto library 
# (torchvision.models.detection.faster_rcnn.FasterRCNN)
#####################

model = YOLO('/home/shengbin/camera_ws/src/camera/camera/weights/yolo_gate_weights.pt')
# Transform to apply on individual frames of the video


class GateDetectorNode(Node):
    def __init__(self):
        super().__init__('gate_detector_node')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/left/gray_world/compressed',
            # '/Hornet/Cam/left/image_rect_color/compressed',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(
            CompressedImage,
            '/gate_detector_node/compressed',
            10
        )
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        
        results = model(cv_image)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        compressed_msg = self.cv_bridge.cv2_to_compressed_imgmsg(annotated_frame)
        self.publisher.publish(compressed_msg)
        
def main(args=None):
    rclpy.init(args=args)
    gate_detector_node = GateDetectorNode()
    rclpy.spin(gate_detector_node)
    gate_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
