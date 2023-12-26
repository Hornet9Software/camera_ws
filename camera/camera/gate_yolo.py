import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from std_msgs.msg import Float32MultiArray

####################
# This node is run in conjunction with the enhance node..
# This node detects for qualification gate (orange/yellow poles) and publishes the bearing and distance to the gate.
# ML method of detection, uses detecto library 
# (torchvision.models.detection.faster_rcnn.FasterRCNN)
#####################

model = YOLO('/home/bb/poolTest_ws/src/camera_ws/camera/weights/yolo_gate_weights.pt')
# Transform to apply on individual frames of the video


class GateDetectorNode(Node):
    def __init__(self):
        super().__init__('gate_detector_node')
        self.subscription = self.create_subscription(
            Image,
            '/left/image_rect_color',
            # '/Hornet/Cam/left/image_rect_color/compressed',
            self.image_callback,
            10
        )
        # self.yolo_pub = self.create_publisher(
        #     CompressedImage,
        #     '/left/yolo/compressed',
        #     10
        # )
        self.bbox_pub = self.create_publisher(
            Float32MultiArray,
            '/left/yolo/box',
            10
        )
        self.cv_bridge = CvBridge()


    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # detection
        results = model(cv_image)
        
        # get the bounding box
        xyxy = results[0].boxes.xyxy.flatten().tolist()
        # xyxy is now a list of 4 floats (x1, x2, y1, y2)

        # terminate if no detection
        if len(xyxy) == 0:
            return

        bbox_msg = Float32MultiArray()
        bbox_msg.data = xyxy

        # publish the message
        self.bbox_pub.publish(bbox_msg)


        # Visualize the results on the frame
        # annotated_frame = results[0].plot()
        # compressed_msg = self.cv_bridge.cv2_to_compressed_imgmsg(annotated_frame)
        # self.yolo_pub.publish(compressed_msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    gate_detector_node = GateDetectorNode()
    rclpy.spin(gate_detector_node)
    gate_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()