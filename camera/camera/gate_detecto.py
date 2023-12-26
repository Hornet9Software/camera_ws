import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
from torchvision import transforms

# make sure you have detecto installed
# pip3 install detecto

from detecto.core import Model
from detecto import utils

####################
# This node is run in conjunction with the enhance node..
# This node detects for qualification gate (orange/yellow poles) and publishes the bearing and distance to the gate.
# ML method of detection, uses detecto library 
# (torchvision.models.detection.faster_rcnn.FasterRCNN)
#####################

labels = ['gate']
model = Model.load('/home/shengbin/camera_ws/src/camera/camera/weights/detecto_model.pth', labels)
# Transform to apply on individual frames of the video

transform_frame = transforms.Compose([
    transforms.ToPILImage(),
    transforms.ToTensor(),
    utils.normalize_transform(),
])

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
        
        predictions = model.predict_top(cv_image)
        for label, box, score in zip(*predictions):
        #     # Create the box around each object detected
        #     # Parameters: frame, (start_x, start_y), (end_x, end_y), (r, g, b), thickness
            if round(score.item(), 2) > 0.95:
                cv2.rectangle(cv_image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 0, 0), 3)
                # Write the label and score for the boxes
                # Parameters: frame, text, (start_x, start_y), font, font scale, (r, g, b), thickness
                cv2.putText(cv_image, '{}: {}'.format(label, round(score.item(), 2)), (int(box[0]), int(box[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

        compressed_msg = self.cv_bridge.cv2_to_compressed_imgmsg(cv_image)
        self.publisher.publish(compressed_msg)
        
def main(args=None):
    rclpy.init(args=args)
    gate_detector_node = GateDetectorNode()
    rclpy.spin(gate_detector_node)
    gate_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
