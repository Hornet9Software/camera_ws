import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from ultralytics import YOLO
from models.experimental import attempt_load
from utils.torch_utils import select_device
from utils.general import non_max_suppression, scale_coords
from utils.plots import plot_one_box, colors

class GateDetectionNode(Node):
    def __init__(self):
        super().__init__('gate_detection_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.device = select_device('')
        self.model = attempt_load('/path/to/yolov8/weights.pt', map_location=self.device)  # load FP32 model

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Preprocess image for YOLOv8
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (640, 640))
        img = np.transpose(img, (2, 0, 1))
        img = torch.from_numpy(img).float().div(255.0).unsqueeze(0)

        # Perform inference
        img = img.to(self.device)
        pred = self.model(img)[0]

        # Apply NMS
        pred = non_max_suppression(pred, 0.4, 0.5)

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], cv_image.shape).round()

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    label = f'{conf:.2f}'
                    plot_one_box(xyxy, cv_image, label=label, color=colors[int(cls)], line_thickness=3)

        # Display the resulting frame
        cv2.imshow('frame', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    gate_detection_node = GateDetectionNode()

    rclpy.spin(gate_detection_node)

    # Destroy the node explicitly
    gate_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()