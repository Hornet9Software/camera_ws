import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO

weights_file = "yolov8n_160124_1.pt"
model = YOLO(f"/home/bb/poolTest_ws/src/camera_ws/camera/camera/weights/{weights_file}")


class GateDetectorNode(Node):
    def __init__(self):
        super().__init__("gate_detector_node")

        self.declare_parameter("camera", "left")
        camera = self.get_parameter("camera").get_parameter_value().string_value

        # Detect on raw image
        self.subscription = self.create_subscription(
            CompressedImage,
            f"/{camera}/compressed",
            self.image_callback,
            10,
        )

        # Detect on calibrated image
        # self.subscription = self.create_subscription(
        #     CompressedImage,
        #     f"/{camera}/calibrated/compressed",
        #     self.image_callback,
        #     10,
        # )

        self.yolo_pub = self.create_publisher(
            CompressedImage, f"/{camera}/yolo/compressed", 10
        )
        self.bbox_pub = self.create_publisher(
            Float32MultiArray, f"/{camera}/yolo/box", 10
        )
        self.cv_bridge = CvBridge()

        # For FPS
        self.cnt = 0
        self.init_time = time.time()

    def image_callback(self, msg):
        # Raw/Calibrated image
        cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)

        # Detection
        results = model(cv_image)

        for result in results:
            # Get the bounding box as (x, y, w, h), normalised to image dimensions
            boxes = result.boxes.xywhn
            classes = result.boxes.cls
            for box, cls in zip(boxes, classes):
                if cls == 0:  # If object is a gate,
                    bbox_msg = Float32MultiArray()
                    box_data = box.tolist()
                    bbox_msg.data = box_data

                    # Publish the message
                    self.bbox_pub.publish(bbox_msg)
                    self.get_logger().info(f"Published bounding box: {box_data}.")
                    break  # Only send the first gate detection

        # Visualize the results on the frame
        annotated_frame = results[0].plot()
        compressed_msg = self.cv_bridge.cv2_to_compressed_imgmsg(annotated_frame)
        self.yolo_pub.publish(compressed_msg)

        # Print FPS
        self.cnt += 1
        elapsed_time = time.time() - self.init_time
        if elapsed_time > 0:
            fps = self.cnt / elapsed_time
        if elapsed_time > 10:  # Reset after 10s
            self.init_time = time.time()
            self.cnt = 0
        self.get_logger().info(f"FPS: {fps}")


def main(args=None):
    rclpy.init(args=args)
    gate_detector_node = GateDetectorNode()
    rclpy.spin(gate_detector_node)
    gate_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
