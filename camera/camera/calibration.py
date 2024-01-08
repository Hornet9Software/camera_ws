import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

# might need to edit the path name
LEFT_CALIBRATION_PATH = "src/camera_ws/camera/calibration/calibrationdata/left.yaml"
RIGHT_CALIBRATION_PATH = "src/camera_ws/camera/calibration/calibrationdata/right.yaml"
BOTTOM_CALIBRATION_PATH = "src/camera_ws/camera/calibration/calibrationdata/bottom.yaml"


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("calibrator_node")
        self.bridge = CvBridge()
        self.subscriber_left = self.create_subscription(Image, "/left/raw", lambda msg: self.image_callback(msg, LEFT_CALIBRATION_PATH), 10)
        self.subscriber_right = self.create_subscription(Image, "/right/raw", lambda msg: self.image_callback(msg, RIGHT_CALIBRATION_PATH), 10)
        self.subscriber_bottom = self.create_subscription(Image, "/bottom/raw", lambda msg: self.image_callback(msg, BOTTOM_CALIBRATION_PATH), 10)

        self.publisher_left = self.create_publisher(CameraInfo, "/left/calibration", 10)
        self.publisher_right = self.create_publisher(
            CameraInfo, "/right/calibration", 10
        )
        self.publisher_btm = self.create_publisher(
            CameraInfo, "/bottom/calibration", 10
        )

        timer_period = 0.1
        # self.timer = self.create_timer(timer_period, self.publish_calibration_data)
    
    def image_callback(self, msg, path):
        data = self.read_data(path)
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        calibrated_img = cv2.undistort(img, data["k"], data["d"], data["r"], data["p"])
        calibrated_imgmsg = self.bridge.cv2_to_imgmsg(calibrated_img)


    def publish_calibration_data(self):
        left_msg = self.generate_message(self.read_data(LEFT_CALIBRATION_PATH))
        right_msg = self.generate_message(self.read_data(RIGHT_CALIBRATION_PATH))
        bottom_msg = self.generate_message(self.read_data(BOTTOM_CALIBRATION_PATH))

        self.publisher_left.publish(left_msg)
        self.publisher_right.publish(right_msg)
        self.publisher_btm.publish(bottom_msg)

        self.get_logger().info(f"left_calibration_data: {left_msg}")
        self.get_logger().info(f"right_calibration_data: {right_msg}")
        self.get_logger().info(f"bottom_calibration_data: {bottom_msg}")

    def read_data(self, path):
        data = {
            # "name": None,
            "height": None,
            "width": None,
            "distortion_model": None,
            "d": None,
            "k": None,
            "r": None,
            "p": None,
        }

        with open(path, "r") as file:
            contents = yaml.safe_load(file)
            # data["name"] = contents["camera_name"]
            data["height"] = contents["image_height"]
            data["width"] = contents["image_width"]
            data["distortion_model"] = contents["distortion_model"]
            data["d"] = np.array(contents["distortion_coefficients"]["data"])
            data["k"] = np.array(contents["camera_matrix"]["data"]).reshape((3, 3))
            data["r"] = np.array(contents["rectification_matrix"]["data"]).reshape((3, 3))
            data["p"] = np.array(contents["projection_matrix"]["data"]).reshape((3, 4))
        return data

    def generate_message(self, data):
        calibration_msg = CameraInfo()
        # not sure if the time is right
        calibration_msg.header.stamp = self.get_clock().now().to_msg()
        calibration_msg.height = data["height"] 
        calibration_msg.width = data["width"] 
        calibration_msg.distortion_model = data["distortion_model"]
        calibration_msg.d = data["d"] 
        calibration_msg.k = data["k"] 
        calibration_msg.r = data["r"] 
        calibration_msg.p = data["p"] 

        return calibration_msg


def main(args=None):
    # reads the calibration data and publishes CameraInfo for other noddes to calibrate with
    rclpy.init(args=args)
    calibration_node = CalibrationNode()

    rclpy.spin(calibration_node)
    calibration_node.destroy_node()
    rclpy.shutdown()
