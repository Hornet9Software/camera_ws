import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage

# import CameraInfo if want to publish calibration data

# file paths are correct tested working when publish CameraInfo
LEFT_CALIBRATION_PATH = "src/camera_ws/camera/calibration/calibrationdata/left.yaml"
RIGHT_CALIBRATION_PATH = "src/camera_ws/camera/calibration/calibrationdata/right.yaml"
BOTTOM_CALIBRATION_PATH = "src/camera_ws/camera/calibration/calibrationdata/bottom.yaml"


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("calibrator_node")

        # read the calibration data file
        self.left_calibration_data = self.read_data(LEFT_CALIBRATION_PATH)
        self.right_calibration_data = self.read_data(RIGHT_CALIBRATION_PATH)
        self.btm_calibration_data = self.read_data(BOTTOM_CALIBRATION_PATH)
        self.bridge = CvBridge()

        # create publishers
        # New topics
        # /left/calibrated/compressed
        # /right/calibrated/compressed
        # /bottom/calibrated/compressed
        self.publisher_left = self.create_publisher(CompressedImage, "/left/calibrated/compressed", 10)
        self.publisher_right = self.create_publisher(CompressedImage, "/right/calibrated/compressed", 10)
        self.publisher_btm = self.create_publisher(CompressedImage, "/bottom/calibrated/compressed", 10)

        # create subscribers to raw image
        self.subscriber_left = self.create_subscription(
            Image,
            "/left/raw",
            lambda msg: self.image_callback(
                msg, self.left_calibration_data, self.publisher_left
            ),
            10,
        )
        self.subscriber_right = self.create_subscription(
            Image,
            "/right/raw",
            lambda msg: self.image_callback(
                msg, self.right_calibration_data, self.publisher_right
            ),
            10,
        )
        self.subscriber_bottom = self.create_subscription(
            Image,
            "/bottom/raw",
            lambda msg: self.image_callback(
                msg, self.btm_calibration_data, self.publisher_btm
            ),
            10,
        )

        # uncomment the following & the btm functions to publish calibration data
        # timer_period = 0.1
        # self.timer = self.create_timer(timer_period, self.publish_calibration_data)

    # callback function to publish the calibrated image once received
    # uses the cv2.undistort & getOptimalNewCameraMatrix functions to process
    def image_callback(self, msg, data, publisher):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        width_height = (data["width"], data["height"])

        # https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        # https://stackoverflow.com/questions/39432322/what-does-the-getoptimalnewcameramatrix-do-in-opencv
        # this chunk hella sus docs may be outdated
        # roi is not used to crop like in the tut
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
            data["k"], data["d"], width_height, 1, width_height
        )
        # rectification & projection matrix not used
        calibrated_img = cv2.undistort(cv_img, data["k"], data["d"], None, newcameramtx)
        calibrated_msg = self.bridge.cv2_to_compressed_imgmsg(calibrated_img)
        calibrated_msg.header.stamp = self.get_clock().now().to_msg()  # check timestamp
        publisher.publish(calibrated_msg)
        # self.get_logger().info(f"published calibrated img: {calibrated_msg}")

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

        # read the yaml file & return the contents in a dict according to CameraInfo params
        # name not included
        with open(path, "r") as file:
            contents = yaml.safe_load(file)
            # data["name"] = contents["camera_name"]
            data["height"] = contents["image_height"]
            data["width"] = contents["image_width"]
            data["distortion_model"] = contents["distortion_model"]

            # reshaped based on the size defined in the yaml file
            data["d"] = np.array(contents["distortion_coefficients"]["data"]).reshape(
                (1, 5)
            )
            data["k"] = np.array(contents["camera_matrix"]["data"]).reshape(
                (3, 3)
            )  # intrinsic matrix
            data["r"] = np.array(contents["rectification_matrix"]["data"]).reshape(
                (3, 3)
            )
            data["p"] = np.array(contents["projection_matrix"]["data"]).reshape((3, 4))

        return data

    # NOT USED BUT WORKS TO JUST PUBLISH CameraInfo
    # def publish_calibration_data(self):
    #     left_msg = self.generate_message(self.read_data(LEFT_CALIBRATION_PATH))
    #     right_msg = self.generate_message(self.read_data(RIGHT_CALIBRATION_PATH))
    #     bottom_msg = self.generate_message(self.read_data(BOTTOM_CALIBRATION_PATH))

    #     self.publisher_left.publish(left_msg)
    #     self.publisher_right.publish(right_msg)
    #     self.publisher_btm.publish(bottom_msg)

    #     self.get_logger().info(f"left_calibration_data: {left_msg}")
    #     self.get_logger().info(f"right_calibration_data: {right_msg}")
    #     self.get_logger().info(f"bottom_calibration_data: {bottom_msg}")

    # def generate_message(self, data):
    #     calibration_msg = CameraInfo()
    #     # not sure if the time is right
    #     calibration_msg.header.stamp = self.get_clock().now().to_msg()
    #     calibration_msg.height = data["height"]
    #     calibration_msg.width = data["width"]
    #     calibration_msg.distortion_model = data["distortion_model"]
    #     calibration_msg.d = data["d"]
    #     calibration_msg.k = data["k"]
    #     calibration_msg.r = data["r"]
    #     calibration_msg.p = data["p"]

    #     return calibration_msg


def main(args=None):
    rclpy.init(args=args)
    calibration_node = CalibrationNode()

    rclpy.spin(calibration_node)
    calibration_node.destroy_node()
    rclpy.shutdown()
