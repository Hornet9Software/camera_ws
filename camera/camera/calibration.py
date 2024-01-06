import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

# might need to edit the path name
LEFT_CALIBRATION_PATH = "camera/calibration/calibrationdata/left.yaml"
RIGHT_CALIBRATION_PATH = "camera/calibration/calibrationdata/right.yaml"
BOTTOM_CALIBRATION_PATH = "camera/calibration/calibrationdata/bottom.yaml"


class CalibrationNode(Node):
    def __init__(self):
        super.__init__("calibrator node")
        self.publisher_left = self.create_publisher(CameraInfo, "/left/calibration", 10)
        self.publisher_right = self.create_publisher(
            CameraInfo, "/right/calibration", 10
        )
        self.publisher_btm = self.create_publisher(
            CameraInfo, "/bottom/calibration", 10
        )

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_calibration_data)

    def publish_calibration_data(self):
        left_msg = self.generate_message(self.read_data(LEFT_CALIBRATION_PATH))
        right_msg = self.generate_message(self.read_data(RIGHT_CALIBRATION_PATH))
        bottom_msg = self.generate_message(self.read_data(BOTTOM_CALIBRATION_PATH))

        self.publisher_left.publish(self.generate_message(left_msg))
        self.publisher_right.publish(self.generate_message(right_msg))
        self.publisher_btm.publish(self.generate_message(bottom_msg))

        self.get_logger().info(f"left_calibration_data: {left_msg}")
        self.get_logger().info(f"right_calibration_data: {right_msg}")
        self.get_logger().info(f"bottom_calibration_data: {bottom_msg}")

    def read_data(self, path):
        data = {
            "name": None,
            "height": None,
            "width": None,
            "model": None,
            "D": None,
            "K": None,
            "R": None,
            "P": None,
        }

        with open(path, "r") as file:
            contents = yaml.safe_load(file)
            data["name"] = contents["camera_name"]
            data["height"] = contents["image_height"]
            data["width"] = contents["image_width"]
            data["model"] = contents["distortion_model"]
            data["D"] = contents["distortion_coefficients"]["data"]
            data["K"] = contents["camera_matrix"]["data"]
            data["R"] = contents["rectification_matrix"]["data"]
            data["P"] = contents["projection_matrix"]["data"]

        return data

    def generate_message(self, data):
        calibration_msg = CameraInfo()

        # not sure if the time is right
        calibration_msg.header.stamp = self.get_clock().now().to_msg()
        calibration_msg.height = data["height"]
        calibration_msg.width = data["width"]
        calibration_msg.distortion_model = data["model"]
        calibration_msg.distortion_coefficients = data["D"]
        calibration_msg.K = data["K"]
        calibration_msg.R = data["R"]
        calibration_msg.p = data["P"]

        return calibration_msg


def main(args=None):
    # reads the calibration data and publishes CameraInfo for other noddes to calibrate with
    calibration_node = CalibrationNode()

    rclpy.init(args=args)

    rclpy.spin(calibration_node)
    calibration_node.destroy_node()
    rclpy.shutdown()
