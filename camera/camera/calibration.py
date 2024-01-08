import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

# might need to edit the path name
LEFT_CALIBRATION_PATH = "src/camera_ws/camera/calibration/calibrationdata/left.yaml"
RIGHT_CALIBRATION_PATH = "src/camera_ws/camera/calibration/calibrationdata/right.yaml"
BOTTOM_CALIBRATION_PATH = "src/camera_ws/camera/calibration/calibrationdata/bottom.yaml"


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("calibrator_node")
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
            data["height"] = int(contents["image_height"])
            data["width"] = int(contents["image_width"])
            data["distortion_model"] = str(contents["distortion_model"])
            data["d"] = list(contents["distortion_coefficients"]["data"])
            data["k"] = list(contents["camera_matrix"]["data"])
            data["r"] = list(contents["rectification_matrix"]["data"])
            data["p"] = list(contents["projection_matrix"]["data"])
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
