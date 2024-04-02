import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32MultiArray

SETTINGS = {
    "LOWER_HUE": 0,
    "UPPER_HUE": 255,
    "LOWER_SATURATION": 0,
    "UPPER_SATURATION": 255,
    "LOWER_VALUE": 0,
    "UPPER_VALUE": 255,
}


class orangeflare_detector(Node):
    def __init__(self):
        super().__init__("orangeflare_detector")
        self.declare_parameter(
            "settings_file_path",
            value="/home/aa/poolTest_ws/src/camera_ws/camera/hsv_bounds/hsv_flare.txt",
        )

        # use the launch file to decalare the files & namespace
        namespace = self.get_namespace()
        self.settings_path = (
            self.get_parameter("settings_file_path").get_parameter_value().string_value
        )
        self.read_settings()

        # Create publishers for debugging
        self.bgr_pub = self.create_publisher(
            CompressedImage, "orange_flare/bgr/compressed", 10
        )
        self.hsv_pub = self.create_publisher(
            CompressedImage, "orange_flare/hsv/compressed", 10
        )
        self.range_pub = self.create_publisher(
            CompressedImage, "orange_flare/range/compressed", 10
        )
        self.mask_pub = self.create_publisher(
            CompressedImage, "orange_flare/mask/compressed", 10
        )
        self.final_mask_pub = self.create_publisher(
            CompressedImage, "orange_flare/finalmask/compressed", 10
        )
        self.bbox_pub = self.create_publisher(
            Int32MultiArray, "/object/orange_flare/box", 10
        )
        self.front_image_feed = self.create_subscription(
            Image,
            "bottom/rect/image",
            self.image_feed_callback,
            10,
            # Image, "bottom/rect/image", self.image_feed_callback, 10
        )
        # self.front_image_feed = self.create_subscription(
        #     CompressedImage,
        #     "bgr/compressed",
        #     self.image_feed_callback,
        #     10)
        self.bridge = CvBridge()

    def image_feed_callback(self, msg):
        # Update the HSV bounds
        lower_bound = np.array(
            [
                SETTINGS["LOWER_HUE"],
                SETTINGS["LOWER_SATURATION"],
                SETTINGS["LOWER_VALUE"],
            ]
        )
        upper_bound = np.array(
            [
                SETTINGS["UPPER_HUE"],
                SETTINGS["UPPER_SATURATION"],
                SETTINGS["UPPER_VALUE"],
            ]
        )

        # bridge is from cv_bridge (for converting ROS image/CompressedImage to opencv images)
        cv_img = self.bridge.imgmsg_to_cv2(msg)
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        bgr_msg = self.bridge.cv2_to_compressed_imgmsg(cv_img)
        self.bgr_pub.publish(bgr_msg)
        # blur the image to remove noise using a 9x9 kernel
        cv_img = cv2.GaussianBlur(cv_img, (3, 3), 0)

        # Convert to HSV
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2HSV)
        hsv_msg = self.bridge.cv2_to_compressed_imgmsg(hsv_img)
        self.hsv_pub.publish(hsv_msg)

        # Threshold the HSV image to get yellow regions
        range_cv_img = cv2.inRange(hsv_img, lower_bound, upper_bound)
        range_msg = self.bridge.cv2_to_compressed_imgmsg(range_cv_img)
        self.range_pub.publish(range_msg)

        # Segment out gate regions using the mask
        kernel1 = np.ones((9, 9), np.uint8)
        kernel2 = np.ones((3, 3), np.uint8)

        # Morphological operations to remove noise
        eroded = cv2.erode(range_cv_img, kernel2, iterations=1)
        dilated = cv2.dilate(eroded, kernel1, iterations=4)
        eroded2 = cv2.erode(dilated, kernel2, iterations=2)

        gate_regions = cv2.bitwise_and(cv_img, cv_img, mask=eroded2)
        mask_img = self.bridge.cv2_to_compressed_imgmsg(gate_regions)
        self.mask_pub.publish(mask_img)

        # Draw Contours and Bounding Box for Gate
        self.draw_contour(dilated, cv_img)
        final_mask_msg = self.bridge.cv2_to_compressed_imgmsg(cv_img)
        self.final_mask_pub.publish(final_mask_msg)

    def draw_contour(self, gray_img, cv_img):
        # find and draw contour
        contours, hierarchy = cv2.findContours(
            gray_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        # Draw contours with green
        cv2.drawContours(cv_img, contours, -1, (0, 255, 0), 2)

        # draw bounding box
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            # draw bounding box only if area is above area threshold
            if cv2.contourArea(c) > 100:
                # draw bounding box
                cv2.rectangle(cv_img, (x, y), (x + w, y + h), (0, 0, 128), 3)

                # get midpoint of contour
                M = cv2.moments(c)

                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0

                # draw centroid
                cv2.circle(cv_img, (cX, cY), 5, (0, 0, 255), -1)
                cv2.putText(
                    cv_img,
                    "orange_flare",
                    (cX - 25, cY - 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (112, 232, 100),
                    2,
                )

                msg = Int32MultiArray()
                msg.data = [cX, cY, w, h]
                self.bbox_pub.publish(msg)

    def read_settings(self):
        keys = SETTINGS.keys()
        try:
            with open(self.settings_path, "r") as f:
                for key in keys:
                    val = f.readline()
                    SETTINGS[key] = int(val)
        except FileNotFoundError:
            raise FileNotFoundError


def main(args=None):
    rclpy.init(args=args)
    orange_flare_detector = orangeflare_detector()
    rclpy.spin(orange_flare_detector)
    orange_flare_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
