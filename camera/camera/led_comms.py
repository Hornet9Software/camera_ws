import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

# import threading

AMBERSETTINGS = {
    "LOWER_HUE": 0,
    "UPPER_HUE": 255,
    "LOWER_SATURATION": 0,
    "UPPER_SATURATION": 255,
    "LOWER_VALUE": 0,
    "UPPER_VALUE": 255,
}

BLUESETTINGS = {
    "LOWER_HUE": 0,
    "UPPER_HUE": 255,
    "LOWER_SATURATION": 0,
    "UPPER_SATURATION": 255,
    "LOWER_VALUE": 0,
    "UPPER_VALUE": 255,
}

GREENSETTINGS = {
    "LOWER_HUE": 0,
    "UPPER_HUE": 255,
    "LOWER_SATURATION": 0,
    "UPPER_SATURATION": 255,
    "LOWER_VALUE": 0,
    "UPPER_VALUE": 255,
}

ORDER_MAPPINGS = {
    "R": "RYB",
    "G": "RBY",
    "B": "BRY",
    "RB": "BYR",
    "RG": "YRB",
    "GB": "YBR",
    "-": "",
    "RGB": "",
}


class CommsDetector(Node):
    def __init__(self):
        super().__init__("comms_detector")
        self.declare_parameter(
            "amber_led_file_path",
            value="/home/aa/poolTest_ws/src/camera_ws/camera/hsv_bounds/led_red.txt",
        )
        self.declare_parameter(
            "blue_led_file_path",
            value="/home/aa/poolTest_ws/src/camera_ws/camera/hsv_bounds/led_blue.txt",
        )
        self.declare_parameter(
            "green_led_file_path",
            value="/home/aa/poolTest_ws/src/camera_ws/camera/hsv_bounds/led_green.txt",
        )

        # use the launch file to decalare the files & namespace
        namespace = self.get_namespace()
        self.amber_settings_path = (
            self.get_parameter("amber_led_file_path").get_parameter_value().string_value
        )
        self.blue_settings_path = (
            self.get_parameter("blue_led_file_path").get_parameter_value().string_value
        )
        self.green_settings_path = (
            self.get_parameter("green_led_file_path").get_parameter_value().string_value
        )
        self.read_settings()

        self.amber_lower_bound = np.array(
            [
                AMBERSETTINGS["LOWER_HUE"],
                AMBERSETTINGS["LOWER_SATURATION"],
                AMBERSETTINGS["LOWER_VALUE"],
            ]
        )
        self.amber_upper_bound = np.array(
            [
                AMBERSETTINGS["UPPER_HUE"],
                AMBERSETTINGS["UPPER_SATURATION"],
                AMBERSETTINGS["UPPER_VALUE"],
            ]
        )
        self.blue_lower_bound = np.array(
            [
                BLUESETTINGS["LOWER_HUE"],
                BLUESETTINGS["LOWER_SATURATION"],
                BLUESETTINGS["LOWER_VALUE"],
            ]
        )
        self.blue_upper_bound = np.array(
            [
                BLUESETTINGS["UPPER_HUE"],
                BLUESETTINGS["UPPER_SATURATION"],
                BLUESETTINGS["UPPER_VALUE"],
            ]
        )
        self.green_lower_bound = np.array(
            [
                GREENSETTINGS["LOWER_HUE"],
                GREENSETTINGS["LOWER_SATURATION"],
                GREENSETTINGS["LOWER_VALUE"],
            ]
        )
        self.green_upper_bound = np.array(
            [
                GREENSETTINGS["UPPER_HUE"],
                GREENSETTINGS["UPPER_SATURATION"],
                GREENSETTINGS["UPPER_VALUE"],
            ]
        )

        # Segment out gate regions using the mask
        # this is q general may need to change below if specific bounds needed
        self.kernel1 = np.ones((5, 5), np.uint8)
        self.kernel2 = np.ones((3, 3), np.uint8)

        # Create publishers for debugging
        # self.bgr_pub = self.create_publisher(CompressedImage, "led/bgr/compressed", 10)
        # self.hsv_pub = self.create_publisher(CompressedImage, "led/hsv/compressed", 10)
        # self.inrange_pub = self.create_publisher(CompressedImage, "led/inrange/compressed", 10)
        # self.mask_pub = self.create_publisher(CompressedImage, "led/mask/compressed", 10)
        self.final_mask_pub = self.create_publisher(
            CompressedImage, "/led/finalmask/compressed", 10
        )  # to visualise the bounding box

        self.comms_color_pub = self.create_publisher(String, "/led/order", 10)

        self.front_image_feed = self.create_subscription(
            Image,
            "bottom/rect/image",  # to be changed do in the launch file
            self.image_feed_callback,
            10,
        )
        ################ Use this when Testing with Bag ###################
        # self.front_image_feed = self.create_subscription(
        #     CompressedImage,
        #     "bgr/compressed", # to be changed do in the launch file
        #     self.image_feed_callback,
        #     10)
        self.bridge = CvBridge()

    def image_feed_callback(self, msg):
        # Update the HSV bounds

        # bridge is from cv_bridge (for converting ROS image/CompressedImage to opencv images)
        cv_img = self.bridge.imgmsg_to_cv2(msg)
        # dont convert then crop does weird stuff
        cv_img = cv_img[100:300, 0:640]
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGBA2RGB)
        # cv_img = cv_img[0:300, 0:640]
        # cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

        # bgr_msg = self.bridge.cv2_to_compressed_imgmsg(cv_img)
        # self.bgr_pub.publish(bgr_msg)
        # blur the image to remove noise using a 9x9 kernel
        cv_img = cv2.GaussianBlur(cv_img, (5, 5), 0)  # check this see if needed

        # Convert to HSV
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2HSV)
        # hsv_msg = self.bridge.cv2_to_compressed_imgmsg(hsv_img)
        # self.hsv_pub.publish(hsv_msg)

        color_amber = self.detect_amber(hsv_img, cv_img)
        color_blue = self.detect_blue(hsv_img, cv_img)
        color_green = self.detect_green(hsv_img, cv_img)
        # color_amber = ""
        # color_green = ""
        order_msg = ""

        if color_amber == "amber":
            order_msg = order_msg + "R"

        if color_green == "green":
            order_msg = order_msg + "G"

        if color_blue == "blue":
            order_msg = order_msg + "B"

        if order_msg == "":
            order_msg = "-"
            # print(color_amber + ' ' + color_blue + ' ' + color_green)

        mapped_order = String()
        mapped_order.data = ORDER_MAPPINGS[order_msg]
        if mapped_order.data != "":
            self.comms_color_pub.publish(mapped_order)

    def detect_amber(self, hsv_img, cv_img):
        # Threshold the HSV image to get amber regions
        amber_range_cv_img = cv2.inRange(
            hsv_img, self.amber_lower_bound, self.amber_upper_bound
        )
        # amber_range_msg = self.bridge.cv2_to_compressed_imgmsg(amber_range_cv_img)
        # self.inrange_pub.publish(amber_range_msg)

        # Morphological operations to remove noise
        # amber_dilated = cv2.dilate(amber_range_cv_img, self.kernel1, iterations=2)
        amber_dilated = cv2.morphologyEx(
            amber_range_cv_img, cv2.MORPH_CLOSE, self.kernel1, iterations=3
        )

        # Visualisation
        # amber_led_regions = cv2.bitwise_and(cv_img, cv_img, mask=amber_dilated)
        # amber_mask_img = self.bridge.cv2_to_compressed_imgmsg(amber_led_regions)
        # self.mask_pub.publish(amber_mask_img)

        # Draw Contours and Bounding Box for amber LED
        return self.draw_contour(0, amber_dilated, cv_img)

        # final_mask_msg = self.bridge.cv2_to_compressed_imgmsg(cv_img)
        # self.final_mask_pub.publish(final_mask_msg)

    def detect_blue(self, hsv_img, cv_img):
        # hsv mask for blue
        blue_range_cv_img = cv2.inRange(
            hsv_img, self.blue_lower_bound, self.blue_upper_bound
        )
        # blue_range_msg = self.bridge.cv2_to_compressed_imgmsg(blue_range_cv_img)
        # self.range_pub.publish(blue_range_msg)

        # Morphological operations to remove noise
        # blue_dilated = cv2.dilate(blue_range_cv_img, self.kernel1, iterations=5)
        # dilated_image = cv2.dilate(blue_range_cv_img, self.kernel1, iterations=2)
        blue_dilated = cv2.morphologyEx(
            blue_range_cv_img, cv2.MORPH_CLOSE, self.kernel1, iterations=3
        )

        # Visualisation
        # blue_led_regions = cv2.bitwise_and(cv_img, cv_img, mask=dilated_image)
        # blue_mask_img = self.bridge.cv2_to_compressed_imgmsg(blue_led_regions)
        # self.mask_pub.publish(blue_mask_img)

        # Draw Contours and Bounding Box for Blue LED
        return self.draw_contour(1, blue_dilated, cv_img)

    def detect_green(self, hsv_img, cv_img):
        # hsv mask for green
        green_range_cv_img = cv2.inRange(
            hsv_img, self.green_lower_bound, self.green_upper_bound
        )
        # green_range_msg = self.bridge.cv2_to_compressed_imgmsg(green_range_cv_img)
        # self.range_pub.publish(green_range_msg)

        # Morphological operations to remove noise
        # green_dilated = cv2.dilate(green_range_cv_img, self.kernel1, iterations=2)
        green_dilated = cv2.morphologyEx(
            green_range_cv_img, cv2.MORPH_CLOSE, self.kernel1, iterations=3
        )
        # Visualisation
        # green_led_regions = cv2.bitwise_and(cv_img, cv_img, mask=green_dilated)
        # blue_mask_img = self.bridge.cv2_to_compressed_imgmsg(blue_led_regions)
        # self.mask_pub.publish(blue_mask_img)

        # Draw Contours and Bounding Box for Blue LED
        return self.draw_contour(2, green_dilated, cv_img)

    def draw_contour(self, color_id, bin_img, cv_img):
        # find and draw contour
        contours, hierarchy = cv2.findContours(
            bin_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        # Draw contours with green
        cv2.drawContours(cv_img, contours, -1, (0, 255, 0), 2)
        label = ""

        # draw bounding box
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            # draw bounding box only if area is above area threshold
            # currently threshold set to be very tight but if loosen uncomment this n try
            # aspect_ratio = float(w)/h
            # if not ((cv2.contourArea(c) > 30) and (aspect_ratio < 1.2 and aspect_ratio > 0.8)):
            # continue

            if color_id == 0:
                label = "amber"
                colour = (0, 0, 128)
            elif color_id == 1:
                label = "blue"
                colour = (0, 128, 0)
            elif color_id == 2:
                label = "green"
                colour = (128, 0, 0)

            # draw bounding box
            cv2.rectangle(cv_img, (x, y), (x + w, y + h), colour, 3)

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
                label,
                (cX - 25, cY - 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (112, 232, 100),
                2,
            )

        final_img = self.bridge.cv2_to_compressed_imgmsg(cv_img)
        self.final_mask_pub.publish(final_img)
        return label

    def read_settings(self):
        keys = AMBERSETTINGS.keys()
        try:
            with open(self.amber_settings_path, "r") as f:
                for key in keys:
                    val = f.readline()
                    AMBERSETTINGS[key] = int(val)

            keys = BLUESETTINGS.keys()
            with open(self.blue_settings_path, "r") as f:
                for key in keys:
                    val = f.readline()
                    BLUESETTINGS[key] = int(val)

            keys = GREENSETTINGS.keys()
            with open(self.green_settings_path, "r") as f:
                for key in keys:
                    val = f.readline()
                    GREENSETTINGS[key] = int(val)

        except FileNotFoundError:
            raise FileNotFoundError


def main(args=None):
    rclpy.init(args=args)
    comms_detection = CommsDetector()
    rclpy.spin(comms_detection)
    comms_detection.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
