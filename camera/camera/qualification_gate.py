import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import threading

# This node is run in conjunction with the dehaze node

# This node detects for qualification gate (orange/yellow poles) 
# and publishes the bearing and distance to the gate [not properly implemented yet]

# Slider Window for HSV Bounds is created upon node creation

# Initial values (to detect yellow/orange)
lower_hue = 12
upper_hue = 28
lower_saturation = 131
upper_saturation = 255
lower_value = 125
upper_value = 255

# Value for red (cone)
# lower_hue = 0
# upper_hue = 12
# lower_saturation = 141
# upper_saturation = 255
# lower_value = 50
# upper_value = 255

sensor_width_in_mm = 3.674 # mm
sensor_resolution_width = 3264 #3280 # pixels
physical_width_resolution = 3.6656 # mm
actual_focal_length = 3.2 # mm
sensor_width_in_pixels = 3264 # pixels


horizontal_field_of_view = 137 #96.2 #62.2 # degrees
 
object_width_in_m = 0.37 # m (EDIT THIS ACCORDING TO THE WIDTH OF THE GATE/OBJECT)
focal_length_in_pixels = sensor_width_in_pixels / (2 * np.tan(np.radians(horizontal_field_of_view/ 2)))



class Gate_Detector(Node):
    def __init__(self):
        super().__init__("qualification_gate_detector")

        # Create sliders
        threading.Thread(target=create_gui, daemon=True).start()
        
        # Create publishers for debugging
        self.pub_gate_img = self.create_publisher(CompressedImage, "/gate/compressed", 10)
        self.gray_pub = self.create_publisher(CompressedImage, "/gate/gray/compressed", 10)
        self.blur_pub = self.create_publisher(CompressedImage, "/gate/blur/compressed", 10)
        self.hsv_pub = self.create_publisher(CompressedImage, "/gate/hsv/compressed", 10)
        self.range_pub = self.create_publisher(CompressedImage, "/gate/range/compressed", 10)
        self.red_mask_pub = self.create_publisher(CompressedImage, "/gate/red/compressed", 10)
        self.mask_pub = self.create_publisher(CompressedImage, "/gate/mask/compressed", 10)
        self.final_mask_pub = self.create_publisher(CompressedImage, "/gate/finalmask/compressed", 10)

        self.front_image_feed = self.create_subscription(
            CompressedImage,
            "gray_world/compressed",
            self.image_feed_callback,
            10)
        self.bridge = CvBridge()
    

    def image_feed_callback(self, msg):
        # Update the HSV bounds
        lower_yellow = np.array([lower_hue, lower_saturation, lower_value])
        upper_yellow = np.array([upper_hue, upper_saturation, upper_value])

        # bridge is from cv_bridge (for converting ROS image/CompressedImage to opencv images)
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)

        # # reduce noice by applying blur
        blur = cv2.GaussianBlur(cv_img, (9,9), 0)

        blur_msg = self.bridge.cv2_to_compressed_imgmsg(blur)
        self.blur_pub.publish(blur_msg)

        # Convert to HSV
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        hsv_msg = self.bridge.cv2_to_compressed_imgmsg(hsv_img)
        self.hsv_pub.publish(hsv_msg)
        
        # Threshold the HSV image to get yellow regions
        gate_mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
        range_msg = self.bridge.cv2_to_compressed_imgmsg(gate_mask)
        self.range_pub.publish(range_msg)

        # Segment out gate regions using the mask
        kernel1 = np.ones((9, 9), np.uint8)
        kernel2 = np.ones((3, 3), np.uint8)
        
        # Morphological operations to remove noise

        # Remove salt and pepper noise
        eroded = cv2.erode(gate_mask, kernel2, iterations=1)

        dilated = cv2.dilate(eroded, kernel1, iterations=3)
        # eroded = cv2.erode(gate_mask, kernel2, iterations=1)        
        #eroded2 = cv2.erode(dilated, kernel1, iterations=1)

        
        gate_regions = cv2.bitwise_and(cv_img, cv_img, mask=dilated)
        mask_img = self.bridge.cv2_to_compressed_imgmsg(gate_regions)
        self.mask_pub.publish(mask_img)

        # Draw Contours and Bounding Box for Gate
        self.draw_contour(dilated, cv_img)
        final_mask_msg = self.bridge.cv2_to_compressed_imgmsg(cv_img)
        self.final_mask_pub.publish(final_mask_msg)

    
    def draw_contour(self, gray_img, cv_img):

        
        # find and draw contour
        contours, hierarchy = cv2.findContours(gray_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Draw contours with green
        cv2.drawContours(cv_img, contours, -1, (0,255,0), 2)
    
        # draw bounding box
        for c in contours:
            x,y,w,h = cv2.boundingRect(c)
            # draw bounding box in dark red
            cv2.rectangle(cv_img,(x,y),(x+w,y+h),(0,0,128),3)
            # get midpoint of contour
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            cv2.circle(cv_img, (cX, cY), 5, (255, 255, 255), -1)
            cv2.putText(cv_img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 0), 2)

            cv2.putText(cv_img, "Image Center: {:.2f}".format(cv_img.shape[1]), (10, 70),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            # calculate yaw angle (currently yaw angle is done naively using x coordinate of centroid)

            # put hfov on image
            cv2.putText(cv_img, "HFOV: {:.2f}".format(horizontal_field_of_view), (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            # pute focal length in pixels on image
            cv2.putText(cv_img, "Focal Length (pixels): {:.2f}".format(focal_length_in_pixels), (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            yaw_angle = calculate_yaw_angle(cX, cv_img.shape[1])

            # put yaw angle on image 
            cv2.putText(cv_img, "Yaw Angle: {:.2f}".format(yaw_angle), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

            # calculate distance (calculation of distance is not properly implemented yet)
            distance = calculate_distance(w, object_width_in_m)

            # put distance on image
            cv2.putText(cv_img, "Distance (m): {:.4f}".format(distance), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2) 


def calculate_yaw_angle(centroid_x, image_width):
    # Calculate the yaw angle
    image_center_x = image_width / 2

    yaw_angle = np.degrees(np.arctan2(centroid_x - image_center_x, focal_length_in_pixels))
    return yaw_angle
    
def calculate_distance(object_width_in_pixels, actual_object_width):

    # convert distance to center in pixels to metres
    distance_to_object = (actual_object_width * focal_length_in_pixels) / object_width_in_pixels
    return distance_to_object

def create_gui():
    cv2.namedWindow('HSV Bounds')
    cv2.createTrackbar('Lower Hue', 'HSV Bounds', lower_hue, 255, update_bounds)
    cv2.createTrackbar('Upper Hue', 'HSV Bounds', upper_hue, 255, update_bounds)
    cv2.createTrackbar('Lower Saturation', 'HSV Bounds', lower_saturation, 255, update_bounds)
    cv2.createTrackbar('Upper Saturation', 'HSV Bounds', upper_saturation, 255, update_bounds)
    cv2.createTrackbar('Lower Value', 'HSV Bounds', lower_value, 255, update_bounds)
    cv2.createTrackbar('Upper Value', 'HSV Bounds', upper_value, 255, update_bounds)
    cv2.createTrackbar('HFOV', 'HSV Bounds', 10, 150, update_bounds)
    while True:
        cv2.waitKey(1)  

def update_bounds(val):
    global lower_hue, upper_hue, lower_saturation, upper_saturation, lower_value, upper_value, horizontal_field_of_view
    try:
        lower_hue = cv2.getTrackbarPos('Lower Hue', 'HSV Bounds')
    except cv2.error:
        pass
    try:
        upper_hue = cv2.getTrackbarPos('Upper Hue', 'HSV Bounds')
    except cv2.error:
        pass
    try:
        lower_saturation = cv2.getTrackbarPos('Lower Saturation', 'HSV Bounds')
    except cv2.error:
        pass
    try:
        upper_saturation = cv2.getTrackbarPos('Upper Saturation', 'HSV Bounds')
    except cv2.error:
        pass
    try:
        lower_value = cv2.getTrackbarPos('Lower Value', 'HSV Bounds')
    except cv2.error:
        pass
    try:
        upper_value = cv2.getTrackbarPos('Upper Value', 'HSV Bounds')
    except cv2.error:
        pass
    try:
        horizontal_field_of_view = cv2.getTrackbarPos('HFOV', 'HSV Bounds')
    except cv2.error:
        pass
def main(args=None):
    rclpy.init(args=args)
    gate_detector = Gate_Detector()
    rclpy.spin(gate_detector)
    gate_detector.destroy_node()
    rclpy.shutdown()
        
if __name__=='__main__':
    main()