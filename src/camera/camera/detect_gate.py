import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge

# Define lower and upper bounds for dark regions (adjust these values based on your image)
lower_dark = np.array([15, 98, 140])  # Lower bound for dark regions in HSV
upper_dark = np.array([48, 255, 255])  # Upper bound for dark regions in HSV


class Detector(Node):

    def __init__(self):
        super().__init__("gate_detector")
        self.pub_gate_img = self.create_publisher(CompressedImage, "/gate/compressed", 10)
        self.enhance_pub = self.create_publisher(CompressedImage, "/gate/enhance/compressed", 10)
        self.gray_pub = self.create_publisher(CompressedImage, "/gate/gray/compressed", 10)
        self.blur_pub = self.create_publisher(CompressedImage, "/gate/blur/compressed", 10)
        self.hsv_pub = self.create_publisher(CompressedImage, "/gate/hsv/compressed", 10)
        self.range_pub = self.create_publisher(CompressedImage, "/gate/range/compressed", 10)
        self.red_mask_pub = self.create_publisher(CompressedImage, "/gate/red/compressed", 10)
        self.mask_pub = self.create_publisher(CompressedImage, "/gate/mask/compressed", 10)
        self.final_mask_pub = self.create_publisher(CompressedImage, "/gate/finalmask/compressed", 10)
        

        self.front_image_feed = self.create_subscription(
            CompressedImage,
            "Hornet/Cam/left/image_rect_color/compressed",
            self.image_feed_callback,
            10)
        self.bridge = CvBridge()

    def image_feed_callback(self, msg):

        # bridge is from cv_bridge (for converting ROS image/CompressedImage to opencv images)
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)

        # Perform Gray World algorithm
        gray_world_image = self.perform_gray_world(cv_img)
        enhance_msg = self.bridge.cv2_to_compressed_imgmsg(gray_world_image)
        self.enhance_pub.publish(enhance_msg)


        # # reduce noice by applying blur
        blur = cv2.GaussianBlur(gray_world_image, (3,3), 0)
        # Apply median blur
        # median_blur = cv2.medianBlur(blur, 5)

        blur_msg = self.bridge.cv2_to_compressed_imgmsg(blur)
        self.blur_pub.publish(blur_msg)

        # Convert to HSV

        hsv_img = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        hsv_msg = self.bridge.cv2_to_compressed_imgmsg(hsv_img)
        self.hsv_pub.publish(hsv_msg)

        #
        # Threshold the HSV image to get dark regions
        gate_mask = cv2.inRange(hsv_img, lower_dark, upper_dark)
        range_msg = self.bridge.cv2_to_compressed_imgmsg(gate_mask)
        self.range_pub.publish(range_msg)

        # Segment out gate regions using the mask
        kernel1 = np.ones((9, 9), np.uint8)
        kernel2 = np.ones((5, 5), np.uint8)
        
        eroded = cv2.erode(gate_mask, kernel2, iterations=1)
        dilated = cv2.dilate(eroded, kernel1, iterations=2)
        gate_regions = cv2.bitwise_and(cv_img, cv_img, mask=dilated)
        mask_img = self.bridge.cv2_to_compressed_imgmsg(gate_regions)
        self.mask_pub.publish(mask_img)

        # Morphological operations to remove noise
        
        # final_mask_img = self.bridge.cv2_to_compressed_imgmsg(dilated)
        # self.final_mask_pub.publish(final_mask_img)

        # edges_image = self.detect_edges(blur)

        # edge_msg = self.bridge.cv2_to_compressed_imgmsg(edges_image)
        # self.edge_pub.publish(edge_msg)
        # # img_dilation = cv2.dilate(edges_image, kernel, iterations=7) 
        # # img_erosion = cv2.erode(img_dilation, kernel, iterations=6) 

        # # kernel = np.ones((5, 5), np.uint8)
        # # dilated = cv2.dilate(edges_image, kernel, iterations=1)
        # # removed_grids = cv2.bitwise_and(cv_img, cv_img, mask=cv2.bitwise_not(dilated))

        # # grid_msg = self.bridge.cv2_to_compressed_imgmsg(removed_grids)
        # # self.removed_grid_pub.publish(grid_msg)

        # # Color Segmentation - Example: Extract gate (assuming a distinct color)
        # lower = np.array([0, 0, 0])
        # upper = np.array([60, 70, 70])
        # mask_gate = cv2.inRange(cv_img, lower, upper)
        # gate_only = cv2.bitwise_and(cv_img, mask_gate, mask=mask_gate)



        # # self.draw_contour(blur, cv_img)


        # # Convert final image (cv_img) and publish
        # final_msg = self.bridge.cv2_to_compressed_imgmsg(gate_only) 
        # self.pub_gate_img.publish(final_msg)

    def perform_gray_world(self, image):
        # Convert image to float32 for accurate calculations
        image_float = image.astype(np.float32)
        
        # Calculate average values for each channel
        avg_r = np.mean(image_float[:, :, 0])
        avg_g = np.mean(image_float[:, :, 1])
        avg_b = np.mean(image_float[:, :, 2])

        # Calculate the average gray value
        avg_gray = (avg_r + avg_g + avg_b) / 3.0

        # Perform gray world algorithm to balance colors
        scale_factor = 1.5
        image_float[:, :, 0] *= (avg_gray / avg_r) * scale_factor
        image_float[:, :, 1] *= (avg_gray / avg_g) * scale_factor
        image_float[:, :, 2] *= (avg_gray / avg_b) * scale_factor

        # Clip the values to ensure they are within the valid range
        image_float = np.clip(image_float, 0, 255)
        
        # Convert back to uint8
        image_normalized = image_float.astype(np.uint8)
        
        return image_normalized


    def detect_edges(self, image):
        # Edge detection (e.g., Canny edge detection)
        edges_image = cv2.Canny(image, 100, 200)
        return edges_image
    
    def draw_contour(self, gray_img, cv_img):

        # find and draw contour
        contours, hierarchy = cv2.findContours(gray_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(cv_img, contours, -1, (0,255,0), 2)
    
        # draw bounding box

        

def main(args=None):
    rclpy.init(args=args)
    detector = Detector()
    rclpy.spin(detector)

    # Below lines are not strictly necessary
    detector.destroy_node()
    rclpy.shutdown()
        
if __name__=='__main__':
    main()