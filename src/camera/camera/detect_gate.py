import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge

class Detector(Node):

    def __init__(self):
        super().__init__("gate_detector")
        self.pub_gate_img = self.create_publisher(CompressedImage, "/gate/compressed", 10)
        self.blur_pub = self.create_publisher(CompressedImage, "/gate/blur/compressed", 10)
        self.edge_pub = self.create_publisher(CompressedImage, "/gate/edge/compressed", 10)
        self.removed_grid_pub = self.create_publisher(CompressedImage, "/gate/remove_grid/compressed", 10)
       
        self.front_image_feed = self.create_subscription(
            CompressedImage,
            "stereo/left/image_raw/compressed",
            self.image_feed_callback,
            10)
        self.bridge = CvBridge()

    def image_feed_callback(self, msg):

        # bridge is from cv_bridge (for converting ROS image/CompressedImage to opencv images)
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        
        # # reduce noice by applying blur

        blur = cv2.GaussianBlur(cv_img, (5,5), 0)

        blur_msg = self.bridge.cv2_to_compressed_imgmsg(blur)
        self.blur_pub.publish(blur_msg)

        edges_image = self.detect_edges(blur)

        edge_msg = self.bridge.cv2_to_compressed_imgmsg(edges_image)
        self.edge_pub.publish(edge_msg)
        # img_dilation = cv2.dilate(edges_image, kernel, iterations=7) 
        # img_erosion = cv2.erode(img_dilation, kernel, iterations=6) 

        kernel = np.ones((5, 5), np.uint8)
        dilated = cv2.dilate(edges_image, kernel, iterations=1)
        removed_grids = cv2.bitwise_and(cv_img, cv_img, mask=cv2.bitwise_not(dilated))

        grid_msg = self.bridge.cv2_to_compressed_imgmsg(removed_grids)
        self.removed_grid_pub.publish(grid_msg)

        # Color Segmentation - Example: Extract gate (assuming a distinct color)
        # lower = np.array([0, 0, 0])  # Define lower and upper bounds for gate color
        # upper = np.array([70, 70, 70])
        # mask_gate = cv2.inRange(removed_grids, lower, upper)
        # gate_only = cv2.bitwise_and(removed_grids, removed_grids, mask=mask_gate)



        # self.draw_contour(blur, cv_img)


        # Convert final image (cv_img) and publish
        # final_msg = self.bridge.cv2_to_compressed_imgmsg(gate_only) 
        # self.pub_gate_img.publish(final_msg)



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