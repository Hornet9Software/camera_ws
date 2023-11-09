import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge

class Detector(Node):

    def __init__(self):
        super().__init__("gate_detector")
        self.pub_hsv_img = self.create_publisher(CompressedImage, "/detect/gate/compressed", 10)
        # self.hsv_pub = self.create_publisher(Image, "hsv_frame", 10)
        # self.blur_pub = self.create_publisher(Image, "blur", 10)
        # self.rawmask_pub = self.create_publisher(Image, "rawmask", 10)
        # self.mask_pub = self.create_publisher(Image, "mask", 10)
        # self.threshold_red_pub = self.create_publisher(Image, "threshold_red", 10)

        self.front_image_feed = self.create_subscription(
            CompressedImage,
            "/Hornet/Cam/left/image_rect_color/compressed",
            self.image_feed_callback,
            10)
        self.bridge = CvBridge()

    def image_feed_callback(self, msg):

        # bridge is from cv_bridge (for converting ROS image/CompressedImage to opencv images)
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        
        # # reduce noice by applying blur
        kernel = np.ones((3,3),np.uint8)
        blur = cv2.GaussianBlur(cv_img, (3,3), cv2.BORDER_DEFAULT)

        # # convert color space to HSV
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        # red_segment = self.find_red(cv_img, hsv, kernel)

        # # convert bgr to greyscale so we can find contours
        # gray_img = cv2.cvtColor(red_segment, cv2.COLOR_BGR2GRAY)
        # self.draw_contour(gray_img, cv_img)

        # Publish hsv image 
        # hsv_msg = self.bridge.cv2_to_imgmsg(hsv, encoding="bgr8") 
        # self.hsv_pub.publish(hsv_msg)

        # Convert final image (cv_img) and publish
        final_msg = self.bridge.cv2_to_compressed_imgmsg(hsv) 
        self.pub_hsv_img.publish(final_msg)

    # def find_red(self, cv_img, hsv, kernel):
        # color thresholding to find red

        # define range of red color in HSV and get a mask
        # lower_red = np.array([160,50,30]) 
        # upper_red = np.array([255,210,255]) 
         
        # mask = cv2.inRange(hsv, lower_red, upper_red)

        # morphological operations (dilution / erosion / opening / closing)
        # dilation = cv2.dilate(mask,kernel,iterations = 13)
        # erode = cv2.erode(dilation,kernel,iterations = 15)

        # red_segment = cv2.bitwise_and(cv_img, cv_img, mask=erode)

        # publish raw mask, mask after morphological operations and segmented red portions 
        # rawmask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8") 
        # self.rawmask_pub.publish(rawmask_msg)

        # mask_msg = self.bridge.cv2_to_imgmsg(erode, encoding="mono8") 
        # self.mask_pub.publish(mask_msg)

        # threshold_red_msg = self.bridge.cv2_to_imgmsg(red_segment, encoding="bgr8") 
        # self.threshold_red_pub.publish(threshold_red_msg)

        # return red_segment
    
    # def draw_contour(self, gray_img, cv_img):

    #     # find and draw contour
    #     contours, hierarchy = cv2.findContours(gray_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #     cv2.drawContours(cv_img, contours, -1, (0,255,0), 2)
    
    #     # draw bounding box

    #     for contour in contours:
    #         # get contour area
    #         contour_area = cv2.contourArea(contour)
    #         # if contour area is above 2400, it is a region of interest
    #         if contour_area > 2400:

    #             # Get the coordinates for corners of bounding rectangle
    #             x,y,w,h = cv2.boundingRect(contour)

    #             # Draw rectangle using coordinates
    #             cv2.rectangle(cv_img, (x, y), (x + w, y + h), (0,0,255), 4)

    #             # Put "Red" text
    #             cv2.putText(cv_img, "Red", (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
    #             # Put text for area of contour
    #             # cv2.putText(cv_img, f"Area: {contour_area:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

def main(args=None):
    rclpy.init(args=args)
    detector = Detector()
    rclpy.spin(detector)

    # Below lines are not strictly necessary
    detector.destroy_node()
    rclpy.shutdown()
        
if __name__=='__main__':
    main()