# Code from curtis to detect red color
# Node looks at bottom camera and detects red color for pail detection

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge

# red = np.uint8([[[40, 120, 255]]])
# # here insert the bgr values which you want to convert to hsv
# bgr2 = np.uint8([[[40,120,255]]])
# bgr = np.uint8([[[38,0,217]]])
# upperLimit = bgr[48][255][255] 
# lowerLimit = bgr[48][100][100]

class Detector(Node):

    def __init__(self):
        super().__init__("gate_detector")
        self.pub_hsv_img = self.create_publisher(CompressedImage, "hsv/compressed", 10)
        self.contour_pub = self.create_publisher(CompressedImage, "contour/compressed", 10)
        # self.blur_pub = self.create_publisher(Image, "blur", 10)
        self.rawmask_pub = self.create_publisher(CompressedImage, "rawmask/compressed", 10)
        self.mask_pub = self.create_publisher(CompressedImage, "mask/compressed", 10)
        # self.threshold_red_pub = self.create_publisher(Image, "threshold_red", 10)
        self.input_pub = self.create_publisher(CompressedImage, "input/compressed", 10)
        self.front_image_feed = self.create_subscription(
            CompressedImage,
            "camera_down/image_raw/compressed",
            # "/Hornet/Cam/Floor/image_rect_color/compressed",
            self.image_feed_callback,
            10)
        self.bridge = CvBridge()

    def image_feed_callback(self, msg):

        # bridge is from cv_bridge (for converting ROS image/CompressedImage to opencv images)
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        input_msg = self.bridge.cv2_to_compressed_imgmsg(cv_img)
        self.input_pub.publish(input_msg)
        s = ColorDetector()
        final_img, contours, mask, hsv, mask_result = s.detectColor(cv_img)

        # classified_img = contour_loop(contours, final_img)
        
        mask_img = self.bridge.cv2_to_compressed_imgmsg(mask)
        hsv_img = self.bridge.cv2_to_compressed_imgmsg(hsv)
        mask_result_img = self.bridge.cv2_to_compressed_imgmsg(mask_result)
        final_msg = self.bridge.cv2_to_compressed_imgmsg(final_img) 
        # Convert final image (cv_img) and publish
        self.mask_pub.publish(mask_img)
        self.contour_pub.publish(final_msg)
        self.rawmask_pub.publish(mask_result_img)
        self.pub_hsv_img.publish(hsv_img)        


class ColorDetector: 
    def __init__(self): 
        pass
    def detectColor(self, img): 
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 50, 0])
        upper_red = np.array([60, 255, 170])
        
        mask = cv2.inRange(hsv_img, lower_red, upper_red)
        result = cv2.bitwise_and(img, img, mask = mask)
        kernel = np.ones((10,10), np.uint8)
        erosion = cv2.erode(result, kernel, iterations = 7)
        dilation = cv2.dilate(erosion, kernel, iterations = 5)
        


        img_gray = cv2.cvtColor(dilation, cv2.COLOR_BGR2GRAY) #convert the image to grayscale, this is a requirement bfr thresholding
        _, thresh = cv2.threshold(img_gray, 70, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(img_gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        mask
        cv2.drawContours(img, contours, -1 , (0, 255, 0), 3) #green contours
        for c in contours: 
            contour_area = cv2.contourArea(c)
            if (contour_area > 4000):
                rect = cv2.boundingRect(c)
                x, y, w, h = rect 
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3)
                cv2.putText(img, str(contour_area), (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        return img, contours, mask, hsv_img, result

class ShapeDetector: 
    def __init__(self):
        pass 
    def detect(self, c): 
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        # if the shape is a triangle, it will have 3 vertices
        if len(approx) == 3:
            shape = "triangle"
        # if the shape has 4 vertices, it is either a square or
        # a rectangle
        elif len(approx) == 4:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)
            # a square will have an aspect ratio that is approximately
            # equal to one, otherwise, the shape is a rectangle
            shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
        # if the shape is a pentagon, it will have 5 vertices
        elif len(approx) == 5:
            shape = "pentagon"
        # otherwise, we assume the shape is a circle
        else:
            shape = "circle"
        # return the name of the shape
        return shape

# Define a function to calculate the distance between two points
def calculate_distance(point1, point2):
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

def contour_loop(contour, image):
    sd = ShapeDetector()
    # print(len(contour))
    Arr = []
    
    for c in contour:
        # These 3 lines find the (x,y) for the centre of an object
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        
        # Check if the new (cX, cY) is within a 10-pixel radius of any point in Arr
        is_duplicate = False
        for point in Arr:
            if calculate_distance((cX, cY), point) < 10:
                is_duplicate = True
                break

        if not is_duplicate:
            Arr.append((cX, cY))
            shape = sd.detect(c)
            print(shape)
            cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    return image

def main(args=None):
    rclpy.init(args=args)
    detector = Detector()
    rclpy.spin(detector)

    # Below lines are not strictly necessary
    detector.destroy_node()
    rclpy.shutdown()
        
if __name__=='__main__':
    main()