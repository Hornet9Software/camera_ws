import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from skimage import filters
# import math (for standard Hough line transformation)
import collections


class FloorTilesNode(Node):
    def __init__(self):
        super().__init__('floor_tiles_node')

        self.tile_img_pub = self.create_publisher(CompressedImage, 'compressed_image_topic', 10)
        self.angle1 = self.create_publisher(Float32, '/angle1', 10)
        self.angle2 = self.create_publisher(Float32, '/angle2', 10)
        self.bottom_image_feed = self.create_subscription(
            CompressedImage,
            'compressed_image_topic',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert compressed image to OpenCV image
        img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Convert to grayscale
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # CLAHE (Contrast Limited Adaptive Histogram Equalization)
        clahe = cv2.createCLAHE(clipLimit=1.2, tileGridSize=(12,12))
        cl1 = clahe.apply(imgGray)

        # Perform Otsu adaptive thresholding
        thresholds = filters.threshold_multiotsu(cl1, classes=2)

        # regions = np.digitize(cl1, bins=thresholds)

        # Perform Thresholding
        threshold = (cl1 > thresholds[0])
        thresholded_img = threshold.astype(np.uint8) * 255

        # Perform Canny edge detection
        edges = cv2.Canny(thresholded_img, 50, 200, None, 3)
        
        # Get Hough lines (used for standard Hough line transformation) 
        # lines = cv2.HoughLines(edges, 1, np.pi / 180, 150, None, 0, 0)
        
        
        cdstP = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        # cdst is used for standard Hough line transformation
        # cdst = np.copy(cdstP)

        # Perform Standard Hough line transformation
        # if lines is not None:
        #     for i in range(0, len(lines)):
                
        #         rho = lines[i][0][0]
        #         theta = lines[i][0][1]
        #         a = math.cos(theta)
        #         b = math.sin(theta)
        #         x0 = a * rho
        #         y0 = b * rho
        #         pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        #         pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        #         cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)

        # Perform Probabilistic Hough line transformation
        linesP = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, None, 50, 10)

        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

        thetas = []
        for line in linesP:
            line = line[0]
            thetas.append(np.arctan2(line[3]-line[1], line[2]-line[0]))

        # Count the occurrences of each angle in the list
        counter = collections.Counter(thetas)

        # Find the two most common angles
        common_angles = counter.most_common(2)

        # common_angles is a list of tuples, where each tuple contains an angle and its count
        # Get the angles from the tuples
        angle1 = np.degrees(common_angles[0][0])
        angle2 = np.degrees(common_angles[1][0])

        # Publish the angles
        self.angle1.publish(angle1)
        self.angle2.publish(angle2)

def main(args=None):
    rclpy.init(args=args)
    floor_tiles_node = FloorTilesNode()
    rclpy.spin(floor_tiles_node)
    floor_tiles_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
