import rclpy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ComputerVisionNode:
    def __init__(self):
        self.node = rclpy.create_node('balance')
        self.subscription = self.node.create_subscription(
            CompressedImage,
            'stereo/left/image_raw/compressed',  # Replace 'compressed_image_input' with your topic name
            self.image_callback,
            10
        )
        self.publisher = self.node.create_publisher(
            CompressedImage,
            'stereo/left/processed/compressed',  # Replace 'processed_image_output' with your desired output topic
            10
        )
        self.balance_publisher = self.node.create_publisher(
            CompressedImage,
            'stereo/left/balance/compressed',  # Replace 'processed_image_output' with your desired output topic
            10
        )
        self.blur_publisher = self.node.create_publisher(
            CompressedImage,
            'stereo/left/blurred/compressed',  # Replace 'processed_image_output' with your desired output topic
            10
        )      
        self.edge_publisher = self.node.create_publisher(
            CompressedImage,
            'stereo/left/edges/compressed',  # Replace 'processed_image_output' with your desired output topic
            10
        )     
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert compressed ROS image message to OpenCV image
            # np_arr = np.frombuffer(msg.data, np.uint8)
            # cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)

            cv_msg = self.bridge.cv2_to_compressed_imgmsg(cv_image)
            self.publisher.publish(cv_msg)

            # Perform white balancing
            balanced_image = self.white_balance(cv_image)
            
            bal_msg = self.bridge.cv2_to_compressed_imgmsg(balanced_image)
            self.balance_publisher.publish(bal_msg)

            # Apply blur
            blurred_image = self.apply_blur(balanced_image)

            blur_msg = self.bridge.cv2_to_compressed_imgmsg(blurred_image)
            self.blur_publisher.publish(blur_msg)

            # Edge detection
            edges_image = self.detect_edges(blurred_image)
     
            edge_msg = self.bridge.cv2_to_compressed_imgmsg(edges_image)
            self.edge_publisher.publish(edge_msg)

            # Draw contours and detect rectangles
            rectangles_image, bearing = self.detect_rectangles(edges_image)
            
            # Convert the processed image to compressed ROS image message
            
            output_msg = self.bridge.cv2_to_compressed_imgmsg(rectangles_image)
            # Publish the processed image
            self.publisher.publish(output_msg)
            
            # Perform bearing calculation (example)
            print("Bearing:", bearing)

        except Exception as e:
            print(e)

    def white_balance(self, image):
        # Perform white balancing algorithm
        # Your white balancing code here
        balanced_image = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        avg_a = np.average(balanced_image[:, :, 1])
        avg_b = np.average(balanced_image[:, :, 2])
        balanced_image[:, :, 1] = balanced_image[:, :, 1] - ((avg_a - 128) * (balanced_image[:, :, 0] / 255.0) * 1.1)
        balanced_image[:, :, 2] = balanced_image[:, :, 2] - ((avg_b - 128) * (balanced_image[:, :, 0] / 255.0) * 1.1)
        balanced_image = cv2.cvtColor(balanced_image, cv2.COLOR_LAB2BGR)
        return balanced_image

    def apply_blur(self, image):
        # Apply blur (e.g., Gaussian blur)
        blurred_image = cv2.GaussianBlur(image, (5, 5), 0)
        return blurred_image

    def detect_edges(self, image):
        # Edge detection (e.g., Canny edge detection)
        edges_image = cv2.Canny(image, 100, 200)
        return edges_image

def detect_rectangles(self, image):
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply threshold to get a binary image
    ret, thresh = cv2.threshold(gray, 127, 255, 0)
    
    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Process each contour to find rectangles
    rectangles_image = image.copy()
    bearing = None  # Placeholder for bearing calculation
    
    for contour in contours:
        # Approximate the contour to a polygon
        approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
        
        # If the contour has four vertices, it may be a rectangle
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(rectangles_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Calculate bearing (example)
            center_x = x + w / 2
            bearing = (center_x / image.shape[1]) * 2 - 1  # Example bearing calculation
            
    return rectangles_image, bearing

def main():
    rclpy.init()
    computer_vision_node = ComputerVisionNode()
    rclpy.spin(computer_vision_node.node)
    computer_vision_node.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
