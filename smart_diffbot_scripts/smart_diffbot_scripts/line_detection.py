import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

import cv2, cv_bridge
import numpy as np
import time


class LineDetection(Node):

    def __init__(self):
        super().__init__('line_detection_node')

        # Start CV bridge
        self.bridge = cv_bridge.CvBridge()

        # Start camera subscriber
        self.create_subscription(Image, 'camera/image_raw', self.image_callback, qos_profile_sensor_data)

        # Start line pose publisher
        self.line_publisher = self.create_publisher(PoseStamped, 'line_detection/midpoint', 10)

        # Settings
        self.rate = 10.0
        self.min_area = 10 # Minimal area for a pattern to be recognized as relevant
        self.filter_below = np.array([0, 0, 100]) # blue, green, red
        self.filter_above = np.array([20, 20, 200])
        self.crop = 0.4 # vertical portion of image to use in center of camera - 1.0 is full width
        self.debug = False # Will visualize camera image, used cropped image and result mask between filter values

        # Init image variable
        self.image = None

        # Start timer
        timer_period = 1/self.rate 
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        # Wait for first image
        if type(self.image) != np.ndarray:
            self.get_logger().info('Waiting for image...')
            time.sleep(1.0)
            return
        
        self.get_logger().info('Found camera image. Line detection running.', once=True)

        # Crop image
        image_cropped = self.crop_image(self.image, self.crop)

        # Find mask of line colors in image
        mask = self.mask_image(image_cropped)

        # Detect line in mask
        line = self.detect_line(mask)

        # Publish line coordinates if line is detected
        if line !=[]:
            self.publish_line(line)


    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.debug:
          cv2.imshow("camera", self.image)
          cv2.waitKey(5)


    def crop_image(self, image, width):
        # Crop a vertical lane of *width* out of the center of the image 
        height, width, _ = image.shape
        start = int(width*(1.0-self.crop)/2.0)
        stop = int(start + width*self.crop)

        crop = image[0:height, start:stop]

        if self.debug:
            cv2.imshow("crop", crop)
            cv2.waitKey(5)

        return crop
    

    def mask_image(self, image):
        # Create mask of all pixels withing specified blure/green/red margins        
        mask = cv2.inRange(image, self.filter_below, self.filter_above)

        if self.debug:
            cv2.imshow("mask", mask)
            cv2.waitKey(5)

        return mask
    

    def detect_line(self, mask):

        line = []
        largest_area = 0
        min_area = 10

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Find line with largest surface area (above a minimum value)
        for contour in contours:
            
            moments = cv2.moments(contour)
            if moments['m00'] > largest_area and moments['m00'] > min_area:

                largest_area = moments['m00'] 

                # Midpoint of line (pixels in mask)
                x_pix = int(moments["m10"]/moments["m00"])
                y_pix = int(moments["m01"]/moments["m00"])

                # With x=0 in the middle and y=0 at the bottom:
                height, width = mask.shape
                x = int(x_pix - width/2.0)
                y = height - y_pix

                line = [x, y]

        return line
    
    def publish_line(self, line):

        # Fill message
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.pose.position.x = float(line[0])
        msg.pose.position.y = float(line[1])

        # Publish message
        self.line_publisher.publish(msg)
    

def main(args=None):

    rclpy.init(args=args)

    line_detection = LineDetection()

    rclpy.spin(line_detection)

    line_detection.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
