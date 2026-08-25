#!/usr/bin/env python3
import math
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
from dodgeball.msg import BallLocation


class Robot(Node):
    def __init__(self):
        super().__init__('ballfinder')
        self.bridge = CvBridge()
        self.raw_image = []
        self.ranges = []
        self.distance_history = []  # Initialize the list
        self.buffer_size = 5        # Set a buffer size for moving average (may not be necessary, note in C++ rewrite)

        self.loc_publisher = self.create_publisher(BallLocation, '/ball_location', 10) #(type, topic, queue)
        self.im_publisher = self.create_publisher(Image, '/ball_image', 10) # publisher for modified image
        self.timer = self.create_timer(0.1, self.main_loop)

        self.create_subscription( #get image data from robot's camera
            Image,
            '/oakd/rgb/preview/image_raw',
            self.handle_image,
            qos_profile_sensor_data,
        )

        self.create_subscription( #get scan data from robot's lidar
            LaserScan,
            '/scan',
            self.handle_scan,
            qos_profile_sensor_data,
        )

    def handle_image(self, msg): #just stores raw image in self.raw_image by converting to OpenCV.
        try:
            self.raw_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError:
            print("Unable to convert ROS image to OpenCV format.")

    def handle_scan(self, msg): 
        self.scan_msg = msg
        self.ranges = msg.ranges # store scan data in self.ranges for use in main loop

    def main_loop(self):
        if len(self.raw_image) == 0 or len(self.ranges) == 0: #if nothing received, dont run.
            return

        image = self.raw_image.copy() #get image, use a copy to not modify original
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #convert image to hsv for filtering

        lower_yellow = np.array([20, 150, 150]) #set bounds for yellow color in hsv
        upper_yellow = np.array([30, 255, 255])

        mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow) #mask (binaryImage, lowBound, upBound), returns binary image where pixels in range are 255 and others are 0
        hfov_deg = 80.0 #confirmed for turtlebot 4 OAK-D PRO Fixed Focus OV9782. Will confirm upon physical testing
        hfov_rad = math.radians(hfov_deg) #convert to radians for trig functions

        height, width = mask.shape
        mask[0:int(0.2*height), :] = 0 #ignore top 20% of image to avoid ceiling
        mask[int(0.8*height):, :] = 0 #ignore bottom 20% of image to avoid floor
        yellow_cols = np.where(mask == 255)[1] #get column indices of yellow pixels,

        focal_length = (width/2) / math.tan(hfov_rad/2) #focal length = dist from camera image plane in pixels
        image_center_x_px = width/2 #image center in pixels (px)
        ball_location = BallLocation() #ball_location = BallLocation object to save data to

        if len(yellow_cols) == 0 or len(yellow_cols) < 50: #if nothing found:
            ball_location.bearing = 0.0 #set invalid/ default values
            ball_location.distance = -1.0
            ball_location.found = False       

        else: #if pixels found... START HERE
            ball_center_x_px = int(np.mean(yellow_cols)) #get average column of yellow pixels, approx ball location in pixels
            ball_location.bearing = np.arctan2(ball_center_x_px-image_center_x_px, focal_length) #calculate bearing with triangle using pixel error as opp and focal length as adj.

            scan_index = int(223 - (ball_center_x_px * 76 / 250))
            scan_index = max(147, min(scan_index, 223))

            distance = self.ranges[scan_index] #store distance for validity check

            if np.isnan(distance) or np.isinf(distance): #ensure distance is valid number, if not set to -1.0 to indicate invalid distance
                ball_location.distance = -1.0
            else:
                self.distance_history.append(float(distance)) #add distance to history
                if len(self.distance_history) > self.buffer_size: #if history exceeds buffer size, remove oldest measurement
                    self.distance_history.pop(0)
                ball_location.distance = np.mean(self.distance_history) #set distance to average of history for
                
            cv2.line(image, (ball_center_x_px, 0), (ball_center_x_px, height), (255, 0, 0), 2) #draw vertical blue line at avg_x to show ball center

            if ball_location.distance > 0: #if ball close to center, set as found
                ball_location.found = True
            else:
                ball_location.found = False

        image[mask>0] = (0, 255, 0) #set yellow pixels in original image to bright yellow for visualization

        try:
            self.im_publisher.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8')) #publish modified image to ROS topic
        except CvBridgeError:
            print('Unable to convert mask to ROS image')
        self.loc_publisher.publish(ball_location)


def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()