#!/usr/bin/env python
import rclpy

from rclpy.node import Node

import sys
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import argparse

class ImageSub(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(Image,
        'techman_image', self.image_callback, 10)
        self.subscription

    def image_callback(self, data):
        self.get_logger().info('Received image')
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(data)
        cv2.imwrite("/tmp/image.jpg",image)
        self.get_logger().info("debug")


        def calculate_centroid_and_angle(moment):
            # Calculate centroid
            if moment["m00"] != 0:
                cx = int(moment["m10"] / moment["m00"])
                cy = int(moment["m01"] / moment["m00"])
            else:
                cx, cy = 0, 0

            # Calculate principal angle
            mu11 = moment["mu11"]
            mu20 = moment["mu20"]
            mu02 = moment["mu02"]
            angle = 0.5 * math.atan2(2 * mu11, mu20 - mu02)

            return (cx, cy), angle

        def process_image(image):

            height, width = image.shape[:2]

            # Crop 5% from each side
            crop_x = int(width * 0.05)
            crop_y = int(height * 0.05)
            cropped_image = image[crop_y:height - crop_y, crop_x:width - crop_x]

            # Convert to grayscale
            gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)

            # Apply Gaussian blur to reduce noise
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            # Apply adaptive thresholding to handle different lighting conditions
            binary = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                        cv2.THRESH_BINARY_INV, 11, 2)

            # Apply morphological operations to remove small noise
            kernel = np.ones((3, 3), np.uint8)
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)


            # Find contours
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)

                # Ignore small contours that may be noise
                if area < 1000:  # Adjust minimum area as needed
                    continue

                # Calculate moments
                moment = cv2.moments(contour)

                # Calculate centroid and principal angle
                centroid, angle = calculate_centroid_and_angle(moment)
                self.get_logger().info(f"Centroid: {centroid}, Angle: {angle:.2f} radians")
                # Adjust centroid position based on the cropped region
                adjusted_centroid = (centroid[0] + crop_x, centroid[1] + crop_y)

                x = adjusted_centroid[0]
                y = adjusted_centroid[1]
                # Print the centroid and angle
                self.get_logger().info(f"Centroid: {adjusted_centroid}, Angle: {angle:.2f} radians")
                return x,y, angle



        x,y,phi = process_image(image)
        x1 = math.cos(math.radians(-45))*x/2
        y1 = y/math.cos(math.radians(45))/2
        phi1 = phi + 135.00
        # # 1 mm = 3 pixels
        # # 10 cm = 300 pixels

        targetP = f"{x1}, {y1}, 500, -180.00, 0.0, {phi1}"
        script = "PTP(\"CPP\","+targetP+",100,200,0,false)"
        send_script(script)

        # targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
        #targetP2 = "280.00, 250, 500, -180.00, 0.0, 135.00"
        # script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        #script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
        # send_script(script1)
        #send_script(script2)
########################################################################################################################

def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()

def set_io(state):
    gripper_node = rclpy.create_node('gripper')
    gripper_cli = gripper_node.create_client(SetIO, 'set_io')

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not availabe, waiting again...')

    io_cmd = SetIO.Request()
    io_cmd.module = 1
    io_cmd.type = 1
    io_cmd.pin = 0
    io_cmd.state = state
    gripper_cli.call_async(io_cmd)
    gripper_node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSub('image_sub')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()