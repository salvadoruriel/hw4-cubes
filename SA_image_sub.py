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

import os
import time

cwd = os.getcwd()
print(f"cwd: {cwd}")
outputFolder = f"{cwd}/src/send_script/send_script/output/"
print(f"outputFolder: {outputFolder}")
now = time.time()
today = time.strftime('%Y-%m-%d',time.localtime(now))
logfile = f"{today}-log.txt"

def ourPrint(self='',string="",log=True):
    now = time.time()
    hour = time.strftime('%H:%M:%S',time.localtime(now))
    
    if self:
        self.get_logger().info(string)
    else:
        print(string)
    if not log: return
    with open(f'{outputFolder}{logfile}', 'a') as file:
        file.write(f'[{hour}] {string}\n')


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
        # cwd = os.getcwd()
        # self.get_logger().info(f"directory> {cwd}")
        # #creates the photo at top level folder, not cwd lol
        # cv2.imwrite(f"{cwd}/output/image3.jpg",image)
        # self.get_logger().info("debug")


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
            print(height)
            print(width)
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

            #ourPrint(self,f'countours: {contours}',log=False)
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
                
            
                # Calculate extended line points based on the image dimensions
                line_length = max(width, height) * 2  # Extend the line beyond the image size
                x_end = int(adjusted_centroid[0] + line_length * math.cos(angle))
                y_end = int(adjusted_centroid[1] + line_length * math.sin(angle))
                x_start = int(adjusted_centroid[0] - line_length * math.cos(angle))
                y_start = int(adjusted_centroid[1] - line_length * math.sin(angle))
                
                # Draw the infinite principal line with thickness 1
                cv2.line(image, (x_start, y_start), (x_end, y_end), (255, 0, 0), 1)
                
                # Draw the centroid
                cv2.circle(image, adjusted_centroid, 5, (0, 0, 255), -1)
                
                # Print the centroid and angle
                ourPrint(self,f"Centroid: {adjusted_centroid}, Angle: {angle:.2f} radians")
                
                # # Display the final image with contours and angles for debugging
                cv2.imshow("Result", image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()   

                # Print the centroid and angle
                self.get_logger().info(f"here")
                self.get_logger().info(f"Centroid: {adjusted_centroid}, Angle: {angle:.2f} radians")
                if not x or not y or not angle:
                    return 0,0,0
                else:
                    return x,y, angle
            ourPrint(self,f'No centroids found. Maybe objects were not present in the camera range')
            return 0,0,0 #


        self.get_logger().info(f"Processing image")
        try:
            x,y,phi = process_image(image)
        except Exception as e:
            ourPrint(self,f"Something happened: {e}")
            x,y,phi = 0,0,0

        # x1 = math.cos(math.radians(50))*x/2
        ycam = (((-y)+960))
        x1=x*0.4497+ycam*0.2467 - 135
        y1 = x*(-0.0705)+ycam*0.2467 + 360
        #/math.cos(math.radians(50))
        phi1 = math.degrees(phi) + 135.00
        self.get_logger().info(f"{x1}{y1}{phi1}")
        ourPrint(self,"converted values:")
        ourPrint(self,f"x1: {x1}")
        ourPrint(self,f"y1: {y1}")
        ourPrint(self,f"phi1: {phi1}")
        # # 1 mm = 3 pixels
        # # 10 cm = 300 pixels
        
        # script = "PTP(\"CPP\","+targetP+",100,200,0,false)"
        # send_script(script)

        x1=80
        y1=100
        targetP1 = f"{x1}, {y1}, 490, -180.00, 0.0, {phi1}"
        #targetP1 = "250.00, 250, 190, -180.00, 0.0, 135.00"
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script1)
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