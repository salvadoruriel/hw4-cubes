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


def cameraToRobotXYZ(u,v):
    """
    Input= centroid's x,y values as u,v
    return = robot tool's x,y coords
    """
    # Camera intrinsics
    fx = 399 #3.99156077e+03  # focal length in x direction
    fy = 398 #3.98157827e+03 # focal length in y direction
    cx = 696  #6.96680719e+02 # principal point x-coordinate
    cy = 406  #4.06275392e+02 # principal point y-coordinate
    # Image centroid and depth
    Z = 730  #Depth, we require another/camera sensor for this value
    # Back-project to 3D in camera frame
    X_camera = (u - cx) * Z / fx
    Y_camera = (v - cy) * Z / fy
    Z_camera = Z
    # 3D point in the camera frame
    P_camera = np.array([X_camera, Y_camera, Z_camera, 1])
    #P_camera = np.array([u, v, Z_camera, 1])

    ######### Frames
    #we saw a ~45 degree rotation from robot to camera
    theta = np.deg2rad(180+45)#counter clockwise
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    # Rotation around Z-axis:
    R = np.array([
        [cos_theta, -sin_theta, 0, 0],
        [sin_theta,  cos_theta, 0, 0],
        [0,          0,         1, 0],
        [0,          0,         0, 1]
    ])
    x_base2cam = 230#-135
    y_base2cam = 230#360
    z_base2cam = 730
    # Translation
    T = np.array([
        [1, 0, 0, x_base2cam],#x
        [0, 1, 0, y_base2cam],#y
        [0, 0, 1, z_base2cam],#z
        [0, 0, 0, 1]
    ])

    # Transformation matrix
    #T_camera_to_robot = T @ R
    T_camera_to_robot = np.array([
        [cos_theta, -sin_theta, 0, x_base2cam],#x
        [sin_theta, cos_theta, 0,  y_base2cam],#y
        [0, 0, 1,                  z_base2cam],#z
        [0, 0, 0,                  1]
    ])

    #From Lesson 06 pg59, Robot vision
    #We just need the x,y values, so in reality it's
    #   all about a transformation from a point in the camera's frame
    #   to a point in the robot's
    #So:
    # Transform the point from the camera frame to the robot frame
    P_robot = T_camera_to_robot @ P_camera

    e_x, e_y, e_z, _ = P_robot
    print(f"Object position in robot's frame {e_x},{e_y}")
    return e_x, e_y



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
        # rightnow = time.time()
        # formatted = time.strftime('%Y-%m-%d-%H_%M_%S',time.localtime(rightnow))
        # cv2.imwrite(f"{outputFolder}image{formatted}.jpg",image)
        # self.get_logger().info("debug")
        #return


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
            crop_x = 0#int(width * 0.05)
            crop_y = 0#int(height * 0.05)
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
        x1=x*0.5851 - 135
        y1 = ycam*(-1*0.3047) + 360
        #/math.cos(math.radians(50))
        phi1 = math.degrees(phi) + 135.00
        self.get_logger().info(f"{x1}{y1}{phi1}")
        ourPrint(self,"converted values:")
        ourPrint(self,f"x1: {x1}")
        ourPrint(self,f"y1: {y1}")
        ourPrint(self,f"phi1: {phi1}")
        # # 1 mm = 3 pixels
        # # 10 cm = 300 pixels

        xt, yt = cameraToRobotXYZ(x,y)
        
        # script = "PTP(\"CPP\","+targetP+",100,200,0,false)"
        # send_script(script)

        targetP1 = f"{xt}, {yt}, 190, -180.00, 0.0, {phi1}"
        #targetP1 = f"{x1}, {y1}, 190, -180.00, 0.0, {phi1}"
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