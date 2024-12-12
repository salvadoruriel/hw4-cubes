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

from .utils import(
    OUTPUTFOLDER,INPUTFOLDER,
    DEFAULT_THETA,DEFAULT_RHO,
    TABLE_Z,SAFE_Z,
    ourPrint)
from .movements import (
    openGrip, closeGrip, 
    moveTo, raiseArm, goGrabObj, 
    goFeed,grab_food,
    play,playDynamic)

########################################################

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

#########################################################################
## Utils
###########
PHOTO_POS = (250,250,550,-180,0,90) 
currPos = (250,250,550,-180,0,90) 

def read_aruco_marker(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    aruco_params = cv2.aruco.DetectorParameters()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    markers = []
    if ids is not None:
        for i, marker_id in enumerate(ids):
            # Extract the corner points and compute the centroid
            corner = corners[i][0]
            centroid = np.mean(corner, axis=0).astype(int)
            markers.append((marker_id[0], centroid))
    return markers

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

        command = ""
        try:
            with open(f"{INPUTFOLDER}commands.txt", "r") as file:
                command = file.read()
                if not command:
                    ourPrint("[image_callback] NO command, quit... & await instruction")
                    return
            #we have a command, consume, use & empty the file
            with open(f"{INPUTFOLDER}commands.txt", "w") as file:
                file.write("")
        except Exception as e:
            ourPrint("Error opening file")


        #lets try cropping and calculating from shorter values
        height, width = image.shape[:2]
        ourPrint(self, f"Image size> { width},{height}")
        # crops 10% w and h , 5 each corner
        # crop_x = int(width * 0.05)
        # crop_y = int(height * 0.05)
        # image = image[crop_y:height - crop_y, crop_x:width - crop_x]
        # height, width = image.shape[:2]
        # ourPrint(self, f"Image size> { width},{height}")

        #save photo
        cwd = os.getcwd()
        self.get_logger().info(f"directory> {cwd}")
        rightnow = time.time()
        formatted = time.strftime('%Y-%m-%d-%H_%M_%S',time.localtime(rightnow))
        cv2.imwrite(f"{OUTPUTFOLDER}image{formatted}.jpg",image)
        self.get_logger().info("debug")
        #return

        contour_area_threshold = 1000
        def process_image(image):

            height, width = image.shape[:2]

            # Crop 5% from each side
            crop_x = int(width * 0.05)
            crop_y = int(height * 0.05)
            cropped_image = image[crop_y : height - crop_y, crop_x : width - crop_x]

            # Detect ArUco markers
            aruco_markers = read_aruco_marker(cropped_image)
            print(f"Number of ArUco markers detected: {len(aruco_markers)}")

            # Convert to HSV color space
            hsv = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)

            # Define range for white color and create a mask
            # Adjusted HSV values for better detection
            lower_white = np.array([0, 0, 150])  # Broadened lower threshold
            upper_white = np.array([180, 80, 255])  # Broadened upper threshold
            mask = cv2.inRange(hsv, lower_white, upper_white)

            # Apply morphological operations to clean up the mask
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            marker_to_contour = {}
            contour_assignments = {}
            print("Test")

            for contour in contours:
                area = cv2.contourArea(contour)
                x, y, w, h = cv2.boundingRect(contour)
                spoonpos = (0, 0)
                toypos = (0, 0)
                spoonposx = 0
                spoonposy = 0
                toyposx = 0
                toyposy  = 0
                # Initialize spoon and toy positions as tuples
                spoonpos = (0, 0)
                toypos = (0, 0)

                # Extract marker positions
                marker1_id, marker1_centroid = aruco_markers[0]
                marker2_id, marker2_centroid = aruco_markers[1]
                print(f"{marker1_centroid}")
                print(f"{marker2_centroid}")
                # Assign positions based on marker IDs
                toypos = marker1_centroid
                spoonpos = marker2_centroid

                # Decompose the tuples into x and y components
                spoonposx, spoonposy = spoonpos
                toyposx, toyposy = toypos
                area = cv2.contourArea(contour)
                print(f"{x}")
                print(f"{spoonposx}")
                print(f"{toyposx}")




                # Ignore small contours that may be noise
                if area < 1000 or (x < 1 and y < 1) or (x > spoonposx + 1 and x < spoonposx - 1) or (y > spoonposy + 1 and y < spoonposy - 1) or (y > toyposy + 1 and y < toyposy - 1) or (x > toyposx + 1 and x < toyposx - 1):
                    continue

                adjusted_contour = contour + np.array([[crop_x, crop_y]])

                # Calculate the centroid of the contour
                M = cv2.moments(adjusted_contour)
                if M["m00"] != 0:
                    contour_centroid = (
                        int(M["m10"] / M["m00"]),
                        int(M["m01"] / M["m00"]),
                    )
                else:
                    contour_centroid = (0, 0)

                # Convert contour points to float32 for PCA
                data_points = adjusted_contour.reshape(-1, 2).astype(np.float32)

                # Calculate angle using PCA
                mean, eigenvectors = cv2.PCACompute(data_points, mean=None)
                principal_axis = eigenvectors[0]  # Primary eigenvector
                angle = -np.degrees(np.arctan2(principal_axis[1], principal_axis[0]))

                # Get the minimum area rectangle
                rect = cv2.minAreaRect(adjusted_contour)
                box = cv2.boxPoints(rect).astype(int)

                # Find closest marker for each contour
                closest_marker = None
                min_distance = float("inf")
                for marker_id, marker_centroid in aruco_markers:
                    dist = np.linalg.norm(
                        np.array(marker_centroid) - np.array(contour_centroid)
                    )
                    if dist < min_distance:
                        closest_marker = (marker_id, marker_centroid)
                        min_distance = dist

                if closest_marker and closest_marker[0] not in contour_assignments:
                    marker_to_contour[closest_marker[0]] = contour_centroid
                    contour_assignments[closest_marker[0]] = (
                        adjusted_contour,
                        contour_centroid,
                        rect,
                        box,
                        angle,
                    )

            output = []
            for marker_id, (
                contour,
                new_centroid,
                rect,
                box,
                angle,
            ) in contour_assignments.items():
                # Draw contours
                cv2.drawContours(image, [contour], 0, (255, 0, 0), 2)

                # Draw bounding box
                cv2.drawContours(image, [box], 0, (0, 255, 0), 2)

                # Draw centroid
                cv2.circle(image, new_centroid, 5, (0, 0, 255), -1)

                # Calculate full-length line endpoints
                angle_radians = np.radians(angle)
                x1 = int(new_centroid[0] - width * np.cos(angle_radians))
                y1 = int(new_centroid[1] + width * np.sin(angle_radians))
                x2 = int(new_centroid[0] + width * np.cos(angle_radians))
                y2 = int(new_centroid[1] - width * np.sin(angle_radians))

                # Draw full-length line
                cv2.line(image, (x1, y1), (x2, y2), (0, 165, 255), 2)  # Orange color in BGR

                # Draw marker ID near the centroid
                text_position = (new_centroid[0] + 10, new_centroid[1] - 10)  # Slightly offset
                cv2.putText(
                    image,
                    f"ID: {marker_id}",
                    text_position,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,  # Font size
                    (255, 51, 204),
                    3,  # Thickness
                    cv2.LINE_AA,
                )

                # Append the marker id, centroid, and angle
                output.append((marker_id, new_centroid, angle))

            # Display the image with all annotations
            cv2.imshow("Result", image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            return output

        

        # self.get_logger().info(f"Processing image")
        try:
            #x,y,phi = process_image(image)
            points = process_image(image)
        except Exception as e:
            ourPrint(self,f"Something happened: {e}")
            #x,y,phi = 0,0,0
            points = []

        self.get_logger().info(f"{points}")
        #x1 = math.cos(math.radians(50))*x/2
        objectPoints = []
        for point in points:
            x, y = point[1]
            phi = point[2]
            ycam = (((-y)+960))
            ourPrint(self,f"x : {x}, y: {ycam}")
            x1= ycam/(2.57) + 152 #160
            y1 = (-x)/(2.57)  + 505 #506
            #/math.cos(math.radians(50)) 
            # phi = phi % 45
            ourPrint(self,f"phi is {phi}")
            #correction applied on the center of the image
            #   where the mass will make the diagonal the bigger mass point
            # if(3*1280/10 < x < 7*1280/10
            #     and 3*960/10 < y < 7*960/10
            #     and 30 <= phi <= 60):
            #     angle_offset= 45#45
            #else: angle_offset = 
            phi1 = phi#90.00 + phi
            ourPrint(self,f"{x1}{y1}{phi1}")
            ourPrint(self,"converted values:")
            ourPrint(self,f"x1: {x1}")
            ourPrint(self,f"y1: {y1}")
            ourPrint(self,f"phi1: {phi1}")
            #objectPoints.append([x1,y1,phi1])
            # x1,y1 = cameraToRobotXYZ(x,y)
            objectPoints.append([x1,y1,TABLE_Z,-180,0,phi])

        # # 1 mm = 3 pixels
        # # 10 cm = 300 pixels
        #goGrabObj(*objectPoints[0])
        #grab_food()
        closeGrip()
        #goGrabObj(250,250,350,-179,0,135)
        #play()
        if command == "play":
            playDynamic()
        elif command == "feed":
            grab_food()
        else:
            ourPrint(f'[ERR] Non recognized command "{command}"')
        
############################################################################
def main(args=None):
    rclpy.init(args=args)
    node = ImageSub('image_sub')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()