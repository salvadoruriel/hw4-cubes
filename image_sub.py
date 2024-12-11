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
    OUTPUTFOLDER,DEFAULT_THETA,
    DEFAULT_RHO,TABLE_Z,SAFE_Z,
    ourPrint)
#from .movements import (openGrip, closeGrip, moveTo, raiseArm, goGrabObj, goFeed)

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
currPos = (250,250,550,-180,0,135) 

#### utils:
def openGrip():
    set_io(0.0)
def closeGrip():
    set_io(1.0) #1.0 close

def moveTo(x,y,z=SAFE_Z,theta=-180.00,rho=0.0,phi=135):
    """move safely to the position"""
    global currPos

    targetP = f"{x}, {y}, {z}, {theta}, {rho}, {phi}"
    script = "PTP(\"CPP\","+targetP+",100,200,0,false)"
    send_script(script)

    currPos = (x,y,z,theta,rho,phi)
    return x,y,z,phi #return as current position

def raiseArm():
    """Raises... the... arm... probs~ iunno"""
    global currPos

    x,y,z,theta,rho,phi = currPos
    return moveTo(x,y,SAFE_Z,theta,rho,phi)

def goGrabObj(x,y,z=TABLE_Z,theta=-180.00,rho=0.0,phi=90):
    """Moves to given position FROM ABOVE & grabs object"""
    print("Grabbing object at ",x,y,z,theta,rho,phi)
    #First raise the arm to avoid collision
    raiseArm()
    openGrip()
    #Now move safely to x,y position of object
    moveTo(x,y,SAFE_Z,DEFAULT_THETA,DEFAULT_RHO, phi)
    #move slowly to object
    moveTo(x,y,z+15,DEFAULT_THETA,DEFAULT_RHO,phi)

    #to see how close we are
    moveTo(x,y,z+13,DEFAULT_THETA,DEFAULT_RHO,phi)
    #return

    #lower arm & grab
    moveTo(x,y,z,DEFAULT_THETA,DEFAULT_RHO,phi)
    closeGrip()
    return x,y,z,DEFAULT_THETA,DEFAULT_RHO,phi

def grab_food():
    moveTo(250,250,300)
    # Grabe the spoon
    # Moove above the bowl
    movebowl = [134.43,378.39,263.67,-157.4,0.29,169.38]
    moveTo(*movebowl)
    in2bowl = [138.74,468.12,175.47,-151.96,0.41,169.4]
    moveTo(*in2bowl)

    out2bowl = [138.74,468.12,175.47,-180.00,0.0,169.4]
    moveTo(*out2bowl)

    #move to cat Position

    # Return centre table
    moveTo(250,250,300)
    return 0

     
def play(seconds=3):
  #1. Rx : -150° TIlt downwards
  #2.  ⁠Rz : 15 -> 60° move toy around
  #3. ⁠repeat 2 back and forth
  center = (250,250,350,-171,0,135)
  moveTo(*center)
  #playDownwards
  play1 = (250,250,200,-171,0,160)
  moveTo(*play1)
  play2 = (250,250,200,-171,0,100)
  moveTo(*play2)

  #play upwards
  moveUp = (250,250,200,160,0,135)
  moveTo(*moveUp)
  play3 = (250,250,200,160,0,160)
  moveTo(*play3)
  play4 = (250,250,200,160,0,100)
  moveTo(*play4)

  #Rectangle movement
  #rightLowerCorner = (565,140,200,180,0,135)
  rightLowerCorner = (565,140,200,180,0,60)
  moveTo(*rightLowerCorner)
  rightUpperCorner = (565,140,600,180,0,60)
  moveTo(*rightUpperCorner)

  #leftUpperCorner = (115,665,600,180,0,135)
  leftUpperCorner = (115,665,600,180,0,200)
  moveTo(*leftUpperCorner)
  leftLowerCorner = (115,665,200,180,0,200)
  moveTo(*leftLowerCorner)
  moveTo(*center)

  #X movement
  moveTo(*rightUpperCorner)
  moveTo(*rightLowerCorner)
  moveTo(*leftUpperCorner)
  moveTo(*leftLowerCorner)
  moveTo(*center)

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
            output = []
            for contour in contours:
                area = cv2.contourArea(contour)

                # Ignore small contours that may be noise
                if area < 1000:  # Adjust minimum area as needed
                    continue

                # Get the minimum area rectangle
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.array([[int(point[0] + crop_x), int(point[1] + crop_y)] for point in box])
                
                # Dimensions of the rectangle
                width, height = rect[1]  # rect[1] contains (width, height)

                # Ensure the longer side is treated as the base
                if width < height:
                    angle = rect[2] + 90  # Adjust angle for consistent base alignment
                else:
                    angle = rect[2]

                # Normalize the angle to ensure it's measured from the horizontal axis
                if angle < 0:
                    angle += 180  # Ensure angle is positive and within [0, 180]

                centroid = (int(rect[0][0])+crop_x, int(rect[0][1])+crop_y)
                x = centroid[0]
                y = centroid[1]    
                print(f"{angle}")
                # Draw bounding box
                cv2.drawContours(image, [box], 0, (0, 255, 0), 2)
                
                # Draw centroid
                cv2.circle(image, centroid, 5, (0, 0, 255), -1)

                # Display the final image with contours and angles for debugging
                cv2.imshow("Result", image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

                # Print the centroid and angle
                self.get_logger().info(f"here")
                self.get_logger().info(f"Centroid: {centroid}, Angle: {angle:.2f} radians")
                if not x or not y or not angle:
                    ourPrint(self,f'No centroids found. Maybe objects were not present in the camera range')
                    return 0,0,0
                else:
                    #return x,y, angle
                    output.append([x,y,angle])
            ourPrint(self,f'No centroids found. Maybe objects were not present in the camera range')
            return output


        # self.get_logger().info(f"Processing image")
        try:
            #x,y,phi = process_image(image)
            points = process_image(image)
        except Exception as e:
            ourPrint(self,f"Something happened: {e}")
            #x,y,phi = 0,0,0
            points = []


        #x1 = math.cos(math.radians(50))*x/2
        objectPoints = []
        for point in points:
            x, y, phi = point
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
        play()
        
############################################################################
def main(args=None):
    rclpy.init(args=args)
    node = ImageSub('image_sub')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()