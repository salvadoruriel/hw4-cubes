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
#Pixel ratio is ALWAYS needed (except for calibration apparently...)
XPIXEL_TO_MM = 1 /2
YPIXEL_TO_MM = 1 /2
#XPIXEL_TO_MM = 660/1280 # mm / 1280
#YPIXEL_TO_MM = 495/960 # /960

x_base2cam = 226#-135
y_base2cam = 704.5#360
z_base2cam = 0

#CROPPED 5%
#imgWidth = 1216
#imgH = 912
#10%
# imgWidth = 1152
# imgH = 864

# x_base2cam = 230
# y_base2cam = 655
# z_base2cam = 0
# XPIXEL_TO_MM = 605/imgWidth # mm / 1280
# YPIXEL_TO_MM = 454/imgH # /960

###############################3
########################################################

def cameraToRobotXYZ(u,v):
    """
    Input= centroid's x,y values as u,v
    return = robot tool's x,y coords
    """
    u= u*XPIXEL_TO_MM
    v= v*YPIXEL_TO_MM
    #camera size:
    # 1280x960
    # center should be ~ 640 x 480
    # Cropped: 5%
    # 1216x912  ~ 608 x 456
    # Camera intrinsics
    fx = 3.99156077e+03  #3.99156077e+03  # focal length in x direction
    fy = 3.98157827e+03  #3.98157827e+03 # focal length in y direction
    cx = 6.96680719e+02  #6.96680719e+02 # principal point x-coordinate
    cy = 4.06275392e+02  #4.06275392e+02 # principal point y-coordinate
    Cam_IntMat = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0,  0, 1]
    ])
    # Image centroid and depth
    Z = 730 - 50 #Depth
    # 730mm Z to End-Effector when taking photo
    #~50mm distance from camera to end-effector
    
    # Back-project to 3D in camera  
    X_camera = (u - cx) * Z / fx
    Y_camera = (v - cy) * Z / fy
    Z_camera = Z
    # 3D point in the camera frame
    P_camera = np.array([X_camera, Y_camera, Z_camera, 1])
    #assuming calibration isn't quite needed...
    P_cameraDirect = np.array([u,v,Z_camera, 1])
    

    """
    #From stack overflow
    """
    #https://stackoverflow.com/questions/13419605/how-to-map-x-y-pixel-to-world-cordinates
    P_tempcamera = np.array([u, v, 1])
    #multiply intrinsic matrix's inverse to woorld coordinates
    World_Coord = np.linalg.inv(Cam_IntMat) @ P_tempcamera
    World_Coord = World_Coord * Z_camera
    #Somehow is always the same as the first P_camera lol
    print(f"Testing: {World_Coord}")

    ## From fdxlabs
    #https://www.fdxlabs.com/calculate-x-y-z-real-world-coordinates-from-a-single-camera-using-opencv/
    scalFactor = 1/3
    uv_1=np.array([[u,v,1]], dtype=np.float32).T

    suv_1=scalFactor *uv_1

    xyz_c=np.linalg.inv(Cam_IntMat) @ suv_1
    tvecs = np.array([[x_base2cam,y_base2cam,0]]).T
    xyz_c=xyz_c - tvecs

    xtheta = np.deg2rad(180)
    xcos_theta = np.cos(xtheta)
    xsin_theta = np.sin(xtheta)
    Rx = np.array([
        [1, 0, 0],
        [0, xcos_theta, -xsin_theta],
        [0, xsin_theta, xcos_theta],
    ])
    ztheta = np.deg2rad(45)
    zcos_theta = np.cos(ztheta)
    zsin_theta = np.sin(ztheta)
    # Rotation around Z-axis:
    Rz = np.array([
        [zcos_theta, -zsin_theta, 0],
        [zsin_theta,  zcos_theta, 0 ],
        [0,          0,         1],
    ])
    #First rotation -> Left-est Matrix mult
    RMat = Rz @ Rx
    XYZ= np.linalg.inv(RMat) @ xyz_c
    print(f"From fdxlabs: {XYZ}")
    
    #Proposed inverse formula:
    # (s* [u,v,1].T @ Cam_IntMat^-1 -t) @ R-1

    ######### Frames
    #Lecture 2 pg 44 Fundamental rotation matrices:
    #Rotation around x-axis
    xtheta = np.deg2rad(180)
    xcos_theta = np.cos(xtheta)
    xsin_theta = np.sin(xtheta)
    Rx = np.array([
        [1, 0, 0, 0],
        [0, xcos_theta, -xsin_theta, 0],
        [0, xsin_theta, xcos_theta, 0],
        [0,          0,         0, 1]
    ])
    #Rotation around y-axis
    ytheta = np.deg2rad(1)
    ycos_theta = np.cos(ytheta)
    ysin_theta = np.sin(ytheta)
    Ry = np.array([
        [ycos_theta, 0, ysin_theta, 0],
        [0,          1, 0,           0],
        [-ysin_theta, 0, ycos_theta, 0],
        [0,          0,         0, 1]
    ])
    ztheta = np.deg2rad(-45)
    zcos_theta = np.cos(ztheta)
    zsin_theta = np.sin(ztheta)
    # Rotation around Z-axis:
    Rz = np.array([
        [zcos_theta, -zsin_theta, 0, 0],
        [zsin_theta,  zcos_theta, 0, 0],
        [0,          0,         1, 0],
        [0,          0,         0, 1]
    ])
    #First rotation -> Left-est Matrix mult
    R = Rz @ Rx
    #print(f"R:\n {R}")

    # R = Ry @ Rz
    #I thought this from the hw1 example but it
    # it doesn't work to this robot's frames...
    #First-est rotation -> Right-est Matrix Mult # Wrong???
    
    #particular rotation matrix
    """ 
    R = np.array([
        [np.cos(theta0), np.cos(theta1), np.cos(theta2), 0],
        [np.cos(theta3), np.cos(theta4), np.cos(theta5), 0],
        [np.cos(theta6), np.cos(theta7), np.cos(theta8), 0],
        [0,            0,             0,              1]
    ]) """
    ## From classes: from 0_to_1
    """ 
    R = np.array([
        [X1*X0, Y1*X0, Z1*X0, 0],
        [X1*Y0, Y1*Y0, Z1*Y0, 0],
        [X1*Z0, Y1*Z0, Z1*Z0, 0],
        [0,         0,     0, 1]
    ]) """

    # Translation
    T = np.array([
        [1, 0, 0, x_base2cam],#x
        [0, 1, 0, y_base2cam],#y
        [0, 0, 1, z_base2cam],#z
        [0, 0, 0, 1]
    ])

    # Transformation matrix
    T_camera_to_robot = T @ R
    """ T_camera_to_robot = np.array([
        [cos_theta, -sin_theta, 0, x_base2cam],#x
        [sin_theta, cos_theta, 0,  y_base2cam],#y
        [0, 0, 1,                  z_base2cam],#z
        [0, 0, 0,                  1]
    ]) """

    #####
    """
    #From stack overflow
    """
    #https://stackoverflow.com/questions/7836134/get-3d-coordinates-from-2d-image-pixel-if-extrinsic-and-intrinsic-parameters-are?noredirect=1&lq=1
    #H = K*[r1, r2, t], 
    #r1 and r2 being the first two columns of the rotation matrix R
    #   t is the translation vector.
    temp = T_camera_to_robot[:, [0,1,3]]
    temp = np.delete(temp, (3),axis=0)
    H = Cam_IntMat @ temp
    projection= H @ [u,v,1]
    projection = projection / Z
    print(f"projection:\n {projection}")

    #From Lesson 06 pg59, Robot vision
    #We just need the x,y values, so in reality it's
    #   all about a transformation from a point in the camera's frame
    #   to a point in the robot's
    #So:
    # Transform the point from the camera frame to the robot frame
    P_robot = T_camera_to_robot @ P_camera
    print(f"T_camera_to_robot:\n {T_camera_to_robot}")
    print(f"P_Camera:\n {P_camera}")
    print(f"P_robot:\n ANS = {P_robot}")

    #Robotics guy's approach
    P_robotDirect = T_camera_to_robot @ P_cameraDirect
    print(f"**P_cameraDirect:\n {P_cameraDirect}")
    print(f"P_robotDirect:\n ANS = {P_robotDirect}")

    e_x, e_y, e_z, _ = P_robot
    e_x, e_y, e_z, _ = P_robotDirect
    #print(f"Object position in robot's frame: {e_x:.2f}, {e_y:.2f}")
    return e_x, e_y

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

#################################################################################
## Utils
###########
###########
#global vars: #I know... TODO change to something else... but if it works, it works
###########
# Assuming taking image initial position
#currPos = (250,250,550,90) #right arm (near door)
currPos = (230,230,730,135) #right arm (near door)
#currPos = (350,350,730,90) #left arm
###########
#### SOME CONSTANTS:
###########
Z_CUBE = 25#mm half of 25, as the cubes are grabbed from almost the top
TABLE_Z = 110#mm  #Approx what the gripper to table position would be, 100mm is gripper length
SAFE_Z = 500#mm   #USE THIS ONE FOR MOVING AROUND
###########
### Printing
cwd = os.getcwd()
print(f"cwd: {cwd}")
outputFolder = f"{cwd}/src/send_script/send_script/output/"
print(f"outputFolder: {outputFolder}")
now = time.time()
today = time.strftime('%Y-%m-%d',time.localtime(now))
logfile = f"{today}-log.txt"
#Printing & logging
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

#### utils:
def openGrip():
    set_io(0.0)
def closeGrip():
    set_io(1.0) #1.0 close

def moveTo(x,y,z=SAFE_Z,phi=90):
    """move safely to the position"""
    global currPos

    targetP = f"{x}, {y}, {z}, -180.00, 0.0, {phi}"
    script = "PTP(\"CPP\","+targetP+",100,200,0,false)"
    send_script(script)

    currPos = (x,y,z,phi)
    return x,y,z,phi #return as current position

def raiseArm():
    """Raises... the... arm... probs~ iunno"""
    global currPos

    x,y,_,phi = currPos
    return moveTo(x,y,SAFE_Z,phi)

def goGrabObj(x,y,z=TABLE_Z,phi=90):
    """Moves to given position FROM ABOVE & grabs object"""
    #First raise the arm to avoid collision
    raiseArm()
    openGrip()
    #Now move safely to x,y position of object
    moveTo(x,y,SAFE_Z,phi)
    #move slowly to object
    moveTo(x,y,z+15,phi)

    #to see how close we are
    moveTo(x,y,z+13,phi)
    #return

    #lower arm & grab
    moveTo(x,y,z,phi)
    closeGrip()
    return x,y,z,phi

#TODO use an object to represent the x,y,z,phi coordinates:
#   we could use operator for dicts, itemgetter; dataclasses etc...
#   To define later on...
#   or we can just keep using x,y,phi and being careful...
def stackObjects(positions=[],endPosition = [400,400,TABLE_Z,90]):
    """"Expects positions as [x,y,phi], a tuple should also work"""
    global currPos
    e_x,e_y,e_z, e_phi = endPosition
    
    ourPrint('',f"[stackObjects] Starting with positions: {positions}")
    for idx,pos in enumerate(positions):
        x,y,phi = pos
        #Go & grab object
        goGrabObj(x,y,phi=phi)
        #return
        raiseArm()

        #Move (safely above to end position)
        moveTo(e_x,e_y, SAFE_Z, e_phi)

        #position cube in table & drop
        extraZ = idx*Z_CUBE +2#mm #extra height to stack them..
        moveTo(e_x,e_y, e_z+extraZ, e_phi)
        openGrip()
    ourPrint('',f"[stackObjects] Finished sending commands...")
    return currPos


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
        cv2.imwrite(f"{outputFolder}image{formatted}.jpg",image)
        self.get_logger().info("debug")
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
                    ourPrint(self,f'No centroids found. Maybe objects were not present in the camera range')
                    return 0,0,0
                else:
                    #return x,y, angle
                    output.append([x,y,angle])
            ourPrint(self,f'No centroids found. Maybe objects were not present in the camera range')
            return output


        self.get_logger().info(f"Processing image")
        try:
            #x,y,phi = process_image(image)
            points = process_image(image)
        except Exception as e:
            ourPrint(self,f"Something happened: {e}")
            #x,y,phi = 0,0,0
            points = []


        # x1 = math.cos(math.radians(50))*x/2
        objectPoints = []
        for point in points:
            x, y, phi = point
            ycam = (((-y)+960))
            ourPrint(self,f"x : {x}, y: {ycam}")
            x1= ycam/(2.57) + 150 #160
            y1 = (-x)/(2.57)  + 512 #506
            #/math.cos(math.radians(50))
            phi = math.degrees(phi) 
            # phi = phi % 45
            ourPrint(self,f"phi is {phi}")
            #correction applied on the center of the image
            #   where the mass will make the diagonal the bigger mass point
            # if(3*1280/10 < x < 7*1280/10
            #     and 3*960/10 < y < 7*960/10
            #     and 30 <= phi <= 60):
            #     angle_offset= 45#45
            #else: angle_offset = 0
            phi1 = 90.00 + phi #- angle_offset
            ourPrint(self,f"{x1}{y1}{phi1}")
            ourPrint(self,"converted values:")
            ourPrint(self,f"x1: {x1}")
            ourPrint(self,f"y1: {y1}")
            ourPrint(self,f"phi1: {phi1}")
            #objectPoints.append([x1,y1,phi1])
            x1,y1 = cameraToRobotXYZ(x,y)
            objectPoints.append([x1,y1,phi])

        # # 1 mm = 3 pixels
        # # 10 cm = 300 pixels
        
        #stackObjects([[x1,y1,phi1]])
        stackObjects(objectPoints)
        # script = "PTP(\"CPP\","+targetP+",100,200,0,false)"
        # send_script(script)
        #set_io(1.0)
        #targetP1 = f"{x1}, {y1}, 130, -180.00, 0.0, {phi1}"
        # targetP1 = "250.00, 250, 120, -180.00, 0.0, 135.00"
        #script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        #send_script(script1)
        #set_io(1.0)
########################################################################################################################

def main(args=None):
    rclpy.init(args=args)
    node = ImageSub('image_sub')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()