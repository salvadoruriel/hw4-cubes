# Abstracted utils
# TODO: test if importing from other files works in this robot
#         or if that can be done...

###########
#### CONST (might better to just copy&paste them into the file always...)
###########
TABLE_Z = 135#mm  #Approx what the gripper to table position would be, 100mm is gripper length
SAFE_Z = 500#mm   #USE THIS ONE FOR MOVING AROUND
Z_CUBE = 25#mm

###########
#### Functions
###########
def openGrip():
	set_io(0)
	
def closeGrip():
	set_io(1.0) #1.0 close

def grabObj(position):
	openGrip()
	#moveTo
	closeGrip()

#TODO really needed?
def saveImg():
	None

import os
import time
cwd = os.getcwd()
print(f"cwd: {cwd}")
#cwd is too far up,
#  navigate to the output folder close to the files we edit:
outputFolder = f"{cwd}/src/send_script/send_script/output/"
print(f"outputFolder: {outputFolder}")
now = time.time()
today = time.strftime('%Y-%m-%d',time.localtime(now))
logfile = f"{today}-log.txt"

def ourPrint(self='',string="",log=True):
    """Printing & logging function...
    call as: 
      ourPrint(self,"Text")
      ourPrint('',"Text")
    """
    now = time.time()
    hour = time.strftime('%H:%M:%S',time.localtime(now))
    
    if self:
        self.get_logger().info(string)
    else:
        print(string)
    if not log: return
    with open(f'{outputFolder}{logfile}', 'a') as file:
        file.write(f'[{hour}] {string}\n')



#####################################################
####################################################
## Math
##########
def cameraToRobotXYZ(x,y):
    """
    Input= centroid's x,y values 
    return = robot tool's x,y coords
    """
    # Camera intrinsics
    fx = 800  # focal length in x direction
    fy = 800  # focal length in y direction
    cx = 320  # principal point x-coordinate
    cy = 240  # principal point y-coordinate
    # Image centroid and depth
    u, v = x,y # Object's centroid
    Z = 1  #Depth, we require another/camera sensor for this value
    # Back-project to 3D in camera frame
    X_camera = (u - cx) * Z / fx
    Y_camera = (v - cy) * Z / fy
    Z_camera = Z
    # 3D point in the camera frame
    P_camera = np.array([X_camera, Y_camera, Z_camera, 1])

    ######### Frames
    #we saw a ~45 degree rotation from robot to camera
    theta = np.deg2rad(45)#counter clockwise
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    # Rotation around Z-axis:
    R = np.array([
        [cos_theta, -sin_theta, 0, 0],
        [sin_theta,  cos_theta, 0, 0],
        [0,          0,         1, 0],
        [0,          0,         0, 1]
    ])
    x_base2cam = -135
    y_base2cam = 360
    # Translation
    T = np.array([
        [1, 0, 0, x_base2cam],#x
        [0, 1, 0, y_base2cam],#y
        [0, 0, 1, 1],#z
        [0, 0, 0, 1]
    ])

    # Transformation matrix
    T_camera_to_robot = T @ R

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



############################################################
############################################################
##### Unused
############################################################
###########
#### Classes
###########
class Position:
    def __init__(self,x: int,y: int, z: int, phi: int):
        self.x = x
        self.y = y
        self.z = z
        self.phi = phi
#adjusting objects to be js~like for easy access
class jsobj(object):
    def __init__(self, _dict):
        self.__dict__.update(_dict)