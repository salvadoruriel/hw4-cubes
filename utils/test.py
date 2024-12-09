import numpy as np
## Testing correct output


x_base2cam = 227#-135
y_base2cam = 705#360
z_base2cam = 0
#from centroid's
x_in, y_in = 730,634
#x_in, y_in = 665, 528 # Is 281, 273 on robot
#x_in, y_in = 145, 179 # Is 215.95, 589.77 on Robot
#x_in = 0 #should output camera's x origin
#y_in = 0 #should output camera's y origin
#Pixel ratio is ALWAYS needed (except for calibration apparently...)
XPIXEL_TO_MM = 1 /2
YPIXEL_TO_MM = 1 /2
XPIXEL_TO_MM = 660/1280 # mm / 1280
YPIXEL_TO_MM = 495/960 # /960

Z_CUBE = 25#mm

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
    tvecs = np.array([[-135,360,0]]).T
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

cameraToRobotXYZ(x_in,y_in)