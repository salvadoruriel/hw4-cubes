#!/usr/bin/env python

import rclpy
import time
import sys
import cv2
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *

from .utils import(INPUTFOLDER,ourPrint)

# arm client
def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()

# gripper client
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

    while True:
        rclpy.init(args=args)

    #--- move command by joint angle ---#
    # script = 'PTP(\"JPP\",45,0,90,0,90,0,35,200,0,false)'

    #--- move command by end effector's pose (x,y,z,a,b,c) ---#
    # targetP1 = "398.97, -122.27, 748.26, -179.62, 0.25, 90.12"s

    # Initial camera position for taking image (Please do not change the values)
    # For right arm: targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
    # For left  arm: targetP1 = "350.00, 350, 730, -180.00, 0.0, 135.00"
    # targetP1 = "250.00, 250, 550, -180.00, 0.0, 135.00"
        targetP1 = "250.00, 250, 550, -180.00, 0.0, 90.00"

        # targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
        script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        #set_io(1.0)
        send_script(script1)

# What does Vision_DoJob do? Try to use it...
# -------------------------------------------------
    # while True:
    #     try:
    #         #save a command to a file before calling vision Do_job
    #         with open(f"{INPUTFOLDER}commands.txt", "rw") as file:
    #             command = file.read()
    #             if command:
    #                 ourPrint("","[send_script] File 'commands.txt' is not empty, waiting...")
    #                 time.sleep(6)
    #                 continue
    #             else:
    #                 file.write("play")
    #                 #file.write("feed")
    #         send_script("Vision_DoJob(job1)")
    #     except Exception as e:
    #         ourPrint("","Error opening file")

    #     time.sleep(3)

        send_script("Vision_DoJob(job1)")
        cv2.waitKey(1)

#--------------------------------------------------

    #set_io(1.0) # 1.0: close gripper, 0.0: open gripper
    # set_io(0.0)
        rclpy.shutdown()
        time.sleep(6)

if __name__ == '__main__':
    main()




