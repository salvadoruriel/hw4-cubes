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