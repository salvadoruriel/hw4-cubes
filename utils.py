#test your python code here before copy pasting to the main script

import os
import time

##################################
#### CONSTANTS
cwd = os.getcwd()
print(f"cwd: {cwd}")
OUTPUTFOLDER = f"{cwd}/src/send_script/send_script/output/" 
print(f"outputFolder: {OUTPUTFOLDER}")
INPUTFOLDER = f"{cwd}/src/send_script/send_script/input/"

#Right arm (close to door) Values:
DEFAULT_THETA = 180
DEFAULT_RHO = 0
TABLE_Z = 107#mm  #Approx what the gripper to table position would be, 100mm is gripper length
SAFE_Z = 500#mm   #USE THIS ONE FOR MOVING AROUND

############toy is smaller
#spoon= 107

###################################
#Useful Functions
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
    with open(f'{OUTPUTFOLDER}{logfile}', 'a') as file:
        file.write(f'[{hour}] {string}\n')
