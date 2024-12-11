#test your python code here before copy pasting to the main script

import os
import time

#from .send_script import (ourPrint)
from .consts import (OUTPUTFOLDER)

def foo():
    print("YOoooooooooo!")
    ourPrint("YOooooooooooooooo!")


### Printing

print(f"outputFolder: {OUTPUTFOLDER}")
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
