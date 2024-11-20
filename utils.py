# Abstracted utils
#		gateway API to keep names consistent in the main code
from send_script import (set_io)

def OpenGrip():
	set_io(0)
	
def CloseGrip():
	set_io(1.0) #1.0 close

def GrabObj():
	OpenGrip()
	#moveTo
	CloseGrip()
	
