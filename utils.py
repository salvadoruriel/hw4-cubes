# Abstracted utils
#		gateway API to keep names consistent in the main code
from send_script import (set_io)

def openGrip():
	set_io(0)
	
def closeGrip():
	set_io(1.0) #1.0 close

def grabObj(position):
	openGrip()
	#moveTo
	closeGrip()
	
def saveImg():
	None