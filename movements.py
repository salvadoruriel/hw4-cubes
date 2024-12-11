from .utils import(
    OUTPUTFOLDER,DEFAULT_THETA,
    DEFAULT_RHO,TABLE_Z,SAFE_Z,
    ourPrint)
#from image_sub import (set_io, send_script)
from .send_script import (set_io, send_script)
"""
This file exists as a separate copy of the movements to do...
TODO:
Import send_script & set_io properly from send_script to make this file work
"""

#File with all movements for the robot
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

def goFeed(spoonPos,bowlPos):
  goGrabObj(spoonPos)
  raiseArm()
  #1. go to center of bowl
  moveTo(bowlPos)
  #2. ⁠move offset towards robot by
  #		bowl radius/half square length)
  #		+ length of spoon
  #		change angle Rx to -155°
  #3. ⁠lower arm to bowl  height + extra height for scooping (e.g. 210mm)
  #		move length of spoon / 2 (towards bowl center)
  #		lower “slowly” to 188mm
  #		move to border of bowl (length of spoon)
  #4. rotate Rx to -180°

  #5. ⁠raise arm
  #6. move towards cat

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

def play(seconds):
  #1. Rx : -150°
  moveTo()
  #2.  ⁠Rz : 15 -> 60°
  #3. ⁠repeat 2 back and forth
  #TOMEASURE TIME FOR ACTION

#laser: move 15* towards 180
#def playLaser():

