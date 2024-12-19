from .utils import(
    OUTPUTFOLDER,DEFAULT_THETA,
    DEFAULT_RHO,TABLE_Z,SAFE_Z,
    ourPrint)
#from image_sub import (set_io, send_script)
from .send_script import (set_io, send_script)
import math
"""
This file exists as a separate copy of the movements to do...
TODO:
Import send_script & set_io properly from send_script to make this file work
"""
PHOTO_POS = (250,250,550,-180,0,90) 
currPos = (250,250,550,-180,0,90) 

#File with all movements for the robot
def openGrip():
    set_io(0.0)
def closeGrip():
    set_io(1.0) #1.0 close

def moveTo(x,y,z=SAFE_Z,theta=-180.00,rho=0.0,phi=135):
    """move safely to the position"""
    global currPos
    if z < TABLE_Z:
       z = TABLE_Z

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
    #avoid rising or other movements too fast
    moveTo(x,y,z+1,DEFAULT_THETA,DEFAULT_RHO,phi+1)
    moveTo(x,y,z+2,DEFAULT_THETA,DEFAULT_RHO,phi-1)
    return x,y,z,DEFAULT_THETA,DEFAULT_RHO,phi

def goDropObj(x,y,z=TABLE_Z,theta=-180.00,rho=0.0,phi=90):
    """Moves to given position FROM ABOVE & Drops"""
    print("Grabbing object at ",x,y,z,theta,rho,phi)
    #First raise the arm to avoid collision
    raiseArm()
    #Now move safely to x,y position of object
    moveTo(x,y,SAFE_Z,DEFAULT_THETA,DEFAULT_RHO, phi)
    #move slowly to object
    moveTo(x,y,z+15,DEFAULT_THETA,DEFAULT_RHO,phi)

    #to see how close we are
    moveTo(x,y,z+13,DEFAULT_THETA,DEFAULT_RHO,phi)
    #return

    #lower arm & drop
    moveTo(x,y,z,DEFAULT_THETA,DEFAULT_RHO,phi)
    openGrip()
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

def points_above_below(x0, y0, h, angle):
    """get points to the sides,
        assuming angle 0 is looking to the side like "|"
    """
    # Convert angle to radians
    radians = math.radians(angle)
    
    # Calculate the offsets
    dx = h * math.sin(radians)
    dy = h * math.cos(radians)
    
    # Calculate the above and below points
    above = (x0 + dx, y0 + dy)
    below = (x0 - dx, y0 - dy)
    
    return above, below

# Example usage:
above, below = points_above_below(5, 5, 5, -45)
above, below = points_above_below(250, 250, 350, -(135-45))
print("Above:", above)
print("Below:", below)

def playDynamic(x=250,y=250,z=350,theta=-179,rho=0,phi=135):
  """
  Given a center focus position, it will play around that area
  """
  theta = -179 #ensuring theta is negative and facing to the center
  #180 = -180 for this robot, facing straight
  #downwards is bigger than -180, e.g. (-160 faces down)
  # upwards is SMALLER than 180, e.g (160 faces up)

  #1. Tilt downwards & move toy around
  center = (x,y,z,theta,rho,phi)
  moveTo(*center)

  downwardSwipe1 = (x,y,z,theta+20,rho,phi+35)
  downwardSwipe2 = (x,y,z,theta+20,rho,phi-35)
  moveTo(*downwardSwipe1)
  moveTo(*downwardSwipe2)
  #upwards
  moveTo(*center)
  upSwipe1 = (x,y,z,-1*theta -20,rho,phi+35)
  upSwipe2 = (x,y,z,-1*theta -20,rho,phi-35)
  moveTo(*upSwipe1)
  moveTo(*upSwipe2)

  moveTo(*center)
  #rightLowerCorner = (565,140,200,180,0,60)
  #rightUpperCorner = (565,140,600,180,0,60)
  #leftUpperCorner = (115,665,600,180,0,200)
  #leftLowerCorner = (115,665,200,180,0,200)
  #rightLowerCorner = (x+150,y-100,z-150, theta,0,phi-60)
  #rightUpperCorner = (x+150,y-100,z+250, theta,0,phi-60)
  #leftUpperCorner = (x-150,y+100,z-150, theta,0,phi+60)
  #leftLowerCorner = (115,665,200,180,0,200)
  above,below = points_above_below(x,y,130,-(phi-45))
  x1,y1 = above
  x2,y2 = below
  rightLowerCorner = (x2,y2,z-150, theta,0,phi-60)
  rightUpperCorner = (x2,y2,z+250, theta,0,phi-60)
  leftLowerCorner = (x1,y1,z-150, theta,0,phi+60)
  leftUpperCorner = (x1,y1,z+250, theta,0,phi+60)
  #Rectangle Movement
  moveTo(*rightLowerCorner)
  moveTo(*rightUpperCorner)
  moveTo(*leftUpperCorner)
  moveTo(*leftLowerCorner)
  #X movement
  moveTo(*rightUpperCorner)
  moveTo(*rightLowerCorner)
  moveTo(*leftUpperCorner)
  moveTo(*leftLowerCorner)
  moveTo(*center)
