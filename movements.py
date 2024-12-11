from image_sub import (set_io, send_script)

#from .movements import (openGrip, closeGrip, moveTo, raiseArm, goGrabObj, goFeed)
CENTERPOS = ()
TABLE_Z = 110#mm  #Approx what the gripper to table position would be, 100mm is gripper length
SAFE_Z = 500#mm   #USE THIS ONE FOR MOVING AROUND

#File with all movements for the robot
def openGrip():
    set_io(0.0)
def closeGrip():
    set_io(1.0) #1.0 close

def moveTo(x,y,z=SAFE_Z,phi=90):
    """move safely to the position"""
    global currPos

    targetP = f"{x}, {y}, {z}, -180.00, 0.0, {phi}"
    script = "PTP(\"CPP\","+targetP+",100,200,0,false)"
    send_script(script)

    currPos = (x,y,z,phi)
    return x,y,z,phi #return as current position

def raiseArm():
    """Raises... the... arm... probs~ iunno"""
    global currPos

    x,y,_,phi = currPos
    return moveTo(x,y,SAFE_Z,phi)

def goGrabObj(x,y,z=TABLE_Z,phi=90):
    """Moves to given position FROM ABOVE & grabs object"""
    #First raise the arm to avoid collision
    raiseArm()
    openGrip()
    #Now move safely to x,y position of object
    moveTo(x,y,SAFE_Z,phi)
    #move slowly to object
    moveTo(x,y,z+15,phi)

    #to see how close we are
    moveTo(x,y,z+13,phi)
    #return

    #lower arm & grab
    moveTo(x,y,z,phi)
    closeGrip()
    return x,y,z,phi

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
     
def play(seconds):
  #1. Rx : -150°
  moveTo()
  #2.  ⁠Rz : 15 -> 60°
  #3. ⁠repeat 2 back and forth
  #TOMEASURE TIME FOR ACTION

#laser: move 15* towards 180
#def playLaser():

