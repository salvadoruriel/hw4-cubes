#####################################3
# pseudocode:
#######################################

###### Definitions:
# def moveTo(x,y,z,ANGLE?):
# def grabObj(x,y,z):
### Raise arm to z=50
### moveTo(x,y)
### Lower arm to z=1
### closeGrip()
### 
# z_cube = 2.5cm
# pos1 = (50,50,0)

#######################################
#######################################
######## Code
# ArrObjsPosition = ScanObjs()
# [(x,y,z),... ]

# grabObj(cube1)
# moveTo(pos1)
# openGrip()

# grabObj(cube2)
# moveTo(pos1 + z_cube + 1mm)
# openGrip()

# grabObj(cube3) 
# moveTo(pos1 + z_cube*2 + 1mm)
# openGrip()