# Robotics 2024 Homework 4
Repository for the Homework 4: Moving the arm
**Just copy code/files from the folder into the working script**

# Robot arm setup
Check Setup.md
Then go into the folder:
`cd workspace2/team14_ws`

# ssh
We can set a connection to the robot with ngrok:
```
ngrok config add-authtoken YOUR_AUTHTOKEN
ngrok tcp 22
```
Then just ssh from your machine:
`ssh <username>@<url> -p <PORT>`

# Robot turning on
The robot should be on & rdy to operate...
Try the manual configuration if its not ready &/or doing anything after running all terminals.

---
# Running
Get 4 terminals to run these commands (in order):

**Terminal 1 (1st)**
```
cd ~/colcon_ws
source install/setup.bash
ros2 run tm_driver tm_driver <robot_ip>
```
right: 192.168.0.69  
left:  192.168.1.103  
**Terminal 2 (2nd)**  
```
ros2 run tm_get_status image_talker
```

**To run the code:**  
_**Terminal 3:**_
```
cd ~/workspace2/team14_ws
#git clone https://github.com/salvadoruriel/hw4-cubes.git
#git pull
#rebuild workspace
colcon build
source install/setup.bash

#run code
ros2 run send_script img_sub
```
_**Terminal 4: (AFTER img_sub)**_
```
cd ~/workspace2/team14_ws
source install/setup.bash
ros2 run send_script send_script
```

## Debugging
If the robot is stuck try:
```
sudo lsof 
sudo pkill -9 
```
