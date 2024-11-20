# Setup
Brief explanation of setup & file structure created

# Robot arm setup
To setup the robot environment:
Follow the [TA's tutorial].
Something like this...
```
#check you are in home dir
ls
#Step 1
mkdir -p ~/workspace2/team14_ws/src
cd workspace2/team14_ws
#step 2
#...
#step 3
#Setup...
```

# Folder structure
Our workspace will look something like this:
~/workspace2/team14_ws
- build
- install
- log
- src
	- build
	- install
	- log
	- send_script
		- __init__.py
		- **image_su.py**
		- **send_script.py**
	...
	- **setup.py**

We should only edit the bolded image_su.py & send_script.py files.
_And do the colcon build each time you do..._




---
[TA's tutorial]: https://hackmd.io/@opk9oqlVTEyViBxe4naBEg/HJbE6Vi7T/%2Fni5q3v8kQ9C0Jw_shnqt-A