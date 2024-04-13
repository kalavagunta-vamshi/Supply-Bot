GitHub link for package: https://github.com/Sameer-Arjun-S/RobotModelling-Project2 

Contents 
1. Assembly 
2. Package 
Dependencies \
* python 3.9 \[works for any python 3 version\] \
* IDE \[I used PyCharm IDE to program the Code
and Execute the Code\] and Jupiter. \
* Libraries: numpy, matplotlib.pyplot,Sympy,math 

How to run the code 
--\> Create a
catkin\_ws and build it, then source it 
--\> Download the package --\>
Paste the package in the source directory 
--\> Build and source the workspace 
--\> Commands to run (open separate terminals and run in the same order)


NEW TERMINAL

cd catkin\_ws 
catkin\_make  
source devel/setup.bash
roslaunch supplybot template\_launch.launch

New terminal(ctrl+shift+t) to run the python code to control mobile base\# 
cd src/scripts/ rosrun supplybot teleop\_template.py


New terminal(ctrl+shift+t) to run the python code for manipulator\#
* rosrun supllybot manipulator_idle.py - THis only brings the robot to home position
* rosrun supplybot teleop_template.py - This enables manual control of the robot
* rosrun supplybot publisher.py - This enables forward motion of the robot
* rosservice call /robot/left_vacuum_gripper/on - This turns vaccum gripper to on state
* rosrun supplybot manipulator_pick.py - This initiates pick action in robot
* rosrun supplybot manipulator_place.py - This initiates place action in robot
* rosservice call /robot/left_vacuum_gripper/off - This turns vaccum gripper to off state



New terminal(ctrl+shift+t) 
* rviz
