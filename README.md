# Wearable Robotic Third Arm

## Goal:

We propose to develop an online controller for the third arm, which will continuously adapt the robot’s motion based on the 
observed human motion. The key challenge is to avoid collisions with the human body and any other obstacles in the workspace. Also, since the third arm is underactuated (5 DoF) this adds to the difficulty in computing online joint-space trajectories towards the target location. Finally, the controller needs to compensate for the motion of the base arm.

## Running the Code on third-arm-computer
`cd third_arm_noetic_ws/`  
`source devel/setup.bash`  
`roslaunch third_arm third_arm_noetic.launch`  

## What to Run when Building
`catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3`  

## Running MotiveTracker
Switch to HRC2 network  
`source devel/setup.bash`  
`roslaunch mocap_optitrack mocap.launch`  
`rostopic echo /tf`  

## Debugging Connections

Port not found:
- Make sure using a port on the back of the PC, 1 from the left looking from behind. There is a USB extension connected to it for convenience.  

Error finding Dynamixel Motor:
- Shut down all terminals  
- Unplug WRTA from computer and power  
- Plug in power  
- Plug in USB cable  
- Rerun scripts  


### Notes and Links
https://emanual.robotis.com/docs/en/dxl/protocol2/  
Goal_Position range - 0 2048 4095 but depends on motor limits  

[Example use of servos in ROS](https://www.theconstructsim.com/morpheus-chair-dynamixel-servos-with-robot-arm-ros-s4-ep-1/)

Motors and their models:
id : 1, model name : MX-64  
id : 2, model name : MX-64  
id : 3, model name : MX-28  
id : 4, model name : AX-12A  
id : 5, model name : AX-12A  
id : 6, model name : MX-28  


### Tasks:

write a service to use dynamixel command - done  
command arm through dynamixel command - done  
write a publisher for JointTrajectory - wip  
	- write a service to use dynamixel execution - wip  
Set WRTA dynamixel wrist tilt motor to wheel mode and command a slow velocity to it

