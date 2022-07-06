# Wearable Robotic Third Arm

## Goal:

We propose to develop an online controller for the third arm, which will continuously adapt the robot’s motion based on the 
observed human motion. The key challenge is to avoid collisions with the human body and any other obstacles in the workspace. Also, since the third arm is underactuated (5 DoF) this adds to the difficulty in computing online joint-space trajectories towards the target location. Finally, the controller needs to compensate for the motion of the base arm.

## Setting up the code  
Install PyPot using command line (Python 3): `python -m pip install PyPot`    
Install PyPot (Python 2.7): `pip install PyPot==3.3.1`  
PyPot is used to control the Dynamixel Motors  
Set USB port permissions, example for ACM0 usb port using: `sudo chmod 0777 /dev/ttyACM0`  

## Running URDF with joint state publisher GUI and RVIZ
Links:
* Base Link <--Horizontal Panning (Revolute)--> Base Motor
* Base Motor <--Vertical Pitching (Revolute)--> Lift Motor
* Extend Link 1 <--Length Extension (Prismatic)--> Extend Link 2
* Extend Link 2 <--Wrist Rotation (Revolute)--> Wrist Rotation Motor
* Wrist Rotation Motor <--Wrist Pitching (Revolute)--> Wrist Tilt Motor

`roslaunch third_arm urdf_5dof.launch`

## Running the Code on third-arm-computer
Plug in Power FIRST  
Plug in USB  
`export ROS_MASTER_URI=http://192.168.0.124:11311`  
`export ROS_IP=192.168.0.124`  
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


### Tasks:

write a service to use dynamixel command - done  
command arm through dynamixel command - done  
write a publisher for JointTrajectory - wip  
	- write a service to use dynamixel execution - wip  
Set WRTA dynamixel wrist tilt motor to wheel mode and command a slow velocity to it

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

this is for the object even though it says robot
('Robot orientation =', [-0.027261753897265567, 0.8300776298749009, 0.29825615225767416, -0.4703947202043823])
('Robot position =', [0.07514090090990067, 2.003984212875366, 0.14993013441562653])
('Robot orientation =', [-0.027261753897265567, 0.8300776298749009, 0.29825615225767416, -0.4703947202043823])
('Robot position =', [0.07478132843971252, 2.0018575191497803, 0.1488160938024521])
('Robot orientation =', [-0.041804371034848915, 0.8302444516663823, 0.3029710460828637, -0.4659990238992184])
('Robot position =', [0.07478132843971252, 2.0018575191497803, 0.1488160938024521])
('Robot orientation =', [-0.041804371034848915, 0.8302444516663823, 0.3029710460828637, -0.4659990238992184])
('Robot position =', [0.07119814306497574, 2.0003042221069336, 0.1495422124862671])
('Robot orientation =', [-0.05553610425074222, 0.8342336751606665, 0.30679464169761433, -0.4548043141596291])