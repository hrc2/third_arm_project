# Wearable Robotic Third Arm

## Overview
This repository provides the necessary tools to control and develop libraries to control the Wearable Robotic Third Arm (WRF).
Currently, the code to control the robot is in the form of a ROS package, supporting ROS Noetic. We have plans to add support for ROS 2 (Humble) as well.
Launch files are provided to run the robot on hardware or in simulation, as well as optionally with an Optitrack Mo-Cap setup (more details below).

## Setup

### Dependencies and Setup
You should have ROS-Noetic installed. Clone the repo into a catkin workspace.
PyPot is used to control the Dynamixel Motors. Install PyPot: `python3 -m pip install PyPot`    
<!-- Install PyPot (Python 2.7): `pip install PyPot==3.3.1`   -->
Temporarily set USB port permissions, example for ACM0 usb port using: `sudo chmod 0777 /dev/ttyACM0`  
Alternatively, add the current user to the dialout group: `sudo adduser $USER dialout`
Build the catking workspace: `catkin_make`

## Running URDF with joint state publisher GUI and RVIZ
The five joints are defined in the file urdf/third_arm_5dof.urdf as follows:
* Base Link <--Horizontal Panning (Revolute)--> Base Motor
* Base Motor <--Vertical Pitching (Revolute)--> Lift Motor
* Extend Link 1 <--Length Extension (Prismatic)--> Extend Link 2
* Extend Link 2 <--Wrist Rotation (Revolute)--> Wrist Rotation Motor
* Wrist Rotation Motor <--Wrist Pitching (Revolute)--> Wrist Tilt Motor

To launch an rviz visualization of the robot with GUI controls for those joints, build and source the workspace, then run `roslaunch third_arm urdf_5dof.launch`


## Running the Third Arm in Simulation
To launch an empty gazebo world with the third arm in it, run:
`roslaunch third_arm third_arm_gazebo.launch`

Currently, there are no actuators or controllers hooked up to joint; this is in progress.

## Running the Code on Third Arm Hardware

### Robot Details
All of the actuators in the third arm are Dynamixel servomotors, (MX-64, MX-64, MX-28, MX-28, and AX-12) respectively.
They are daisy-chained on a single serial bus via a USB2AX module and powered with a 12V supply (recommended 10A rating).
Note that the USB2AX is no longer produced or avaiable to purchase. However, a very similar setup can be achieved using a U2D2 with the accompanying U2D2 power board. 

To control the robot using the provided libraries, do the following:
1. Plug the 12V supply in to the USB2AX or U2D2.
2. Connect the USB cable fromt he USB2AX or U2D2 to your computer.
3. Source your catkin workspace.
4. Run `roslaunch third_arm third_arm_noetic.launch`.

If you have an optitrack motion-tracking system, you can use our provided code with our experimental "online" controller. This uses a PID loop to update the servo velocities according towards the current location of a tracked object (e.g. a person's hand).

Otherwise, motion-tracking is disabled by default. You can specify a target location and the robot will move to the correct joint angles to realize that pose.


## Old README contents
Plug in Power FIRST  
Plug in USB  
`export ROS_MASTER_URI=http://192.168.0.124:11311`
`export ROS_IP=192.168.0.124`  
`cd third_arm_noetic_ws/`  
`source devel/setup.bash`  
`roslaunch third_arm third_arm_noetic.launch`  




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
