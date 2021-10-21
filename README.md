run motor rosservice call /dynamixel_workbench/dynamixel_command "command: ''
id: 6
addr_name: ''
value: 500" 

Goal_Position


<node name="control_motors" pkg="third_arm" type="control_motors.py" output="screen"/>
https://www.theconstructsim.com/morpheus-chair-dynamixel-servos-with-robot-arm-ros-s4-ep-1/

TO RUN:
cd third_arm_noetic_ws/
source devel/setup.bash
roslaunch third_arm third_arm_noetic.launch

catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3

TO RUN MOTIVETRACKER:
Switch to HRC2 network
source devel/setup.bash
roslaunch mocap_optitrack mocap.launch
rostopic echo /tf


Goal_Position - 0 2048 4095 but depends on motor

https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
https://emanual.robotis.com/docs/en/dxl/protocol2/

if port not found make sure using port 1 from left on back looking from behind
if still not work, shut down all terminals and unplug and replug and redo scripts

base_swivel: 1
    back: 0, 4000
    forward: 2000
    left: 1000
    right: 3000

vertical_tilt: 2
    bottom: 0
    top: 3500
	
#Extension should be in multi-turn mode        
arm_extension: 3
    min:1850
    max:0
        
wrist: 4
    500 - rotate right
    level, upright- 0
    max - 1000, level upright

wrist_tilt: 5
    min - 0
    level - 500
    max - 1000

gripper: 6 
    open - 2000
    closed - 2700
    
[ INFO] [1634248959.076008682]: id : 1, model name : MX-64
[ INFO] [1634248959.076013415]: id : 2, model name : MX-64
[ INFO] [1634248959.076017586]: id : 3, model name : MX-28
[ INFO] [1634248959.076021599]: id : 4, model name : AX-12A
[ INFO] [1634248959.076025555]: id : 5, model name : AX-12A
[ INFO] [1634248959.076029448]: id : 6, model name : MX-28



write a service to use dynamixel command - done
command arm through dynamixel command - done
write a publisher for JointTrajectory - wip
	- write a service to use dynamixel execution - wip




http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

try to send a velocity profile 0 to 32737, at 10 Hz to update

be in velocity control mode and set to slow

and then actuate it back and backforth
