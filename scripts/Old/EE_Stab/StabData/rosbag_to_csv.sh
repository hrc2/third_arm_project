#!/bin/bash

fname='Painting_No_Pred_2020_08_05'

bagfile='wrf_stab_test_bag_2020-08-05-19-54-28.bag'
echo "Bag File being processed is: $bagfile"

outpath="Stab_Proc/$fname"
mkdir -p $outpath

robot_cmd_topics=('/base_swivel_controller/command' '/vertical_tilt_controller/command' '/arm_extension_controller/command' '/wrist_controller/command' '/wrist_tilt_controller/command' '/gripper_controller/command')
robot_cmd_out_fnames=('dof1_cmd.csv' 'dof2_cmd.csv' 'dof3_cmd.csv' 'dof4_cmd.csv' 'dof5_cmd.csv' 'dof6_cmd.csv')

robot_st_topics=('/base_swivel_controller/state' '/vertical_tilt_controller/state' '/arm_extension_controller/state' '/wrist_controller/state' '/wrist_tilt_controller/state' '/gripper_controller/state')
robot_st_out_fnames=('dof1_state.csv' 'dof2_state.csv' 'dof3_state.csv' 'dof4_state.csv' 'dof5_state.csv' 'dof6_state.csv')

tag_topics=('/mocap_node/wrf_base/pose' '/mocap_node/end_eff/pose' '/mocap_node/elbow/pose' '/mocap_node/hand/pose' '/mocap_filtered')
tag_out_fnames=('base_pose.csv' 'ee_pose.csv' 'elbow_pose.csv' 'hand_pose.csv' 'mocap_filt.csv')


i=0
while [ $i -lt ${#robot_cmd_topics[@]} ]
do
    rostopic echo -b "$bagfile" -p ${robot_cmd_topics[i]} > "$outpath/${robot_cmd_out_fnames[i]}"
    rostopic echo -b "$bagfile" -p ${robot_st_topics[i]} > "$outpath/${robot_st_out_fnames[i]}"
    ((i++))  
done

i=0
while [ $i -lt ${#tag_topics[@]} ]
do
    rostopic echo -b "$bagfile" -p ${tag_topics[i]} > "$outpath/${tag_out_fnames[i]}"
    ((i++))  
done

echo "All Bags written to CSV"
