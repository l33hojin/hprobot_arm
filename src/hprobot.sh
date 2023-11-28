#!/bin/bash


source ~/opt/ros/noetic/setup.bash
source /home/hprobot/interbotix_ws/devel/setup.bash
#source ~/ws_hprobot/devel/setup.bash
#source ~/ws_realsense/devel/setup.bash
source ~/hprobot_arm_ws/devel/setup.bash
#roslaunch interbotix_xsarm_moveit xsarm_moveit.launch use_fake:=true robot_model:=vx300 use_hprobot_node:=true dof:=5 use_moveit_rviz:=true

roslaunch interbotix_xsarm_moveit xsarm_moveit.launch use_actual:=true robot_model:=vx300 use_hprobot_node:=true dof:=5 use_moveit_rviz:=false
roslaunch mqtt_client standalone.launch
