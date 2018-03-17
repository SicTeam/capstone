#!/bin/bash
#Author: Samuel Strba
#Email: sstrba@pdx.edu
#Run this script when you are running the multiple drones Gazebo simulation.
#The script grabs the video feeds from the cameras in Gazebo and saves them to .avi files in the ~/Videos folder.
rosrun image_view video_recorder image:=/sd5_1/camera_sd5_left/image_raw _filename:=$HOME/Videos/output_left.avi _codec:=MJPG &
rosrun image_view video_recorder image:=/sd5_1/camera_sd5_right/image_raw _filename:=$HOME/Videos/output_right.avi _codec:=MJPG &
rosrun image_view video_recorder image:=/sd5_1/camera_sd5_rear/image_raw _filename:=$HOME/Videos/output_rear.avi _codec:=MJPG
