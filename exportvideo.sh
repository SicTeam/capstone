#!/bin/bash

rosrun image_view video_recorder image:=/sd5_1/camera_sd5_left/image_raw _filename:=$HOME/Videos/output_left.avi _codec:=MJPG &
rosrun image_view video_recorder image:=/sd5_1/camera_sd5_right/image_raw _filename:=$HOME/Videos/output_right.avi _codec:=MJPG &
rosrun image_view video_recorder image:=/sd5_1/camera_sd5_rear/image_raw _filename:=$HOME/Videos/output_rear.avi _codec:=MJPG

#todo: add _fps:=30 parameter when the input image feed is also 30 fps
