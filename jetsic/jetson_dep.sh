#!/bin/bash

# Copyright (c) 2018 SicTeam.
# TODO needs appropriate licensing.
# This is a modified ROS install script from https://github.com/jetsonhacks/installROSTX1/
# And https://github.com/NVIDIA-Jetson/redtail/blob/master/ros/scripts/jetson_ros_install.sh

echo "This script will install in this order: \n"
echo "\t1. OpenCV3.3 and its dependencies \n"
echo "\t2. ROS Base and Required Packages for skyNET \n"
echo "\t3. Other stuff \n" 

#Install OpenCV and its dependencies
# This part of the script is from https://raw.githubusercontent.com/jetsonhacks/buildOpenCVTX1/master/buildOpenCV.sh
echo "\t1. Installing OpenCV3.3... " 
cd $HOME
sudo apt-get install -y \
    libglew-dev \
    libtiff5-dev \
    zlib1g-dev \
    libjpeg-dev \
    libpng12-dev \
    libjasper-dev \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libpostproc-dev \
    libswscale-dev \
    libeigen3-dev \
    libtbb-dev \
    libgtk2.0-dev \
    cmake \
    pkg-config

# Python 2.7
sudo apt-get install -y python-dev python-numpy python-py python-pytest
# GStreamer support
sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev 


git clone https://github.com/opencv/opencv.git
cd opencv
git checkout -b v3.3.0 3.3.0
# This is for the test data
cd $HOME
git clone https://github.com/opencv/opencv_extra.git
cd opencv_extra
git checkout -b v3.3.0 3.3.0

cd $HOME/opencv
mkdir build
cd build
# Jetson TX1 
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr \
    -DBUILD_PNG=OFF \
    -DBUILD_TIFF=OFF \
    -DBUILD_TBB=OFF \
    -DBUILD_JPEG=OFF \
    -DBUILD_JASPER=OFF \
    -DBUILD_ZLIB=OFF \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_opencv_java=OFF \
    -DBUILD_opencv_python2=ON \

# Update the repositories to include universe, multiverse and restricted

echo "Prep for ROS install, updating respositories, adding: universe, multiverse, and restricted..."
sudo apt-add-repository universe
sudo apt-add-repository multivere
sudo apt-add-repository restricted
sudo apt-get update

# ROS kinetic install. Taken from http://wiki.ros.org/kinetic/Installation/Ubuntu with minor modifications
# including custom package selection

echo "2. Installing ROS Kinetic "

# Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup your keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Update package list
sudo apt-get update

# Install ROS base and MAVROS
sudo apt-get install -y ros-kinetic-ros-base ros-kinetic-mavros ros-kinetic-mavros-extras

sudo apt-get install -y ros-kinetic-vision-open_cv \
                        #ros-kinetic-PACKAGE \
                        ros-kinetic-vision-opencv3  

# For some reason, SSL certificates get messed up on TX1 so Python scripts like rosdep will fail. Rehash the certs.
sudo c_rehash /etc/ssl/certs

# MAVROS requires GeographicLib datasets starting v0.20 .
sudo geographiclib-get-geoids egm96-5

# Initialize rosdep
sudo rosdep init
rosdep update

# Environment setup - optional. Do not run if multiple versions of ROS are present.
echo "source /opt/ros/kinetic/setup.bash" >> $HOME/.bashrc
source /opt/ros/kinetic/setup.bash


echo "3. No other stuff yet"

# TODO add more ros packages
# TODO stopped on line 51 of redtail/jetson_install.sh
