#!/bin/bash

# Copyright (c) 2018 SicTeam.
# TODO needs appropriate licensing.
# This is a modified ROS install script from https://github.com/jetsonhacks/installROSTX1/
# And https://github.com/NVIDIA-Jetson/redtail/blob/master/ros/scripts/jetson_ros_install.sh


red=`tput setaf 1`
reset=`tput sgr0`

# Should not run this script as sudo.
if [ "$EUID" = 0 ]; then
    echo "${red}Please run this script as a non-root user.${reset}"
    exit
fi

echo "This script will install in this order: "
echo "  1. OpenCV3.2 and its dependencies "
echo "  2. ROS Base and Required Packages for skyNET "
#echo "  3. Other stuff "

#Install OpenCV and its dependencies
# This part of the script is from https://raw.githubusercontent.com/jetsonhacks/buildOpenCVTX1/master/buildOpenCV.sh
echo "1. Installing OpenCV3.2... " 
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
# TODO Gstreamer "plugins-bad" needed for ROS?

cd $HOME
OPENCV_BUILD=$HOME/opencv_build
OPENCV_VERSION="3.2.0"
SKIP_OPENCV=false
OPENCV_CMAKE=cmake \
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
    -DBUILD_opencv_python3=OFF \
    -DENABLE_PRECOMPILED_HEADERS=OFF \
    -DWITH_OPENCL=OFF \
    -DWITH_OPENMP=OFF \
    -DWITH_FFMPEG=ON \
    -DWITH_GSTREAMER=ON \
    -DWITH_GSTREAMER_0_10=OFF \
    -DWITH_CUDA=ON \
    -DWITH_GTK=ON \
    -DWITH_VTK=OFF \
    -DWITH_TBB=ON \
    -DWITH_1394=OFF \
    -DWITH_OPENEXR=OFF \
    -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-8.0 \
    -DCUDA_ARCH_BIN=5.3 \
    -DCUDA_ARCH_PTX="" \
    -DINSTALL_C_EXAMPLES=ON \
    -DINSTALL_TESTS=ON \
    -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules \
    -DOPENCV_TEST_DATA_PATH=../opencv_extra/testdata \
    ../


# Check for existance of opencv
if [ "$(pkg-config --cflags opencv)" = "-I/usr/include/opencv" ]; then
    echo "OpenCV found, checking version"
    if [ "$(pkg-config --modversion opencv)" != "$OPENCV_VERSION" ]; then
        echo "${red}Need $OPENCV_VERSION, different version found uninstall and try again.. ${reset}"
        exit
    else
        echo "OpenCV $OPENCV_VERSION is already installed, skipping"
        SKIP_OPENCV=true
    fi
fi

echo "OpenCV not found, preparing to build"

if [ ! -d "$OPENCV_BUILD" && "$SKIP_OPENCV" = false ]; then
    mkdir -p $OPENCV_BUILD
    cd $OPENCV_BUILD
    echo " Cloning opencv.."
    git clone https://github.com/opencv/opencv.git
    cd opencv
    git checkout -b v$OPENCV_VERSION $OPENCV_VERSION

    cd $OPENCV_BUILD
    git clone https://github.com/opencv/opencv_extra.git
    cd opencv_extra
    git checkout -b v$OPENCV_VERSION $OPENCV_VERSION

    cd $OPENCV_BUILD
    git clone https://github.com/opencv/opencv_contrib.git
    cd opencv_contrib
    git checkout -b v$OPENCV_VERSION $OPENCV_VERSION
elif [ "$SKIP_OPENCV" = false ]; then 
    echo " Updating opencv sources "

    cd $OPENCV_BUILD/opencv
    git checkout -b v$OPENCV_VERSION $OPENCV_VERSION
    git pull

    cd $OPENCV_BUILD/opencv_extra
    git checkout -b v$OPENCV_VERSION $OPENCV_VERSION
    git pull

    cd $OPENCV_BUILD/opencv_contrib
    git checkout -b v$OPENCV_VERSION $OPENCV_VERSION
    git pull
fi

if [ "$SKIP_OPENCV" = false ]; then
    # Build opencv from source
    cd $OPENCV_BUILD/opencv
    mkdir build
    cd build
    exec $OPENCV_CMAKE

    # Consider running jetson_clocks.sh before compiling
    make -j4

    sudo make install

    # Verify install
    if [ "$(pkg-config --modversion opencv)" != "$OPENCV_VERSION" ]; then
        echo "${red}OpenCV v$OPENCV_VERSION not successfully installed check 'pkg-config --modversion opencv' .. ${reset}"
        exit
    fi

    make clean

    # TODO Can this remove be done in a safer manner?
    cd $HOME
    sudo rm -rf $OPENCV_BUILD

else
    echo "Skipped OPENCV"
fi
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

sudo apt-get install -y ros-kinetic-vision-opencv 
#ros-kinetic-PACKAGE \

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

#fi

#echo "3. No other stuff yet"

