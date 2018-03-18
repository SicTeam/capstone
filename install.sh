#!/bin/bash

##Bash script for setting up a PX4 development environment on Ubuntu LTS (16.04).
## It can be used for installing simulators (only) or for installing the preconditions for Snapdragon Flight or Raspberry Pi.
##
## Installs:
## - Common dependencies and tools for all targets (including: Ninja build system, Qt Creator, pyulog)
## - FastRTPS and FastCDR
## - jMAVSim simulator dependencies

# Preventing sudo timeout https://serverfault.com/a/833888
trap "exit" INT TERM; trap "kill 0" EXIT; sudo -v || exit $?; sleep 1; while true; do sleep 60; sudo -nv; done 2>/dev/null &

# Ubuntu Config
sudo apt-get remove modemmanager -y


# Common dependencies
echo "Installing common dependencies"
sudo apt-get update -y
sudo apt-get install git zip qtcreator cmake build-essential genromfs ninja-build -y
# Required python packages
sudo apt-get install python-argparse python-empy python-toml python-numpy python-dev python-pip -y
sudo -H pip install --upgrade pip
sudo -H pip install pandas jinja2 pyserial
# optional python tools
sudo -H pip install pyulog

# Install FastRTPS 1.5.0 and FastCDR-1.0.7
fastrtps_dir=$HOME/eProsima_FastRTPS-1.5.0-Linux
echo "Installing FastRTPS to: $fastrtps_dir"
if [ -d "$fastrtps_dir" ]
then
    echo " FastRTPS already installed."
else
    pushd .
    cd ~
    wget http://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-5-0/eprosima_fastrtps-1-5-0-linux-tar-gz -O eprosima_fastrtps-1-5-0-linux.tar.gz
    tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz eProsima_FastRTPS-1.5.0-Linux/
    tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz requiredcomponents
    tar -xzf requiredcomponents/eProsima_FastCDR-1.0.7-Linux.tar.gz
    cpucores=$(( $(lscpu | grep Core.*per.*socket | awk -F: '{print $2}') * $(lscpu | grep Socket\(s\) | awk -F: '{print $2}') ))
    cd eProsima_FastCDR-1.0.7-Linux; ./configure --libdir=/usr/lib; make -j$cpucores; sudo make install
    cd ..
    cd eProsima_FastRTPS-1.5.0-Linux; ./configure --libdir=/usr/lib; make -j$cpucores; sudo make install
    cd ..
    rm -rf requiredcomponents eprosima_fastrtps-1-5-0-linux.tar.gz
    popd
fi

# jMAVSim simulator dependencies
echo "Installing jMAVSim simulator dependencies"
sudo apt-get install ant openjdk-8-jdk openjdk-8-jre -y

# Clone PX4/Firmware
px4_dir=~/src
echo "Cloning PX4 to: $px4_dir."
if [ -d "$px4_dir" ]
then
    echo " Firmware already cloned."
else
    mkdir -p $px4_dir
    cd $px4_dir
    git clone https://github.com/PX4/Firmware.git
fi

#NUTTX

## Set up a PX4 development environment for Pixhawk/NuttX targets on Ubuntu LTS (16.04).
## It can be used for installing simulators and the NuttX toolchain.
##
## Installs:
## - NuttX toolchain (i.e. gcc compiler)

# NuttX
sudo apt-get install python-serial openocd \
    flex bison libncurses5-dev autoconf texinfo \
    libftdi-dev libtool zlib1g-dev -y

# Clean up old GCC
sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded -y
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa -y


# GNU Arm Embedded Toolchain: 7-2017-q4-major December 18, 2017
gcc_dir=$HOME/gcc-arm-none-eabi-7-2017-q4-major
echo "Installing GCC to: $gcc_dir"
if [ -d "$gcc_dir" ]
then
    echo " GCC already installed."
else
    pushd .
    cd ~    
    wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
    tar -jxf gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
    exportline="export PATH=$HOME/gcc-arm-none-eabi-7-2017-q4-major/bin:\$PATH"
    if grep -Fxq "$exportline" ~/.profile; then echo " GCC path already set." ; else echo $exportline >> ~/.profile; fi
    . ~/.profile
    popd
fi

## Setting up a ROS/Gazebo development environment for PX4 on Ubuntu LTS (16.04). 
## It installs the common dependencies for all targets (including Qt Creator) and the ROS Kinetic/Gazebo 7 (the default).
##
## Installs:
## - ROS Kinetic (including Gazebo7)
## - MAVROS

# ROS Kinetic/Gazebo (ROS Kinetic includes Gazebo7 by default)
## Gazebo simulator dependencies
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y

## ROS Gazebo: http://wiki.ros.org/kinetic/Installation/Ubuntu
## Setup keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
## For keyserver connection problems substitute hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 above.
sudo apt-get update
## Get ROS/Gazebo
sudo apt-get install ros-kinetic-desktop-full -y
## Initialize rosdep
sudo rosdep init
rosdep update
## Setup environment variables
rossource="source /opt/ros/kinetic/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
else echo "$rossource" >> ~/.bashrc; fi
eval $rossource
## Get rosinstall
sudo apt-get install python-rosinstall -y

# MAVROS: https://dev.px4.io/en/ros/mavros_installation.html
## Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

## Install dependencies
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools -y

## Initialise wstool
wstool init ~/catkin_ws/src

## Build MAVROS
### Get source (upstream - released)
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
### Get latest released mavlink package
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
### Setup workspace & install deps
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
if ! rosdep install --from-paths src --ignore-src --rosdistro kinetic -y; then
    # (Use echo to trim leading/trailing whitespaces from the unsupported OS name
    unsupported_os=$(echo $(rosdep db 2>&1| grep Unsupported | awk -F: '{print $2}'))
    rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --os ubuntu:xenial
fi
## Build!
catkin build
## Re-source environment to reflect new packages/build environment
catkin_ws_source="source ~/catkin_ws/devel/setup.bash"
if grep -Fxq "$catkin_ws_source" ~/.bashrc; then echo ROS catkin_ws setup.bash already in .bashrc; 
else echo "$catkin_ws_source" >> ~/.bashrc; fi
eval $catkin_ws_source

if [ -f $HOME/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh ]; then 
    sudo bash $HOME/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
fi

clone_dir=~/sicteam_src
echo "Cloning to: $clone_dir."
if [ -d "$clone_dir" ]; then
    echo "Code already cloned."
else
    mkdir -p $clone_dir
    cd $clone_dir
    git clone https://github.com/SicTeam/Firmware.git
    git clone https://github.com/SicTeam/capstone.git
fi

cd $HOME
sudo apt-get install espeak libespeak-dev libudev-dev libsdl2-dev -y
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage

echo "RESTART YOUR COMPUTER to complete installation of PX4 development toolchain"
