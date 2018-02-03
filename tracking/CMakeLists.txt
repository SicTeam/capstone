#contains the build of detection/track directory

#XXX keep track of this cmake version requirement, might not work on jetson
cmake_minimum_required(VERSION 2.8)
project(Drone_Detect)
find_package(OpenCV REQUIRED)

#Detection
add_executable(detect drone_track.cpp drone_main.cpp)
target_link_libraries(detect ${OpenCV_LIBS})
