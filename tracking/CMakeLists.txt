#contains the build of detection/track directory

#XXX keep track of this cmake version requirement, might not work on jetson
#cmake_minimum_required(VERSION 2.8)
#project(Drone_Detect)
find_package(OpenCV REQUIRED)

#add_library(detect drone_track.cpp drone_main.cpp)
#add_library(detect_test drone_track.cpp detect_test.cpp)
#add_library(track_test drone_track.cpp track_test.cpp)

#target_include_directories (detect PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
#target_include_directories (detect_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
#target_include_directories (track_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

#Detection
add_executable(detect drone_track.cpp drone_main.cpp)
add_executable(detect_test drone_track.cpp detect_test.cpp)
add_executable(track_test drone_track.cpp track_test.cpp) 
target_link_libraries(detect ${OpenCV_LIBS})
target_link_libraries(detect_test ${OpenCV_LIBS})
target_link_libraries(track_test ${OpenCV_LIBS})
