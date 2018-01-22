## Detection and Tracking of Targets ##

This directory contains the software necessary to detect and track viable targets
within the drones airspace. The tracking method being used is called kcf which
stands for Kernelized Correlation Filters. More can be read about this method
at the following [article](https://www.learnopencv.com/object-tracking-using-opencv-cpp-python/)


### Generic Build Instructions ###

To run initial build:
    
    cmake .
    make

To run with test videos:
    
    ./detect test/Video_1.avi
