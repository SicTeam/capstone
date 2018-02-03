## Detection and Tracking of Targets ##

This directory contains the software necessary to detect and track viable targets
within the drones airspace. The tracking method being used is called kcf which
stands for Kernelized Correlation Filters.


### Generic Build Instructions ###

To run initial build:   (remove CMakeCache.txt upon inital download if present)
    
    cmake .   
    make

To run with test videos:
    
    ./detect test/Video_1.avi


### Unit Testing Instructions ###

This is a work in progress. I plan on storing the tests in the tests directory.
I will need to look into cmake a bit more in order to have both directories built
simultaneously.
