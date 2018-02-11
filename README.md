## Tracking and Pursuit Module ##

This module has been developed to take in video feed from a stereo camera to 
conduct object recognition of drones. Once the object is detected the location 
of the bounding box is used to determine a directional vector to send to PX4 via
Mavlink/ROS nodes


### Generic Build Instructions ### 

To run initial build:

Within Directory: /capstone/

    cmake .
    make

To run with test videos:

Within Directory: /capstone/tracking/

    ./detect test/Video_1.avi

To run face recognition test through webcam:

    ./detect


### CMake Instructions ###

Cmake was used for this project in order to provide a structured build system
that will build and compile all code as the module is expanded. The CMakeLists.txt 
file will hold the project creation and will need to contain additional 
add-subdirectories(-----) if additional directories are added. Within these new 
directories, a CMakeLists.txt will contain code necessary to either add a new 
library or add an executable. The .gitignore file is preventing inclusion of 
cmake files in order to make the module more portable.


### Unit Testing Instructions ###

Work in progress. Plans are to use Cmake's CTest in order to provide an automated
testing suite.
