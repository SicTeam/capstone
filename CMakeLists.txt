# CMakeLists files in this project can refer to the root source 
# directory of the project as ${SkyNet_SOURCE_DIR} and to the 
# root binary directory of the project as ${SkyNet_BINARY_DIR}.
cmake_minimum_required (VERSION 2.8)
project (SkyNet)

# XXX is it possible to put cmake files in one folder without going there?
# XXX until then build in build folder to consolidate new files
# XXX Also needs to move cascade.xml to build folder

# Set the output folder where your program will be created
#set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
#set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# The following folder will be included
#include_directories("${PROJECT_SOURCE_DIR}")

# Recurse into the "Hello" and "Demo" subdirectories. This does not actually
# cause another cmake executable to run. The same process will walk through
# the project's entire directory structure.
add_subdirectory (tracking)
#add_subdirectory (pursuit)
