cmake_minimum_required(VERSION 2.6.0)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

FIND_PACKAGE( Boost COMPONENTS thread system )

add_definitions( "-std=c++0x" )

add_subdirectory( src )
add_subdirectory( tests )
