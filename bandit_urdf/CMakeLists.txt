cmake_minimum_required(VERSION 2.6.0)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_executable(parser src/parser.cc)
target_link_libraries(parser freeimage)

set(urdf_files "")

# Xacro files
#file(GLOB model_xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.xacro)

# Dependency files
file(GLOB urdf_xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.urdf.xacro)

foreach(it ${urdf_xacro_files})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)

  message(" processing file: ${it}")

  set(expanded_file "${basepath}/${basename}.urdf")
  message("    generated expanded file: ${expanded_file}")

  xacro_add_xacro_file(${it} ${expanded_file})

  set(urdf_files ${urdf_files} ${expanded_file})
endforeach(it)

add_custom_target(media_files ALL DEPENDS ${urdf_files})
