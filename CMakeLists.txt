cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

INCLUDE(cmake/base.cmake)

#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("jrl-dynamics")

rosbuild_add_boost_directories()

# Make sure Boost.Filesystem v2 is used.
add_definitions(-DBOOST_FILESYSTEM_VERSION=2)

rosbuild_add_library(${PROJECT_NAME} src/parser.cc)
rosbuild_link_boost(${PROJECT_NAME} filesystem system)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} jrl-dynamics)

rosbuild_add_executable(display-robot examples/display-robot.cc)
target_link_libraries(display-robot ${PROJECT_NAME})

add_subdirectory(tests)
