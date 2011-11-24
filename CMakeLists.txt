cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

INCLUDE(cmake/base.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("jrl-dynamics")

rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_library(${PROJECT_NAME} src/parser.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} jrl-dynamics)

#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
