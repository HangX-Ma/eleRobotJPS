cmake_minimum_required(VERSION 3.0.2)
project(elerobot_simple_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf2_ros
)


catkin_package(
  # INCLUDE_DIRS
  # LIBRARIES 
  CATKIN_DEPENDS tf2_ros
  # DEPENDS 
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

