cmake_minimum_required(VERSION 3.0.2)
project(rokae_pick_place_with_vacuum_gripper)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation   # Add message_generation here, after the other packages
)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  GripperState.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  LIBRARIES rokae_vaccum_gripper
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vacuum_gripper_control_client src/vacuum_gripper_control_client.cpp)
target_link_libraries(vacuum_gripper_control_client ${catkin_LIBRARIES})

add_executable(vacuum_gripper_control_server src/vacuum_gripper_control_server.cpp)
target_link_libraries(vacuum_gripper_control_server ${catkin_LIBRARIES})
