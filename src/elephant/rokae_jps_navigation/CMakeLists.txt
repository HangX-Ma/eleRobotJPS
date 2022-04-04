cmake_minimum_required(VERSION 3.0.2)
project(rokae_jps_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  message_generation
  moveit_msgs
  moveit_ros_planning_interface
)

find_package(Boost REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

add_service_files(
  FILES
  Goto.srv
  eefState.srv
  joint2pose.srv
  CheckCollision.srv
)

generate_messages(
  DEPENDENCIES
  octomap_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    moveit_core
    moveit_ros_planning_interface
    message_runtime
  DEPENDS 
    Boost
    OCTOMAP
)

link_libraries(${OCTOMAP_LIBRARIES})

include_directories(
  include
  ${OCTOMAP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# USE THIS PLANNER NEED TO RE-ANNOTATE THE FOLLOWING CODES

set(ROKAE_IKFAST_WRAPPER rokae_ikfast_wrapper)
add_library(${ROKAE_IKFAST_WRAPPER} src/JPS_Modules/rokae_ikfast_wrapper.cpp)

add_executable(rokae_arm_eef_state src/JPS_Modules/rokae_arm_eef_state.cpp)
target_link_libraries(rokae_arm_eef_state ${catkin_LIBRARIES})

add_executable(rokae_collision_detection src/JPS_Modules/rokae_collision_detection.cpp)
target_link_libraries(rokae_collision_detection ${catkin_LIBRARIES} ${ROKAE_IKFAST_WRAPPER})

add_executable(rokae_joint2pose src/JPS_Modules/rokae_joint2pose.cpp)
target_link_libraries(rokae_joint2pose ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${ROKAE_IKFAST_WRAPPER})


set(ROKAE_GRAPH_SEARCH rokae_graph_search)
add_library(${ROKAE_GRAPH_SEARCH} src/JPS_Planner/rokae_graph_search.cpp)

add_executable(rokae_jps_planner src/JPS_Planner/rokae_jps_planner.cpp)
target_link_libraries(rokae_jps_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${ROKAE_GRAPH_SEARCH})

add_executable(rokae_arm_main src/JPS_Modules/rokae_arm_main.cpp)
target_link_libraries(rokae_arm_main ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# debug mode
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")