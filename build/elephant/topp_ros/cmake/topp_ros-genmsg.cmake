# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "topp_ros: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(topp_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GetHelixPoints.srv" NAME_WE)
add_custom_target(_topp_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "topp_ros" "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GetHelixPoints.srv" "trajectory_msgs/JointTrajectory:trajectory_msgs/JointTrajectoryPoint:std_msgs/Header"
)

get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GenerateTrajectory.srv" NAME_WE)
add_custom_target(_topp_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "topp_ros" "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GenerateTrajectory.srv" "trajectory_msgs/JointTrajectory:trajectory_msgs/JointTrajectoryPoint:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(topp_ros
  "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GetHelixPoints.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/topp_ros
)
_generate_srv_cpp(topp_ros
  "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GenerateTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/topp_ros
)

### Generating Module File
_generate_module_cpp(topp_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/topp_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(topp_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(topp_ros_generate_messages topp_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GetHelixPoints.srv" NAME_WE)
add_dependencies(topp_ros_generate_messages_cpp _topp_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GenerateTrajectory.srv" NAME_WE)
add_dependencies(topp_ros_generate_messages_cpp _topp_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(topp_ros_gencpp)
add_dependencies(topp_ros_gencpp topp_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS topp_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(topp_ros
  "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GetHelixPoints.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/topp_ros
)
_generate_srv_eus(topp_ros
  "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GenerateTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/topp_ros
)

### Generating Module File
_generate_module_eus(topp_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/topp_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(topp_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(topp_ros_generate_messages topp_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GetHelixPoints.srv" NAME_WE)
add_dependencies(topp_ros_generate_messages_eus _topp_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GenerateTrajectory.srv" NAME_WE)
add_dependencies(topp_ros_generate_messages_eus _topp_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(topp_ros_geneus)
add_dependencies(topp_ros_geneus topp_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS topp_ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(topp_ros
  "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GetHelixPoints.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/topp_ros
)
_generate_srv_lisp(topp_ros
  "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GenerateTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/topp_ros
)

### Generating Module File
_generate_module_lisp(topp_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/topp_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(topp_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(topp_ros_generate_messages topp_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GetHelixPoints.srv" NAME_WE)
add_dependencies(topp_ros_generate_messages_lisp _topp_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GenerateTrajectory.srv" NAME_WE)
add_dependencies(topp_ros_generate_messages_lisp _topp_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(topp_ros_genlisp)
add_dependencies(topp_ros_genlisp topp_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS topp_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(topp_ros
  "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GetHelixPoints.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/topp_ros
)
_generate_srv_nodejs(topp_ros
  "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GenerateTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/topp_ros
)

### Generating Module File
_generate_module_nodejs(topp_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/topp_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(topp_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(topp_ros_generate_messages topp_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GetHelixPoints.srv" NAME_WE)
add_dependencies(topp_ros_generate_messages_nodejs _topp_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GenerateTrajectory.srv" NAME_WE)
add_dependencies(topp_ros_generate_messages_nodejs _topp_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(topp_ros_gennodejs)
add_dependencies(topp_ros_gennodejs topp_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS topp_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(topp_ros
  "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GetHelixPoints.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/topp_ros
)
_generate_srv_py(topp_ros
  "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GenerateTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/topp_ros
)

### Generating Module File
_generate_module_py(topp_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/topp_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(topp_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(topp_ros_generate_messages topp_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GetHelixPoints.srv" NAME_WE)
add_dependencies(topp_ros_generate_messages_py _topp_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/topp_ros/srv/GenerateTrajectory.srv" NAME_WE)
add_dependencies(topp_ros_generate_messages_py _topp_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(topp_ros_genpy)
add_dependencies(topp_ros_genpy topp_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS topp_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/topp_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/topp_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(topp_ros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(topp_ros_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(topp_ros_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/topp_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/topp_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(topp_ros_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET trajectory_msgs_generate_messages_eus)
  add_dependencies(topp_ros_generate_messages_eus trajectory_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(topp_ros_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/topp_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/topp_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(topp_ros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(topp_ros_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(topp_ros_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/topp_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/topp_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(topp_ros_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET trajectory_msgs_generate_messages_nodejs)
  add_dependencies(topp_ros_generate_messages_nodejs trajectory_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(topp_ros_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/topp_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/topp_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/topp_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(topp_ros_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(topp_ros_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(topp_ros_generate_messages_py geometry_msgs_generate_messages_py)
endif()
