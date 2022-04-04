# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rokae_jps_navigation: 0 messages, 4 services")

set(MSG_I_FLAGS "-Ioctomap_msgs:/opt/ros/melodic/share/octomap_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rokae_jps_navigation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/Goto.srv" NAME_WE)
add_custom_target(_rokae_jps_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rokae_jps_navigation" "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/Goto.srv" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/eefState.srv" NAME_WE)
add_custom_target(_rokae_jps_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rokae_jps_navigation" "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/eefState.srv" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/joint2pose.srv" NAME_WE)
add_custom_target(_rokae_jps_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rokae_jps_navigation" "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/joint2pose.srv" "std_msgs/Float32"
)

get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/CheckCollision.srv" NAME_WE)
add_custom_target(_rokae_jps_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rokae_jps_navigation" "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/CheckCollision.srv" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/Goto.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_jps_navigation
)
_generate_srv_cpp(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/eefState.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_jps_navigation
)
_generate_srv_cpp(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/joint2pose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_jps_navigation
)
_generate_srv_cpp(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/CheckCollision.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_jps_navigation
)

### Generating Module File
_generate_module_cpp(rokae_jps_navigation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_jps_navigation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rokae_jps_navigation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rokae_jps_navigation_generate_messages rokae_jps_navigation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/Goto.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_cpp _rokae_jps_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/eefState.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_cpp _rokae_jps_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/joint2pose.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_cpp _rokae_jps_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/CheckCollision.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_cpp _rokae_jps_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rokae_jps_navigation_gencpp)
add_dependencies(rokae_jps_navigation_gencpp rokae_jps_navigation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rokae_jps_navigation_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/Goto.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_jps_navigation
)
_generate_srv_eus(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/eefState.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_jps_navigation
)
_generate_srv_eus(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/joint2pose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_jps_navigation
)
_generate_srv_eus(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/CheckCollision.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_jps_navigation
)

### Generating Module File
_generate_module_eus(rokae_jps_navigation
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_jps_navigation
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rokae_jps_navigation_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rokae_jps_navigation_generate_messages rokae_jps_navigation_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/Goto.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_eus _rokae_jps_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/eefState.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_eus _rokae_jps_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/joint2pose.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_eus _rokae_jps_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/CheckCollision.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_eus _rokae_jps_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rokae_jps_navigation_geneus)
add_dependencies(rokae_jps_navigation_geneus rokae_jps_navigation_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rokae_jps_navigation_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/Goto.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_jps_navigation
)
_generate_srv_lisp(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/eefState.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_jps_navigation
)
_generate_srv_lisp(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/joint2pose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_jps_navigation
)
_generate_srv_lisp(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/CheckCollision.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_jps_navigation
)

### Generating Module File
_generate_module_lisp(rokae_jps_navigation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_jps_navigation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rokae_jps_navigation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rokae_jps_navigation_generate_messages rokae_jps_navigation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/Goto.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_lisp _rokae_jps_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/eefState.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_lisp _rokae_jps_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/joint2pose.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_lisp _rokae_jps_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/CheckCollision.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_lisp _rokae_jps_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rokae_jps_navigation_genlisp)
add_dependencies(rokae_jps_navigation_genlisp rokae_jps_navigation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rokae_jps_navigation_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/Goto.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_jps_navigation
)
_generate_srv_nodejs(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/eefState.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_jps_navigation
)
_generate_srv_nodejs(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/joint2pose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_jps_navigation
)
_generate_srv_nodejs(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/CheckCollision.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_jps_navigation
)

### Generating Module File
_generate_module_nodejs(rokae_jps_navigation
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_jps_navigation
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rokae_jps_navigation_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rokae_jps_navigation_generate_messages rokae_jps_navigation_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/Goto.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_nodejs _rokae_jps_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/eefState.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_nodejs _rokae_jps_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/joint2pose.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_nodejs _rokae_jps_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/CheckCollision.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_nodejs _rokae_jps_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rokae_jps_navigation_gennodejs)
add_dependencies(rokae_jps_navigation_gennodejs rokae_jps_navigation_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rokae_jps_navigation_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/Goto.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_jps_navigation
)
_generate_srv_py(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/eefState.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_jps_navigation
)
_generate_srv_py(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/joint2pose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_jps_navigation
)
_generate_srv_py(rokae_jps_navigation
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/CheckCollision.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_jps_navigation
)

### Generating Module File
_generate_module_py(rokae_jps_navigation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_jps_navigation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rokae_jps_navigation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rokae_jps_navigation_generate_messages rokae_jps_navigation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/Goto.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_py _rokae_jps_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/eefState.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_py _rokae_jps_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/joint2pose.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_py _rokae_jps_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_jps_navigation/srv/CheckCollision.srv" NAME_WE)
add_dependencies(rokae_jps_navigation_generate_messages_py _rokae_jps_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rokae_jps_navigation_genpy)
add_dependencies(rokae_jps_navigation_genpy rokae_jps_navigation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rokae_jps_navigation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_jps_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_jps_navigation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET octomap_msgs_generate_messages_cpp)
  add_dependencies(rokae_jps_navigation_generate_messages_cpp octomap_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rokae_jps_navigation_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(rokae_jps_navigation_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_jps_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_jps_navigation
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET octomap_msgs_generate_messages_eus)
  add_dependencies(rokae_jps_navigation_generate_messages_eus octomap_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rokae_jps_navigation_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(rokae_jps_navigation_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_jps_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_jps_navigation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET octomap_msgs_generate_messages_lisp)
  add_dependencies(rokae_jps_navigation_generate_messages_lisp octomap_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rokae_jps_navigation_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(rokae_jps_navigation_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_jps_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_jps_navigation
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET octomap_msgs_generate_messages_nodejs)
  add_dependencies(rokae_jps_navigation_generate_messages_nodejs octomap_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rokae_jps_navigation_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(rokae_jps_navigation_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_jps_navigation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_jps_navigation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_jps_navigation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET octomap_msgs_generate_messages_py)
  add_dependencies(rokae_jps_navigation_generate_messages_py octomap_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rokae_jps_navigation_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(rokae_jps_navigation_generate_messages_py geometry_msgs_generate_messages_py)
endif()
