# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rokae_arm_toppra: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rokae_arm_toppra_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_arm_toppra/srv/ToppRa_srv.srv" NAME_WE)
add_custom_target(_rokae_arm_toppra_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rokae_arm_toppra" "/home/contour/ws_catkin_elephant/src/elephant/rokae_arm_toppra/srv/ToppRa_srv.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(rokae_arm_toppra
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_arm_toppra/srv/ToppRa_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_arm_toppra
)

### Generating Module File
_generate_module_cpp(rokae_arm_toppra
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_arm_toppra
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rokae_arm_toppra_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rokae_arm_toppra_generate_messages rokae_arm_toppra_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_arm_toppra/srv/ToppRa_srv.srv" NAME_WE)
add_dependencies(rokae_arm_toppra_generate_messages_cpp _rokae_arm_toppra_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rokae_arm_toppra_gencpp)
add_dependencies(rokae_arm_toppra_gencpp rokae_arm_toppra_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rokae_arm_toppra_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(rokae_arm_toppra
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_arm_toppra/srv/ToppRa_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_arm_toppra
)

### Generating Module File
_generate_module_eus(rokae_arm_toppra
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_arm_toppra
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rokae_arm_toppra_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rokae_arm_toppra_generate_messages rokae_arm_toppra_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_arm_toppra/srv/ToppRa_srv.srv" NAME_WE)
add_dependencies(rokae_arm_toppra_generate_messages_eus _rokae_arm_toppra_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rokae_arm_toppra_geneus)
add_dependencies(rokae_arm_toppra_geneus rokae_arm_toppra_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rokae_arm_toppra_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(rokae_arm_toppra
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_arm_toppra/srv/ToppRa_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_arm_toppra
)

### Generating Module File
_generate_module_lisp(rokae_arm_toppra
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_arm_toppra
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rokae_arm_toppra_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rokae_arm_toppra_generate_messages rokae_arm_toppra_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_arm_toppra/srv/ToppRa_srv.srv" NAME_WE)
add_dependencies(rokae_arm_toppra_generate_messages_lisp _rokae_arm_toppra_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rokae_arm_toppra_genlisp)
add_dependencies(rokae_arm_toppra_genlisp rokae_arm_toppra_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rokae_arm_toppra_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(rokae_arm_toppra
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_arm_toppra/srv/ToppRa_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_arm_toppra
)

### Generating Module File
_generate_module_nodejs(rokae_arm_toppra
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_arm_toppra
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rokae_arm_toppra_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rokae_arm_toppra_generate_messages rokae_arm_toppra_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_arm_toppra/srv/ToppRa_srv.srv" NAME_WE)
add_dependencies(rokae_arm_toppra_generate_messages_nodejs _rokae_arm_toppra_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rokae_arm_toppra_gennodejs)
add_dependencies(rokae_arm_toppra_gennodejs rokae_arm_toppra_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rokae_arm_toppra_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(rokae_arm_toppra
  "/home/contour/ws_catkin_elephant/src/elephant/rokae_arm_toppra/srv/ToppRa_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_arm_toppra
)

### Generating Module File
_generate_module_py(rokae_arm_toppra
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_arm_toppra
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rokae_arm_toppra_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rokae_arm_toppra_generate_messages rokae_arm_toppra_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/contour/ws_catkin_elephant/src/elephant/rokae_arm_toppra/srv/ToppRa_srv.srv" NAME_WE)
add_dependencies(rokae_arm_toppra_generate_messages_py _rokae_arm_toppra_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rokae_arm_toppra_genpy)
add_dependencies(rokae_arm_toppra_genpy rokae_arm_toppra_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rokae_arm_toppra_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_arm_toppra)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_arm_toppra
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rokae_arm_toppra_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_arm_toppra)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_arm_toppra
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rokae_arm_toppra_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_arm_toppra)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_arm_toppra
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rokae_arm_toppra_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_arm_toppra)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_arm_toppra
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rokae_arm_toppra_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_arm_toppra)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_arm_toppra\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_arm_toppra
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rokae_arm_toppra_generate_messages_py std_msgs_generate_messages_py)
endif()
