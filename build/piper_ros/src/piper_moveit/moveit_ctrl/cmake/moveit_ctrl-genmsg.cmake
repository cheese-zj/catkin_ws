# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "moveit_ctrl: 0 messages, 1 services")

set(MSG_I_FLAGS "-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(moveit_ctrl_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jameszhao2004/catkin_ws/src/piper_ros/src/piper_moveit/moveit_ctrl/srv/JointMoveitCtrl.srv" NAME_WE)
add_custom_target(_moveit_ctrl_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "moveit_ctrl" "/home/jameszhao2004/catkin_ws/src/piper_ros/src/piper_moveit/moveit_ctrl/srv/JointMoveitCtrl.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(moveit_ctrl
  "/home/jameszhao2004/catkin_ws/src/piper_ros/src/piper_moveit/moveit_ctrl/srv/JointMoveitCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/moveit_ctrl
)

### Generating Module File
_generate_module_cpp(moveit_ctrl
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/moveit_ctrl
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(moveit_ctrl_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(moveit_ctrl_generate_messages moveit_ctrl_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jameszhao2004/catkin_ws/src/piper_ros/src/piper_moveit/moveit_ctrl/srv/JointMoveitCtrl.srv" NAME_WE)
add_dependencies(moveit_ctrl_generate_messages_cpp _moveit_ctrl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(moveit_ctrl_gencpp)
add_dependencies(moveit_ctrl_gencpp moveit_ctrl_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS moveit_ctrl_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(moveit_ctrl
  "/home/jameszhao2004/catkin_ws/src/piper_ros/src/piper_moveit/moveit_ctrl/srv/JointMoveitCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/moveit_ctrl
)

### Generating Module File
_generate_module_eus(moveit_ctrl
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/moveit_ctrl
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(moveit_ctrl_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(moveit_ctrl_generate_messages moveit_ctrl_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jameszhao2004/catkin_ws/src/piper_ros/src/piper_moveit/moveit_ctrl/srv/JointMoveitCtrl.srv" NAME_WE)
add_dependencies(moveit_ctrl_generate_messages_eus _moveit_ctrl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(moveit_ctrl_geneus)
add_dependencies(moveit_ctrl_geneus moveit_ctrl_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS moveit_ctrl_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(moveit_ctrl
  "/home/jameszhao2004/catkin_ws/src/piper_ros/src/piper_moveit/moveit_ctrl/srv/JointMoveitCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/moveit_ctrl
)

### Generating Module File
_generate_module_lisp(moveit_ctrl
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/moveit_ctrl
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(moveit_ctrl_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(moveit_ctrl_generate_messages moveit_ctrl_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jameszhao2004/catkin_ws/src/piper_ros/src/piper_moveit/moveit_ctrl/srv/JointMoveitCtrl.srv" NAME_WE)
add_dependencies(moveit_ctrl_generate_messages_lisp _moveit_ctrl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(moveit_ctrl_genlisp)
add_dependencies(moveit_ctrl_genlisp moveit_ctrl_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS moveit_ctrl_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(moveit_ctrl
  "/home/jameszhao2004/catkin_ws/src/piper_ros/src/piper_moveit/moveit_ctrl/srv/JointMoveitCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/moveit_ctrl
)

### Generating Module File
_generate_module_nodejs(moveit_ctrl
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/moveit_ctrl
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(moveit_ctrl_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(moveit_ctrl_generate_messages moveit_ctrl_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jameszhao2004/catkin_ws/src/piper_ros/src/piper_moveit/moveit_ctrl/srv/JointMoveitCtrl.srv" NAME_WE)
add_dependencies(moveit_ctrl_generate_messages_nodejs _moveit_ctrl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(moveit_ctrl_gennodejs)
add_dependencies(moveit_ctrl_gennodejs moveit_ctrl_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS moveit_ctrl_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(moveit_ctrl
  "/home/jameszhao2004/catkin_ws/src/piper_ros/src/piper_moveit/moveit_ctrl/srv/JointMoveitCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/moveit_ctrl
)

### Generating Module File
_generate_module_py(moveit_ctrl
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/moveit_ctrl
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(moveit_ctrl_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(moveit_ctrl_generate_messages moveit_ctrl_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jameszhao2004/catkin_ws/src/piper_ros/src/piper_moveit/moveit_ctrl/srv/JointMoveitCtrl.srv" NAME_WE)
add_dependencies(moveit_ctrl_generate_messages_py _moveit_ctrl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(moveit_ctrl_genpy)
add_dependencies(moveit_ctrl_genpy moveit_ctrl_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS moveit_ctrl_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/moveit_ctrl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/moveit_ctrl
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(moveit_ctrl_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/moveit_ctrl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/moveit_ctrl
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(moveit_ctrl_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/moveit_ctrl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/moveit_ctrl
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(moveit_ctrl_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/moveit_ctrl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/moveit_ctrl
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(moveit_ctrl_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/moveit_ctrl)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/moveit_ctrl\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/moveit_ctrl
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(moveit_ctrl_generate_messages_py sensor_msgs_generate_messages_py)
endif()
