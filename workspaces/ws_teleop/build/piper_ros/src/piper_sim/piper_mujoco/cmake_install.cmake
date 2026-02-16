# Install script for directory: /home/jameszhao2004/catkin_ws/workspaces/ws_teleop/src/piper_ros/src/piper_sim/piper_mujoco

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/build/piper_ros/src/piper_sim/piper_mujoco/catkin_generated/installspace/piper_mujoco.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/piper_mujoco/cmake" TYPE FILE FILES
    "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/build/piper_ros/src/piper_sim/piper_mujoco/catkin_generated/installspace/piper_mujocoConfig.cmake"
    "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/build/piper_ros/src/piper_sim/piper_mujoco/catkin_generated/installspace/piper_mujocoConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/piper_mujoco" TYPE FILE FILES "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/src/piper_ros/src/piper_sim/piper_mujoco/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/piper_mujoco" TYPE PROGRAM FILES "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/build/piper_ros/src/piper_sim/piper_mujoco/catkin_generated/installspace/piper_no_gripper_mujoco_ros.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/piper_mujoco" TYPE PROGRAM FILES "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/build/piper_ros/src/piper_sim/piper_mujoco/catkin_generated/installspace/piper_mujoco_ros.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/piper_mujoco" TYPE PROGRAM FILES "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/build/piper_ros/src/piper_sim/piper_mujoco/catkin_generated/installspace/piper_mujoco_pid.py")
endif()

