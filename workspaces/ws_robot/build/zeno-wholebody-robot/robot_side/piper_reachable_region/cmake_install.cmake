# Install script for directory: /home/jameszhao2004/catkin_ws/workspaces/ws_robot/src/zeno-wholebody-robot/robot_side/piper_reachable_region

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jameszhao2004/catkin_ws/workspaces/ws_robot/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jameszhao2004/catkin_ws/workspaces/ws_robot/build/zeno-wholebody-robot/robot_side/piper_reachable_region/catkin_generated/installspace/piper_reachable_region.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/piper_reachable_region/cmake" TYPE FILE FILES
    "/home/jameszhao2004/catkin_ws/workspaces/ws_robot/build/zeno-wholebody-robot/robot_side/piper_reachable_region/catkin_generated/installspace/piper_reachable_regionConfig.cmake"
    "/home/jameszhao2004/catkin_ws/workspaces/ws_robot/build/zeno-wholebody-robot/robot_side/piper_reachable_region/catkin_generated/installspace/piper_reachable_regionConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/piper_reachable_region" TYPE FILE FILES "/home/jameszhao2004/catkin_ws/workspaces/ws_robot/src/zeno-wholebody-robot/robot_side/piper_reachable_region/package.xml")
endif()

