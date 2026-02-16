# Install script for directory: /home/jameszhao2004/catkin_ws/workspaces/ws_teleop/src/zeno-wholebody-teleop/common/dm_drivers/dm_hw

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dm_hw/msg" TYPE FILE FILES "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/src/zeno-wholebody-teleop/common/dm_drivers/dm_hw/msg/MotorState.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dm_hw/cmake" TYPE FILE FILES "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/build/zeno-wholebody-teleop/common/dm_drivers/dm_hw/catkin_generated/installspace/dm_hw-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/devel/include/dm_hw")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/devel/share/roseus/ros/dm_hw")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/devel/share/common-lisp/ros/dm_hw")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/devel/share/gennodejs/ros/dm_hw")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/devel/lib/python3/dist-packages/dm_hw")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/devel/lib/python3/dist-packages/dm_hw")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/build/zeno-wholebody-teleop/common/dm_drivers/dm_hw/catkin_generated/installspace/dm_hw.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dm_hw/cmake" TYPE FILE FILES "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/build/zeno-wholebody-teleop/common/dm_drivers/dm_hw/catkin_generated/installspace/dm_hw-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dm_hw/cmake" TYPE FILE FILES
    "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/build/zeno-wholebody-teleop/common/dm_drivers/dm_hw/catkin_generated/installspace/dm_hwConfig.cmake"
    "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/build/zeno-wholebody-teleop/common/dm_drivers/dm_hw/catkin_generated/installspace/dm_hwConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dm_hw" TYPE FILE FILES "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/src/zeno-wholebody-teleop/common/dm_drivers/dm_hw/package.xml")
endif()

