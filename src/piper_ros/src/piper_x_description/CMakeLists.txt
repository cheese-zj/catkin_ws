cmake_minimum_required(VERSION 2.8.3)
project(piper_x_description)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir config launch meshes rviz urdf)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
  endif()
endforeach()
