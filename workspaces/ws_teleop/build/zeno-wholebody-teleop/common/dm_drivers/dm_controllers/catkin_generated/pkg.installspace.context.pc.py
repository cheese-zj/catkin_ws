# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "controller_interface;hardware_interface;pluginlib;roscpp;std_msgs;geometry_msgs;dm_common".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ldm_controllers".split(';') if "-ldm_controllers" != "" else []
PROJECT_NAME = "dm_controllers"
PROJECT_SPACE_DIR = "/home/jameszhao2004/catkin_ws/workspaces/ws_teleop/install"
PROJECT_VERSION = "0.0.0"
