cmake_minimum_required(VERSION 3.0.2)
project(scripts)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  moveit_commander
  moveit_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  moveit_commander
  moveit_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
 ${catkin_INCLUDE_DIRS}
)




## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
   src/config.py
   src/joy_sub.py
   src/tf_sub.py
   src/joy_control.py
   src/move_pub.py
   src/gripper_pub.py
   src/time_pub.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


