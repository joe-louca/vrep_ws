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
  OpenCV
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
 ${OpenCV_INCLUDE_DIRS}
)




## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
   src/inputs/config.py
   src/inputs/joy_sub.py
   src/inputs/tf_sub.py
   src/inputs/joy_control.py
   src/inputs/move_pub.py
   src/inputs/gripper_pub.py
   src/inputs/time_pub.py
   src/feedbacks/cam1_sub.py
   src/feedbacks/cam2_sub.py
   src/feedbacks/ft_sub.py
   src/feedbacks/joy_vibrate.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


