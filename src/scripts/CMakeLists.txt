cmake_minimum_required(VERSION 3.0.2)
project(scripts)

set(CMAKE_CXX_FLAGS "-std=c++14 -lnifalcon")#Falcon lib is build agains c++14

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
  iiwaPy
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
   src/inputs/config_UR10.py
   src/inputs/config_kuka.py
   src/inputs/joy_sub.py
   src/inputs/falcon_sub.py
   src/inputs/tf_sub.py
   src/inputs/joy_control.py   
   src/inputs/joy_control_CartVel.py
   src/inputs/move_pub.py
   src/inputs/move_pub_CartVel.py
   src/inputs/gripper_pub.py
   src/inputs/time_pub.py
   src/feedbacks/cam1_sub.py
   src/feedbacks/cam2_sub.py
   src/feedbacks/ft_sub.py
   src/feedbacks/ft_delay_pub.py
   src/feedbacks/joy_vibrate.py
   src/kuka/kuka_joint_sub.py
   src/UR10/vrep_to_UR10.py
   src/UR10/UR10_joint_sub.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

add_executable(
falcon_node src/inputs/falcon_pub.cpp
)

add_dependencies(falcon_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(
   falcon_node
   nifalcon
   nifalcon_cli_base
   usb-1.0
   ${catkin_LIBRARIES}
)

