cmake_minimum_required(VERSION 3.0.2)
project(iiwaPy-master)

set(CMAKE_CXX_FLAGS "-std=c++14 -lnifalcon")#Falcon lib is build agains c++14

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  sys
  socket
  time
  io
  math
  datetime
  rospy
  std_msgs
  matplotlib
  numpy
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
  sys
  socket
  time
  io
  math
  datetime
  rospy
  std_msgs
  matplotlib
  numpy
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
   iiwaPy-master/python_client/GeneralPurpose.py
   iiwaPy-master/python_client/Getters.py
   iiwaPy-master/python_client/iiwaPy.py
   iiwaPy-master/python_client/__init__.py
   iiwaPy-master/python_client/mySock.py
   iiwaPy-master/python_client/MyTestPy.py
   iiwaPy-master/python_client/PTP.py
   iiwaPy-master/python_client/RealTime.py
   iiwaPy-master/python_client/Senders.py
   iiwaPy-master/python_client/Setters.py
   iiwaPy-master/python_client/sunrisePy.py
   iiwaPy-master/python_client/Tutorial_realTimeImpedence.py
   iiwaPy-master/python_client/Tutorial_realTimeImpedence_with_Feedback.py
   iiwaPy-master/python_client/vrep_joints_to_kuka.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

