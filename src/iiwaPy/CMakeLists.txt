cmake_minimum_required(VERSION 3.0.2)
project(iiwaPy)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

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
  rospy
  std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   src/python_client/GeneralPurpose.py
   src/python_client/Getters.py
   src/python_client/iiwaPy.py
   src/python_client/__init__.py
   src/python_client/mySock.py
   src/python_client/MyTestPy.py
   src/python_client/PTP.py
   src/python_client/RealTime.py
   src/python_client/Senders.py
   src/python_client/Setters.py
   src/python_client/sunrisePy.py
   src/python_client/Tutorial_realTimeImpedence.py
   src/python_client/Tutorial_realTimeImpedence_with_Feedback.py
   src/python_client/vrep_joints_to_kuka.py
   src/python_client/vrep_to_iiwa.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

