cmake_minimum_required(VERSION 3.0.2)
project(elite_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs)


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   RobotState.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   # -- robot --
   RobotServoOn.srv
   # -- io --
   SetIO.srv
   SetAnalogIO.srv
   # -- move --
   StopMove.srv
   JointMove.srv
   CartMove.srv
   JointTrajectoryMove.srv
   # -- kinematic --
   InverseKinematic.srv
   ForwardKinematic.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur_msgs
   CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

#############
## Install ##
#############

#############
## Testing ##
#############