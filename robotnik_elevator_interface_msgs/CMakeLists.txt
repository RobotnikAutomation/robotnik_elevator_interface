cmake_minimum_required(VERSION 2.8.3)
project(robotnik_elevator_interface_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib_msgs std_msgs message_generation)


#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
    FILES
    ElevatorState.msg
)

## Generate services in the 'srv' folder
add_service_files(
    DIRECTORY srv
    FILES
    SetElevatorControl.srv
    GoToFloor.srv
    SetDoorState.srv
)

#add_action_files(
#    FILES
#    SetElevator.action
#)


## Generate added messages and services with any dependencies listed here
generate_messages(
DEPENDENCIES
   actionlib_msgs
   std_msgs  # Or other packages containing msgs
)

catkin_package(CATKIN_DEPENDS
  actionlib_msgs
  message_runtime
  std_msgs
)


