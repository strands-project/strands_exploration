cmake_minimum_required(VERSION 2.8.3)
project(exploration_task)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  flir_pantilt_d46
  message_generation
  rospy
  sensor_msgs
  std_msgs
  strands_executive_msgs
  strands_gazing
  strands_webserver
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  Clicks.msg
#)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Empty.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

#############
## Install ##
#############

install(PROGRAMS
  scripts/exploration_task_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
