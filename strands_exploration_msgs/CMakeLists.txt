cmake_minimum_required(VERSION 2.8.3)
project(strands_exploration_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs sensor_msgs actionlib_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  SFrelement.msg
  Frelement.msg
  FremenGrid.msg
  ExplorationSchedule.msg
)

add_service_files(
  FILES
  AddView.srv
  Visualize.srv
  SaveGrid.srv
  LoadGrid.srv
  GetExplorationTasks.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES strands_exploration_msgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


