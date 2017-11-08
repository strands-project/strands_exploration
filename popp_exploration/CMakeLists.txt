cmake_minimum_required(VERSION 2.8.3)
project(popp_exploration)

find_package(catkin REQUIRED COMPONENTS
  exploration_bid_manager
  mongodb_store
  multi_detect_temporal_patterns
  region_observation
  rospy
  std_msgs
  strands_executive_msgs
  strands_navigation_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  ExplorationChoice.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES popp_exploration
  CATKIN_DEPENDS exploration_bid_manager mongodb_store multi_detect_temporal_patterns region_observation rospy strands_executive_msgs strands_navigation_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# install(PROGRAMS
#   src/popp_exploration/exploration.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(DIRECTORY launch
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
