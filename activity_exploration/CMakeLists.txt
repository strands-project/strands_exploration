cmake_minimum_required(VERSION 2.8.3)
project(activity_exploration)

find_package(catkin REQUIRED COMPONENTS
  tf
  rospy
  std_msgs
  soma_manager
  mongodb_store
  geometry_msgs
  soma_map_manager
  visualization_msgs
  human_trajectory # poisson stuff
  strands_navigation_msgs
  strands_exploration_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  RegionObservationTime.msg
  PoissonMsg.msg
)

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES region_observation
  CATKIN_DEPENDS mongodb_store rospy soma_manager tf soma_map_manager geometry_msgs visualization_msgs human_trajectory strands_navigation_msgs strands_exploration_msgs std_msgs
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

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/counting_manager.py
  # src/region_observation/observe.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
