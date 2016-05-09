cmake_minimum_required(VERSION 2.8.3)
project(edge_exploration)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs topological_navigation strands_navigation_msgs)

#catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES edge_exploration
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

