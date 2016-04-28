cmake_minimum_required(VERSION 2.8.3)
project(topological_exploration)

find_package(catkin REQUIRED COMPONENTS mongodb_store roscpp strands_executive_msgs std_msgs std_srvs strands_navigation_msgs mongodb_store_msgs geometry_msgs dynamic_reconfigure image_transport tf scitos_ptu strands_exploration_msgs) 

#add_service_files(
#  FILES
#  AddView.srv
#  Visualize.srv
#  SaveGrid.srv
#  LoadGrid.srv
#)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  SFrelement.msg
#  Frelement.msg
#  FremenGrid.msg
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#  strands_exploration_msgs
#)

generate_dynamic_reconfigure_options(
  cfg/exploration.cfg
)

set ( CMAKE_CXX_FLAGS "-Ofast -march=native -mno-avx -ggdb")

catkin_package(CATKIN_DEPENDS roscpp strands_executive_msgs std_msgs std_srvs strands_navigation_msgs mongodb_store_msgs geometry_msgs std_msgs mongodb_store scitos_ptu)

include_directories(${catkin_INCLUDE_DIRS}) 

add_executable(topological_exploration src/topological_exploration.cpp)

add_dependencies(topological_exploration ${catkin_EXPORTED_TARGETS} mongodb_store ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_genmsg ${PROJECT_NAME}_generated_messages)

add_library(frelement_exp src/CFrelementExp.cpp)
add_library(fremen_timer_exp src/CTimerExp.cpp)
add_library(fremengridlib src/CFremenGrid.cpp)
add_library(exploration_timer src/CTimerExp.cpp)
add_library(exploration_fremengridset src/CFremenGridSet.cpp)

target_link_libraries(frelement_exp fremen_timer_exp)
target_link_libraries(fremengridlib frelement_exp)
target_link_libraries(exploration_fremengridset fremengridlib)
target_link_libraries(exploration_fremengridset fremengridlib)

target_link_libraries (topological_exploration ${catkin_LIBRARIES} exploration_fremengridset)


