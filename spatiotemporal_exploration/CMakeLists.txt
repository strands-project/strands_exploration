cmake_minimum_required(VERSION 2.8.3)
project(spatiotemporal_exploration)

find_package(catkin REQUIRED COMPONENTS mongodb_store roscpp strands_executive_msgs std_msgs std_srvs strands_navigation_msgs mongodb_store_msgs geometry_msgs dynamic_reconfigure image_transport tf scitos_ptu strands_exploration_msgs) 


generate_dynamic_reconfigure_options(
  cfg/exploration.cfg
)

set ( CMAKE_CXX_FLAGS "-Ofast -march=native -mno-avx -ggdb")

catkin_package(CATKIN_DEPENDS roscpp strands_executive_msgs std_msgs std_srvs strands_navigation_msgs mongodb_store_msgs geometry_msgs std_msgs mongodb_store scitos_ptu)

include_directories(${catkin_INCLUDE_DIRS}) 

add_executable(spatiotemporal_exploration src/spatiotemporal_exploration.cpp)

add_dependencies(spatiotemporal_exploration ${catkin_EXPORTED_TARGETS} mongodb_store ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_genmsg ${PROJECT_NAME}_generated_messages)

add_library(fremenExp_frelement src/CFrelementExp.cpp)
add_library(fremenExp_timer src/CTimerExp.cpp)
add_library(fremenExp_grid src/CFremenGridExp.cpp)
add_library(fremenExp_set  src/CFremenGridSet.cpp)

target_link_libraries(fremenExp_frelement fremenExp_timer)
target_link_libraries(fremenExp_grid fremenExp_frelement)
target_link_libraries(fremenExp_set fremenExp_grid)

target_link_libraries (spatiotemporal_exploration ${catkin_LIBRARIES} fremenExp_set)

# Mark executables and/or libraries for installation
install(TARGETS spatiotemporal_exploration fremenExp_frelement fremenExp_grid fremenExp_set
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/grid_store.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
