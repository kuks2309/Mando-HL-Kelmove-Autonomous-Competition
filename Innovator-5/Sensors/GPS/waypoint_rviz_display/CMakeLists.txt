cmake_minimum_required(VERSION 3.0.2)
project(waypoint_rviz_display)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  geometry_msgs
)


catkin_package(
# INCLUDE_DIRS include
# LIBRARIES waypoint_rviz_display
  CATKIN_DEPENDS roscpp rospy std_msgs visualization_msgs
# DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_rviz_display_node src/waypoints_rviz_display.cpp)
add_dependencies(waypoint_rviz_display_node ${waypoint_rviz_display_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_rviz_display_node ${catkin_LIBRARIES} )
