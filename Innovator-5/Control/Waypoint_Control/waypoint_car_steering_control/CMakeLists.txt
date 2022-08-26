cmake_minimum_required(VERSION 3.0.2)
project(waypoint_car_steering_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2_geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES waypoint_car_control
   CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf2_geometry_msgs
   DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_car_steering_control_node src/waypoint_car_steering_control_node.cpp)
add_dependencies(waypoint_car_steering_control_node ${waypoint_car_steering_control_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_car_steering_control_node ${catkin_LIBRARIES} )
