cmake_minimum_required(VERSION 3.0.2)
project(gps_msgs_package)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_calibration
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs nav_msgs tf
  DEPENDS system_lib
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


add_executable(gps_msgs_package_node src/gps_msgs_package.cpp)
add_dependencies(gps_msgs_package_node ${gps_msgs_package_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_msgs_package_node ${catkin_LIBRARIES} )


