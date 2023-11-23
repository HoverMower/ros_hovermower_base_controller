cmake_minimum_required(VERSION 3.5)
project(ros_hovermower_base_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosmower_msgs REQUIRED)


#catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ros_hovermower_base_controller_nodelet
#   CATKIN_DEPENDS roscpp rosmower_msgs nodelet pluginlib 
#  DEPENDS system_lib
#   CATKIN_DEPENDS message_runtime
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/ros_hovermower_base_controller
  include ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
 #add_library(
  #ros_hovermower_base_controller_nodelet
  # src/hovermower_bumper2pc.cpp
 #)

 add_executable(base_controller 
 src/hovermower_base_controller.cpp
 src/main.cpp)
 
 ament_target_dependencies(base_controller rclcpp std_msgs
pluginlib
sensor_msgs
tf2
tf2_ros
tf2_geometry_msgs
rosmower_msgs)

#target_link_libraries(ros_hovermower_base_controller_nodelet )
ament_target_dependencies(base_controller rclcpp std_msgs
pluginlib
sensor_msgs
tf2
tf2_ros
tf2_geometry_msgs
rosmower_msgs)

install(TARGETS
  base_controller 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
#add_dependencies(ros_hovermower_base_controller ${PROJECT_NAME}_gencfg)

ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_package()

