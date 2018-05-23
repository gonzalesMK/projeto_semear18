cmake_minimum_required(VERSION 2.8.3)
project(projeto_semear)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  actionlib
  genmsg
  geometry_msgs
  std_msgs
  tf
  roscpp
  message_generation
)

add_message_files(
  FILES
  Pose.msg
)

add_action_files(
  FILES 
  moveEletroima.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs geometry_msgs std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES motor_library
  CATKIN_DEPENDS actionlib_msgs message_runtime #geometry_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(motor_library src/kine_control.cpp
)
# Link Library with respective dependencies
target_link_libraries(motor_library ${catkin_LIBRARIES})


add_executable(eletroima_control_server src/eletroima_control_server.cpp)
target_link_libraries(eletroima_control_server
  ${catkin_LIBRARIES}
)

add_executable(eletroima_control_demo src/eletroima_control_demo.cpp)
target_link_libraries(eletroima_control_demo
  ${catkin_LIBRARIES}
)

add_executable(kine_control_demo src/kine_control_demo.cpp)
target_link_libraries(kine_control_demo
  ${catkin_LIBRARIES}
  motor_library
)
