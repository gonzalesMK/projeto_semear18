cmake_minimum_required(VERSION 2.8.3)
project(robot_strategy)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

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
  cv_bridge
  image_transport
  sensor_msgs
)


find_package(OpenCV REQUIRED)
# find_library(PIGPIO_LIBRARY pygpio /path)

catkin_python_setup()

# add_message_files(
#   FILES
#   PololuLineSensor.msg
#   Colors.msg
#   Pose.msg
# )

# Generate services in the 'srv' folder
#add_service_files(
#  FILES
#)

# add_action_files(
#   FILES 
#   moveEletroima.action
#   setEletroima.action
# )
    
# generate_messages(
#   DEPENDENCIES
#   actionlib_msgs geometry_msgs std_msgs
# )

catkin_package(
  # INCLUDE_DIRS include
  # CATKIN_DEPENDS actionlib_msgs message_runtime #geometry_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_library(motorControlLib src/motorControlLib.cpp)
add_library(arduinoLibrary src/arduinoInterfaceLib.cpp)

# Link Library with respective dependencies
target_link_libraries(motorControlLib ${catkin_LIBRARIES})
target_link_libraries(arduinoLibrary ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
