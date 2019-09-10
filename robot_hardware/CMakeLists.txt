cmake_minimum_required(VERSION 2.8.3)
project(robot_hardware)

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


# find_package(OpenCV REQUIRED)
# find_library(PIGPIO_LIBRARY pygpio /path)

# catkin_python_setup()

# add_message_files(
#   FILES
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
  INCLUDE_DIRS include 
  CATKIN_DEPENDS actionlib_msgs message_runtime #geometry_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(arduino_library src/arduinoInterfaceLib.cpp)

# Link Library with respective dependencies
target_link_libraries(arduino_library ${catkin_LIBRARIES})


add_executable(sensorCode src/sensorCode.cpp)
target_link_libraries(sensorCode
  ${catkin_LIBRARIES}
  arduino_library
)
target_link_libraries(sensorCode ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_executable(publishEncoder rasp/publishEncoder.cpp)
#target_link_libraries(publishEncoder
# ${catkin_LIBRARIES}
#  encoderLib
#)
#add_dependencies(publishEncoder ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(motorCode src/motorCode.cpp)
target_link_libraries(motorCode
 ${catkin_LIBRARIES}
  arduino_library
 )

