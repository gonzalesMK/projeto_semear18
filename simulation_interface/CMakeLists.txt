cmake_minimum_required(VERSION 2.8.3)
project(simulation_interface)

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

# catkin_python_setup()

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

add_action_files(
  FILES 
  moveEletroima.action
  setEletroima.action
)
    
generate_messages(
  DEPENDENCIES
  actionlib_msgs geometry_msgs std_msgs
)

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS actionlib_msgs message_runtime #geometry_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(convert_colorsensor src/convert_colorsensor.cpp)
target_link_libraries(convert_colorsensor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
# add_dependencies(convert_colorsensor ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(eletroimaInterface src/eletroimaInterface.cpp)
target_link_libraries(eletroimaInterface
  ${catkin_LIBRARIES}
)
# add_dependencies(eletroimaInterface ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(arduinoMotorConversion src/arduinoMotorConversion.cpp)
target_link_libraries(arduinoMotorConversion
  ${catkin_LIBRARIES}
)
# add_dependencies(arduinoMotorConversion ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(raspEncoderConversion src/raspEncoderConversion.cpp)
target_link_libraries(raspEncoderConversion
  ${catkin_LIBRARIES}
)
# add_dependencies(raspEncoderConversion ${${PROJECT_NAME}_EXPORTED_TARGETS})
