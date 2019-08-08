cmake_minimum_required(VERSION 2.8.3)
project(projeto_semear)

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

add_message_files(
  FILES
  PololuLineSensor.msg
  Colors.msg
  Pose.msg
)

# Generate services in the 'srv' folder
#add_service_files(
#  FILES
#)

add_action_files(
  FILES 
  goToFirstPose.action
  goToContainer.action
  goFromContainerToIntersection.action
  goFromDockToIntersection.action
  goToDock.action
  changeIntersection.action
  moveEletroima.action
  setEletroima.action
  goToTrain.action
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
  ${OpenCV_INCLUDE_DIRS}
)

add_library(arduino_library src/arduino_interface.cpp)

# Link Library with respective dependencies
target_link_libraries(arduino_library ${catkin_LIBRARIES})
# add_dependencies(arduino_library ${${PROJECT_NAME}_EXPORTED_TARGETS})


catkin_install_python(PROGRAMS scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(convert_colorsensor src/convert_colorsensor.cpp)
target_link_libraries(convert_colorsensor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(convert_colorsensor ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(eletroimaInterface simulation_interface_code/eletroimaInterface.cpp)
target_link_libraries(eletroimaInterface
  ${catkin_LIBRARIES}
)
add_dependencies(eletroimaInterface ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(arduinoMotorConversion simulation_interface_code/arduinoMotorConversion.cpp)
target_link_libraries(arduinoMotorConversion
  ${catkin_LIBRARIES}
)
add_dependencies(arduinoMotorConversion ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(raspEncoderConversion simulation_interface_code/raspEncoderConversion.cpp)
target_link_libraries(raspEncoderConversion
  ${catkin_LIBRARIES}
)
add_dependencies(raspEncoderConversion ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(sensorCode src/sensorCode.cpp)
target_link_libraries(sensorCode
  ${catkin_LIBRARIES}
  arduino_library
)
add_dependencies(sensorCode ${${PROJECT_NAME}_EXPORTED_TARGETS})


#add_executable(publishEncoder rasp/publishEncoder.cpp)
#target_link_libraries(publishEncoder
# ${catkin_LIBRARIES}
#  rotary_encoder
#)
#add_dependencies(publishEncoder ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(motorCode src/motorCode.cpp)
target_link_libraries(motorCode
 ${catkin_LIBRARIES}
  arduino_library
 )

