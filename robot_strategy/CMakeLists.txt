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
find_library(PIGPIO_LIBRARY pigpio ~/Documents/pigpio-master)

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
add_library(lineSensorLib src/lineSensorLib.cpp)
add_library(arduinoLibrary src/arduinoInterfaceLib.cpp)
add_library(stateLib src/stateLib.cpp)


# Link Library with respective dependencies
target_link_libraries(arduinoLibrary ${catkin_LIBRARIES})
target_link_libraries(motorControlLib ${catkin_LIBRARIES} arduinoLibrary)
target_link_libraries(lineSensorLib ${catkin_LIBRARIES} arduinoLibrary)
target_link_libraries(stateLib ${catkin_LIBRARIES} arduinoLibrary lineSensorLib motorControlLib)

catkin_install_python(PROGRAMS scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(testarMovimentoBase src/testar_movimento_base.cpp)
target_link_libraries(testarMovimentoBase
 ${catkin_LIBRARIES}
  motorControlLib
 )

add_executable(motor_code src/motorCode.cpp)
target_link_libraries(motor_code
 ${catkin_LIBRARIES}
  motorControlLib
 )


add_executable(testar_lineSensor src/testar_lineSensor.cpp)
target_link_libraries(testar_lineSensor
 ${catkin_LIBRARIES}
  lineSensorLib
)

add_executable(testar_gpio src/testar_gpio.cpp)
target_link_libraries(testar_gpio
 ${catkin_LIBRARIES}
 ${PIGPIO_LIBRARY}
)

add_executable(main src/main.cpp)
target_link_libraries(main
 ${catkin_LIBRARIES}
 ${PIGPIO_LIBRARY}
 ${OpenCV_LIBRARIES}
 lineSensorLib
 motorControlLib
 stateLib
)


add_executable(testar_states src/testar_states.cpp)
target_link_libraries(testar_states ${catkin_LIBRARIES} arduinoLibrary lineSensorLib motorControlLib stateLib)

add_executable(testar_claw src/testar_claw.cpp)
target_link_libraries(testar_claw ${catkin_LIBRARIES} arduinoLibrary lineSensorLib )