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
add_service_files(
  FILES
  EscolherContainer.srv
  Strategy.srv
  GetPose.srv
  PathPlanning.srv
	DepositarContainer.srv
  SetContainer.srv
  GetContainerInfo.srv
  MoveContainer.srv
  DescobrirCor.srv
  #AlignRobot.srv
)

add_action_files(
  FILES 
  goToFirstPose.action
  goToContainer.action
  goFromContainerToIntersection.action
  goFromDockToIntersection.action
  goToBlueDock.action
  goToDock.action
  goToGreenDock.action
  changeIntersection.action
  moveEletroima.action
  setEletroima.action
  navigation.action
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


catkin_install_python(PROGRAMS scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Declare a C++ library
add_library(motor_library src/kine_control.cpp
)
# add_library(rotary_encoder rasp/rotary_encoder.cpp
# )

# Link Library with respective dependencies
target_link_libraries(motor_library ${catkin_LIBRARIES})
add_dependencies(motor_library ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(eletroima_control_server src/eletroima_control_server.cpp)
target_link_libraries(eletroima_control_server
  ${catkin_LIBRARIES}
)
add_dependencies(eletroima_control_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(eletroima_control_demo src/tests/eletroima_control_demo.cpp)
target_link_libraries(eletroima_control_demo
  ${catkin_LIBRARIES}
)
add_dependencies(eletroima_control_demo ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(kine_control_demo src/tests/kine_control_demo.cpp)
target_link_libraries(kine_control_demo
  ${catkin_LIBRARIES}
  motor_library
)

add_executable(escolher_container src/escolher_container.cpp)
target_link_libraries(escolher_container
  ${catkin_LIBRARIES}
)
add_dependencies(escolher_container ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(estrategia_server src/estrategia_server.cpp)
target_link_libraries(estrategia_server
  ${catkin_LIBRARIES}
)
add_dependencies(estrategia_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(gps src/gps.cpp)
target_link_libraries(gps
  ${catkin_LIBRARIES}
)
add_dependencies(gps ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(gps_demo src/tests/gps_demo.cpp)
target_link_libraries(gps_demo
  ${catkin_LIBRARIES}
)
add_dependencies(gps_demo ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(navigating_server src/navigating_server.cpp)
target_link_libraries(navigating_server
  ${catkin_LIBRARIES}
  motor_library
)
add_dependencies(navigating_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(path_planning_demo src/tests/path_planning_demo.cpp)
target_link_libraries(path_planning_demo
  ${catkin_LIBRARIES}
)
add_dependencies(path_planning_demo ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(path_planning_server src/path_planning_server.cpp)
target_link_libraries(path_planning_server
  ${catkin_LIBRARIES}
)
add_dependencies(path_planning_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(convert_colorsensor src/convert_colorsensor.cpp)
target_link_libraries(convert_colorsensor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(convert_colorsensor ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(navigating_server_demo src/tests/navigating_server_demo.cpp)
target_link_libraries(navigating_server_demo
  ${catkin_LIBRARIES}
  motor_library
)
add_dependencies(navigating_server_demo ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(depositar_server src/depositar_server.cpp)
target_link_libraries(depositar_server
  ${catkin_LIBRARIES}
  motor_library
)
add_dependencies(depositar_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(depositar_client src/tests/depositar_client.cpp)
target_link_libraries(depositar_client
  ${catkin_LIBRARIES}
  motor_library
)

add_dependencies(depositar_client ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(mapa_container_server src/mapa_container_server.cpp)
target_link_libraries(mapa_container_server
  ${catkin_LIBRARIES}
  motor_library
)
add_dependencies(mapa_container_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(mapa_container_demo src/tests/mapa_container_demo.cpp)
target_link_libraries(mapa_container_demo
  ${catkin_LIBRARIES}
  motor_library
)
add_dependencies(mapa_container_demo ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(descobrir_cor_server src/descobrir_cor_server.cpp)
target_link_libraries(descobrir_cor_server
  ${catkin_LIBRARIES}
  motor_library
)
add_dependencies(descobrir_cor_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(descobrir_cor_demo src/tests/descobrir_cor_demo.cpp)
target_link_libraries(descobrir_cor_demo
  ${catkin_LIBRARIES}
  motor_library
)
add_dependencies(descobrir_cor_demo ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(main src/main.cpp)
target_link_libraries(main
  ${catkin_LIBRARIES}
  motor_library
)
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(pid_demo src/tests/pid_demo.cpp)
target_link_libraries(pid_demo
  ${catkin_LIBRARIES}
  motor_library
)
add_dependencies(pid_demo ${${PROJECT_NAME}_EXPORTED_TARGETS})

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

#add_executable(publishEncoder rasp/publishEncoder.cpp)
#target_link_libraries(publishEncoder
# ${catkin_LIBRARIES}
#  rotary_encoder
#)
#add_dependencies(publishEncoder ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(motorCode rasp/motorCode.cpp)
target_link_libraries(motorCode
 ${catkin_LIBRARIES}
)
