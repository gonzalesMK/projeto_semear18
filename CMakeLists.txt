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

add_message_files(
  FILES
  Pose.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  GetPose.srv
  PathPlanning.srv
	DepositarContainer.srv
  #AlignRobot.srv
)

add_action_files(
  FILES 
  moveEletroima.action
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

# Declare a C++ library
add_library(motor_library src/kine_control.cpp
)
# Link Library with respective dependencies
target_link_libraries(motor_library ${catkin_LIBRARIES})


add_executable(eletroima_control_server src/eletroima_control_server.cpp)
target_link_libraries(eletroima_control_server
  ${catkin_LIBRARIES}
)
add_dependencies(eletroima_control_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(eletroima_control_demo src/eletroima_control_demo.cpp)
target_link_libraries(eletroima_control_demo
  ${catkin_LIBRARIES}
)
add_dependencies(eletroima_control_demo ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(kine_control_demo src/kine_control_demo.cpp)
target_link_libraries(kine_control_demo
  ${catkin_LIBRARIES}
  motor_library
)

add_executable(gps src/gps.cpp)
target_link_libraries(gps
  ${catkin_LIBRARIES}
)
add_dependencies(gps ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(gps_demo src/gps_demo.cpp)
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

add_executable(path_planning_demo src/path_planning_demo.cpp)
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

add_executable(transicao_demo src/transicao_demo.cpp)
target_link_libraries(transicao_demo
  ${catkin_LIBRARIES}
  motor_library
)
add_dependencies(transicao_demo ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(linha_preta src/linha_preta.cpp)
target_link_libraries(linha_preta
  ${catkin_LIBRARIES}
  motor_library
)
add_dependencies(linha_preta ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(navigating_server_demo src/navigating_server_demo.cpp)
target_link_libraries(navigating_server_demo
  ${catkin_LIBRARIES}
  motor_library
)
add_dependencies(navigating_server_demo ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(depositar_container src/depositar_container.cpp)
target_link_libraries(depositar_container
  ${catkin_LIBRARIES}
  motor_library
)
add_dependencies(depositar_container ${${PROJECT_NAME}_EXPORTED_TARGETS})

<<<<<<< HEAD
<<<<<<< c4a561123ef9295e5956866ed09dd400bb8ce443
<<<<<<< 2c46a2b66ba647f7d62e380a82543e05b67d08b3
<<<<<<< af1c596b3bf6c68830cd7427b35d4db629194fdc
<<<<<<< f0f03972ca605d198e7bab65fbea3c85f33eca65
=======
>>>>>>> Atalização para adição de serviço depositar container
=======
>>>>>>> 078292d76bcf1d5a143daf6018daace08842d8fe
add_executable(depositar_server src/depositar_server.cpp)
target_link_libraries(depositar_server
  ${catkin_LIBRARIES}
  motor_library
)
add_dependencies(depositar_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(depositar_client src/depositar_client.cpp)
target_link_libraries(depositar_client
  ${catkin_LIBRARIES}
  motor_library
)
<<<<<<< HEAD
<<<<<<< c4a561123ef9295e5956866ed09dd400bb8ce443
add_dependencies(depositar_client ${${PROJECT_NAME}_EXPORTED_TARGETS})
=======

>>>>>>> atualização do CMakeLists
=======
add_executable(pegar_container src/pegar_container.cpp)
target_link_libraries(pegar_container
  ${catkin_LIBRARIES}
  motor_library
)
add_dependencies(pegar_container ${${PROJECT_NAME}_EXPORTED_TARGETS})
>>>>>>> Primeiro commit
=======
>>>>>>> Arrumar CMake
=======
add_dependencies(depositar_client ${${PROJECT_NAME}_EXPORTED_TARGETS})
>>>>>>> Atalização para adição de serviço depositar container
=======
add_dependencies(depositar_client ${${PROJECT_NAME}_EXPORTED_TARGETS})
>>>>>>> 078292d76bcf1d5a143daf6018daace08842d8fe
