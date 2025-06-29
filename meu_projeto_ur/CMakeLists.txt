cmake_minimum_required(VERSION 3.0.2)
project(meu_projeto_ur)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
# A palavra-chave FILES foi descomentada.
add_service_files(
  FILES
  DetectObjects.srv
)

## Generate messages with dependencies
# Esta seção foi movida para depois de add_service_files para manter a ordem lógica.
generate_messages(
  DEPENDENCIES
  geometry_msgs   # Porque o seu serviço usa Pose
  std_msgs        # Porque o seu serviço usa string
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES meu_projeto_ur
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    moveit_core
    moveit_ros_planning_interface
    message_runtime  # <-- ESSENCIAL
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pick_place_ur_cubes src/pick_place_ur_cubes.cpp)

target_link_libraries(pick_place_ur_cubes ${catkin_LIBRARIES})
# O resto do ficheiro são exemplos comentados e podem permanecer como estão.