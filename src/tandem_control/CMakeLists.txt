cmake_minimum_required(VERSION 2.8.3)
project(tandem_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  catkin_simple
  roscpp
  rosbag
  std_msgs
  message_generation
  actionlib_msgs
  trajectory_msgs
)
find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  kpkv_msg.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  traj.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  actionlib_msgs
  trajectory_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tandem_control
#  CATKIN_DEPENDS actionlib catkin_simple roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
## Declare a C++ library
# add_library(tandem_control
#   src/${PROJECT_NAME}/tandem_control.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(tandem_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(tandem_control_node src/tandem_control_node.cpp)
add_executable(tandem_control_controller src/tandem_control_controller.cpp)
add_executable(tandem_control_kpkv_serviceClient src/kpkv_serviceClient.cpp)
add_executable(tandem_control_client src/tandem_control_client.cpp)
add_executable(floating_platform src/floating_platform.cpp)
add_executable(record_bag_node src/record_bag_node.cpp)
add_executable(end_point_position src/end_point_position.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(tandem_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(tandem_control_controller tandem_control_gencpp)
add_dependencies(tandem_control_kpkv_serviceClient tandem_control_gencpp)
add_dependencies(tandem_control_client tandem_control_gencpp)
add_dependencies(floating_platform tandem_control_gencpp)
add_dependencies(record_bag_node tandem_control_gencpp)
add_dependencies(end_point_position tandem_control_gencpp)

target_link_libraries(tandem_control_controller ${catkin_LIBRARIES})
target_link_libraries(tandem_control_kpkv_serviceClient ${catkin_LIBRARIES})
target_link_libraries(tandem_control_client ${catkin_LIBRARIES})
target_link_libraries(floating_platform ${catkin_LIBRARIES})
target_link_libraries(record_bag_node ${catkin_LIBRARIES})
target_link_libraries(end_point_position ${catkin_LIBRARIES})
