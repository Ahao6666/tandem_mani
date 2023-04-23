cmake_minimum_required(VERSION 2.8.3)
project(hexarotor_mani)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  catkin_simple
  roscpp
  std_msgs
  message_generation
  actionlib_msgs
  trajectory_msgs
)

## Generate services in the 'srv' folder
# add_service_files(
  # FILES

# )

## Generate actions in the 'action' folder
add_action_files(
  FILES
  hexarotor_mani_traj.action
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
#  LIBRARIES hexarotor_mani
#  CATKIN_DEPENDS actionlib catkin_simple roscpp
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(hexarotor_mani_controller src/hexarotor_mani_controller.cpp)
add_executable(hexarotor_mani_command src/hexarotor_mani_command.cpp)
add_executable(hexarotor_mani_client src/hexarotor_mani_client.cpp)
add_executable(hexarotor_mani_server src/hexarotor_mani_server.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(hexarotor_mani_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(hexarotor_mani_controller hexarotor_mani_gencpp)
add_dependencies(hexarotor_mani_command hexarotor_mani_gencpp)
add_dependencies(hexarotor_mani_client hexarotor_mani_gencpp)
add_dependencies(hexarotor_mani_server hexarotor_mani_gencpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(hexarotor_mani_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(hexarotor_mani_controller ${catkin_LIBRARIES})
target_link_libraries(hexarotor_mani_command ${catkin_LIBRARIES})
target_link_libraries(hexarotor_mani_client ${catkin_LIBRARIES})
target_link_libraries(hexarotor_mani_server ${catkin_LIBRARIES})
