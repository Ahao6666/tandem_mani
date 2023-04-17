cmake_minimum_required(VERSION 2.8.3)
project(my_gazebo_plugin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  std_msgs
  catkin_simple
  actionlib
  message_generation
  actionlib_msgs
  trajectory_msgs
)
find_package(gazebo REQUIRED)
# The motor_model node is actually added in the CMakeLists.txt file, so it exists.
# However, it is commented out and not being built because it is not being linked to any other targets.
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tandem_control
#  CATKIN_DEPENDS actionlib catkin_simple roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)

# link_directories(
#   ${GAZEBO_LIBRARY_DIRS}
# )

# add_library(my_gazebo_plugin SHARED
#   src/my_gazebo_plugin.cpp
# )
add_executable(motor_model src/motor_model.cpp)
add_dependencies(motor_model my_gazebo_plugin_gencpp)
target_link_libraries(motor_model 
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
)

# target_link_libraries(my_gazebo_plugin
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
#   ${SDFormat_LIBRARIES}
# )

# set(CATKIN_PACKAGE_LIB_DESTINATION lib/my_gazebo_plugin)

# install(TARGETS my_gazebo_plugin
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )
