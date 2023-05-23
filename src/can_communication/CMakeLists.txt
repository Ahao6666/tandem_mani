cmake_minimum_required(VERSION 2.8.3)
project(can_communication)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  socketcan_interface
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(can_node src/can_node.cpp)
target_link_libraries(can_node ${catkin_LIBRARIES})
