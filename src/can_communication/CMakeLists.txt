cmake_minimum_required(VERSION 2.8.3)
project(can_communication)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  catkin_simple
  std_msgs
  socketcan_interface
  message_generation
)
add_message_files(
  FILES 
  MotorPosTar.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  std_msgs 
  message_runtime
)
include_directories(include ${catkin_INCLUDE_DIRS})

# add_executable(can_node src/can_node.cpp)
add_executable(can_motor src/can_motor.cpp)
add_executable(motor_MS src/motor_MS.c)
add_executable(Motor_Pos_Tar src/Motor_Pos_Tar.cpp)

# target_link_libraries(can_node ${catkin_LIBRARIES})
target_link_libraries(can_motor ${catkin_LIBRARIES})
target_link_libraries(motor_MS ${catkin_LIBRARIES})
target_link_libraries(Motor_Pos_Tar ${catkin_LIBRARIES})
