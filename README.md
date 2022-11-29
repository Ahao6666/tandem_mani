# minimal_robot_gripper
 - `catkin build` for make

To get program running, first compile it using catkin_make in your ros workspace. Then running these scripts in seperate terminals:

  - `roscore`
  
  - `rosrun gazebo_ros gazebo`
  
  - `cd ~/tandem_mani/src/tandem_mani/urdf/`

  - `rosrun gazebo_ros spawn_model -file tandem_mani.urdf -urdf -model tandem_mani` (in the urdf directory)
  
  - `rosrun tandem_control tandem_control_controller`
  
  - `rosrun tandem_control tandem_control_server`
  
  - `rosrun tandem_control tandem_control_client`


or you can run `.start.sh` directly

