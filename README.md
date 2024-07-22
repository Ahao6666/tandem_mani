# minimal_robot_gripper
 - `catkin build` for make

To get program running, first compile it using `catkin_make` in your ros workspace. Then running these scripts in seperate terminals:

  - `roscore`  
  - `rosrun gazebo_ros gazebo`  
  - `rosrun gazebo_ros spawn_model -urdf -file ~/tandem_mani/src/tandem_mani/urdf/tandem_mani.urdf -model tandem_mani`  
  - `rosrun tandem_control tandem_control_controller`  
  - `rosrun tandem_control tandem_control_client`  

or you can run `.start.sh` or `roslaunch start.launch` directly

  - `rosrun tandem_control floating_platform` to move the platform  

# note: 
  1. need to **modify the relative path to absolute path** in sdf file
  
  
# 20240718

  ## start PX4
 - `roslaunch px4 posix_sitl_arm.launch ` 
 
  ## start QGC
 - `QGC` 
 
  ## srart tandem_control
 - `cd ~/tandem_mani`
 - `source devel/setup.bash`
 - `roslaunch start.launch`

  ## takeoff UAV
 - `source devel/setup.bash`
 - `roslaunch robot_connection.launch`
 - new terminal
 - `source devel/setup.bash`
 - `rosrun qt_joystick qt_joystick`
