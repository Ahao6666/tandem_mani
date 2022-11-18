# minimal_robot_gripper
Simple robot gripper demonstration in ROS (5th homework for ROS class)


The folder ps5_yxl1450 is the package that can be compiled in ROS environment. My ROS version is Indigo, you can tested it in other versions. The video "tandem_control_grasp_test.mp4" basicly has demonstrated the structure of this simple robot gripper and how it works.


To get program running, first compile it using catkin_make in your ros workspace. Then running these scripts in seperate terminals:

  $ roscore
  
  $ rosrun gazebo_ros gazebo
  
  $ cd ~/tandem_mani/src/tandem_mani/urdf/

  $ rosrun gazebo_ros spawn_model -file tandem_mani.urdf -urdf -model tandem_mani (in the urdf directory)
  
  $ rosrun tandem_control tandem_control_controller
  
  $ rosrun tandem_control tandem_control_server
  
  $ rosrun tandem_control tandem_control_client


As the video demonstrate, you can play with the beer or table to see how the capability of a simple robot gripper. However the beer is weirdly behaved because the gravity has been set to -0.1m/s^2 for a better control of robot joints. You can play with those parameters also. More detailed description can be found in the pdf file "ps5_writeup_yxl1450.pdf".

The video has been uploaded to Youtube: [https://youtu.be/uPjAnxeLxwU](https://youtu.be/uPjAnxeLxwU)
