#!/bin/bash
 
module_delay=1
#roscore
gnome-terminal -x  bash -c "roscore; exec bash -i"
sleep 5
echo "roscore ready."

# 0. gazebo start   
gnome-terminal -x  bash -c "rosrun gazebo_ros gazebo"
sleep ${module_delay}   
echo "gazebo ready."

# 1. establich model 
gnome-terminal -x  bash -c "rosrun gazebo_ros spawn_model -file ~/tandem_mani/src/tandem_mani/urdf/tandem_mani.urdf -urdf -model tandem_mani"
sleep ${module_delay}
echo "model ready."

# 2. tandem_control_controller
gnome-terminal -x  bash -c "rosrun tandem_control tandem_control_controller"
sleep ${module_delay}
echo "tandem_control_controller ready."

# # 3. tandem_control_server
# gnome-terminal -x  bash -c "rosrun tandem_control tandem_control_server"
# sleep ${module_delay}
# echo "tandem_control_server ready."

# 4. tandem_control_client
gnome-terminal -x  bash -c "rosrun tandem_control tandem_control_client"
sleep ${module_delay}
echo "tandem_control_client ready."

# 5.record_bag_node
#gnome-terminal -x  bash -c "rosrun tandem_control record_bag_node"
#sleep ${module_delay}
#echo "record_bag_node ready."


echo "System is started."


