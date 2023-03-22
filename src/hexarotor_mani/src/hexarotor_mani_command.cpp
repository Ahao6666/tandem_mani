/******************************************************************************
 * Copyright (c) 2022-2024 The IUSL_UAV Shen Jiahao. All rights reserved.
 * See the AUTHORS file for names of contributors.
 *****************************************************************************/

/******************************************************************************
 * @file hexarotor_mani_command
 * @author Shen Jiahao <shenjiahao@westlake.edu.cn>
 *****************************************************************************/
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>

int main(int argc, char** argv) {
	ros::init(argc, argv, "hexarotor_mani_command_node");
	ros::NodeHandle nh;	// to be passed in the instantiation of class
	std::vector<std::string> joint_names = {"joint1","joint2","joint3","joint4","joint5"};
	//initialize command publishers for each joints
	std::vector<ros::Publisher> joint_pos_cmd_publisher_;
	joint_pos_cmd_publisher_.resize(5);
	for (int i=0; i<5; i++) {
		joint_pos_cmd_publisher_[i] = nh.advertise<std_msgs::Float64>(
			joint_names[i] + "_pos_cmd", 1, true);
	}

	while (ros::ok()) {
		std::vector<double> cmd_data = {1.0, 0.5, 0.1, -0.8, -0.5};
		std_msgs::Float64 cmd_msg;
		for (int i=0; i<5; i++){
			cmd_msg.data = cmd_data[i];
			joint_pos_cmd_publisher_[i].publish(cmd_msg);
		}

		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}

	return 0;
}

