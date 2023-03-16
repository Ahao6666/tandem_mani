/******************************************************************************
 * Copyright (c) 2022-2024 The IUSL_UAV Shen Jiahao. All rights reserved.
 * See the AUTHORS file for names of contributors.
 *****************************************************************************/

/******************************************************************************
 * @file floating_platform.cpp
 * @author Shen Jiahao <shenjiahao@westlake.edu.cn>
 *****************************************************************************/
#include <ros/ros.h>
#include <math.h>
#include <string>
#include <gazebo_msgs/SetLinkState.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "floating_platform_node");
	ros::NodeHandle node_handle;
	ros::ServiceClient client = node_handle.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");

    gazebo_msgs::SetLinkState objstate;

    objstate.request.link_state.link_name = "tandem_mani::platform";
	objstate.request.link_state.reference_frame = "world";

	ros::Rate rate_timer(1 / 0.01);
	while(ros::ok()) {
		objstate.request.link_state.pose.position.x = 0.5* cos(0.25 * ros::Time::now().toSec());
		objstate.request.link_state.pose.position.y = 0.5* sin(0.25 * ros::Time::now().toSec());
		objstate.request.link_state.pose.position.z = 0.2;
		objstate.request.link_state.pose.orientation.w = 1;
		objstate.request.link_state.pose.orientation.x = 0;
		objstate.request.link_state.pose.orientation.y = 0;
		objstate.request.link_state.pose.orientation.z = 0;
		client.call(objstate);

		ros::spinOnce();
		rate_timer.sleep(); // sleep the sample time
	}
	return 0;
}

