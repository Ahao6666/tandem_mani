/******************************************************************************
 * Copyright (c) 2022-2024 The IUSL_UAV Shen Jiahao. All rights reserved.
 * See the AUTHORS file for names of contributors.
 *****************************************************************************/

/******************************************************************************
 * @file hexarotor_mani_client
 * @author Shen Jiahao <shenjiahao@westlake.edu.cn>
 *****************************************************************************/


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <hexarotor_mani/hexarotor_mani_trajAction.h>
#include <vector>
#include <math.h>
#include <string>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Point.h>


// callback to get "result" message from action server
void doneCb(const actionlib::SimpleClientGoalState& state,
		const hexarotor_mani::hexarotor_mani_trajResultConstPtr& result) {
	ROS_INFO("doneCb: server responded with state [%s]", state.toString().c_str());
}

// trajectory action client for the gripper robot
int main(int argc, char** argv) {
	ros::init(argc, argv, "hexarotor_mani_client_node");
	ros::NodeHandle nh;

	// initialize an action client
	actionlib::SimpleActionClient<hexarotor_mani::hexarotor_mani_trajAction> action_client(
		"hexarotor_mani", true);
	// try to connect the client to action server
	bool server_exist = action_client.waitForServer(ros::Duration(5.0));
	ros::Duration sleep1s(1);
	if(!server_exist) {
		ROS_WARN("could not connect to server; retrying");
		bool server_exist = action_client.waitForServer(ros::Duration(1.0));
		sleep1s.sleep();
	}
	// if here, then connected to the server
	ROS_INFO("connected to action server");

	hexarotor_mani::hexarotor_mani_trajGoal goal;
	// instantiate goal message
	trajectory_msgs::JointTrajectory trajectory;
	trajectory_msgs::JointTrajectoryPoint trajectory_points;
	// joint_names field
	trajectory.joint_names.resize(5);
	trajectory.joint_names[0] = "tandem_mani::joint1";
	trajectory.joint_names[1] = "tandem_mani::joint2";
	trajectory.joint_names[2] = "tandem_mani::joint3";
	trajectory.joint_names[3] = "tandem_mani::joint4";
	trajectory.joint_names[4] = "tandem_mani::joint5";
	// positions and velocities field
	trajectory_points.positions.resize(5);

	// initialize a service client to get joint positions
	ros::ServiceClient get_jnt_state_client = nh.serviceClient<gazebo_msgs::GetJointProperties>(
		"/gazebo/get_joint_properties");
	gazebo_msgs::GetJointProperties get_joint_state_srv_msg;
	// initialize a service client to get model state
	ros::ServiceClient get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>(
		"/gazebo/get_model_state");
	gazebo_msgs::GetModelState get_model_state_srv_msg;

	// parameters for flow control, time assignment
	int time_1 = 3; // time for task 1
	double time_delay = 1.0; // delay between every task
	std::vector<double> start_jnts; // start joints for each move task
	std::vector<double> end_jnts; // end joints for each move task
	double fraction_of_range;
	start_jnts.resize(5);
	end_jnts.resize(5);

	while(ros::ok()){
		// get the original joint positions when this node is invoked
		std::vector<double> origin_jnts;
		origin_jnts.resize(5);
		for (int i=0; i<5; i++) {
			get_joint_state_srv_msg.request.joint_name = trajectory.joint_names[i];
			get_jnt_state_client.call(get_joint_state_srv_msg);
			origin_jnts[i] = get_joint_state_srv_msg.response.position[0];
		}
		// assign current joints to start joints
		start_jnts = origin_jnts;

		// define the safe point, avoid singularity at origin
		std::vector<double> safe_jnts;
		safe_jnts.resize(5);
		safe_jnts[0] = 0; // joint1, 
		safe_jnts[1] = -M_PI/2; // joint2, 
		safe_jnts[2] = 0; // joint3, 
		safe_jnts[3] = 0; // joint4, 
		safe_jnts[4] = sin(0.25 * ros::Time::now().toSec());
		// assign the safe joints to end joints
		end_jnts = safe_jnts;

		// prepare the goal message
		trajectory.points.clear();
		for (int i=0; i<time_1+1; i++) { // there are time_1+1 points, including start and end
			fraction_of_range = (double)i/time_1; // cautious, convert to double
			for (int j=0; j<5; j++) { // there are 5 joints
				trajectory_points.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
			}
			trajectory_points.time_from_start = ros::Duration((double)i);
			trajectory.points.push_back(trajectory_points);
		}
		// copy this trajectory into our action goal
		goal.trajectory = trajectory;
		// send out the goal
		action_client.sendGoal(goal, &doneCb);
		// if here, task 1 is finished successfully
		ros::Duration(time_delay).sleep(); // delay before jumping to next task
	}

	
	return 0;
}

