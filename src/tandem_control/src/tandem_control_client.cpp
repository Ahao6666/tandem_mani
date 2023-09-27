/******************************************************************************
 * Copyright (c) 2022-2024 The IUSL_UAV Shen Jiahao. All rights reserved.
 * See the AUTHORS file for names of contributors.
 *****************************************************************************/

/******************************************************************************
 * @file tandem_control_client
 * @brief This node controls the tandem manipulator in Gazebo simulation environment
 * @details This node subscribes to the joint states of the tandem manipulator and publishes the desired end-effector pose and velocity to the /tandem_manipulator/joint_trajectory_controller/command topic.
 * @note Make sure to run the gazebo simulation and the joint trajectory controller before running this node.
 * @note If you encounter an error "process has died [pid XXXX, exit code -11]", try increasing the memory limit by running "ulimit -s unlimited" in the terminal before running the node.
 * @author Shen Jiahao
 *****************************************************************************/

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <string>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/GetLinkState.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include "manipulator_kinematic.h"

geometry_msgs::Pose end_point_pose;
void planEEPoseAndVel(Manipulator_Kinematic K_model,Vector3d &desiredEEPos,Matrix3d &desiredEERotation,VectorXd &desiredEEVel){
	double currentTime=ros::Time::now().toSec();
	VectorXd jointAngle(5);
	VectorXd jointAngleVel(5);

	//input desired joints traj
	for (int j = 0; j < 5; j++) {
		// jointAngle[j]= 5 * sin(currentTime) * M_PI / 30;
		// jointAngleVel[j]=5 * cos(currentTime) * M_PI / 30;
		jointAngle[j]= 0.1;
		jointAngleVel[j]=0;
		}
	//compute desired EE's pose
	K_model.forward_kinematic(jointAngle);
	desiredEEPos = K_model.T_from_base.at(5).block<3, 1>(0, 3);
	desiredEERotation= K_model.T_from_base.at(5).block<3, 3>(0, 0);

	// compute desired vel by jacobian
	K_model.get_jacobian();
	MatrixXd jacobian(5,5);
	jacobian=K_model.jacobian.block<5, 5>(0, 0);
    desiredEEVel.resize(5);
	desiredEEVel=jacobian*jointAngleVel;
}

void getEEPose(Manipulator_Kinematic K_model,VectorXd current_jnts,Vector3d &EEPos,Matrix3d &EERotation){
	//compute EE's pose
	current_jnts.resize(5);
	K_model.forward_kinematic(current_jnts);
	EEPos = K_model.T_from_base.at(5).block<3, 1>(0, 3);
	EERotation= K_model.T_from_base.at(5).block<3, 3>(0, 0);
}

void EEposeCmdCB(const geometry_msgs::Pose& EEpose) {
	// ROS_INFO("received value of %s_pos_cmd is: %f", joint_name.c_str(), pos_cmd_msg.data); 
	end_point_pose.position = EEpose.position;
	end_point_pose.orientation = EEpose.orientation;
}

// trajectory action client for the robot
int main(int argc, char** argv) {
	ros::init(argc, argv, "tandem_control_client_node");
	ros::NodeHandle nh;

	ROS_INFO("client starts");
	// initialize a service client to get joint positions
	ros::ServiceClient get_jnt_state_client = nh.serviceClient<gazebo_msgs::GetJointProperties>(
		"/gazebo/get_joint_properties");
	gazebo_msgs::GetJointProperties get_joint_state_srv_msg;

	ros::ServiceClient get_link_state_client = nh.serviceClient<gazebo_msgs::GetLinkState>(
		"/gazebo/get_link_state");
	gazebo_msgs::GetLinkState get_link_state_srv_msg;

    Manipulator_Kinematic K_model;

	// publish desired joints
	std::vector<double> cmd_jnts_pos; // command joints position to be sent
	std::vector<double> cmd_jnts_vel; // command joints position to be sent
	std::vector<ros::Publisher> cmd_jnts_pos_publisher;  // initialization will be in executeCB()
	std::vector<ros::Publisher> cmd_jnts_vel_publisher;  // initialization will be in executeCB()
	cmd_jnts_pos.resize(5);
	cmd_jnts_pos_publisher.resize(5);
	cmd_jnts_vel.resize(5);
	cmd_jnts_vel_publisher.resize(5);

	// subscribe EEPose
	ros::Subscriber EEpose_subscriber = nh.subscribe("end_point_pose", 1, &EEposeCmdCB);

	std::string jointNames[]={"joint1","joint2","joint3","joint4","joint5"};
	//initialize command publishers for each joints，first advertise,then publish
	for(int j=0; j < 5; j++){
			cmd_jnts_pos_publisher[j] = nh.advertise<std_msgs::Float64>( 
			jointNames[j] + "_cmd_jnts_pos", 1, true);
			cmd_jnts_vel_publisher[j] = nh.advertise<std_msgs::Float64>( 
			jointNames[j] + "_cmd_jnts_vel", 1, true);
	}

	VectorXd current_jnts(5);
	VectorXd desiredEEPose(5); 
	VectorXd desiredEEVel(5);
	VectorXd poseError(5);
	VectorXd EEVel(5);
	MatrixXd jacobian(5,5);
	MatrixXd pseudoInverseJacbian(5,5);
	VectorXd desiredJointsVel(5);
	double kp = 1;
	double lambda_max = 0.005;
	double lambda;
	double sigma=0.5;

	double dt=0.01;
    ros::Rate rate_timer(1 / dt);
	static VectorXd desiredJoints =VectorXd::Zero(5);//static memory

    while(ros::ok()) {
		//call for current joints
		// for (int i=0; i<5; i++) {
		// 	get_joint_state_srv_msg.request.joint_name = jointNames[i];
		// 	get_jnt_state_client.call(get_joint_state_srv_msg);
		// 	current_jnts[i] = get_joint_state_srv_msg.response.position[0];
		// }

	// //secondly,compute EE pose
	// Vector3d EEPos;Matrix3d EERotation;
	// getEEPose(K_model,current_jnts,EEPos,EERotation);
	// // ROS_INFO("EEpos1 %f, %f, %f",EEPos[0],EEPos[1],EEPos[2]);
	// // ROS_INFO("EEpos2 %f, %f, %f",end_point_pose.position.x,end_point_pose.position.y,end_point_pose.position.z);

	// //thirdly,compute desired EE pose and vel
	// Vector3d desiredEEPos; Matrix3d desiredEERotation;VectorXd desiredEEVel(5);
	// planEEPoseAndVel(K_model,desiredEEPos,desiredEERotation,desiredEEVel);
	// //ROS_INFO("EEpose %f, %f, %f, %f,%f",desiredEEPose[0],desiredEEPose[1],desiredEEPose[2],desiredEEPose[3],desiredEEPose[4]);

	// //ClIK method 
	// //firstly,compute position and rotation error
	// Vector3d pos_error=desiredEEPos-EEPos;//position error
	// // ROS_INFO("pos_error %f, %f, %f",pos_error[0],pos_error[1],pos_error[2]);

	// //compute orientation , a little complex
	// Vector3d n=EERotation.block<3,1>(0,0);Vector3d s=EERotation.block<3,1>(0,1);Vector3d a=EERotation.block<3,1>(0,2);

	// Vector3d desired_n=desiredEERotation.block<3,1>(0,0);Vector3d desired_s=desiredEERotation.block<3,1>(0,1);
	// Vector3d desired_a=desiredEERotation.block<3,1>(0,2);
	// Vector3d rotation_error=1.0/2*(n.cross(desired_n)+s.cross(desired_s)+a.cross(desired_a));

	// VectorXd poseError(5);VectorXd EEVel(5); double kp=1;
	// poseError.block<3,1>(0,0)=pos_error;
	// poseError.block<2,1>(3,0)=rotation_error.block<2,1>(0,0);
	// // ROS_INFO("rotation_error %f, %f, %f",rotation_error[0],rotation_error[1],rotation_error[2]);

	// EEVel=desiredEEVel+kp*poseError;

	// //DLS
	// K_model.forward_kinematic(current_jnts);//for jacobian
	// K_model.get_jacobian();
	// MatrixXd jacobian(5,5);
	// jacobian=K_model.jacobian.block<5, 5>(0, 0);

	// MatrixXd J_mid=jacobian * jacobian.transpose();
	// lambda=lambda_max*exp(-J_mid.determinant()/(2*pow(sigma,2)));
	// pseudoInverseJacbian = jacobian.transpose() *(J_mid+ pow(lambda,2) * MatrixXd::Identity(5, 5)).inverse();
	// desiredJointsVel=pseudoInverseJacbian * EEVel;

	// desiredJoints=desiredJoints+desiredJointsVel*dt;
	// desiredJoints<<0.1,-0.1,0.1,0.1,0.1;
	desiredJoints<<1,1,1,1,1;

	desiredJointsVel.setZero();
	//ROS_INFO("desiredJoints %f, %f, %f,%f,%f",desiredJoints[0],desiredJoints[1],desiredJoints[2],desiredJoints[3],desiredJoints[4]);

		std_msgs::Float64 cmd_jnts_pos_msg;
		std_msgs::Float64 cmd_jnts_vel_msg;
		for(int j=0; j < 5; j++){
			cmd_jnts_pos_msg.data = desiredJoints[j];
			cmd_jnts_pos_publisher[j].publish(cmd_jnts_pos_msg);

			cmd_jnts_vel_msg.data = desiredJointsVel[j];
			cmd_jnts_vel_publisher[j].publish(cmd_jnts_vel_msg);
		}

		ros::spinOnce();
		rate_timer.sleep(); // sleep the sample time
	}

	return 0;
}