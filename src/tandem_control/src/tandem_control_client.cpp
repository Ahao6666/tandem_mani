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

#include "manipulator_kinematic.h"

//compute Euler angles by R,定义在前面，定义在后面会报错
Vector3d computeEulerAngles(Matrix3d R) {
	double theta = 0, psi = 0, pfi = 0;
	if (abs(R(2, 0)) < 1 - FLT_MIN || abs(R(2, 0)) > 1 + FLT_MIN) { // abs(R(2, 0)) != 1
		double theta1 = -asin(R(2, 0));
		double theta2 = M_PI- theta1;
		double psi1 = atan2(R(2, 1) / cos(theta1), R(2, 2) / cos(theta1));
		double psi2 = atan2(R(2, 0) / cos(theta2), R(2, 2) / cos(theta2));
		double pfi1 = atan2(R(1, 0) / cos(theta1), R(0, 0) / cos(theta1));
		double pfi2 = atan2(R(1, 0) / cos(theta2), R(0, 0) / cos(theta2));
		theta = theta1;
		psi = psi1;
		pfi = pfi1;
	}
	else {
		double phi = 0;
		double delta = atan2(R(0, 1), R(0, 2));
		if (R(2, 0) > -1 - FLT_MIN && R(2, 0) < -1 + FLT_MIN) { // R(2,0) == -1
			theta = M_PI/ 2;
			psi = phi + delta;
		}
		else {
			theta = -M_PI/ 2;
			psi = -phi + delta;
		}
	}
	Vector3d EulerAngles;
	EulerAngles[0] = psi;
	EulerAngles[1] = theta;
	EulerAngles[2] = pfi;
	return EulerAngles;
}

void planEEPoseAndVel(Manipulator_Kinematic K_model,VectorXd &desiredEEPose,VectorXd &desiredEEVel){
	double currentTime=ros::Time::now().toSec();
	VectorXd jointAngle(5);
	VectorXd jointAngleVel(5);

	//input desired joints traj
	for (int j = 0; j < 5; j++) {
		jointAngle[j]= 5 * sin(currentTime) * M_PI / 30;
		jointAngleVel[j]=5 * cos(currentTime) * M_PI / 30;
		// jointAngle[j]= 1;
		// jointAngleVel[j]=0;
		}
	//compute desired EE's pose
	K_model.get_R_t(jointAngle);
	Matrix3d EERotationMatrix;
	EERotationMatrix= K_model.T.at(5).block<3, 3>(0, 0);

	Vector3d pos = K_model.T.at(5).block<3, 1>(0, 3);
	Vector3d EulerAngles=computeEulerAngles(EERotationMatrix);

	desiredEEPose.resize(5);
	desiredEEPose.block<3,1>(0,0)=pos;
	desiredEEPose.block<2,1>(3,0)=EulerAngles.block<2,1>(0,0);

	// compute desired vel by jacobian
	K_model.get_jacobain(jointAngle);
	MatrixXd jacobian(5,5);
	jacobian=K_model.jacob.block<5, 5>(0, 0);
	desiredEEVel=jacobian*jointAngleVel;
}

void getEEPose(Manipulator_Kinematic K_model,VectorXd current_jnts,VectorXd &EEPose){
	//compute EE's pose
	current_jnts.resize(5);
	K_model.get_R_t(current_jnts);
	Matrix3d EERotationMatrix;
	EERotationMatrix= K_model.T.at(5).block<3, 3>(0, 0);

	Vector3d pos = K_model.T.at(5).block<3, 1>(0, 3);
	Vector3d EulerAngles=computeEulerAngles(EERotationMatrix);

	EEPose.resize(5);
	EEPose.block<3,1>(0,0)=pos;
	EEPose.block<2,1>(3,0)=EulerAngles.block<2,1>(0,0);
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
	std::vector<ros::Publisher> cmd_jnts_pos_publisher;  // initialization will be in executeCB()
	cmd_jnts_pos.resize(5);
	cmd_jnts_pos_publisher.resize(5);

	std::string jointNames[]={"joint1","joint2","joint3","joint4","joint5"};
	//initialize command publishers for each joints，先advertise一下,再publish一下
	for(int j=0; j < 5; j++){
			cmd_jnts_pos_publisher[j] = nh.advertise<std_msgs::Float64>( 
			jointNames[j] + "_cmd_jnts_pos", 1, true);
	}

	double dt=0.02;
    ros::Rate rate_timer(1 / dt);

    VectorXd current_jnts(5);
    while(ros::ok()) {

		//call for current joints
		for (int i=0; i<5; i++) {
			get_joint_state_srv_msg.request.joint_name = jointNames[i];
			get_jnt_state_client.call(get_joint_state_srv_msg);
			current_jnts[i] = get_joint_state_srv_msg.response.position[0];
			ROS_INFO("joint position = %f",get_joint_state_srv_msg.response.position[0]);
		}

		//call for current EE pose
		VectorXd EEPose_gz(3);
		get_link_state_srv_msg.request.link_name = "link5";
		//get_link_state_srv_msg.request.reference_frame = "wrold";
		get_link_state_client.call(get_link_state_srv_msg);
		EEPose_gz[0]= get_link_state_srv_msg.response.link_state.pose.position.x;
		EEPose_gz[1]= get_link_state_srv_msg.response.link_state.pose.position.y;
		EEPose_gz[2]= get_link_state_srv_msg.response.link_state.pose.position.z;
		// ROS_INFO("EEpose_gz %f, %f, %f",EEPose_gz[0],EEPose_gz[1],EEPose_gz[2]);

		//secondly,compute EE pose
		VectorXd EEPose(5);
		getEEPose(K_model,current_jnts,EEPose);
		// ROS_INFO("EEpose %f, %f, %f",EEPose[0],EEPose[1],EEPose[2]);

		//thirdly,compute desired EE pose and vel
		VectorXd desiredEEPose(5); VectorXd desiredEEVel(5);
		planEEPoseAndVel(K_model,desiredEEPose,desiredEEVel);

		//ClIK method 
		VectorXd poseError(5);VectorXd EEVel(5); double kp=1;
		poseError=desiredEEPose-EEPose;
		EEVel=desiredEEVel+kp*poseError;

		//DLS
		K_model.get_jacobain(current_jnts);
		MatrixXd jacobian(5,5);
		jacobian=K_model.jacob.block<5, 5>(0, 0);
		JacobiSVD<MatrixXd> svd(jacobian, ComputeThinU | ComputeThinV);
		double sigma= svd.singularValues()[4];// 第五个特征值
		double epsilon= 1 / M_PI;
		double lambda_max = 0.001;
		double lambda;
		if (sigma >= epsilon)
		{
			lambda = 0;
		}
		else
		{
           lambda = lambda_max*(1-sigma / epsilon);
		}
		
		MatrixXd pseudoInverseJacbian(5,5);VectorXd desiredJointsVel(5);VectorXd desiredJoints(5);
		pseudoInverseJacbian = jacobian.transpose() *(jacobian * jacobian.transpose()+ pow(lambda,2) * MatrixXd::Identity(5, 5)).inverse();
		desiredJointsVel=pseudoInverseJacbian * EEVel;
		desiredJoints=current_jnts+desiredJointsVel*dt;

		for(int j=0; j < 5; j++){
			std_msgs::Float64 cmd_jnts_pos_msg;
			cmd_jnts_pos_msg.data = desiredJoints[j];
			//cmd_jnts_pos_msg.data = 0;
			cmd_jnts_pos_publisher[j].publish(cmd_jnts_pos_msg);
		}

		ros::spinOnce();
		rate_timer.sleep(); // sleep the sample time
	}

	return 0;
}