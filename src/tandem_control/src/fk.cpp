
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

int main(int argc, char** argv) {
	ros::init(argc, argv, "fk_node");
	ros::NodeHandle nh;
    Manipulator_Kinematic K_model;

    double dt=0.01;
    ros::Rate rate_timer(1 / dt);

    VectorXd current_jnts;
    current_jnts.resize(5);
    ros::Subscriber EEpose_subscriber = nh.subscribe("end_point_pose", 1, &EEposeCmdCB);
    ROS_INFO("fk starts");
	// initialize a service client to get joint positions
	ros::ServiceClient get_jnt_state_client = nh.serviceClient<gazebo_msgs::GetJointProperties>(
		"/gazebo/get_joint_properties");
    	gazebo_msgs::GetJointProperties get_joint_state_srv_msg;
        std::string jointNames[]={"joint1","joint2","joint3","joint4","joint5"};

    while(ros::ok()) {

    // double time=ros::Time::now().toSec();
    // current_jnts<<0.5*sin(0.1*time),0.5*sin(0.1*time),0.5*sin(0.1*time),0.5*sin(0.1*time),0.5*sin(0.1*time);

//call for current joints
		for (int i=0; i<5; i++) {
			get_joint_state_srv_msg.request.joint_name = jointNames[i];
			get_jnt_state_client.call(get_joint_state_srv_msg);
			current_jnts[i] = get_joint_state_srv_msg.response.position[0];
		}

	Vector3d EEPos;Matrix3d EERotation;
	getEEPose(K_model,current_jnts,EEPos,EERotation);
	ROS_INFO("EEpos1 %f, %f, %f",EEPos[0],EEPos[1],EEPos[2]);
	ROS_INFO("EEpos2 %f, %f, %f",end_point_pose.position.x,end_point_pose.position.y,end_point_pose.position.z);

	ros::spinOnce();
	rate_timer.sleep(); // sleep the sample time
    }

    return 0;

}
