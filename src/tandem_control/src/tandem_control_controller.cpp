/******************************************************************************
 * Copyright (c) 2022-2024 The IUSL_UAV Shen Jiahao. All rights reserved.
 * See the AUTHORS file for names of contributors.
 *****************************************************************************/

/******************************************************************************
 * @file tandem_control_controller
 * @author Shen Jiahao <shenjiahao@westlake.edu.cn>
 *****************************************************************************/
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <string>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <sensor_msgs/JointState.h>
#include <tandem_control/kpkv_msg.h>
#include "manipulator_IDynamic.h"


// define class to instantiate joints
class Joint {
public:
	Joint(ros::NodeHandle nh, std::string joint_name, double dt); // constructor
	~Joint() {}; // destructor
	void getJointState();
	void jointTrqControl(double torque);
	void kpkvSetting(double kp, double kv);

	double pos_cur; // current joint position
	double vel_cur; // current joint velocity
	double pos_cmd; // joint position from commander
	double vel_cmd; // joint velocity from commander
	double kp;
	double kv;
private:
	// callback for the pos_cmd subscriber
	void posCmdCB(const std_msgs::Float64& pos_cmd_msg);
	void velCmdCB(const std_msgs::Float64& vel_cmd_msg);
	// callback for kpkv service server
	bool kpkvCallback(tandem_control::kpkv_msgRequest& request, tandem_control::kpkv_msgResponse& response);
	// service clients
	ros::ServiceClient get_jnt_state_client;
	ros::ServiceClient set_trq_client;
	// publisher objects
	ros::Publisher trq_publisher;
	ros::Publisher joint_state_publisher;
	// subscriber object
	ros::Subscriber pos_cmd_subscriber;
	ros::Subscriber vel_cmd_subscriber;

	// gazebo/sensor messages
	gazebo_msgs::GetJointProperties get_joint_state_srv_msg;
	gazebo_msgs::ApplyJointEffort effort_cmd_srv_msg;
	sensor_msgs::JointState joint_state_msg;
	// position/velocity/torque messages to be published
	std_msgs::Float64 trq_msg; // torque
	// kpkv service server
	ros::ServiceServer kpkv_server;
	// other parameters
	std::string joint_name;
};

Joint::Joint(ros::NodeHandle nh, std::string joint_name, double dt) {
	// initialize parameters
	this -> joint_name = joint_name;
	pos_cmd = 0.0;
	vel_cmd = 0.0;
	ros::Duration duration(dt);
	kp = 10.0;
	kv = 3.0;

	// initialize gazebo clients
	get_jnt_state_client = nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
	set_trq_client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
	// initialize publisher objects
	trq_publisher = nh.advertise<std_msgs::Float64>(joint_name + "_trq", 1);
	joint_state_publisher = nh.advertise<sensor_msgs::JointState>(joint_name + "_states", 1); 
	// initialize subscriber object
	pos_cmd_subscriber = nh.subscribe(joint_name + "_cmd_jnts_pos", 1, &Joint::posCmdCB, this);
	vel_cmd_subscriber = nh.subscribe(joint_name + "_cmd_jnts_vel", 1, &Joint::velCmdCB, this);

	// initialize kpkv service server
	kpkv_server = nh.advertiseService(joint_name + "_kpkv_service", &Joint::kpkvCallback, this);

	// set up get_joint_state_srv_msg
	get_joint_state_srv_msg.request.joint_name = joint_name;
	// set up effort_cmd_srv_msg
	effort_cmd_srv_msg.request.joint_name = joint_name;
	effort_cmd_srv_msg.request.effort = 0.0;
	effort_cmd_srv_msg.request.duration = duration;
	// set up joint_state_msg
	joint_state_msg.header.stamp = ros::Time::now();
	joint_state_msg.name.push_back(joint_name);
	joint_state_msg.position.push_back(0.0);
	joint_state_msg.velocity.push_back(0.0);
}

void Joint::posCmdCB(const std_msgs::Float64& pos_cmd_msg) {
	// too much information
	// ROS_INFO("received value of %s_pos_cmd is: %f", joint_name.c_str(), pos_cmd_msg.data); 
	pos_cmd = pos_cmd_msg.data;
}

void Joint::velCmdCB(const std_msgs::Float64& vel_cmd_msg) {
	// too much information
	// ROS_INFO("received value of %s_pos_cmd is: %f", joint_name.c_str(), vel_cmd_msg.data); 
	vel_cmd = vel_cmd_msg.data;
}

void Joint::getJointState() {
	// get joint state
	get_jnt_state_client.call(get_joint_state_srv_msg);
	pos_cur = get_joint_state_srv_msg.response.position[0];
	vel_cur = get_joint_state_srv_msg.response.rate[0];
	// publish joint_state_msg
	joint_state_msg.header.stamp = ros::Time::now();
	joint_state_msg.position[0] = pos_cur;
	joint_state_msg.velocity[0] = vel_cur;
	joint_state_publisher.publish(joint_state_msg);
}

// calculate joint torque, publish them, send to gazebo
void Joint::jointTrqControl(double torque) {
	// publish the torque message
	trq_msg.data = torque;
	trq_publisher.publish(trq_msg);
	// send torque command to gazebo
	effort_cmd_srv_msg.request.effort = torque;
	set_trq_client.call(effort_cmd_srv_msg);
	// make sure service call was successful
	bool result = effort_cmd_srv_msg.response.success;
	if (!result)
		ROS_WARN("service call to apply_joint_effort failed!");
}

void Joint::kpkvSetting(double kp, double kv) {
	this -> kp = kp;
	this -> kv = kv;
}

bool Joint::kpkvCallback(tandem_control::kpkv_msgRequest& request, tandem_control::kpkv_msgResponse& response) {
	ROS_INFO("kpkvCallback activated");
	kp = request.kp;
	kv = request.kv;
	// joint_name has to be converted to c_str() for ROS_INFO output
	ROS_INFO("%s: kp has been set to %f, kv has been set to %f", joint_name.c_str(), kp, kv);
	response.setting_is_done = true;
	return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tandem_control_controller_node");
    ros::NodeHandle nh;
	ros::Duration half_sec(0.1);
	Manipulator_IDynamic ID_model;

	// make sure apply_joint_effort service is ready
	bool service_ready = false;
	while (!service_ready) {
		service_ready = ros::service::exists("/gazebo/apply_joint_effort",true);
		ROS_INFO("waiting for apply_joint_effort service");
		half_sec.sleep();
	}
	ROS_INFO("apply_joint_effort service exists");
	// make sure get_joint_state_client service is ready
	service_ready = false;
	while (!service_ready) {
		service_ready = ros::service::exists("/gazebo/get_joint_properties",true);
		ROS_INFO("waiting for /gazebo/get_joint_properties service");
		half_sec.sleep();
	}
	ROS_INFO("/gazebo/get_joint_properties service exists");

	double dt = 0.01; // sample time for the controller

	// instantiate 4 joint instance
	Joint joint1(nh, "joint1", dt);
	Joint joint2(nh, "joint2", dt);
	Joint joint3(nh, "joint3", dt);
	Joint joint4(nh, "joint4", dt);
	Joint joint5(nh, "joint5", dt);

	joint1.kpkvSetting(50, 10);
	joint2.kpkvSetting(50, 10);
	joint3.kpkvSetting(50, 10);
	joint4.kpkvSetting(50, 10);
	joint5.kpkvSetting(50, 10);

	std::vector<double> pos_error; pos_error.resize(5);
	std::vector<double> vel_error; vel_error.resize(5);
    std::vector<double> desired_accel; desired_accel.resize(5);
	std::vector<double> pos_cur_all;pos_cur_all.resize(5);
	std::vector<double> vel_cur_all;vel_cur_all.resize(5);
	std::vector<double> torque;torque.resize(5);

	ros::Rate rate_timer(1 / dt);
	while(ros::ok()) {
		// get joint state(pos, vel) and publish them
		joint1.getJointState();
		joint2.getJointState();
		joint3.getJointState();
		joint4.getJointState();
		joint5.getJointState();

		// compute torque, not perfect
		//firstly,compute joint pos error
		pos_error[0]=joint1.pos_cmd-joint1.pos_cur;
		pos_error[1]=joint2.pos_cmd-joint2.pos_cur;
		pos_error[2]=joint3.pos_cmd-joint3.pos_cur;
		pos_error[3]=joint4.pos_cmd-joint4.pos_cur;
		pos_error[4]=joint5.pos_cmd-joint5.pos_cur;

		ROS_INFO("pos_error %f,%f,%f,%f,%f",pos_error[0],pos_error[1],pos_error[2],pos_error[3],pos_error[4]);

        //compute joint vel error
		vel_error[0]=joint1.vel_cmd-joint1.vel_cur;
		vel_error[1]=joint2.vel_cmd-joint2.vel_cur;
		vel_error[2]=joint3.vel_cmd-joint3.vel_cur;
		vel_error[3]=joint4.vel_cmd-joint4.vel_cur;
		vel_error[4]=joint5.vel_cmd-joint5.vel_cur;
		// ROS_INFO("vel_error %f, %f, %f",vel_error[0],vel_error[1],vel_error[2]);
		
	    //compute desired acceleration by PD control
		desired_accel[0]=joint1.kp*pos_error[0]+joint1.kv*vel_error[0];
		desired_accel[1]=joint2.kp*pos_error[1]+joint2.kv*vel_error[1];
		desired_accel[2]=joint3.kp*pos_error[2]+joint3.kv*vel_error[2];
		desired_accel[3]=joint4.kp*pos_error[3]+joint4.kv*vel_error[3];
		desired_accel[4]=joint5.kp*pos_error[4]+joint5.kv*vel_error[4];

		//transfor to vector
		pos_cur_all[0]=joint1.pos_cur;
		pos_cur_all[1]=joint2.pos_cur;
		pos_cur_all[2]=joint3.pos_cur;
		pos_cur_all[3]=joint4.pos_cur;
		pos_cur_all[4]=joint5.pos_cur;

		vel_cur_all[0]=joint1.vel_cur;
		vel_cur_all[1]=joint2.vel_cur;
		vel_cur_all[2]=joint3.vel_cur;
		vel_cur_all[3]=joint4.vel_cur;
		vel_cur_all[4]=joint5.vel_cur;

        torque = ID_model.rnea(pos_cur_all, vel_cur_all, desired_accel);
		// torque=desired_accel;
		// calculate the torque for each joint and publish them
		joint1.jointTrqControl(torque[0]);
		joint2.jointTrqControl(torque[1]);
		joint3.jointTrqControl(torque[2]);
		joint4.jointTrqControl(torque[3]);
		joint5.jointTrqControl(torque[4]);
		ROS_INFO("torque %f,%f,%f,%f,%f",torque[0],torque[1],torque[2],torque[3],torque[4]);

		ros::spinOnce(); // update pos_cmd, kpkv
		rate_timer.sleep(); // sleep the sample time
	}
}

