/******************************************************************************
 * Copyright (c) 2022-2024 The IUSL_UAV Shen Jiahao. All rights reserved.
 * See the AUTHORS file for names of contributors.
 *****************************************************************************/

/******************************************************************************
 * @file hexarotor_mani_controller
 * @author Shen Jiahao <shenjiahao@westlake.edu.cn>
 *****************************************************************************/
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <string>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <sensor_msgs/JointState.h>

// define class to instantiate joints
class Joint {
public:
	Joint(ros::NodeHandle nh, std::string joint_name, double dt); // constructor
	~Joint() {}; // destructor
	void getJointState();
	void jointTrqControl();
	void paramSetting(double kp, double kv);
private:
	// callback for the pos_cmd subscriber
	void posCmdCB(const std_msgs::Float64& pos_cmd_msg);
	// service clients
	ros::ServiceClient get_jnt_state_client;
	ros::ServiceClient set_trq_client;
	// publisher objects
	ros::Publisher trq_publisher;
	ros::Publisher joint_state_publisher;
	// subscriber object
	ros::Subscriber pos_cmd_subscriber;
	// gazebo/sensor messages
	gazebo_msgs::GetJointProperties get_joint_state_srv_msg;
	gazebo_msgs::ApplyJointEffort effort_cmd_srv_msg;
	sensor_msgs::JointState joint_state_msg;
	// position/velocity/torque messages to be published
	std_msgs::Float64 trq_msg; // torque

	// control parameters
	double pos_cur; // current joint position
	double vel_cur; // current joint velocity
	double pos_cmd; // joint position from commander
	double pos_err; // error between pos_cmd and pos_cur
	double trq_cmd; // torque to be published
	double kp;
	double kv;
	// other parameters
	std::string joint_name;
};

Joint::Joint(ros::NodeHandle nh, std::string joint_name, double dt) {
	// initialize parameters
	std::string joint_name_new = "tandem_mani::" + joint_name;
	this -> joint_name = joint_name_new;

	pos_cmd = 0.0;
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
	// pos_cmd_subscriber = nh.subscribe(joint_name + "_pos_cmd", 1, boost::bind(&Joint::posCmdCB, this, _1));
	// why boost::bind gives compile errors here?
	pos_cmd_subscriber = nh.subscribe(joint_name + "_pos_cmd", 1, &Joint::posCmdCB, this);

	// set up get_joint_state_srv_msg
	get_joint_state_srv_msg.request.joint_name = joint_name_new;
	// set up effort_cmd_srv_msg
	effort_cmd_srv_msg.request.joint_name = joint_name_new;
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
void Joint::jointTrqControl() {
	pos_err = pos_cmd - pos_cur;
	// watch for periodicity
	if (pos_err > M_PI)
		pos_err = pos_err - 2 * M_PI;
	if (pos_err < -M_PI)
		pos_err = pos_err + 2 * M_PI;
	// control algorithm in one line
	trq_cmd  = kp * pos_err - kv * vel_cur;
	// publish the torque message
	trq_msg.data = trq_cmd;
	trq_publisher.publish(trq_msg);
	// send torque command to gazebo
	effort_cmd_srv_msg.request.effort = trq_cmd;
	set_trq_client.call(effort_cmd_srv_msg);
	// make sure service call was successful
	bool result = effort_cmd_srv_msg.response.success;
	if (!result)
		ROS_WARN("service call to apply_joint_effort failed!");
}

void Joint::paramSetting(double kp, double kv) {
	this -> kp = kp;
	this -> kv = kv;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tandem_manipulator_controller");
    ros::NodeHandle nh;
	ros::Duration half_sec(0.5);

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

	// instantiate 4 joint instances
	Joint joint1(nh, "joint1", dt);
	Joint joint2(nh, "joint2", dt);
	Joint joint3(nh, "joint3", dt);
	Joint joint4(nh, "joint4", dt);
	Joint joint5(nh, "joint5", dt);

	joint1.paramSetting(20, 1.0 );
	joint2.paramSetting(20, 0.8 );
	joint3.paramSetting(3, 0.1 );
	joint4.paramSetting(2, 0.05 );
	joint5.paramSetting(0.5, 0.01 );

	while(ros::ok()) {
		// get joint state(pos, vel) and publish them
		joint1.getJointState();
		joint2.getJointState();
		joint3.getJointState();
		joint4.getJointState();
		joint5.getJointState();

		// calculate the torque for each joint and publish them
		joint1.jointTrqControl();
		joint2.jointTrqControl();
		joint3.jointTrqControl();
		joint4.jointTrqControl();
		joint5.jointTrqControl();

		ros::spinOnce(); // update pos_cmd, kpkv
		ros::Duration(dt).sleep();
	}
}

