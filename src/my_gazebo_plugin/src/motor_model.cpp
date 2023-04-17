/******************************************************************************
 * Copyright (c) 2022-2024 The IUSL_UAV Shen Jiahao. All rights reserved.
 * See the AUTHORS file for names of contributors.
 *****************************************************************************/

/******************************************************************************
 * @file motor_model.cpp
 * @author Shen Jiahao <shenjiahao@westlake.edu.cn>
 *****************************************************************************/
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <string>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <sensor_msgs/JointState.h>

#define kt 0.1 // 电机转矩常数，单位：N·m/A
#define ke 0.01 // 电机电动势常数，单位：V·s/rad
#define b 0.001 // 电机摩擦系数，单位：N·s/rad
#define j 0.01 // 电机转动惯量，单位：N·m·s^2
#define R 10.0 // 电机电阻，单位：Ω
// define class to instantiate joints
class Joint {
    public:
        Joint(ros::NodeHandle nh, std::string joint_name, double dt); // constructor
        ~Joint() {}; // destructor
        void jointTrqControl();
        double tau_motor(double, double);
        void omega_motor_callback(const std_msgs::Float64::ConstPtr& msg);
        void i_motor_callback(const std_msgs::Float64::ConstPtr& msg);
    private:
        // service clients
        ros::ServiceClient set_trq_client;
        // publisher objects
        ros::Publisher trq_pub;
        // gazebo/sensor messages
        gazebo_msgs::ApplyJointEffort effort_cmd_srv_msg;
        // torque messages to be published
        std_msgs::Float64 trq_msg; // torque
        // control parameters
        double trq_cmd; // torque to be published
        // other parameters
        std::string joint_name;
        // motor speed and current
        double omega_motor; // 电机转速，单位：rad/s
        double i_motor;     // 电流，单位：A
};
// calculate the motor trque mainly according to current
double Joint::tau_motor(double i_motor, double omega_motor) {
    double tau = kt*i_motor - b*omega_motor - 
        (ke*ke/R)*omega_motor + (ke/R)*sqrt((kt*kt*i_motor*i_motor + 
        (b*j + ke*omega_motor)*(b*j + ke*omega_motor))/(R*R) - 4*kt*b*i_motor*omega_motor/(R*R));
    return tau;
}
// callback function for omega_motor topic
void Joint::omega_motor_callback(const std_msgs::Float64::ConstPtr& msg) {
    omega_motor = msg->data;
}

// callback function for i_motor topic
void Joint::i_motor_callback(const std_msgs::Float64::ConstPtr& msg) {
    i_motor = msg->data;
}

Joint::Joint(ros::NodeHandle nh, std::string joint_name, double dt) {
    // initialize parameters
    this -> joint_name = joint_name;
    ros::Duration duration(dt);

    // initialize gazebo clients
    set_trq_client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
    // subscribe to omega_motor topic
    ros::Subscriber omega_motor_sub = nh.subscribe("/omega_motor", 1, &Joint::omega_motor_callback, this);
    // subscribe to i_motor topic
    ros::Subscriber i_motor_sub = nh.subscribe("/i_motor", 1, &Joint::i_motor_callback, this);
    // initialize publisher objects
    trq_pub = nh.advertise<std_msgs::Float64>(joint_name + "_trq", 1);

    // set up effort_cmd_srv_msg
    effort_cmd_srv_msg.request.joint_name = joint_name;
    effort_cmd_srv_msg.request.effort = 0.0f;
    effort_cmd_srv_msg.request.duration = duration;
}

// calculate joint torque, publish them, send to gazebo
void Joint::jointTrqControl() {
    // control algorithm in one line
    trq_cmd  = tau_motor(omega_motor, i_motor);
    // publish the torque message
    // ROS_INFO("trq_cmd:%f", trq_cmd);
    trq_msg.data = trq_cmd;
    trq_pub.publish(trq_msg);
    // send torque command to gazebo
    effort_cmd_srv_msg.request.effort = trq_cmd;
    effort_cmd_srv_msg.request.start_time.sec = 0;
    effort_cmd_srv_msg.request.duration.sec = 10000;        
    set_trq_client.call(effort_cmd_srv_msg);
    // make sure service call was successful
    bool result = effort_cmd_srv_msg.response.success;
    if (!result) {
        ROS_WARN("service call to apply_joint_effort failed!");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_model_node");
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

    double dt = 0.01; // sample time for the controller

    Joint joint1(nh, "joint1", dt);
    Joint joint2(nh, "joint2", dt);
    Joint joint3(nh, "joint3", dt);
    Joint joint4(nh, "joint4", dt);
    Joint joint5(nh, "joint5", dt);

    ros::Rate rate_timer(1 / dt);
    while(ros::ok()) {// 
        // calculate the torque for each joint and publish them
        joint1.jointTrqControl();
        joint2.jointTrqControl();
        joint3.jointTrqControl();
        joint4.jointTrqControl();
        joint5.jointTrqControl();

        ros::spinOnce(); // update pos_cmd, kpkv
        rate_timer.sleep(); // sleep the sample time
    }
}
