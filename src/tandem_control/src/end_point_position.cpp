/******************************************************************************
 * Copyright (c) 2022-2024 The IUSL_UAV Shen Jiahao. All rights reserved.
 * See the AUTHORS file for names of contributors.
 *****************************************************************************/

/******************************************************************************
 * @file end_point_position.cpp
 * @author Shen Jiahao <shenjiahao@westlake.edu.cn>
 *****************************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/GetLinkState.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "end_point_position_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("end_point_pose", 1);
    ros::ServiceClient get_link_state_client = nh.serviceClient<gazebo_msgs::GetLinkState>(
        "/gazebo/get_link_state");
    gazebo_msgs::GetLinkState get_link_state_srv_msg;
    geometry_msgs::Pose EEPose;
    
    double dt;
    ros::Rate rate_timer(1/dt);

    while(ros::ok()) {
        // Get the link state of the 'tandem_mani::end_point' link from Gazebo
        get_link_state_srv_msg.request.link_name = "tandem_mani::end_point";
        get_link_state_srv_msg.request.reference_frame = "world";
        get_link_state_client.call(get_link_state_srv_msg);
        geometry_msgs::Pose pose = get_link_state_srv_msg.response.link_state.pose;

        EEPose.position.x = pose.position.x;
        EEPose.position.y = pose.position.y;
        EEPose.position.z = pose.position.z;

        // ROS_INFO("Position: (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z);
        EEPose.orientation.x= pose.orientation.w;
	    EEPose.orientation.y= pose.orientation.x;
	    EEPose.orientation.z= pose.orientation.y;
	    EEPose.orientation.w= pose.orientation.z;

        pub.publish(EEPose);

        ros::spinOnce();
        rate_timer.sleep(); // sleep the sample time
    }

    return 0;
}
