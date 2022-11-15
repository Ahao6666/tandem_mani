#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>


geometry_msgs::PoseStamped poseMav;



void myCallback(const geometry_msgs::TransformStamped& message_holder){
//   pose = message_holder;
   poseMav.pose.position.x = message_holder.transform.translation.x;
   poseMav.pose.position.y = message_holder.transform.translation.y;
   poseMav.pose.position.z = message_holder.transform.translation.z;
   poseMav.pose.orientation.x = message_holder.transform.rotation.x;
   poseMav.pose.orientation.y = message_holder.transform.rotation.y;
   poseMav.pose.orientation.z = message_holder.transform.rotation.z;
   poseMav.pose.orientation.w = message_holder.transform.rotation.w;
   poseMav.header.stamp = ros::Time::now();;
}

int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "vicon_px4");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("vicon/Turing/Turing", 10, myCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);

    ros::Rate naptime(40);
    while (ros::ok())
    {
        /* code for loop body */
        // ROS_INFO("x= %f, y= %f, z= %f",pose.transform.translation.x,pose.transform.translation.y,pose.transform.translation.z);
        ros::spinOnce();
        pub.publish(poseMav);
        ROS_INFO("x= %f, y= %f, z= %f",poseMav.pose.position.x,poseMav.pose.position.y,poseMav.pose.position.z);
        naptime.sleep();
    }
    return 0;
}
