#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <string>
#include <vector>
#include <std_msgs/Float64.h>
 
// 定义一个函数，用于录制多个话题的数据到ROSbag文件
// 参数：
// - topics: 一个包含多个话题名的vector
// - bag_filename: ROSbag文件名
// - duration: 录制数据的时间长度（单位：秒），如果duration<=0，则持续录制直到手动停止

// callback function
void pos_callback(const std_msgs::Float64& pos_cmd_msg) {
    std::string bag_filename = "/home/ji/bag_joint_pos.bag";
    double pos_cmd;
    pos_cmd = pos_cmd_msg.data;

    static rosbag::Bag bag;
    if (!bag.isOpen()) {
        bag.open(bag_filename, rosbag::bagmode::Write);  
    }

    // 将消息写入bag文件
    bag.write("my_topic", ros::Time::now(), pos_cmd_msg);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "record_bag_node");
    ros::NodeHandle nh;
    
    // subscribe topic
    ros::Subscriber pos_cmd_subscriber;
    pos_cmd_subscriber = nh.subscribe("joint1_pos", 1, pos_callback);

    double duration = 50; // 持续10秒录制
    double dt = 0.02; // sample time for the controller

    // 持续录制一段时间后停止
    ROS_INFO_STREAM("Start recording for " << duration << " seconds...");

    ros::Time start_time = ros::Time::now();
    ros::Rate rate_timer(1 / dt);
    while ((ros::Time::now() - start_time).toSec() < duration) {
        ros::spinOnce();
        rate_timer.sleep(); // sleep the sample time
    }
    // ros::spin();
    return 0;
}