#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <socketcan_interface/socketcan.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int sockfd;

void receiveCallback(const can_msgs::Frame::ConstPtr& msg)
{
  // 处理接收到的CAN帧数据
  // 在这里你可以根据CAN帧的ID、数据等信息进行相应的处理
  // 例如打印CAN帧的ID和数据
  ROS_INFO("Received CAN frame - ID: 0x%03X, Data: %02X %02X %02X %02X %02X %02X %02X %02X",
           msg->id, msg->data[0], msg->data[1], msg->data[2], msg->data[3],
           msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
}

int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "can_publisher_subscriber");
  ros::NodeHandle nh;

  const char* ifname = "can0"; // CAN接口名，根据实际情况修改
  // 创建套接字
  sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sockfd == -1) {
      std::cerr << "Failed to create socket." << std::endl;
      return 1;
  }
  // 绑定到CAN接口
  struct ifreq ifr;
  std::strcpy(ifr.ifr_name, ifname);
  if (ioctl(sockfd, SIOCGIFINDEX, &ifr) == -1) {
      std::cerr << "Failed to get interface index." << std::endl;
      close(sockfd);
      return 1;
  }
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
      std::cerr << "Failed to bind socket to interface." << std::endl;
      close(sockfd);
      return 1;
  }

  // 创建CAN消息发布者
  ros::Publisher can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 1000);

  // 创建CAN消息订阅者
  ros::Subscriber can_sub = nh.subscribe("received_messages", 1000, receiveCallback);

  // 创建一个CAN帧
  can_msgs::Frame frame;
  frame.header.frame_id = "can_frame";
  frame.header.stamp = ros::Time::now();
  frame.is_extended = false;
  frame.is_error = false;
  frame.dlc = 8;  // 设置数据长度
  frame.id = 0x17;  // 设置CAN帧的ID
  // 设置CAN帧的数据
  frame.data[0] = 0x01;
  frame.data[1] = 0x01;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;

  // 发布和订阅CAN消息
  while (ros::ok())
  {
    // 设置时间戳
    frame.header.stamp = ros::Time::now();

    // 发布CAN消息
    can_pub.publish(frame);

    // 延时一段时间后再发布下一条CAN消息
    ros::Duration(1.0).sleep();

    // 处理订阅的消息
    ros::spinOnce();
  }

  return 0;
}
