#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <vector>
#include "can_communication/MotorPosTar.h"

// 全局变量用于存储键盘输入的数据
int16_t num_input;
std::vector<double> pos_tar_input;
std::vector<double> time_dur_input;
// 键盘输入线程函数
void keyboardInputThread()
{
    while (ros::ok()) {
        double pos_tar_temp;
        double time_dur_temp;
        int16_t num_input_temp;
        std::vector<double> pos_tar_input_temp;
        std::vector<double> time_dur_input_temp;
        // 读取键盘输入的数据
        std::cout << "Enter the motor number (<5): ";
        std::cin >> num_input_temp;
        for(int i=0; i<num_input_temp; i++){
            std::cout << "Enter pos_tar of the " << i+1 << " motor(deg): ";
            std::cin >> pos_tar_temp;
            std::cout << "Enter time_during of the " << i+1 << " motor(ms): ";
            std::cin >> time_dur_temp;
            pos_tar_input_temp.push_back(pos_tar_temp);
            time_dur_input_temp.push_back(time_dur_temp);
        }
        num_input = num_input_temp;
        pos_tar_input = pos_tar_input_temp;
        time_dur_input = time_dur_input_temp;
        pos_tar_input_temp.clear();
        time_dur_input_temp.clear();
    }
}

// ROS发布消息线程函数
void publishMessageThread(ros::Publisher& MotorPosTar_Pub)
{
    ros::Rate rate(10);  // 定义发布频率为10Hz

    while (ros::ok()) {

        // 创建消息对象并填充数据
        can_communication::MotorPosTar MotorPosTar_data;

        MotorPosTar_data.num = num_input;
        MotorPosTar_data.pos_tar = pos_tar_input;
        MotorPosTar_data.time_dur = time_dur_input;
        // 发布消息
        MotorPosTar_Pub.publish(MotorPosTar_data);

        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Motor_Pos_Tar_node");
    ros::NodeHandle nh;

    ros::Publisher MotorPosTar_Pub = nh.advertise<can_communication::MotorPosTar>("MotorPosTar", 10);

    // 创建键盘输入线程和ROS发布消息线程
    std::thread input_thread(keyboardInputThread);
    std::thread publish_thread(publishMessageThread, std::ref(MotorPosTar_Pub));

    ros::spin();  // 进入ROS事件循环

    // 等待线程结束
    input_thread.join();
    publish_thread.join();

    return 0;
}
