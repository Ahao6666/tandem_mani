#include "ros/ros.h"
#include "can_communication/MotorPIDSet.h"
#include <cstdlib>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Motor_PID_Set_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<can_communication::MotorPIDSet>("Motor_PID_set");
    can_communication::MotorPIDSet srv;

    std::string input;
    std::cout << "Enter KP value: ";
    std::cin >> input;
    srv.request.kp = std::stoi(input);

    std::cout << "Enter KI value: ";
    std::cin >> input;
    srv.request.ki = std::stoi(input);

    std::cout << "Enter KD value: ";
    std::cin >> input;
    srv.request.kd = std::stoi(input);

    if (client.call(srv))
    {
        if(srv.response.pid_setting_is_done)
            ROS_INFO("Motor PID set successed!");
        else
            ROS_INFO("Motor PID set failed!");
    }
    else
    {
        ROS_ERROR("Failed to call service Motor_PID_set");
        return 1;
    }

    return 0;
 }
