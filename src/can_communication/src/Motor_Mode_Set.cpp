#include "ros/ros.h"
#include "can_communication/MotorModeSet.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Motor_Mode_Set_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<can_communication::MotorModeSet>("Motor_mode_set");
    can_communication::MotorModeSet srv;

    std::cout << "Enter Motor Mode value\n(0 for RTORQUE_MODE,\n 1 for RSPEED_MODE,\n 2 for RPOSITION_MODE,\n 3 for RPOSITION_MODE_T): ";
    std::cin >> srv.request.mode;
    
    if (client.call(srv))
    {
        if(srv.response.success)
            ROS_INFO("Motor mode set successed!");
        else
            ROS_INFO("Motor mode set failed!");
    }
    else
    {
        ROS_ERROR("Failed to call service Motor_mode_set");
        return 1;
    }

    return 0;
 }
