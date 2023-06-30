#include "ros/ros.h"
#include "can_communication/MotorModeSet.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Motor_Mode_Set_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<can_communication::MotorModeSet>("Motor_mode_set");
    can_communication::MotorModeSet srv;
    srv.request.mode = atoll(argv[1]);
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
