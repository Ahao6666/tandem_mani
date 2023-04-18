#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <string>
#include <std_msgs/Float64.h>
#include <ros/ros.h>

#define kt 0.1 // 电机转矩常数，单位：N·m/A
#define ke 0.01 // 电机电动势常数，单位：V·s/rad
#define b 0.001 // 电机摩擦系数，单位：N·s/rad
#define j 0.01 // 电机转动惯量，单位：N·m·s^2
#define R 1.4 // 电机电阻，单位：Ω

namespace gazebo
{
  class motor_plugin : public ModelPlugin
  {
    public: 
      motor_plugin() {}
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        // 获取关节名称
        if (_sdf->HasElement("joint_name"))
        {
          jointName_ = _sdf->Get<std::string>("joint_name");
        }
        // 获取关节
        joint_ = _model->GetJoint(jointName_);
        if (!joint_)
        {
          std::cout << "Unable to find joint with name [" << jointName_ << "]" << std::endl;
          return;
        }
        else
        { 
          std::cout << "Connect to " << jointName_ << std::endl;
        }

        // 订阅关节力矩的命令
        ros::NodeHandle nh;
        joint_trq_sub = nh.subscribe(jointName_+"_trq", 1, &motor_plugin::CmdCB, this);
      }

      // // 订阅关节力矩的命令
      void CmdCB(const std_msgs::Float64ConstPtr &_msg)
      {
        i_motor = _msg->data;
        omega_motor = 100;
        double tau = kt * i_motor - b * omega_motor - 
            (ke * ke / R) * omega_motor + (ke / R) * sqrt((kt * kt * i_motor * i_motor + 
            (b * j + ke * omega_motor) * (b * j + ke * omega_motor)) / (R * R) - 4 * kt * b * i_motor * omega_motor / (R * R));
        // ROS_WARN("joint_trq: %f", _msg->data);
        // 应用关节力矩
        joint_->SetForce(0, _msg->data);
      }
    private:
      std::string jointName_; // 关节名称
      physics::JointPtr joint_; // 关节指针
      ros::Subscriber joint_trq_sub;
      double omega_motor; // 电机转速，单位：rad/s
      double i_motor;     // 电流，单位：A
      double tau;   //力矩，单位：Nm

  };

  GZ_REGISTER_MODEL_PLUGIN(motor_plugin)
}

