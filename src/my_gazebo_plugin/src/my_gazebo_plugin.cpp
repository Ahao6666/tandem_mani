#include <stdio.h>
#include <math.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>
#include <string>
#include <ignition/math/Vector3.hh>

double tau_motor(double i_motor, double omega_motor, double kt, double ke, double b, double j, double R) {
    double tau = kt*i_motor - b*omega_motor - (ke*ke/R)*omega_motor + (ke/R)*sqrt((kt*kt*i_motor*i_motor + (b*j + ke*omega_motor)*(b*j + ke*omega_motor))/(R*R) - 4*kt*b*i_motor*omega_motor/(R*R));
    return tau;
}


namespace gazebo
{
  class Moter_Model : public ModelPlugin
  {
    public: 
      Moter_Model() {
        gzerr <<  "11111111111111111111111122222222222222222222222333333333333333333" <<std::endl;
      }

      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        // 获取关节名称
        if (_sdf->HasElement("joint_name"))
          jointName_ = _sdf->Get<std::string>("joint_name");
        // 获取关节
        joint_ = _model->GetJoint(jointName_);
        if (!joint_)
        {
          gzerr << "Unable to find joint with name [" << jointName_ << "]" << std::endl;
          return;
        }
        else{
          gzerr <<  "11111111111111111111111122222222222222222222222333333333333333333" <<std::endl;
          gzerr <<  "Connect to "<< jointName_ <<std::endl;
        }

        // 订阅关节力矩的命令
        node_ = transport::NodePtr(new transport::Node());
        node_->Init();
        cmdSub_ = node_->Subscribe("~/"+_model->GetName()+"/"+jointName_+"/cmd", &Moter_Model::OnCmd, this);
      }

      // // 订阅关节力矩的命令
      void OnCmd(ConstVector3dPtr &_msg)
      {
      //   // 获取关节力矩命令
      //   math::Vector3 cmd = msgs::Convert(*_msg);

      //   // 应用关节力矩
      //   joint_->SetForce(0, cmd.x);
      }

    private:
      std::string jointName_; // 关节名称
      physics::JointPtr joint_; // 关节指针
      transport::NodePtr node_; // 传输节点
      transport::SubscriberPtr cmdSub_; // 订阅者
  };

  GZ_REGISTER_MODEL_PLUGIN(Moter_Model)
}

