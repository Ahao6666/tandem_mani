
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <math.h>
#include <float.h>
#include "manipulator_kinematic.h"
#include "manipulator_IDynamic.h"

Manipulator_Kinematic K_model;
Manipulator_IDynamic ID_model;

// using namespace Eigen;

// VectorXd add(VectorXd x){
// static VectorXd p=VectorXd::Zero(5);
// p=p+x;
// return p;
// }

int main() {
//    Quaterniond quaternion;
//    Vector3d eulerAngles;
//    eulerAngles<< 0,M_PI/2,M_PI/2;
// //    quaternion= Eigen::AngleAxisd(eulerAngles[0], Eigen::Vector3d::UnitZ()) * 
// //                   Eigen::AngleAxisd(eulerAngles[1], Eigen::Vector3d::UnitY()) * 
// //                   Eigen::AngleAxisd(eulerAngles[2], Eigen::Vector3d::UnitX());
//    quaternion={1,0,0,0};
//    Vector3d euler1 = quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
//    std::cout << "EulerAngles: " << euler1<< std::endl;

//    double yaw=M_PI/2;double pitch=0;double roll=-M_PI/2;
// 	Matrix3d rotationMatrix;
// 	rotationMatrix= AngleAxisd(yaw, Vector3d::UnitZ())*
// 	AngleAxisd(pitch, Vector3d::UnitY())*
// 	AngleAxisd(roll, Vector3d::UnitX());

// 	Vector3d EulerAngles2=rotationMatrix.eulerAngles(2,1,0);
// 	std::cout << "EulerAngles: " << EulerAngles2<< std::endl

// desired_accel.insert(desired_accel.begin(),0);
// std::vector<Eigen::Vector3d> omega;    //angular velocity
// std::vector<double> desired_accel_.insert()
// std::vector<double>  q;
std::vector<double> dq;
std::vector<double>  ddq;
// q.resize(5);q={10*M_PI/180,20*M_PI/180,30*M_PI/180,40*M_PI/180,50*M_PI/180};
// dq.resize(5);dq={1,2,3,4,5};
// ddq.resize(5);ddq={0.5,1,1.5,0,0};


// std::vector<double>  torque = ID_model.rnea(q, dq, ddq);
VectorXd q;
q.resize(5);
q<<1,1,1,1,1;
// q<<0,0,0,0,0;

K_model.forward_kinematic(q);
Vector3d desiredEEPos0 = K_model.T_from_base.at(1).block<3, 1>(0, 3);
Vector3d desiredEEPos1 = K_model.T_from_base.at(2).block<3, 1>(0, 3);

Vector3d desiredEEPos2 = K_model.T_from_base.at(4).block<3, 1>(0, 3);
Vector3d desiredEEPos3 = K_model.T_from_base.at(5).block<3, 1>(0, 3);

// Matrix3d desiredEERotation= K_model.T_from_base.at(5).block<3, 3>(0, 0);

// K_model.get_jacobian();
// MatrixXd jacobian(6,5);
// jacobian=K_model.jacobian;
// dq.erase(dq.begin());
// std::cout<< desiredEEPos0<<std::endl;

// std::cout<< desiredEEPos1<<std::endl;
double a=1.0/2;
std::cout<< desiredEEPos3<<std::endl;
std::cout<< a<<std::endl;



}