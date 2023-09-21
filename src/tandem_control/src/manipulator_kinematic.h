#pragma once
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <vector>
#include <Eigen/StdVector>
using namespace Eigen;

class Manipulator_Kinematic {
public:
	Manipulator_Kinematic();
	void get_jacobian();
	void forward_kinematic(VectorXd q);
	Matrix <double, 6, 5> jacobian;
	Matrix4d base;
        Matrix4d MDH_transform(int i, double theta);
        std::vector<double> MDH_d;
        std::vector<double> MDH_a;
        std::vector<double> MDH_alpha;
        std::vector<double> MDH_offset;
		std::vector<Eigen::Matrix4d> T_from_base;  
		const int DOFs = 5;
		Vector3d z; //axis of link

private:
	std::vector<Eigen::Matrix4d> T_from_the_last;  
	Matrix4d T_from_joint5; 
	Matrix4d T_base; 
};

Manipulator_Kinematic::Manipulator_Kinematic() {
	//MDH para 

    MDH_d.resize(DOFs);
    MDH_a.resize(DOFs);
    MDH_alpha.resize(DOFs);
    MDH_offset.resize(DOFs);
		
    MDH_d = { -0.093,0,0.189,0,0 };
    MDH_a = { 0.000158,0,0,0,0.157};
    MDH_alpha = { 0,M_PI/2,M_PI/2,-M_PI/2,0 };
    MDH_offset = { M_PI/2, M_PI/2, 0, -M_PI/2,0};
	T_from_joint5 <<0,1,0,0,
	0,0,1,-0.12,
	1,0,0,-0.0005,
	0,0,0,1;
	z << 0,0,1;
	T_base<<1,0,0,0,
	0,1,0,0,
	0,0,1,1.5,
	0,0,0,1;
	T_from_base.resize(DOFs+1);
	T_from_the_last.resize(DOFs+1);
}

void Manipulator_Kinematic::forward_kinematic(VectorXd q) {
	q.resize(5);
	for (int i = 0; i < DOFs; i++)
	{
          T_from_the_last.at(i)=Manipulator_Kinematic::MDH_transform(i, q[i]);
	}

	T_from_the_last.at(DOFs)=T_from_joint5; // EE from joint5
	T_from_base.at(0)=T_base*T_from_the_last.at(0);
	for (int i = 1; i <= DOFs; i++)
	{
		T_from_base.at(i)=T_from_base.at(i-1)*T_from_the_last.at(i);
	}
}

void Manipulator_Kinematic::get_jacobian() {
	Vector3d zp;
	Vector3d z_in_base;
	Vector3d p_EE_from_link;
	Vector3d p_EE=T_from_base.at(DOFs).block<3,1>(0,3);
	Vector3d p_link_i;
	Matrix3d R_link_to_base;
	for (int i = 0; i < 5; i++) {
		p_link_i=T_from_base.at(i).block<3,1>(0,3);
		p_EE_from_link=p_EE-p_link_i; //in the world/base frame
		R_link_to_base=T_from_base.at(i).block<3,3>(0,0);// rotation from link i to the base
		z_in_base=R_link_to_base*z;  //transfer to the world/base frame
		zp=z_in_base.cross(p_EE_from_link);
		jacobian.col(i) << zp, z_in_base;
	}
}

Matrix4d Manipulator_Kinematic::MDH_transform(int i, double theta) {
    
    Matrix4d T;
    Matrix4d RX;
    Matrix4d DX;
    Matrix4d RZ;
    Matrix4d DZ;
  
    RX<<1,0,0,0,
        0,cos(MDH_alpha[i]),-sin(MDH_alpha[i]),0,
        0,sin(MDH_alpha[i]),cos(MDH_alpha[i]),0,
        0,0,0,1;
    DX<<1,0,0,MDH_a[i],
        0,1,0,0,
        0,0,1,0,
        0,0,0,1;
    RZ<<cos(theta + MDH_offset[i]),-sin(theta + MDH_offset[i]),0,0,
        sin(theta + MDH_offset[i]),cos(theta + MDH_offset[i]),0,0,
        0,0,1,0,
        0,0,0,1;
    DZ<<1,0,0,0,
        0,1,0,0,
        0,0,1,MDH_d[i],
        0,0,0,1;
    T=RX*DX*RZ*DZ;
    return T;
}