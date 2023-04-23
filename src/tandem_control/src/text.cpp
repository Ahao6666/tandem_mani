
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include "manipulator_kinematic.h"


//compute Euler angles by R
Vector3d computeEulerAngles(Matrix3d R) {
	Vector3d EulerAngles;

	float theta = 0, psi = 0, pfi = 0;
	if (abs(R(2, 0)) < 1 - FLT_MIN || abs(R(2, 0)) > 1 + FLT_MIN) { // abs(R(2, 0)) != 1
		float theta1 = -asin(R(2, 0));
		float theta2 = M_PI- theta1;
		float psi1 = atan2(R(2, 1) / cos(theta1), R(2, 2) / cos(theta1));
		float psi2 = atan2(R(2, 0) / cos(theta2), R(2, 2) / cos(theta2));
		float pfi1 = atan2(R(1, 0) / cos(theta1), R(0, 0) / cos(theta1));
		float pfi2 = atan2(R(1, 0) / cos(theta2), R(0, 0) / cos(theta2));
		theta = theta1;
		psi = psi1;
		pfi = pfi1;
	}
	else {
		float phi = 0;
		float delta = atan2(R(0, 1), R(0, 2));
		if (R(2, 0) > -1 - FLT_MIN && R(2, 0) < -1 + FLT_MIN) { // R(2,0) == -1
			theta = M_PI/ 2;
			psi = phi + delta;
		}
		else {
			theta = -M_PI/ 2;
			psi = -phi + delta;
		}
	}
		EulerAngles[0] = psi;
		EulerAngles[1] = theta;
		EulerAngles[2] = pfi;
	return EulerAngles;
}

void getEEPose(Manipulator_Kinematic K_model,VectorXd current_jnts,VectorXd &EEPose){
	//compute EE's pose
	current_jnts.resize(5);
	K_model.get_R_t(current_jnts);
	Matrix3d EERotationMatrix;
	EERotationMatrix= K_model.T.at(5).block<3, 3>(0, 0);

	Vector3d pos = K_model.T.at(5).block<3, 1>(0, 3);
	Vector3d EulerAngles=computeEulerAngles(EERotationMatrix);

	EEPose.resize(5);
	EEPose.block<3,1>(0,0)=pos;
	EEPose.block<2,1>(3,0)=EulerAngles.block<2,1>(0,0);
}

int main() {

double p=2;

		Eigen::Matrix3d EERotationMatrix;
        Eigen::Matrix4d rot;// 创建4行4列的double型矩阵（方阵）
        Eigen::Matrix3d rot2;
        Vector3d pos;

        double kp=2;
        std::string jointNames[]={"joint1","joint2","joint3","joint4","joint5"};

        rot2 << 1.0, 0.0, 0.0,
		0.0, cos(p), -sin(p),
		0.0, sin(p), cos(p);

        //Vector3d  EulerAngles=computeEulerAngles(rot2); 


         //std::cout << "Dot product: " << EulerAngles<< std::endl;

		 EERotationMatrix= rot2.block<3, 3>(0, 0);//这个样子直接赋值不可以，会报错
         std::cout << "Dot product: " << EERotationMatrix<< std::endl;
        //  EERotationMatrix=rot;//可以
         EERotationMatrix.block<3, 3>(0, 0)= rot2.block<3, 3>(0, 0);//可以
         pos=rot2.block<3, 1>(0, 0);

         std::cout << "Dot product: " << pos<< std::endl;

         VectorXd current_jnts(5);

         VectorXd EEPose(5);
          Manipulator_Kinematic K_model;
	     getEEPose(K_model,current_jnts,EEPose);
		 K_model.get_jacobain(current_jnts);
		 MatrixXd jacobian(5,5);
	    jacobian=K_model.jacob.block<5, 5>(0, 0);
         std::cout << "Dot product: " << jacobian<< std::endl;
	
        return 0;
}