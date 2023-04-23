#pragma once
#include <iostream>
#include <vector>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

// program
#include <Eigen/StdVector>
using namespace Eigen;
class manipulator {
public:
	std::vector<double> PID_control(std::vector<double> q, std::vector<double> dq, std::vector<double> ddq, std::vector<double> q_des, std::vector<double> dq_des, std::vector<double> ddq_des, std::vector<double> tau_dynamic);
	manipulator();
private:

	std::vector<double> eq;
	std::vector<double> edq;
	std::vector<double> eddq;
	std::vector<double> torque;
	std::vector<double> tau_PID;
	std::vector<double> kp;
	std::vector<double> kd;
	std::vector<double> ki;


};
manipulator::manipulator() {
	eq.resize(5);
	edq.resize(5);
	eddq.resize(5);
	torque.resize(5);
	tau_PID.resize(5);
	kp.resize(5);
	kd.resize(5);
	ki.resize(5);
	eddq = { 0,0,0,0,0 };
	//t=0.01
	//kp = { 140, 140, 140, 140, 140 };
	//kd = { 15, 15, 15, 15, 15 };
	//ki = { 0.15, 0.015, 0.015, 0.015, 0.015 };
	
	//t=0.05
		//kp = { 1.4, 1.4, 1.4, 1.4, 1.4 };
		//kd = { 0.15, 0.15, 0.15, 0.15, 0.15 };
	kp = { 14, 14, 14, 14, 14 };
	kd = { 0.01, 0.01, 0.01, 0.01, 0.01 };
	//ki = { 0.15, 0.015, 0.015, 0.015, 0.015 };
}
std::vector<double> manipulator::PID_control(std::vector<double> q, std::vector<double> dq, std::vector<double> ddq, std::vector<double> q_des, std::vector<double> dq_des, std::vector<double> ddq_des, std::vector<double> tau_dynamic) {

	for (int i = 0; i < 5; i++) {
		eq[i] = q[i] - q_des[i];//(5x1)
		edq[i] = dq[i] - dq_des[i];
		eddq[i] = ddq[i] - ddq_des[i];
		tau_PID[i] = -kp[i] * eq[i] - kd[i] * edq[i];
		//torque[i] = tau_PID[i];
		torque[i] = tau_PID[i] + tau_dynamic[i];

	}
	return torque;
}
