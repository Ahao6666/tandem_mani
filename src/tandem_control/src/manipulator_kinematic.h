#pragma once
//#include <Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <vector>
#include <Eigen/StdVector>
using namespace Eigen;


class Manipulator_Kinematic {
public:
	void get_jacobain(VectorXd q);
	void get_R_t(VectorXd q);
	Manipulator_Kinematic();
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > T;
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > T2;
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > TF;
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > TI;
	Matrix <double, 6, 5> jacob;
	Matrix4d base;
	Eigen::Matrix3d computeR(Matrix4d Tb, Eigen::Matrix3d R);
	Eigen::Vector3d computet(Matrix4d Tb, Eigen::Vector3d& t);
	std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > R;
	std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > t;
private:
	double L1 = 0.0928, L2 = 0.1888, L3 = 0.157, L4 = 0;
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > S;
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > S_;
	std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > B;
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > Tb;
	Matrix <double, 5, 6> slist;
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > M;
	std::vector<double> q0;
	Matrix4d N;
	Matrix <double, 6, 6> Adt;

};

Manipulator_Kinematic::Manipulator_Kinematic() {
	
	S.resize(5);
	S_.resize(5);
	Tb.resize(5);
	M.resize(6);
	T.resize(6);
	q0.resize(5);
	T2.resize(5);
	TF.resize(5);
	TI.resize(5);
	q0 = { 0,0,0,0,0 };
	base << 1,0,0,0,0,1,0,0,0,0,1,1.5,0,0,0,1;
	S.at(0) << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	S.at(1) << 0, 0, 0, 0, 0, 0, -1, -L1, 0, 1, 0, 0, 0, 0, 0, 0;
	S.at(2) << 0, 0, 1, L1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0;
	S.at(3) << 0, 0, 0, 0, 0, 0, -1, -L1, 0, 1, 0, -L2, 0, 0, 0, 0;
	S.at(4) << 0, 0, 0, 0, 0, 0, -1, -L1, 0, 1, 0, -(L2 + L3), 0, 0, 0, 0;
	M.at(0) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -L1, 0, 0, 0, 1;
	M.at(1) << 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, -L1, 0, 0, 0, 1;
	M.at(2) << 0, 1, 0, 0, 0, 0, -1, 0, -1, 0, 0, -L1, 0, 0, 0, 1;
	M.at(3) << 0, 0, 1, 0, 1, 0, 0, L2, 0, 1, 0, -L1, 0, 0, 0, 1;
	M.at(4) << 0, 0, 1, 0, 1, 0, 0, L2 + L3, 0, 1, 0, -L1, 0, 0, 0, 1;
	M.at(5) << 1, 0, 0, 0, 0, 1, 0, L2 + L3, 0, 0, 1, -(L1 + L4), 0, 0, 0, 1;
	T.at(0).setZero();
	T.at(1).setZero();
	T.at(2).setZero();
	T.at(3).setZero();
	T.at(4).setZero();
	T.at(5).setZero();
	T2.at(0).setZero();
	T2.at(1).setZero();
	T2.at(2).setZero();
	T2.at(3).setZero();
	T2.at(4).setZero();
	TF.at(0).setZero();
	TF.at(1).setZero();
	TF.at(2).setZero();
	TF.at(3).setZero();
	TF.at(4).setZero();
	TI.at(0).setZero();
	TI.at(1).setZero();
	TI.at(2).setZero();
	TI.at(3).setZero();
	TI.at(4).setZero();
	jacob << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	Adt << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	slist << 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, -L1, 0, 0, 1, 0, L1, 0, 0, 1, 0, 0, 0, -L1, -L2, 1, 0, 0, 0, -L1, -(L2 + L3);
	R.resize(5);
	t.resize(5);
	B.resize(5);


	for (int i = 0; i < 5; i++)
	{
		R.at(i).setZero();
		t.at(i).setZero();
		B.at(i).setZero();
	}
}

//֮���붯��ѧ��ϣ�д��һ��class��
void Manipulator_Kinematic::get_jacobain(VectorXd q) {
	q.resize(5);
	//DH d=1/0; a=1;标准DH参数
	T2.at(0) << -sin(q[0]), 0, cos(q[0]), 0, cos(q[0]), 0, sin(q[0]), 0, 0, 1, 0, 0, 0, 0, 0, 1;
	T2.at(1) << -sin(q[1]), 0, cos(q[1]), 0, cos(q[1]), 0, sin(q[1]), 0, 0, 1, 0, 0, 0, 0, 0, 1;
	T2.at(2) << -cos(q[2]), 0, -sin(q[2]), 0, -sin(q[2]), 0, cos(q[2]), 0, 0, 1, 0, L2, 0, 0, 0, 1;
	T2.at(3) << -sin(q[3]), -cos(q[3]), 0, -L3 * sin(q[3]), cos(q[3]), -sin(q[3]), 0, L3* cos(q[3]), 0, 0, 1, 0, 0, 0, 0, 1;
	T2.at(4) << sin(q[4]), cos(q[4]), 0, L4* sin(q[4]), -cos(q[4]), sin(q[4]), 0, -L4 * cos(q[4]), 0, 0, 1, 0, 0, 0, 0, 1;

	//T05 T15...T45
	TI.at(0) = T2.at(0) * T2.at(1) * T2.at(2) * T2.at(3) * T2.at(4);
	TI.at(1) = T2.at(1) * T2.at(2) * T2.at(3) * T2.at(4);
	TI.at(2) = T2.at(2) * T2.at(3) * T2.at(4);
	TI.at(3) = T2.at(3) * T2.at(4);
	TI.at(4) = T2.at(4);
	//T01 T02...T005
	TF.at(0) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
	TF.at(1) = T2.at(0);
	TF.at(2) = T2.at(0) * T2.at(1);
	TF.at(3) = T2.at(0) * T2.at(1) * T2.at(2);
	TF.at(4) = T2.at(0) * T2.at(1) * T2.at(2) * T2.at(3);
	for (int i = 0; i < 5; i++) {
		Eigen::Vector3d z;
		Eigen::Vector3d zp;
		Eigen::Vector3d pa;
		z = TF.at(i).block<3, 1>(0, 2);
		pa = TF.at(i).block<3, 3>(0, 0) * TI.at(i).block<3, 1>(0, 3);
		zp = z.cross(pa);
		jacob.col(i) << zp, z;
	}
}

void Manipulator_Kinematic::get_R_t(VectorXd q) {

q.resize(5);
	for (int i = 0; i < 5; i++)
	{
		N = S.at(i) * q[i];
		S_.at(i) = N.exp();
	
		/*R.at(i) = S_.at(i).block<3, 3>(0, 0);
		B.at(i) = S_.at(i).block<3, 3>(0, 1);
		//t.at(i) = S.at(i).block(1,3,0,1);
		t.at(i) = B.at(i).col(2);*/
		R.at(i) = computeR(S_.at(i), R.at(i));
		t.at(i) = computet(S_.at(i), t.at(i));
	}
	T.at(5) = base * S_.at(0) * S_.at(1) * S_.at(2) * S_.at(3) * S_.at(4) * M.at(5);
	T.at(4) = base * S_.at(0) * S_.at(1) * S_.at(2) * S_.at(3) * M.at(4);
	T.at(3) = base * S_.at(0) * S_.at(1) * S_.at(2) * M.at(3);
	T.at(2) = base * S_.at(0) * S_.at(1) * M.at(2);
	T.at(1) = base * S_.at(0) * M.at(1);
	T.at(0) = base * M.at(0);
	for (int i = 0; i < 5; i++)
	{
		R.at(i) = computeR(T.at(i), R.at(i));
		t.at(i) = computet(T.at(i), t.at(i));
	}

}

Eigen::Matrix3d Manipulator_Kinematic::computeR(Matrix4d Tb, Eigen::Matrix3d R) {

	R = Tb.block<3, 3>(0, 0);
	return R;
}
Eigen::Vector3d Manipulator_Kinematic::computet(Matrix4d Tb, Eigen::Vector3d& t) {
	Matrix3d B;
	B = Tb.block<3, 3>(0, 1);
	t = B.col(2);
	return t;
}
