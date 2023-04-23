#include "manipulator_control.h"
#include "manipulator_kinematic.h"
#include "manipulator_IDynamic.h"
#define pi 3.14159


Matrix3d get_Rot(double pitch, double yaw) {
	Matrix3d R;
	R << cos(yaw) * cos(pitch), -sin(yaw), 0, cos(pitch)* sin(yaw), cos(yaw), 0, -sin(pitch), 0, 1 ;
	return R;
}

std::vector<float> computeEularAngles(Eigen::Matrix3f& R, bool israd) {
	std::vector<float> result(3, 0);
	//const float pi = 3.14159265397932384626433;

	float theta = 0, psi = 0, pfi = 0;
	if (abs(R(2, 0)) < 1 - FLT_MIN || abs(R(2, 0)) > 1 + FLT_MIN) { // abs(R(2, 0)) != 1
		float theta1 = -asin(R(2, 0));
		float theta2 = pi - theta1;
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
			theta = pi / 2;
			psi = phi + delta;
		}
		else {
			theta = -pi / 2;
			psi = -phi + delta;
		}
	}

	// psi is along x-axis, theta is along y-axis, pfi is along z axis
	if (israd) { // for rad 
		result[0] = psi;
		result[1] = theta;
		result[2] = pfi;
	}
	else {
		result[0] = psi * 180 / pi;
		result[1] = theta * 180 / pi;
		result[2] = pfi * 180 / pi;
	}
	return result;
}

int main() {
	Manipulator_Kinematic K_model;
	Manipulator_IDynamic ID_model;
	manipulator P_model;
	Eigen::Vector3d eulerAngle;
	Vector3d pos;
	std::vector<double> tau_dynamic;
	std::vector<double> torque;
	tau_dynamic.resize(5);
	torque.resize(5);

	std::vector<double> t0;
	std::vector<double> t1;
	std::vector<double> t2;
	std::vector<double> tt;
	std::vector<double> tt1;
	std::vector<double> t3;
	std::vector<double> t4;
	std::vector<double> t5;
	std::vector<double> tg;
	Matrix <double, 5, 5> M;
	t0.resize(5);
	t1.resize(5);
	t2.resize(5);
	t3.resize(5);
	t4.resize(5);
	t5.resize(5);
	tg.resize(5);
	tt.resize(5);
	tt1.resize(5);
	tt = { 0,0,0,0,0 };
	tt1 = { 1,1,1,1,1 };

	Matrix <double, 5, 5> jacob2;

	int T = 10;
	double dd = 0.02;
	int L = T / dd;
	Matrix <double, 5, Dynamic> q;
	Matrix <double, 5, Dynamic> dq;
	Matrix <double, 5, Dynamic> ddq;
	Matrix <double, 5, Dynamic> q_des;
	Matrix <double, 5, Dynamic> dq_des;
	Matrix <double, 5, Dynamic> ddq_des;
	Matrix <double, 5, Dynamic> d_state_des;
	Matrix <double, 6, Dynamic> state_des;
	Matrix <double, 6, Dynamic> state_actual;
	Matrix <double, 5, Dynamic> d_state_des2;
	Matrix <double, 5, Dynamic> ddaq_des;
	Matrix <double, 5, Dynamic> daq_des;
	Matrix <double, 5, Dynamic> aq_des;
	Matrix <double, 5, Dynamic> E;
	Matrix3d Rot;
	Eigen::Matrix3d RY;
	E.resize(5, L);
	q.resize(5, L);
	dq.resize(5, L);
	ddq.resize(5, L);
	q_des.resize(5, L);
	dq_des.resize(5, L);
	daq_des.resize(5, L);
	ddaq_des.resize(5, L);
	aq_des.resize(5, L);
	ddq_des.resize(5, L);
	state_des.resize(6, L);
	state_actual.resize(6, L);
	d_state_des.resize(5, L);
	d_state_des2.resize(5, L);
	q.setZero();
	dq.setZero();
	ddq.setZero();
	q_des.setZero();
	dq_des.setZero();
	ddq_des.setZero();
	ddaq_des.setZero();
	d_state_des.setZero();
	d_state_des2.setZero();
	state_des.setZero();
	state_actual.setZero();
	aq_des.setZero();
	std::vector<double> a;
	std::vector<double> da;
	std::vector<double> dda;
	std::vector<double> a_des;
	std::vector<double> da_des;
	std::vector<double> dda_des;
	std::vector<double> bbb;
	bbb.resize(3);
	a.resize(5);
	da.resize(5);
	dda.resize(5);
	a_des.resize(5);
	da_des.resize(5);
	dda_des.resize(5);
	double sigma;
	double sigma_th;
	double lambda_max;
	double lambda;
	Matrix <double, 5, 5> B;
	//���˶�ѧģ��
	std::vector<double> f;
	f.resize(5);
	f = { 0,0,0,0,0 };

	std::vector<double> q2;
	q2.resize(5);
	q2 = { 0,0,0,0,0 };

	std::vector<double> f2;
	f2.resize(5);
	f2 = { 0,0,0,0,0 };
	q_des.col(0) << 0, 0, 0, 0, 0;
	q.col(0) << 0, 0, 0, 0, 0;

	for (int i = 0; i < L; i++) {
		for (int j = 0; j < 5; j++) {
			aq_des.col(i)[j] = 5 * sin(i * dd) * pi / 30;
			f[j] = aq_des.col(i)[j];
		}
		//���˶�ѧ���ɹ켣
		K_model.get_R_t(f);
		Eigen::Matrix3f Rad;
		Rad.setZero();
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				Rad(i,j) = K_model.T.at(5).block<3, 3>(0, 0)(i,j);
			}
		}
		std::vector<float> result;
		result.resize(3);
		result=computeEularAngles(Rad, false);

		eulerAngle[0] = result[0] / 180 * pi;
		eulerAngle[1] = result[1] / 180 * pi;
		eulerAngle[2] = result[2] / 180 * pi;

		pos = K_model.T.at(5).block<3, 1>(0, 3);
		state_des.col(i)[0] = pos[0];
		state_des.col(i)[1] = pos[1];
		state_des.col(i)[2] = pos[2];
		state_des.col(i)[3] = eulerAngle[0];
		state_des.col(i)[4] = eulerAngle[1];
		state_des.col(i)[5] = eulerAngle[2];
		//std::cout << state_des.col(i)[3] << std::endl;
	}
	
	
	for (int i = 1; i < L ; i++) {
     
		//
		dq.col(0) << 5 * pi / 30, 5 * pi / 30, 5 * pi / 30, 5 * pi / 30, 5 * pi / 30;
		//dq.col(0) << 0,0,0, 5 * pi / 30,0;
		for (int j = 0; j < 5; j++) {
			//
			a[j] = q.col(i-1)[j];
			da[j] = dq.col(i-1)[j];
			dda[j] = ddq.col(i-1)[j];
		}
		
		//
		K_model.get_R_t(a);
		Eigen::Matrix3f Rbd;
		Rbd.setZero();
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				Rbd(i, j) = K_model.T.at(5).block<3, 3>(0, 0)(i, j);
			}
		}
		std::vector<float> result;
		result.resize(3);
		result = computeEularAngles(Rbd, false);

		eulerAngle[0] = result[0] / 180 * pi;//roll
		eulerAngle[1] = result[1] / 180 * pi;//pitch
		eulerAngle[2] = result[2] / 180 * pi;//yaw


		pos = K_model.T.at(5).block<3, 1>(0, 3);
		//����ٶȡ�ŷ�����ٶ�ת������
		Rot = get_Rot(eulerAngle[1], eulerAngle[2]);
		//��������ĩ���ٶ�
		d_state_des.col(i)[0] = (state_des.col(i)[0] - pos[0]) / dd;
		d_state_des.col(i)[1] = (state_des.col(i)[1] - pos[1]) / dd;
		d_state_des.col(i)[2] = (state_des.col(i)[2] - pos[2]) / dd;
		bbb[0] = (state_des.col(i)[3] - eulerAngle[0]) / dd;
		bbb[1] = (state_des.col(i)[4] - eulerAngle[1]) / dd;
		bbb[2] = (state_des.col(i)[5] - eulerAngle[2]) / dd;
		
		//transform to omega
		d_state_des.col(i)[3] = Rot(0, 0) * bbb[0] + Rot(0, 1) * bbb[1] + Rot(0, 2) * bbb[2];
		d_state_des.col(i)[4] = Rot(1, 0) * bbb[0] + Rot(1, 1) * bbb[1] + Rot(1, 2) * bbb[2];
		//d_state_des.col(i)[5] = Rot(2, 0) * bbb[0] + Rot(2, 1) * bbb[1] + Rot(2, 2) * bbb[2];
		
		//DLS
		K_model.get_jacobain(a);
		jacob2=K_model.jacob.block<5, 5>(0, 0);
		
		//��svd
		JacobiSVD<MatrixXd> svd(jacob2, ComputeThinU | ComputeThinV);
		sigma= svd.singularValues()[4];
		sigma_th = 1 / pi;
		lambda_max = 0.01;
		if (sigma >= sigma_th)
		{
		    lambda = 0;
		}
		else
		{
           lambda = lambda_max - lambda_max * sigma / sigma_th;
		}
		
		B = jacob2 * jacob2.transpose() + lambda * MatrixXd::Identity(5, 5);

		//�������ؽ��ٶ�
		dq_des.col(i) = jacob2.transpose() * B.inverse() * d_state_des.col(i);
		
		//�������ؽڽǶȺͼ��ٶ�
		switch (i) {
		case 0:
			q_des.col(0) << 0, 0, 0, 0, 0;
			break;
		default:
			q_des.col(i) = q.col(i - 1) + dq_des.col(i) * dd ;// + (dq_des.col(i) - dq_des.col(i - 1)) * dd / 2
			break;
		}
		switch (i) {
		case 0:
			ddq_des.col(0).setZero();
			break;
		default:
			ddq_des.col(i) =(dq_des.col(i) - dq.col(i - 1)) / dd;
			break;
		}
		
		//std::cout << q_des.col(i)[1] << std::endl;
		
		
		for (int j = 0; j < 5; j++) {
			a_des[j] = q_des.col(i)[j];
			da_des[j] = dq_des.col(i)[j];
			dda_des[j] = ddq_des.col(i)[j];
		}
		
	    //����
		ID_model.get_R_t(a);
		tau_dynamic = ID_model.rnea(a, da, dda_des);//ֻ�м��ٶ�������ֵ,û�мӻ������ƶ�����UAM�в�һ��
	    //PID controller
		torque = P_model.PID_control(a, da, dda, a_des, da_des, dda_des, tau_dynamic);

		//������ѧ��ؽڽǶȣ��õ���ǰλ�ã������ſɱȾ����������
		dda = { 0,0,0,0,0 };
		ID_model.get_R_t(a);
		tg = ID_model.rnea(a, tt, tt);//g+c
		t0 = ID_model.rnea(a, da, tt);
		tt1 = { 1, 0, 0, 0, 0 };
		t1 = ID_model.rnea(a, tt, tt1);
		tt1 = { 0, 1, 0, 0, 0 };
		t2 = ID_model.rnea(a, tt, tt1);
		tt1 = { 0, 0, 1, 0, 0 };
		t3 = ID_model.rnea(a, tt, tt1);
		tt1 = { 0, 0, 0, 1, 0 };
		t4 = ID_model.rnea(a, tt, tt1);
		tt1 = { 0, 0, 0, 0, 1 };
		t5 = ID_model.rnea(a, tt, tt1);
		for (int i = 0; i < 5; i++) {
			M.col(0)[i] = t1[i] - tg[i];
			M.col(1)[i] = t2[i] - tg[i];
			M.col(2)[i] = t3[i] - tg[i];
			M.col(3)[i] = t4[i] - tg[i];
			M.col(4)[i] = t5[i] - tg[i];
		}
		for (int i = 0; i < 5; i++) {
			for (int j = 0; j < 5; j++) {
				dda[i] = dda[i] + M.inverse().row(i)[j] * (torque[j] - t0[j]);//E���Ŷ�
				//dda[i] = dda[i] + M.inverse().row(i)[j] * (tau_dynamic[j] - t0[j]);
			}
		}
		//compute q dq ddq

		for (int j = 0; j < 5; j++) {
			ddq.col(i)[j] = dda[j];
			dq.col(i)[j] = dq.col(i-1)[j] + dd * ddq.col(i)[j];
			q.col(i)[j] = q.col(i-1)[j] + dd * dq.col(i)[j] ;
		}
		//std::cout << q_des.col(i)[1] << std::endl;
		//std::cout << q.col(i)[1] << std::endl;
		// 
	
		// 
		state_actual.col(i - 1)[0] = pos[0];
		state_actual.col(i - 1)[1] = pos[1];
		state_actual.col(i - 1)[2] = pos[2];
		state_actual.col(i - 1)[3] = eulerAngle[0];
		state_actual.col(i - 1)[4] = eulerAngle[1];
		state_actual.col(i - 1)[5] = eulerAngle[2];


		std::cout << state_des.col(i-1)[0] << std::endl;
		std::cout << state_actual.col(i-1)[0] << std::endl;

		

		
	}

}



