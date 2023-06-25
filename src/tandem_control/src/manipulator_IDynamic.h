#pragma once
//#define DYNAMICAL_MODEL_H
#include "manipulator_kinematic.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Dense>

class Manipulator_IDynamic : public Manipulator_Kinematic {

public:
    Manipulator_IDynamic();
    std::vector<double> rnea(std::vector<double> q, std::vector<double> dq, std::vector<double> ddq);
    std::vector<Eigen::Vector3d> f_;        //linear forces applied at the origin of the RF
    std::vector<Eigen::Vector3d> tau_;      //tourques

private:
    const int DOFS = 5;
    std::vector<double> m_;
    std::vector<Eigen::Matrix3d> inertiaTensor; // 3X3N tensor contains inertia matrix for each link
    std::vector<Eigen::Vector3d> cog_;     //3XN matrix contains cog vector for each link
    std::vector<double> alpha_;
    std::vector<double> l_;
    std::vector<double> d_;

    std::vector<Eigen::Vector3d> omega_;    //angular velocity
    std::vector<Eigen::Vector3d> d_omega_;  //angular acceleration
    std::vector<Eigen::Vector3d> a_;      //linear acceleration at frame i in frame i
    std::vector<Eigen::Vector3d> ac_;     //linear acceleration of the cog in frame i

    //Eigen::Vector3d u_;                     //tau_ after projection
    std::vector<double> u_;
    void initializeMatrices();
    void forwardRecursion(std::vector <double> q, std::vector<double> dq, std::vector<double> ddq);
    void backwardRecursion(std::vector<double> q);
};


Manipulator_IDynamic::Manipulator_IDynamic() {

    //##########Dynamic Parameters #########//
    //masses
    m_.resize(DOFS);
    m_ = { 1.5, 1, 1.2, 1.2 , 0.8 };
    u_.resize(DOFS);

    //inertias
    inertiaTensor.resize(DOFS);
    inertiaTensor.at(0) << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    inertiaTensor.at(1) << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    inertiaTensor.at(2) << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    inertiaTensor.at(3) << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    inertiaTensor.at(4) << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;

    //cogs expressed wrt the next frame so ready to use in the formulas without transformation
    cog_.resize(DOFS);
    cog_.at(0) << 0, 0, 0.075;
    cog_.at(1) << 0, 0.125, 0;
    cog_.at(2) << 0, 0, 0.015;
    cog_.at(3) << 0, 0.015, 0;
    cog_.at(4) << 0, 0.1, 0;

    //##########initialization velocities, accelleration#########//
    omega_.resize(DOFS + 1);
    d_omega_.resize(DOFS + 1);
    a_.resize(DOFS + 1);
    ac_.resize(DOFS);

    //##########initialization forces and torques#########//
    f_.resize(DOFS + 1);
    tau_.resize(DOFS + 1);
}

std::vector<double> Manipulator_IDynamic::rnea(std::vector<double> q, std::vector<double> dq, std::vector<double> ddq) {
    initializeMatrices();
    forwardRecursion(q, dq, ddq);
    backwardRecursion(q);
    return  u_;
}

//��һ���ĵط��������ơ�����һ����Ҫת��
void Manipulator_IDynamic::forwardRecursion(std::vector<double> q, std::vector<double> dq, std::vector<double> ddq) {
    Eigen::Vector3d z;
    z << 0, 0, 1;

    Eigen::Vector3d tp;
    tp << 0, 0, 0;

    // Eigen::Matrix3d R;
    // Eigen::Vector3d p;

     //std::vector<double> dh_parameters;
     //dh_parameters.resize(4);
    double m;
    for (short int i = 0; i < DOFS; i++) {
        // cog_.at(i) = cog_.at(i) + tp;
        omega_.at(i + 1) = R.at(i).transpose() * omega_.at(i) + dq[i] * z;
        d_omega_.at(i + 1) = R.at(i).transpose() * (d_omega_.at(i) + omega_.at(i).cross(dq[i] * z)) + ddq[i] * z;
        a_.at(i + 1) = R.at(i).transpose() * (a_.at(i) + d_omega_.at(i).cross(tp) + omega_.at(i).cross(omega_.at(i).cross(tp)));
        ac_.at(i) = a_.at(i + 1) + d_omega_.at(i + 1).cross(cog_.at(i)) + omega_.at(i + 1).cross(omega_.at(i + 1).cross(cog_.at(i)));
        //std::cout << R.transpose().eval() << std::endl;
        tp = t.at(i);
    }
}


void  Manipulator_IDynamic::backwardRecursion(std::vector<double> q) {

    Eigen::Vector3d g;
    g << 0, 0, 9.81;
    Eigen::Vector3d z;
    z << 0, 0, 1;
    Eigen::Matrix3d Rp1;
    Rp1 << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    // Eigen::Matrix3d R, Rp1;
    // Eigen::Vector3d p;

    // std::vector<double> dh_parameters;
    //dh_parameters.resize(4);
    //double m;
    //the first transfomation is identity because it transfom
    //from the tip to the environment so it remain like it is
    //Rp1 << 1, 0, 0, 0, 1, 0,  0, 0, 1;
    for (short int i = DOFS - 1; i >= 0; i--) {
        //cog_.at(i) = cog_.at(i) + t.at(i);
        f_.at(i) = Rp1 * f_.at(i + 1) + m_[i] * (ac_.at(i));
        tau_.at(i) = Rp1 * tau_.at(i + 1) + t.at(i).cross(Rp1 * f_.at(i + 1)) + cog_.at(i).cross(m_[i] * ac_.at(i)) +
            inertiaTensor.at(i) * (d_omega_.at(i + 1)) + omega_.at(i + 1).cross(inertiaTensor.at(i) * (omega_.at(i + 1)));
        u_[i] = tau_.at(i).transpose() * z;//�ؽ�Ť�أ�ΪʲôҪ���һ��RT
        Rp1 = R.at(i);
    }
}

void  Manipulator_IDynamic::initializeMatrices() {

    for (short int i = 0; i < DOFS + 1; i++) {
        omega_.at(i).setZero();
        d_omega_.at(i).setZero();
        a_.at(i).setZero();
        f_.at(i).setZero();
        tau_.at(i).setZero();
        if (i < DOFS) ac_.at(i).setZero();
    }
    a_.at(0) << 0, 0, 9.81;
}

