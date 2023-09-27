//在控制器里面使用，做一个前馈，计算期望的扭矩发送给gazebo

#include "manipulator_kinematic.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Dense>

class Manipulator_IDynamic : public Manipulator_Kinematic {
 
public:
    Manipulator_IDynamic();
    std::vector<double>  rnea(std::vector<double> q, std::vector<double> dq, std::vector<double> ddq);

    std::vector<Eigen::Vector3d> f;        //force exerted on link i by i-1
    std::vector<Eigen::Vector3d> n;      //tourque exerted on link i by i-1
    std::vector<double> tau;
    //注意，以上那种形式的定义，后面必须resize一下。
private:
    std::vector<double> m;
    std::vector<Eigen::Matrix3d> inertia; //3X3N tensor contains inertia matrix for each link
    std::vector<Eigen::Vector3d> com;     //3XN matrix contains cog vector for each link

    std::vector<Eigen::Vector3d> omega;    //angular velocity
    std::vector<Eigen::Vector3d> domega;  //angular acceleration
    std::vector<Eigen::Vector3d> dv;      //linear acceleration at frame i in frame i
    std::vector<Eigen::Vector3d> F;      //net force
    std::vector<Eigen::Vector3d> N;      //net torque

    std::vector<Eigen::Matrix3d> R;  //rotation matrix
    std::vector<Eigen::Vector3d> P;  // the vector from i to i+1

    void initializeMatrices();
};


Manipulator_IDynamic::Manipulator_IDynamic() {

    //##########Dynamic Parameters #########//
    //masses
    m.resize(DOFs+1);
    double mb=0; //base mass
    m= { mb,0.245, 0.23, 0.25, 0.255 , 0.05 };
    // m = {mb, 2, 2, 2, 2 ,2};

    //inertias
    inertia.resize(DOFs+1);
    inertia.at(1) << 9.36e-5, 6.39e-8, 9.1e-8,
        6.39e-8, 1.11e-4, -2.21e-6,
        9.1e-8, -2.21e-6, 8.45e-5;
    inertia.at(2) << 7.49e-5, -3.29e-13, -1.56e-12,
        -3.29e-13, 8.19e-5, -5.78e-7,
        -1.56e-12, -5.78e-7, 6.85e-5;
    inertia.at(3) << 1.42e-4, -2.7e-8, -6e-9,
        -2.7e-8, 1.54e-4, -4.14e-6,
        -6e-9, -4.14e-6, 7.47e-5;
    inertia.at(4) << 5.98e-5, 0, 6.6e-7,
        0, 2.5e-4, 0,
        6.6e-7, 0, 2.75e-4;
    inertia.at(5) << 4.38e-5, -1.71e-12, 1.57e-13,
        -1.71e-12, 1.65e-5, 5.15e-7,
        1.57e-13, 5.15e-7, 3.59e-5;

    // inertia.at(1) << 2,0,0,
    //     0,2,0,
    //     0,0,2;
    // inertia.at(2) << 2,0,0,
    //     0,2,0,
    //     0,0,2;
    // inertia.at(3) << 2,0,0,
    //     0,2,0,
    //     0,0,2;
    // inertia.at(4) << 2,0,0,
    //     0,2,0,
    //     0,0,2;
    // inertia.at(5) << 2,0,0,
    //     0,2,0,
    //     0,0,2;
    
    //cogs expressed wrt the next frame so ready to use in the formulas without transformation
    com.resize(DOFs+1);
    com.at(1) << 0, 0, 0.0041;
    com.at(2) << 0, -0.0656, 0;
    com.at(3) << 0, 0, -0.009;
    com.at(4) << 0.14, 0, 0;
    com.at(5) << 0, -0.04, 0;
    //看一下质心位置是否对应，此项会影响机械臂运动
    // com.at(1) << 0, 0, 0.032;
    // com.at(2) << 0, 0, 0;
    // com.at(3) << 0, 0, -0.068;
    // com.at(4) << 0, 0, 0;
    // com.at(5) << 0, 0, 0;

    //##########initialization velocities, accelleration#########//
    omega.resize(DOFs + 1);
    domega.resize(DOFs + 1);
    dv.resize(DOFs + 1);
    F.resize(DOFs+1);
    N.resize(DOFs+1);
    R.resize(DOFs+2);
    P.resize(DOFs+2);

    //##########initialization forces and torques#########//
    f.resize(DOFs + 2);
    n.resize(DOFs + 2);
}

void  Manipulator_IDynamic::initializeMatrices() {

     //input foloating base 
     //note: 0指的是floating base的omega和domega，1指的才是关节1
    omega.at(0).setZero();
    domega.at(0).setZero();
    dv.at(0) << 0, 0, 9.81;
    // external force and torque exerted on the end effector,set to be zero
    f.at(DOFs+1).setZero();
    n.at(DOFs+1).setZero();
    R.at(DOFs+1).setIdentity(3,3); //末端坐标系转化到第5个坐标系，默认为单位矩阵
    P.at(DOFs+1).setZero(); //link5到末端的位置，默认为0，若末端没有力，这个不影响
    tau.resize(DOFs+1);//
}

std::vector<double> Manipulator_IDynamic::rnea(std::vector<double> q, std::vector<double> dq, std::vector<double> ddq) {
// std::vector<Eigen::Vector3d> Manipulator_IDynamic::rnea(std::vector<double> q, std::vector<double> dq, std::vector<double> ddq) {

    initializeMatrices();
    q.insert(q.begin(),0);
    dq.insert(dq.begin(),0);
    ddq.insert(ddq.begin(),0);

    Vector3d dvc; //linear acceleration of the com in frame i

   //compute rotation matrix R and p in the link frame, 编号1才是关节1，因此R的编号加1
    for (short int i = 1; i <= DOFs; i++) {
    Matrix4d T = Manipulator_IDynamic::MDH_transform(i-1, q[i]);
    R.at(i) = T.block<3, 3>(0, 0);
    P.at(i) =T.block<3,1>(0,3);
    }

   //forward recursion
    for (short int i = 1; i <= DOFs; i++) {
        omega.at(i) = R.at(i).transpose() *omega.at(i-1) + z*dq[i];   
        domega.at(i) = R.at(i).transpose() *domega.at(i-1) + (R.at(i).transpose() *omega.at(i-1)).cross( z*dq[i])+z*ddq[i] ;
        dv.at(i) = R.at(i).transpose() * (dv.at(i-1) + domega.at(i-1).cross(P.at(i)) + omega.at(i-1).cross(omega.at(i-1).cross(P.at(i))));
        dvc = dv.at(i) + domega.at(i).cross(com.at(i)) + omega.at(i).cross(omega.at(i).cross(com.at(i)));
        F.at(i)= m.at(i) * dvc;
        N.at(i)= inertia.at(i) * domega.at(i) + omega.at(i).cross(inertia.at(i)* omega.at(i));
    }

   //backward recursion
    for (short int i = DOFs; i >= 1; i--) {
       f.at(i) = R.at(i+1)*f.at(i + 1) + F.at(i);
       n.at(i) = R.at(i+1)*n.at(i+1) + P.at(i+1).cross(R.at(i+1)*f.at(i + 1))+ com.at(i).cross(F.at(i)) + N.at(i);
       tau[i]= n.at(i).transpose() *z;
    }
    // force and torque exerted on link 1 by base
    Vector3d force1;
    Vector3d tau1;
    tau.erase(tau.begin());//remeber this value must be initialized first, or it will be reduced one by one!!
    return  tau;
}

