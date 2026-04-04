#ifndef T_R_MAT_OUT
#define T_R_MAT_OUT

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>//cin和cout所在库

std::vector<Eigen::Matrix4d> Com_MDH_Trans( 
    const Eigen::VectorXd& alpha,
    const Eigen::VectorXd& a,
    const Eigen::VectorXd& d,
    const Eigen::VectorXd& q);

std::vector<Eigen::Matrix3d> Ext_Rot(int n,const std::vector<Eigen::Matrix4d> & T);

#endif