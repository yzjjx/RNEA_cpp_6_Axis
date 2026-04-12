#ifndef SE4_TAU_F
#define SE4_TAU_F

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>//cin和cout所在库

std::vector<double>  Com_tau_f(const int n,Eigen::Matrix<double,6,1> dq);

#endif