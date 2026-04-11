#ifndef SR4_RNEA_TAU
#define SR4_RNEA_TAU

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>//cin和cout所在库

std::vector<double> Com_tau(const int n,Eigen::Matrix<double,6,1> q,
                            Eigen::Matrix<double,6,1> dq,
                            Eigen::Matrix<double,6,1> ddq);

#endif