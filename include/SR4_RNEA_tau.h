#ifndef SR4_RNEA_TAU
#define SR4_RNEA_TAU

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>//cin和cout所在库

std::vector<double> Com_tau(const int n,const Eigen::Matrix<double,6,1>& q,
                            const Eigen::Matrix<double,6,1>& dq,
                            const Eigen::Matrix<double,6,1>& ddq);

#endif