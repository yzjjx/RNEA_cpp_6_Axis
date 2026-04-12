#ifndef TOTAL_TAU
#define TOTAL_TAU

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>//cin和cout所在库
#include "SR4_RNEA_tau.h"
#include "SR4_tau_f.h"

std::vector<double> Com_total_tau(const int n,
                                  const Eigen::Matrix<double,6,1>& q,
                                  const Eigen::Matrix<double,6,1>& dq,
                                  const Eigen::Matrix<double,6,1>& ddq);

#endif