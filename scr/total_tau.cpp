#include "SR4_RNEA_tau.h"
#include "SR4_tau_f.h"

#include <vector>
#include <stdexcept>
#include <algorithm>
#include <Eigen/Dense>

// 总力矩 = 动力学力矩 + 摩擦力矩
std::vector<double> Com_total_tau(const int n,
                                  const Eigen::Matrix<double,6,1>& q,
                                  const Eigen::Matrix<double,6,1>& dq,
                                  const Eigen::Matrix<double,6,1>& ddq)
{
    // 分别计算动力学力矩和摩擦力矩
    std::vector<double> tau   = Com_tau(n, q, dq, ddq);
    std::vector<double> tau_f = Com_tau_f(n, dq);

    // 计算总力矩
    std::vector<double> total_tau(tau.size(), 0.0);
    for (size_t i = 0; i < 7; i++)
    {
        total_tau[i] = tau[i+1] + tau_f[i];
    }

    return total_tau;
}