#include <iostream>
#include <cmath>
#include "SR4_TAU_MIN.h"

int main()
{
    const data_t PI = std::acos(-1.0);

    // 输入关节角
    data_t q[n] = {
        30.0 * PI / 180.0,
        60.0 * PI / 180.0,
        45.0 * PI / 180.0,
        20.0 * PI / 180.0,
        50.0 * PI / 180.0,
        45.0 * PI / 180.0
    };

    // 输入关节角速度
    data_t dq[n] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

    // 输入关节角加速度
    data_t ddq[n] = {0.01, 0.05, 0.1, 0.2, 0.4, 0.05};

    // 由 q 计算 sinq / cosq
    data_t sinq[n];
    data_t cosq[n];
    for (int i = 0; i < n; i++)
    {
        sinq[i] = std::sin(q[i]);
        cosq[i] = std::cos(q[i]);
    }

    // 输出量
    data_t tau_dyn[n + 2]   = {0.0};
    data_t tau_fric[n + 2]  = {0.0};
    data_t tau_total[n + 2] = {0.0};

    // 计算
    SR4_rnea_hls(q, dq, ddq, sinq, cosq, tau_dyn);
    tau_f(dq, tau_fric);
    total_tau(q, dq, ddq, sinq, cosq, tau_total);

    // 输出动力学力矩
    std::cout << "=========== tau_dyn ===========" << std::endl;
    for (int i = 1; i <= n; i++)
    {
        std::cout << "tau_dyn[" << i << "] = " << tau_dyn[i] << std::endl;
    }

    // 输出摩擦力矩
    std::cout << "\n=========== tau_fric ===========" << std::endl;
    for (int i = 1; i <= n; i++)
    {
        std::cout << "tau_fric[" << i << "] = " << tau_fric[i] << std::endl;
    }

    // 输出总力矩
    std::cout << "\n=========== tau_total ===========" << std::endl;
    for (int i = 1; i <= n; i++)
    {
        std::cout << "tau_total[" << i << "] = " << tau_total[i] << std::endl;
    }

    return 0;
}