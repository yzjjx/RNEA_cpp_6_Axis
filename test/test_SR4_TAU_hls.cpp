#include <iostream>
#include <cmath>
#include "SR4_TAU_hls.h"

int main()
{
    // 输入关节角
    data_t q[DOF] = {
        30.0f * static_cast<data_t>(M_PI) / 180.0f,
        60.0f * static_cast<data_t>(M_PI) / 180.0f,
        45.0f * static_cast<data_t>(M_PI) / 180.0f,
        20.0f * static_cast<data_t>(M_PI) / 180.0f,
        50.0f * static_cast<data_t>(M_PI) / 180.0f,
        45.0f * static_cast<data_t>(M_PI) / 180.0f
    };

    // 输入关节角速度
    data_t dq[DOF] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};

    // 输入关节角加速度
    data_t ddq[DOF] = {0.01f, 0.05f, 0.1f, 0.2f, 0.4f, 0.05f};

    // 由 q 计算 sinq / cosq
    data_t sinq[DOF];
    data_t cosq[DOF];
    for (int i = 0; i < DOF; i++)
    {
        sinq[i] = std::sin(q[i]);
        cosq[i] = std::cos(q[i]);
    }

    // 输出总力矩 tau1~tau6
    data_t tau_out[DOF];

    // 调用 HLS 顶层函数
    total_tau_hls(q, dq, ddq, sinq, cosq, tau_out);

    // 输出结果
    std::cout << "=========== tau_out ===========" << std::endl;
    for (int i = 0; i < DOF; i++)
    {
        std::cout << "tau_out[" << i << "] = " << tau_out[i] << std::endl;
    }

    return 0;
}