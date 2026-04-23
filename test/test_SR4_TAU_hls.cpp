#include <iostream>
#include <cmath>
#include <chrono>
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

    // 开始计时
    auto start = std::chrono::high_resolution_clock::now();

    // 调用 HLS 顶层函数
    total_tau_hls(q, dq, ddq, sinq, cosq, tau_out);

    // 结束计时
    auto end = std::chrono::high_resolution_clock::now();

    // 计算耗时
    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // 输出结果
    std::cout << "=========== tau_out ===========" << std::endl;
    for (int i = 0; i < DOF; i++)
    {
        std::cout << "tau_out[" << i << "] = " << tau_out[i] << std::endl;
    }

    std::cout << "=========== time ===========" << std::endl;
    std::cout << "计算时间: " << duration_ns << " ns" << std::endl;
    std::cout << "计算时间: " << duration_us << " us" << std::endl;

    return 0;
}