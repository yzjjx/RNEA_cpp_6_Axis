#include <iostream>
#include <cmath>
#include <chrono>
#include "SR4_TAU_hls.h"

int main()
{
    constexpr int rounds = 10;
    constexpr int iters_per_round = 100000;

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

    // 先热身，避免首次调用带来的额外开销
    for (int i = 0; i < 1000; i++)
    {
        total_tau_hls(q, dq, ddq, sinq, cosq, tau_out);
    }

    volatile data_t checksum = 0;
    double total_avg_ns = 0.0;

    std::cout << "=========== benchmark start ===========" << std::endl;
    std::cout << "rounds = " << rounds << std::endl;
    std::cout << "iters_per_round = " << iters_per_round << std::endl;

    for (int r = 0; r < rounds; r++)
    {
        auto start = std::chrono::steady_clock::now();

        for (int i = 0; i < iters_per_round; i++)
        {
            total_tau_hls(q, dq, ddq, sinq, cosq, tau_out);
            checksum += tau_out[0];
        }

        auto end = std::chrono::steady_clock::now();

        auto round_total_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

        double avg_ns = static_cast<double>(round_total_ns) / iters_per_round;
        double avg_us = avg_ns / 1000.0;

        total_avg_ns += avg_ns;

        std::cout << "[Round " << (r + 1) << "] "
                  << "总时间: " << round_total_ns << " ns, "
                  << "平均单次: " << avg_ns << " ns ("
                  << avg_us << " us)" << std::endl;
    }

    double final_avg_ns = total_avg_ns / rounds;
    double final_avg_us = final_avg_ns / 1000.0;

    std::cout << "=========== final average ===========" << std::endl;
    std::cout << "10轮平均单次时间: " << final_avg_ns << " ns" << std::endl;
    std::cout << "10轮平均单次时间: " << final_avg_us << " us" << std::endl;
    std::cout << "checksum = " << checksum << std::endl;

    // 再调用一次，输出结果检查是否正常
    total_tau_hls(q, dq, ddq, sinq, cosq, tau_out);

    std::cout << "=========== tau_out ===========" << std::endl;
    for (int i = 0; i < DOF; i++)
    {
        std::cout << "tau_out[" << i << "] = " << tau_out[i] << std::endl;
    }

    return 0;
}