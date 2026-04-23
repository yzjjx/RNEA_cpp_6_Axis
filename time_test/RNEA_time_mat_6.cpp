#include "SR4_RNEA_tau.h"
#include "SR4_tau_f.h"

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <chrono>

int main()
{
    try
    {
        const int n = 6;
        constexpr int rounds = 10;
        constexpr int iters_per_round = 100000;

        // q1 和 q2 输入修正量：-82.875/180*pi
        const double q_offset = 82.875 / 180.0 * M_PI;

        // -----------------------------
        // 1) 直接输入 q, dq, ddq
        // -----------------------------
        Eigen::Matrix<double, 6, 1> q;
        Eigen::Matrix<double, 6, 1> dq;
        Eigen::Matrix<double, 6, 1> ddq;

        q << 30.0 * M_PI / 180.0,
             60.0 * M_PI / 180.0,
             45.0 * M_PI / 180.0,
             20.0 * M_PI / 180.0,
             50.0 * M_PI / 180.0,
             45.0 * M_PI / 180.0;

        dq << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
        ddq << 0.01, 0.05, 0.10, 0.20, 0.40, 0.05;

        // q1、q2 输入时都减去 82.875/180*pi
        Eigen::Matrix<double, 6, 1> q_corrected = q;
        q_corrected(1) -= q_offset;   // q1
        q_corrected(2) -= q_offset;   // q2

        std::cout << std::fixed << std::setprecision(10);

        std::cout << "=========== input ===========" << std::endl;
        std::cout << "q original   = " << q.transpose() << std::endl;
        std::cout << "q corrected  = " << q_corrected.transpose() << std::endl;
        std::cout << "dq           = " << dq.transpose() << std::endl;
        std::cout << "ddq          = " << ddq.transpose() << std::endl;

        // -----------------------------
        // 2) 热身
        // -----------------------------
        volatile double warmup_checksum = 0.0;
        for (int i = 0; i < 1000; ++i)
        {
            std::vector<double> tau  = Com_tau(n, q_corrected, dq, ddq);
            std::vector<double> tauf = Com_tau_f(n, dq);

            warmup_checksum += tau[1] + tauf[1];
        }

        // -----------------------------
        // 3) Benchmark
        // -----------------------------
        std::vector<double> tau;
        std::vector<double> tauf;

        volatile double checksum = 0.0;
        double total_avg_ns = 0.0;

        std::cout << "\n=========== benchmark start ===========" << std::endl;
        std::cout << "rounds = " << rounds << std::endl;
        std::cout << "iters_per_round = " << iters_per_round << std::endl;

        for (int r = 0; r < rounds; ++r)
        {
            auto start = std::chrono::steady_clock::now();

            for (int i = 0; i < iters_per_round; ++i)
            {
                tau  = Com_tau(n, q_corrected, dq, ddq);
                // tauf = Com_tau_f(n, dq);

                // checksum += tau[1] + tauf[1];
            }

            auto end = std::chrono::steady_clock::now();

            auto round_total_ns =
                std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

            double avg_ns = static_cast<double>(round_total_ns) / iters_per_round;
            double avg_us = avg_ns / 1000.0;

            total_avg_ns += avg_ns;

            std::cout << "[Round " << (r + 1) << "] "
                      << "总时间 = " << round_total_ns << " ns, "
                      << "平均单次 = " << avg_ns << " ns ("
                      << avg_us << " us)" << std::endl;
        }

        double final_avg_ns = total_avg_ns / rounds;
        double final_avg_us = final_avg_ns / 1000.0;

        // -----------------------------
        // 4) 输出最终统计
        // -----------------------------
        std::cout << "\n=========== final average ===========" << std::endl;
        std::cout << "10轮平均单次时间: " << final_avg_ns << " ns" << std::endl;
        std::cout << "10轮平均单次时间: " << final_avg_us << " us" << std::endl;
        std::cout << "checksum = " << checksum << std::endl;

        // -----------------------------
        // 5) 再算一次，输出结果检查
        // -----------------------------
        tau  = Com_tau(n, q_corrected, dq, ddq);
        tauf = Com_tau_f(n, dq);

        if (tau.size() < n + 1)
        {
            throw std::runtime_error("Com_tau 返回维度不足 7");
        }
        if (tauf.size() < n + 1)
        {
            throw std::runtime_error("Com_tau_f 返回维度不足 7");
        }

        std::cout << "\n=========== tau ===========" << std::endl;
        for (int i = 0; i <= 6; ++i)
        {
            std::cout << "tau[" << i << "] = " << tau[i] << std::endl;
        }

        std::cout << "\n=========== tauf ===========" << std::endl;
        for (int i = 0; i <= 6; ++i)
        {
            std::cout << "tauf[" << i << "] = " << tauf[i] << std::endl;
        }

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "程序出错: " << e.what() << std::endl;
        return -1;
    }
}