#include <iostream>
#include <cmath>
#include <SR4_RNEA_FPGA.h>
// 假设你的 SR4_rnea_hls 和相关常量、函数都在同一个文件里
// 如果分成头文件和源文件，就 include 对应头文件

int main()
{
    // 输入关节角
    data_t q[n] = {
        30.0 * M_PI / 180.0,
        60.0 * M_PI / 180.0,
        45.0 * M_PI / 180.0,
        20.0 * M_PI / 180.0,
        50.0 * M_PI / 180.0,
        45.0 * M_PI / 180.0
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
    data_t tau_out[n];
    data_t omega_out[n+1][3];

    // 调用
    SR4_rnea_hls(q, dq, ddq, sinq, cosq, tau_out, omega_out);

    // 打印基座和各连杆角速度
    std::cout << "=========== omega ===========" << std::endl;
    for (int i = 0; i <= n; i++)
    {
        std::cout << "omega[" << i << "] = [ "
                  << omega_out[i][0] << ", "
                  << omega_out[i][1] << ", "
                  << omega_out[i][2] << " ]" << std::endl;
    }

    return 0;
}