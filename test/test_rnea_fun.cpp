#include "SR4_RNEA_tau.h"
#include "SR4_tau_f.h"

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <iomanip>

int main()
{
    const int n = 6;

    // 输入关节角度
    Eigen::Matrix<double, n, 1> q;
    q << M_PI * 30.0 / 180.0,
         M_PI * 60.0 / 180.0,
         M_PI * 45.0 / 180.0,
         M_PI * 20.0 / 180.0,
         M_PI * 50.0 / 180.0,
         M_PI * 45.0 / 180.0;

    // 输入关节角速度
    Eigen::Matrix<double, n, 1> dq;
    dq << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

    // 输入关节角加速度
    Eigen::Matrix<double, n, 1> ddq;
    ddq << 0.01, 0.05, 0.1, 0.2, 0.4, 0.05;

    // 调用函数
    std::vector<double> tau  = Com_tau(n, q, dq, ddq);
    std::vector<double> tauf = Com_tau_f(n, dq);

    std::cout << std::fixed << std::setprecision(10);

    // 打印输入
    std::cout << "================ 输入数据 ================" << std::endl;
    std::cout << "q =" << std::endl << q << std::endl << std::endl;
    std::cout << "dq =" << std::endl << dq << std::endl << std::endl;
    std::cout << "ddq =" << std::endl << ddq << std::endl << std::endl;

    // 打印返回长度
    std::cout << "tau.size()  = " << tau.size() << std::endl;
    std::cout << "tauf.size() = " << tauf.size() << std::endl << std::endl;

    // 打印 tau
    std::cout << "================ tau 输出 ================" << std::endl;
    for (size_t i = 0; i < tau.size(); i++)
    {
        std::cout << "tau[" << i << "] = " << tau[i] << std::endl;
    }
    std::cout << std::endl;

    // 打印 tauf
    std::cout << "================ tauf 输出 ================" << std::endl;
    for (size_t i = 0; i < tauf.size(); i++)
    {
        std::cout << "tauf[" << i << "] = " << tauf[i] << std::endl;
    }
    std::cout << std::endl;

    return 0;
}