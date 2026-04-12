#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include "total_tau.h"

// 读取一个文本文件，每一行读取 6 个数
// 兼容：空格 / 制表符 / 逗号 分隔
std::vector<Eigen::Matrix<double,6,1>> readTXT6(const std::string& filename)
{
    std::vector<Eigen::Matrix<double,6,1>> data;
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "错误：无法打开文件 " << filename << std::endl;
        return data;
    }

    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty()) continue;

        // 把逗号和制表符替换成空格，便于统一解析
        for (char& ch : line)
        {
            if (ch == ',' || ch == '\t')
            {
                ch = ' ';
            }
        }

        std::stringstream ss(line);
        std::vector<double> values;
        double val;

        while (ss >> val)
        {
            values.push_back(val);
        }

        // 每行必须正好 6 个数据
        if (values.size() != 6)
        {
            std::cerr << "警告：" << filename
                      << " 中某一行数据个数不是 6，已跳过该行。"
                      << std::endl;
            continue;
        }

        Eigen::Matrix<double,6,1> vec;
        for (int i = 0; i < 6; ++i)
        {
            vec(i,0) = values[i];
        }

        data.push_back(vec);
    }

    file.close();
    return data;
}

int main()
{
    const int n = 6;

    // q1 和 q2 输入修正量：-82.875/180*pi
    const double q_offset = 82.875 / 180.0 * M_PI;

    // 输入文件名
    const std::string q_file   = "../data_in/q.txt";
    const std::string dq_file  = "../data_in/dq.txt";
    const std::string ddq_file = "../data_in/ddq.txt";

    // 读取输入数据
    std::vector<Eigen::Matrix<double,6,1>> q_data   = readTXT6(q_file);
    std::vector<Eigen::Matrix<double,6,1>> dq_data  = readTXT6(dq_file);
    std::vector<Eigen::Matrix<double,6,1>> ddq_data = readTXT6(ddq_file);

    if (q_data.empty() || dq_data.empty() || ddq_data.empty())
    {
        std::cerr << "错误：输入文件为空或读取失败！" << std::endl;
        return -1;
    }

    // 取最小行数，防止三个文件行数不一致
    size_t rows = std::min({q_data.size(), dq_data.size(), ddq_data.size()});

    if (q_data.size() != dq_data.size() || q_data.size() != ddq_data.size())
    {
        std::cerr << "警告：q/dq/ddq 行数不一致，将按最小行数 "
                  << rows << " 进行计算。" << std::endl;
    }

    // 输出 CSV 文件
    std::ofstream outfile("total_tau.csv");
    if (!outfile.is_open())
    {
        std::cerr << "错误：无法创建输出文件 total_tau.csv" << std::endl;
        return -1;
    }

    // 写表头
    outfile << "tau0,tau1,tau2,tau3,tau4,tau5,tau6\n";

    // 逐行计算总力矩
    for (size_t i = 0; i < rows; ++i)
    {
        // 复制当前时刻 q，并对 q1、q2 做输入修正
        Eigen::Matrix<double,6,1> q_corrected = q_data[i];
        q_corrected(1,0) -= q_offset;  // q1 = q1 - 82.875/180*pi
        q_corrected(2,0) -= q_offset;  // q2 = q2 - 82.875/180*pi

        std::vector<double> total_tau = Com_total_tau(n, q_corrected, dq_data[i], ddq_data[i]);

        if (total_tau.size() < static_cast<size_t>(n + 1))
        {
            std::cerr << "警告：第 " << i + 1
                      << " 行计算结果长度不足，已跳过。" << std::endl;
            continue;
        }

        // 写入 csv
        for (int j = 0; j <= n; ++j)
        {
            outfile << total_tau[j];
            if (j < n) outfile << ",";
        }
        outfile << "\n";
    }

    outfile.close();

    std::cout << "总力矩已输出到 total_tau.csv" << std::endl;
    return 0;
}