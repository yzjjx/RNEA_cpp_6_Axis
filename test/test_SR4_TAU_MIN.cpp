#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <stdexcept>
#include <filesystem>

#include "SR4_TAU_MIN.h"

using Row6 = std::array<data_t, n>;

// 读取一个 txt 文件，每行 6 个数
// 支持逗号 / 空格 / 制表符分隔
std::vector<Row6> readTxt6(const std::string& filename)
{
    std::vector<Row6> data;
    std::ifstream file(filename);

    if (!file.is_open())
    {
        throw std::runtime_error("无法打开文件: " + filename);
    }

    std::string line;
    int line_no = 0;

    while (std::getline(file, line))
    {
        line_no++;

        if (line.empty()) continue;

        // 把逗号和制表符替换成空格
        for (char& ch : line)
        {
            if (ch == ',' || ch == '\t')
            {
                ch = ' ';
            }
        }

        std::stringstream ss(line);
        Row6 row{};

        for (int i = 0; i < n; i++)
        {
            if (!(ss >> row[i]))
            {
                throw std::runtime_error(
                    "文件 " + filename + " 第 " + std::to_string(line_no) +
                    " 行数据不足 6 个");
            }
        }

        data.push_back(row);
    }

    return data;
}

int main()
{
    try
    {
        // 输入文件
        const std::string q_file   = "../data_in/q.txt";
        const std::string dq_file  = "../data_in/dq.txt";
        const std::string ddq_file = "../data_in/ddq.txt";

        // 输出文件
        const std::string out_dir        = "../data_out";
        const std::string total_tau_file = "../data_out/total_tau.txt";

        // q1、q2 的角度偏置
        const data_t q_offset = 82.875 / 180.0 * M_PI;

        // 读取数据
        std::vector<Row6> q_data   = readTxt6(q_file);
        std::vector<Row6> dq_data  = readTxt6(dq_file);
        std::vector<Row6> ddq_data = readTxt6(ddq_file);

        // 检查行数是否一致
        if (q_data.size() != dq_data.size() || q_data.size() != ddq_data.size())
        {
            throw std::runtime_error("q、dq、ddq 三个文件的行数不一致");
        }

        if (q_data.empty())
        {
            throw std::runtime_error("输入文件为空");
        }

        // 创建输出目录
        std::filesystem::create_directories(out_dir);

        std::ofstream fout(total_tau_file);
        if (!fout.is_open())
        {
            throw std::runtime_error("无法创建输出文件: " + total_tau_file);
        }

        // 逐行计算
        for (size_t row = 0; row < q_data.size(); row++)
        {
            data_t q[n];
            data_t dq[n];
            data_t ddq[n];
            data_t sinq[n];
            data_t cosq[n];

            // 读入原始数据
            for (int i = 0; i < n; i++)
            {
                q[i]   = q_data[row][i];
                dq[i]  = dq_data[row][i];
                ddq[i] = ddq_data[row][i];
            }

            // q1、q2 输入时都减去 82.875/180*pi
            // 这里沿用你原来的写法
            q[1] -= q_offset;   // 第2个关节
            q[2] -= q_offset;   // 第3个关节

            // 用修正后的 q 计算 sin / cos
            for (int i = 0; i < n; i++)
            {
                sinq[i] = std::sin(q[i]);
                cosq[i] = std::cos(q[i]);
            }

            // 总力矩输出
            data_t tau_total[n + 1] = {0};

            total_tau(q, dq, ddq, sinq, cosq, tau_total);

            // 输出 total_tau1 ~ total_tau6
            for (int i = 1; i <= n; i++)
            {
                fout << tau_total[i];
                if (i < n) fout << ",";
            }
            fout << "\n";
        }

        fout.close();

        std::cout << "计算完成，共处理 " << q_data.size() << " 行数据。" << std::endl;
        std::cout << "q1、q2 已减去偏置: " << q_offset << " rad" << std::endl;
        std::cout << "total_tau 已保存到: " << total_tau_file << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "错误: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}