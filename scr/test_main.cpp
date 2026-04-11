#include "SR4_RNEA_tau.h"
#include "SR4_tau_f.h"

#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>

// 去除字符串首尾空白
static std::string trim(const std::string& s)
{
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

// 解析一行 6 维数据：a,b,c,d,e,f
static bool parseLineToVec6(const std::string& line, Eigen::Matrix<double, 6, 1>& vec)
{
    std::string cleaned = trim(line);
    if (cleaned.empty()) return false;

    // 去掉 UTF-8 BOM
    if (cleaned.size() >= 3 &&
        static_cast<unsigned char>(cleaned[0]) == 0xEF &&
        static_cast<unsigned char>(cleaned[1]) == 0xBB &&
        static_cast<unsigned char>(cleaned[2]) == 0xBF)
    {
        cleaned = cleaned.substr(3);
    }

    std::stringstream ss(cleaned);
    std::string item;
    int count = 0;

    while (std::getline(ss, item, ','))
    {
        item = trim(item);
        if (item.empty())
        {
            std::cerr << "发现空字段，原始行: " << line << std::endl;
            return false;
        }

        try
        {
            if (count >= 6)
            {
                std::cerr << "一行数据超过 6 个元素，原始行: " << line << std::endl;
                return false;
            }
            vec(count) = std::stod(item);
            count++;
        }
        catch (const std::exception&)
        {
            std::cerr << "数字解析失败，原始行: " << line << std::endl;
            return false;
        }
    }

    if (count != 6)
    {
        std::cerr << "一行数据不是 6 个元素，原始行: " << line << std::endl;
        return false;
    }

    return true;
}

// 读取整个 txt 文件，每行转成一个 6x1 向量
static std::vector<Eigen::Matrix<double, 6, 1>> readTxtFile(const std::string& filename)
{
    std::ifstream fin(filename);
    if (!fin.is_open())
    {
        throw std::runtime_error("无法打开文件: " + filename);
    }

    std::vector<Eigen::Matrix<double, 6, 1>> data;
    std::string line;
    int line_num = 0;

    while (std::getline(fin, line))
    {
        line_num++;
        Eigen::Matrix<double, 6, 1> vec;

        if (trim(line).empty())
        {
            continue; // 跳过空行
        }

        if (!parseLineToVec6(line, vec))
        {
            throw std::runtime_error("文件 " + filename + " 第 " + std::to_string(line_num) + " 行格式错误");
        }

        data.push_back(vec);
    }

    return data;
}

// 写 tau.csv，输出 tau0~tau6
static void writeTauCsv(const std::string& filename,
                        const std::vector<std::vector<double>>& tau_all)
{
    std::ofstream fout(filename);
    if (!fout.is_open())
    {
        throw std::runtime_error("无法创建文件: " + filename);
    }

    fout << "tau0,tau1,tau2,tau3,tau4,tau5,tau6\n";
    fout << std::fixed << std::setprecision(15);

    for (const auto& row : tau_all)
    {
        if (row.size() != 7)
        {
            throw std::runtime_error("tau 数据维度不是 7");
        }

        for (int i = 0; i < 7; i++)
        {
            fout << row[i];
            if (i != 6) fout << ",";
        }
        fout << "\n";
    }
}

// 写 tauf.csv，输出 tauf0~tauf6
static void writeTauFCsv(const std::string& filename,
                         const std::vector<std::vector<double>>& tauf_all)
{
    std::ofstream fout(filename);
    if (!fout.is_open())
    {
        throw std::runtime_error("无法创建文件: " + filename);
    }

    fout << "tauf0,tauf1,tauf2,tauf3,tauf4,tauf5,tauf6\n";
    fout << std::fixed << std::setprecision(15);

    for (const auto& row : tauf_all)
    {
        if (row.size() != 7)
        {
            throw std::runtime_error("tauf 数据维度不是 7");
        }

        for (int i = 0; i < 7; i++)
        {
            fout << row[i];
            if (i != 6) fout << ",";
        }
        fout << "\n";
    }
}

int main()
{
    try
    {
        const int n = 6;

        // q1 和 q2 输入修正量：-82.875/180*pi
        const double q_offset = 82.875 / 180.0 * M_PI;

        // 输入文件名
        const std::string q_file   = "../data_in/q.txt";
        const std::string dq_file  = "../data_in/dq.txt";
        const std::string ddq_file = "../data_in/ddq.txt";

        // 读取数据
        std::vector<Eigen::Matrix<double, 6, 1>> q_all   = readTxtFile(q_file);
        std::vector<Eigen::Matrix<double, 6, 1>> dq_all  = readTxtFile(dq_file);
        std::vector<Eigen::Matrix<double, 6, 1>> ddq_all = readTxtFile(ddq_file);

        // 检查行数一致
        if (q_all.size() != dq_all.size() || q_all.size() != ddq_all.size())
        {
            throw std::runtime_error("q.txt、dq.txt、ddq.txt 的行数不一致");
        }

        std::vector<std::vector<double>> tau_all;
        std::vector<std::vector<double>> tauf_all;

        tau_all.reserve(q_all.size());
        tauf_all.reserve(q_all.size());

        // 逐行计算
        for (size_t i = 0; i < q_all.size(); i++)
        {
            // 复制一份当前时刻的关节角，用于修正
            Eigen::Matrix<double, 6, 1> q_corrected = q_all[i];

            // q1、q2 输入时都减去 82.875/180*pi
            q_corrected(1) -= q_offset;   // q1
            q_corrected(2) -= q_offset;   // q2

            std::vector<double> tau  = Com_tau(n, q_corrected, dq_all[i], ddq_all[i]);
            std::vector<double> tauf = Com_tau_f(n, dq_all[i]);

            // 默认 Com_tau / Com_tau_f 返回:
            // [0]=tau0, [1]=tau1, ..., [6]=tau6
            if (tau.size() < n + 1)
            {
                throw std::runtime_error("Com_tau 返回维度不足 7，出错行: " + std::to_string(i + 1));
            }
            if (tauf.size() < n + 1)
            {
                throw std::runtime_error("Com_tau_f 返回维度不足 7，出错行: " + std::to_string(i + 1));
            }

            std::vector<double> tau_row(7);
            std::vector<double> tauf_row(7);

            for (int j = 0; j <= 6; j++)
            {
                tau_row[j]  = tau[j];
                tauf_row[j] = tauf[j];
            }

            tau_all.push_back(tau_row);
            tauf_all.push_back(tauf_row);
        }

        // 输出 CSV
        writeTauCsv("tau.csv", tau_all);
        writeTauFCsv("tauf.csv", tauf_all);

        std::cout << "计算完成！" << std::endl;
        std::cout << "已生成: tau.csv" << std::endl;
        std::cout << "已生成: tauf.csv" << std::endl;
        std::cout << "q1 和 q2 已在输入时减去 82.875/180*pi。" << std::endl;
        std::cout << "tau.csv 中包含 tau0~tau6。" << std::endl;
        std::cout << "tauf.csv 中包含 tauf0~tauf6。" << std::endl;
        std::cout << "总计处理 " << q_all.size() << " 行数据。" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "程序出错: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}