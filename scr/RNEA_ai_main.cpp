#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <stdexcept>
#include <algorithm>

#include "SR4_rnea_AI.h"

// 读取一个 txt 文件，每行 6 个数，支持逗号 / 空格 / 制表符分隔
static std::vector<std::array<data_t, DOF>> read_txt_6col(const std::string& filename)
{
    std::vector<std::array<data_t, DOF>> data;
    std::ifstream fin(filename);
    if (!fin.is_open()) {
        throw std::runtime_error("无法打开文件: " + filename);
    }

    std::string line;
    int line_no = 0;
    while (std::getline(fin, line)) {
        ++line_no;

        if (line.empty()) {
            continue;
        }

        for (char& ch : line) {
            if (ch == ',' || ch == '\t') {
                ch = ' ';
            }
        }

        std::stringstream ss(line);
        std::array<data_t, DOF> row{};
        for (int i = 0; i < DOF; ++i) {
            if (!(ss >> row[i])) {
                throw std::runtime_error(
                    "文件 " + filename + " 第 " + std::to_string(line_no) +
                    " 行数据不足 6 列"
                );
            }
        }

        data.push_back(row);
    }

    return data;
}

static void write_txt_6col(const std::string& filename,
                           const std::vector<std::array<data_t, DOF>>& data)
{
    std::ofstream fout(filename);
    if (!fout.is_open()) {
        throw std::runtime_error("无法创建输出文件: " + filename);
    }

    fout.setf(std::ios::fixed);
    fout.precision(10);

    for (const auto& row : data) {
        for (int i = 0; i < DOF; ++i) {
            fout << row[i];
            if (i < DOF - 1) {
                fout << ",";
            }
        }
        fout << "\n";
    }
}

int main()
{
    try {
        const std::string q_file   = "../data_in/q.txt";
        const std::string dq_file  = "../data_in/dq.txt";
        const std::string ddq_file = "../data_in/ddq.txt";
        const std::string tau_file = "../data_in/tau.txt";

        std::vector<std::array<data_t, DOF>> q_data   = read_txt_6col(q_file);
        std::vector<std::array<data_t, DOF>> dq_data  = read_txt_6col(dq_file);
        std::vector<std::array<data_t, DOF>> ddq_data = read_txt_6col(ddq_file);

        if (q_data.size() != dq_data.size() || q_data.size() != ddq_data.size()) {
            throw std::runtime_error("q、dq、ddq 三个文件的行数不一致");
        }

        std::vector<std::array<data_t, DOF>> tau_data;
        tau_data.reserve(q_data.size());

        for (size_t k = 0; k < q_data.size(); ++k) {
            data_t q[DOF];
            data_t dq[DOF];
            data_t ddq[DOF];
            data_t sinq[DOF];
            data_t cosq[DOF];
            data_t tau_out[DOF];

            for (int i = 0; i < DOF; ++i) {
                q[i]   = q_data[k][i];
                dq[i]  = dq_data[k][i];
                ddq[i] = ddq_data[k][i];

                sinq[i] = static_cast<data_t>(std::sin(q[i]));
                cosq[i] = static_cast<data_t>(std::cos(q[i]));
            }

            SR4_rnea_hls(q, dq, ddq, sinq, cosq, tau_out);

            std::array<data_t, DOF> tau_row{};
            for (int i = 0; i < DOF; ++i) {
                tau_row[i] = tau_out[i];
            }
            tau_data.push_back(tau_row);
        }

        write_txt_6col(tau_file, tau_data);

        std::cout << "计算完成，共处理 " << q_data.size() << " 行数据。" << std::endl;
        std::cout << "输出文件: " << tau_file << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}