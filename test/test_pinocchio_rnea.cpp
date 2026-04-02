/*该代码用于pinocchio的RNEA的前向递推解*/
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include <Eigen/Dense>
#include <iostream>

#include <fstream>
#include <vector>

int main()
{
    using namespace pinocchio;
    // urdf路径
    const std::string urdf_filename = "/home/yzj_robot/20260402_RNEA_cpp/urdf_and_xml/xMateSR4C_gen.urdf";

    // 建立机器人模型
    Model SR4C_model;
    pinocchio::urdf::buildModel(urdf_filename, SR4C_model);

    // 创建 data
    Data SR4C_data(SR4C_model);

    // 输出质量、质心坐标、惯性矩阵
    std::cout<<"---------------输出每个刚体的质量、质心以及惯量-----------------\n";
    for (JointIndex i = 1; i < SR4C_model.njoints; ++i)
    {
        // &引用，表示 Y 不是复制出一个新对象，而是直接给原来的那个元素起了一个别名
        const auto &Y = SR4C_model.inertias[i];

        std::cout << "关节id = " << i << std::endl;
        std::cout << "质量 = " << Y.mass() << std::endl;
        std::cout << "质心坐标 = " << Y.lever().transpose() << std::endl;
        std::cout << "惯性矩阵 = \n" << Y.inertia().matrix() << std::endl;
        std::cout << "----------------------------------------\n";
    }

    return 0;
}