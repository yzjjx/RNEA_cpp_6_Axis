/*该代码用于pinocchio的RNEA的前向递推解*/
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/spatial/classic-acceleration.hpp"

#include <Eigen/Dense>
#include <iostream>

#include <fstream>
#include <vector>

int main()
{
    using namespace pinocchio;
    // urdf路径
    // const std::string urdf_filename = "/home/yzj_robot/20260402_RNEA_cpp/urdf_and_xml/xMateSR4C_gen.urdf";
    // 改为相对路径
    const std::string urdf_filename = "../urdf_and_xml/xMateSR4C_gen.urdf";

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

    // 前向牛顿欧拉计算结果
    // 定义关节初始状态q,v,a
    Eigen::VectorXd q = Eigen::VectorXd::Zero(SR4C_model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(SR4C_model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(SR4C_model.nv);

    // 给第一个关节测试值
    q[0] = 0.3;
    v[0] = 1.2;
    a[0] = 0.8;

    // 前向运动学
    pinocchio::forwardKinematics(SR4C_model,SR4C_data,q,v,a);

    //取第一个关节
    pinocchio::JointIndex joi_1 = SR4C_model.getJointId("xmate_joint_1");

    //计算^1 omega_1
    pinocchio::Motion v_1 = pinocchio::getVelocity(SR4C_model,SR4C_data,joi_1,pinocchio::LOCAL);
    Eigen::Vector3d omega_1 = v_1.angular();
    std::cout<<"Omega_1 = "<<omega_1<<"\n";

    //计算^1 d_omega_1
    pinocchio::Motion a_1 = pinocchio::getAcceleration(SR4C_model,SR4C_data,joi_1,pinocchio::LOCAL);
    Eigen::Vector3d d_omega_1 = a_1.angular(); 
    std::cout<<"d_Omega_1 = " << d_omega_1 << "\n";

    // 计算计算连杆坐标系原点线加速度 ^1 d_v_1
    pinocchio::Motion d_v_11 = pinocchio::getClassicalAcceleration(SR4C_model,SR4C_data,joi_1,pinocchio::LOCAL);
    Eigen::Vector3d d_v_1 = d_v_11.linear(); 
    std::cout<<"d_v_1 = " << d_v_1 << "\n";

    // 下面的部分都缺少对应的函数，因此选择直接代入公式计算结果
    // 计算连杆坐标系线 ^1 v_dc1
    // ^i M_{i+1}
    pinocchio::SE3 iMi1 = SR4C_data.oMi[0].actInv(SR4C_data.oMi[1]);
    // ^i R_{i+1}
    Eigen::Matrix3d iRi1 = iMi1.rotation();
    // ^{i+1} R_i
    Eigen::Matrix3d i1Ri = iRi1.transpose();
    std::cout<<"d_v_1 = " << i1Ri << "\n";

    // 计算^1F_1

    // 计算^1N_1

    return 0;
}