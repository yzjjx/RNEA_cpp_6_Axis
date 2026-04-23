#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/joint/joint-revolute.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <Eigen/Dense>
#include <array>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <chrono>

using namespace pinocchio;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::VectorXd;

// -----------------------------
// 4x4 基本变换
// -----------------------------
static Matrix4d RotX4(double alpha)
{
    Matrix4d T = Matrix4d::Identity();
    double c = std::cos(alpha);
    double s = std::sin(alpha);

    T(1,1) =  c;  T(1,2) = -s;
    T(2,1) =  s;  T(2,2) =  c;
    return T;
}

static Matrix4d TransX4(double a)
{
    Matrix4d T = Matrix4d::Identity();
    T(0,3) = a;
    return T;
}

static Matrix4d TransZ4(double d)
{
    Matrix4d T = Matrix4d::Identity();
    T(2,3) = d;
    return T;
}

// -----------------------------
// MDH 固定安装位姿
// -----------------------------
static SE3 mdhFixedPlacement(double alpha, double a, double d)
{
    Matrix4d T = RotX4(alpha) * TransX4(a) * TransZ4(d);
    return SE3(T.block<3,3>(0,0), T.block<3,1>(0,3));
}

int main()
{
    constexpr int n = 6;

    Model model;

    // -----------------------------
    // 1) 重力
    // -----------------------------
    model.gravity.linear()  << 0.0, 0.0, -9.8;
    model.gravity.angular() << 0.0, 0.0,  0.0;

    // -----------------------------
    // 2) 6轴 MDH 参数
    // -----------------------------
    Eigen::Matrix<double, n, 1> alpha;
    alpha << 0.0,
            -M_PI / 2.0,
             M_PI,
            -M_PI / 2.0,
             M_PI / 2.0,
            -M_PI / 2.0;

    Eigen::Matrix<double, n, 1> a_mdh;
    a_mdh << 0.0,
             0.0,
             0.05,
            -0.05,
             0.0,
             0.0;

    Eigen::Matrix<double, n, 1> d_mdh;
    d_mdh << 0.355,
             0.0,
             0.4,
             0.4,
             0.136,
             0.1035;

    // -----------------------------
    // 3) 动力学参数
    // -----------------------------
    const std::array<Vector3d, n> pc = {
        Vector3d( 1.09999999999679e-05, -0.00799199999999364, -0.0921580000000931),
        Vector3d( 0.169304139589595  , -0.0167375750841445 , -0.0698168604257776),
        Vector3d(-0.00101677311184785,  0.0992317670799201 , -0.00599764278732221),
        Vector3d(-2.50011383739252e-05, -0.0109230011023275, -0.071266999970154 ),
        Vector3d(-0.00141300709545952,  0.0118448713738878, -0.0209142000136706),
        Vector3d( 7.29993135608786e-05,  7.90001800890199e-05, -0.044852999792348 )
    };

    const std::array<double, n> m = {
        3.16699999486378,
        3.89505775503759,
        1.55161965449885,
        1.8819999954753,
        1.34621855234916,
        0.633999997351584
    };

    std::array<Matrix3d, n> I;

    I[0] <<
         0.0311679899460595,  -6.49999998879087e-07, -4.26999999258665e-06,
        -6.49999998879087e-07, 0.0307127099468431,  -0.00223655999611672,
        -4.26999999258665e-06, -0.00223655999611672, 0.00587739998995415;

    I[1] <<
         0.0179531824691758,   0.0309224033095523,  -0.00240843390062569,
         0.0309224033095523,   0.132852985273582,    0.00071426442226283,
        -0.00240843390062569,  0.00071426442226283,  0.138139839236999;

    I[2] <<
         0.0205839772508243,   0.0034186607558711,   0.000323904256666548,
         0.0034186607558711,   0.00322986952841047,  0.00178215039218484,
         0.000323904256666548, 0.00178215039218484,  0.0202148376332874;

    I[3] <<
         0.0133198499755109,  -1.85000009038544e-06,  8.69999115245914e-07,
        -1.85000009038544e-06, 0.0125198899767503,  -0.00143893999778065,
         8.69999115245914e-07, -0.00143893999778065, 0.00278827999533888;

    I[4] <<
         0.00490782058915933,  -1.04779487811945e-05, -9.95872429112993e-06,
        -1.04779487811945e-05, 0.00384857289176222,   0.000384368912752211,
        -9.95872429112993e-06,  0.000384368912752211, 0.00258053503197348;

    I[5] <<
         0.000997019998284638, -1.37999999641383e-06,  8.99999979434996e-07,
        -1.37999999641383e-06, 0.00100252999827485,   1.5800000042753e-06,
         8.99999979434996e-07,  1.5800000042753e-06,  0.000447949999234091;

    // -----------------------------
    // 4) 手工建模 6 个 RZ 关节
    // -----------------------------
    std::array<JointIndex, n> joint_ids;
    JointIndex parent = 0;

    for(int i = 0; i < n; ++i)
    {
        SE3 joint_placement = mdhFixedPlacement(alpha[i], a_mdh[i], d_mdh[i]);

        joint_ids[i] = model.addJoint(
            parent,
            JointModelRZ(),
            joint_placement,
            "joint" + std::to_string(i + 1)
        );

        model.appendBodyToJoint(
            joint_ids[i],
            Inertia(m[i], pc[i], I[i]),
            SE3::Identity()
        );

        parent = joint_ids[i];
    }

    Data data(model);

    // -----------------------------
    // 5) 输入 q, dq, ddq
    // -----------------------------
    VectorXd q(model.nq), dq(model.nv), ddq(model.nv);

    q << M_PI * 30.0 / 180.0,
         M_PI * 60.0 / 180.0,
         M_PI * 45.0 / 180.0,
         M_PI * 20.0 / 180.0,
         M_PI * 50.0 / 180.0,
         M_PI * 45.0 / 180.0;

    dq << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
    ddq << 0.01, 0.05, 0.10, 0.20, 0.40, 0.05;

    std::cout << std::fixed << std::setprecision(10);

    std::cout << "================ 模型信息 ================\n";
    std::cout << "model.nq = " << model.nq << "\n";
    std::cout << "model.nv = " << model.nv << "\n";
    std::cout << "gravity  = " << model.gravity.linear().transpose() << "\n\n";

    std::cout << "q   = " << q.transpose() << "\n";
    std::cout << "dq  = " << dq.transpose() << "\n";
    std::cout << "ddq = " << ddq.transpose() << "\n\n";

    // -----------------------------
    // 6) RNEA Benchmark
    // -----------------------------
    constexpr int rounds = 10;
    constexpr int iters_per_round = 100000;

    for(int i = 0; i < 1000; ++i)
    {
        rnea(model, data, q, dq, ddq);
    }

    VectorXd tau(model.nv);
    volatile double checksum = 0.0;
    double total_avg_ns = 0.0;

    std::cout << "================ RNEA Benchmark ================\n";
    std::cout << "rounds          = " << rounds << "\n";
    std::cout << "iters per round = " << iters_per_round << "\n\n";

    for(int round = 0; round < rounds; ++round)
    {
        auto start = std::chrono::steady_clock::now();

        for(int i = 0; i < iters_per_round; ++i)
        {
            tau = rnea(model, data, q, dq, ddq);
            checksum += tau[0];
        }

        auto end = std::chrono::steady_clock::now();

        auto round_total_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

        double avg_ns = static_cast<double>(round_total_ns) / iters_per_round;
        double avg_us = avg_ns / 1000.0;

        total_avg_ns += avg_ns;

        std::cout << "[Round " << (round + 1) << "] "
                  << "总时间 = " << round_total_ns << " ns, "
                  << "平均单次 = " << avg_ns << " ns ("
                  << avg_us << " us)" << "\n";
    }

    double final_avg_ns = total_avg_ns / rounds;
    double final_avg_us = final_avg_ns / 1000.0;

    std::cout << "\n================ 最终结果 ================\n";
    std::cout << "10轮平均单次时间: " << final_avg_ns << " ns" << std::endl;
    std::cout << "10轮平均单次时间: " << final_avg_us << " us" << std::endl;
    std::cout << "checksum = " << checksum << std::endl << std::endl;

    std::cout << "================ RNEA 输出 ================\n";
    std::cout << "tau = " << tau.transpose() << "\n\n";

    for(int i = 0; i < n; ++i)
    {
        const Force &fi = data.f[joint_ids[i]];

        std::cout << "--- Joint " << (i + 1) << " ---\n";
        std::cout << "f   = " << fi.linear().transpose() << "\n";
        std::cout << "n_f = " << fi.angular().transpose() << "\n";
        std::cout << "tau = " << tau[i] << "\n\n";
    }

    return 0;
}