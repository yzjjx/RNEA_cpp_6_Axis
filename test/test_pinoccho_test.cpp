#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/joint/joint-revolute.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <cmath>

int main()
{
    using namespace pinocchio;
    using Eigen::Matrix3d;
    using Eigen::Vector3d;
    using Eigen::VectorXd;

    Model model;

    // -----------------------------
    // 1) 重力设置
    // -----------------------------
    model.gravity.linear()  << 0.0, 0.0, 9.8;
    model.gravity.angular() << 0.0, 0.0,  0.0;

    // -----------------------------
    // 2) 你的参数
    // -----------------------------
    const int n = 3;

    const Vector3d pc[3] = {
        Vector3d( 1.09 , -0.008, -0.092),
        Vector3d( 0.17 , -0.017, -0.070),
        Vector3d(-0.001,  0.100, -0.006)
    };

    const double m[3] = {3.0, 4.0, 5.0};

    Matrix3d I[3];
    I[0] <<
        0.0311679899460595,  -6.49999998879087e-07, -4.26999999258665e-06,
       -6.49999998879087e-07, 0.0307127099468431,  -0.00223655999611672,
       -4.26999999258665e-06, -0.00223655999611672, 0.00587739998995415;

    I[1] <<
         0.0179531824691758,  0.0309224033095523,  -0.00240843390062569,
         0.0309224033095523,  0.132852985273582,    0.00071426442226283,
        -0.00240843390062569, 0.00071426442226283,  0.138139839236999;

    I[2] <<
         0.0205839772508243,   0.0034186607558711,   0.000323904256666548,
         0.0034186607558711,   0.00322986952841047,  0.00178215039218484,
         0.000323904256666548, 0.00178215039218484,  0.0202148376332874;

    // 按你前面的写法：alpha = 0, d = 0，只保留 a
    const double a1 = 0.0;
    const double a2 = 5.0;
    const double a3 = 10.0;

    // -----------------------------
    // 3) 建模
    // joint_placement = 父关节到当前关节的固定安装位姿
    // -----------------------------
    const SE3 M1(Eigen::Matrix3d::Identity(), Vector3d(a1, 0.0, 0.0));
    const SE3 M2(Eigen::Matrix3d::Identity(), Vector3d(a2, 0.0, 0.0));
    const SE3 M3(Eigen::Matrix3d::Identity(), Vector3d(a3, 0.0, 0.0));

    JointIndex j1 = model.addJoint(0,  JointModelRZ(), M1, "joint1");
    model.appendBodyToJoint(j1, Inertia(m[0], pc[0], I[0]), SE3::Identity());

    JointIndex j2 = model.addJoint(j1, JointModelRZ(), M2, "joint2");
    model.appendBodyToJoint(j2, Inertia(m[1], pc[1], I[1]), SE3::Identity());

    JointIndex j3 = model.addJoint(j2, JointModelRZ(), M3, "joint3");
    model.appendBodyToJoint(j3, Inertia(m[2], pc[2], I[2]), SE3::Identity());

    Data data_fk(model);
    Data data_rnea(model);
    Data data_g(model);
    Data data_nle(model);

    // -----------------------------
    // 4) 输入 q, dq, ddq
    // q: rad, dq: rad/s, ddq: rad/s^2
    // -----------------------------
    VectorXd q(model.nq), v(model.nv), a(model.nv);

    q << M_PI * 30.0 / 180.0,
         M_PI * 60.0 / 180.0,
         M_PI * 45.0 / 180.0;

    v << 1.0, 2.0, 3.0;
    a << 0.01, 0.05, 0.10;

    std::cout << std::fixed << std::setprecision(10);

    std::cout << "================ 模型信息 ================\n";
    std::cout << "model.nq = " << model.nq << "\n";
    std::cout << "model.nv = " << model.nv << "\n";
    std::cout << "gravity  = " << model.gravity.linear().transpose() << "\n\n";

    std::cout << "q   = " << q.transpose() << "\n";
    std::cout << "dq  = " << v.transpose() << "\n";
    std::cout << "ddq = " << a.transpose() << "\n\n";

    // =========================================================
    // A. 外推：forwardKinematics
    // =========================================================
    forwardKinematics(model, data_fk, q, v, a);

    std::cout << "================ 外推结果（含重力版） ================\n";
    for(int i = 0; i < n; ++i)
    {
        JointIndex jid = static_cast<JointIndex>(i + 1);

        // 关节局部速度
        Motion vj = getVelocity(model, data_fk, jid, LOCAL);

        // 关节局部空间加速度
        Motion aj_spatial = getAcceleration(model, data_fk, jid, LOCAL);

        // 关节局部经典加速度（纯运动学）
        Motion aj_classical = getClassicalAcceleration(model, data_fk, jid, LOCAL);

        Vector3d omega   = vj.angular();
        Vector3d d_omega = aj_spatial.angular();

        // 纯运动学经典线加速度
        Vector3d d_v_kin = aj_classical.linear();

        // 把世界系重力变到该关节局部系
        Vector3d g_local = data_fk.oMi[jid].rotation().transpose() * model.gravity.linear();

        // 与你手写牛顿欧拉更接近的 d_v（加入重力）
        Vector3d d_v = d_v_kin - g_local;

        // 质心线加速度（加入重力）
        Vector3d d_v_c = d_v
                       + d_omega.cross(pc[i])
                       + omega.cross(omega.cross(pc[i]));

        // 连杆力 / 连杆力矩
        Vector3d F = m[i] * d_v_c;
        Vector3d N = I[i] * d_omega + omega.cross(I[i] * omega);

        std::cout << "--- Link " << (i + 1) << " ---\n";
        std::cout << "omega      = " << omega.transpose()   << "\n";
        std::cout << "d_omega    = " << d_omega.transpose() << "\n";
        std::cout << "d_v_kin    = " << d_v_kin.transpose() << "\n";
        std::cout << "g_local    = " << g_local.transpose() << "\n";
        std::cout << "d_v        = " << d_v.transpose()     << "\n";
        std::cout << "d_v_c      = " << d_v_c.transpose()   << "\n";
        std::cout << "F          = " << F.transpose()       << "\n";
        std::cout << "N          = " << N.transpose()       << "\n\n";
    }

    // =========================================================
    // B. 内推：RNEA
    // =========================================================
    VectorXd tau_full = rnea(model, data_rnea, q, v, a);

    std::cout << "================ 内推结果 ================\n";
    for(int i = 0; i < n; ++i)
    {
        JointIndex jid = static_cast<JointIndex>(i + 1);

        const Force &fi = data_rnea.f[jid];

        Vector3d f_linear  = fi.linear();   // 合力
        Vector3d n_angular = fi.angular();  // 合力矩

        std::cout << "--- Joint " << (i + 1) << " ---\n";
        std::cout << "f   = " << f_linear.transpose()  << "\n";
        std::cout << "n_f = " << n_angular.transpose() << "\n";
        std::cout << "tau = " << tau_full[i]           << "\n\n";
    }

    // =========================================================
    // C. 分项验证：重力项 / 非线性项 / 全项
    // =========================================================
    VectorXd tau_g = computeGeneralizedGravity(model, data_g, q);
    VectorXd tau_nle = rnea(model, data_nle, q, v, VectorXd::Zero(model.nv));

    std::cout << "================ 力矩分项 ================\n";
    std::cout << "tau_g    = " << tau_g.transpose()   << "\n";
    std::cout << "tau_nle  = " << tau_nle.transpose() << "\n";
    std::cout << "tau_full = " << tau_full.transpose() << "\n\n";

    // =========================================================
    // D. 末端总输出
    // =========================================================
    std::cout << "================ 结束 ================\n";
    return 0;
}