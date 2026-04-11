/*该代码用于实现牛顿欧拉法的前向递推过程，将计算结果与手算结果以及Pinocchio结果进行对比*/
/*该代码在三自由度机器人验证正确*/
#include <Eigen/Dense>
#include <iostream>
#include "T_R_Mat_out.h"

int main()
{
    const Eigen::Vector3d Z(0, 0, 1);

    // 输入关节数量
    const int n = 6;

    // 输入各个连杆质心坐标向量，用于计算质心线加速度
    const Eigen::Vector3d P_c[n] = {
        Eigen::Vector3d(1.09999999999679e-05 ,-0.00799199999999364, -0.0921580000000931),
        Eigen::Vector3d(0.169304139589595 ,-0.0167375750841445, -0.0698168604257776),
        Eigen::Vector3d(-0.00101677311184785 ,0.0992317670799201, -0.00599764278732221),
        Eigen::Vector3d(-2.50011383739252e-05 ,-0.0109230011023275, -0.071266999970154),
        Eigen::Vector3d(-0.00141300709545952 ,0.0118448713738878,-0.0209142000136706),
        Eigen::Vector3d(7.29993135608786e-05, 7.90001800890199e-05, -0.044852999792348)
    };

    // 输入各个连杆质量，用于计算力
    const double m[n] = {3.16699999486378,3.89505775503759,1.55161965449885,1.8819999954753,1.34621855234916,0.633999997351584};

    // 输入各个连杆的惯性张量矩阵，用于计算力矩
    const Eigen::Matrix3d I[n] = {
        (Eigen::Matrix3d() <<
            0.0311679899460595,  -6.49999998879087e-07, -4.26999999258665e-06,
        -6.49999998879087e-07, 0.0307127099468431,  -0.00223655999611672,
        -4.26999999258665e-06, -0.00223655999611672, 0.00587739998995415
        ).finished(),

        (Eigen::Matrix3d() <<
            0.0179531824691758,   0.0309224033095523,  -0.00240843390062569,
            0.0309224033095523,   0.132852985273582,    0.00071426442226283,
        -0.00240843390062569,  0.00071426442226283,  0.138139839236999
        ).finished(),

        (Eigen::Matrix3d() <<
            0.0205839772508243,   0.0034186607558711,   0.000323904256666548,
            0.0034186607558711,   0.00322986952841047,  0.00178215039218484,
            0.000323904256666548, 0.00178215039218484,  0.0202148376332874
        ).finished(),

        (Eigen::Matrix3d() <<
            0.0133198499755109,  -1.85000009038544e-06,  8.69999115245914e-07,
        -1.85000009038544e-06, 0.0125198899767503,  -0.00143893999778065,
            8.69999115245914e-07, -0.00143893999778065, 0.00278827999533888
        ).finished(),

        (Eigen::Matrix3d() <<
            0.00490782058915933,  -1.04779487811945e-05, -9.95872429112993e-06,
        -1.04779487811945e-05, 0.00384857289176222,   0.000384368912752211,
        -9.95872429112993e-06,  0.000384368912752211, 0.00258053503197348
        ).finished(),

        (Eigen::Matrix3d() <<
            0.000997019998284638, -1.37999999641383e-06,  8.99999979434996e-07,
        -1.37999999641383e-06, 0.00100252999827485,   1.5800000042753e-06,
            8.99999979434996e-07,  1.5800000042753e-06,  0.000447949999234091
        ).finished()
    };

    // 在此编辑DH表，MDH
    // MDH 参数
    Eigen::Matrix<double, n, 1> alpha;
    alpha << 0.0,
            -M_PI / 2.0,
            M_PI,
            -M_PI / 2.0,
            M_PI / 2.0,
            -M_PI / 2.0;

    Eigen::Matrix<double, n, 1> a;
    a << 0.0,
        0.0,
        0.05,
        -0.05,
        0.0,
        0.0;

    Eigen::Matrix<double, n, 1> d;
    d << 0.355,
        0.0,
        0.4,
        0.4,
        0.136,
        0.1035;

    // 输入关节角度（在实际计算中，关节角度需要接受外界未知量）
    Eigen::Matrix<double, n, 1> q;
    q << M_PI*30/180, M_PI*60/180 , M_PI*45/180,M_PI*20/180, M_PI*50/180 , M_PI*45/180;

    // 输入关节角速度
    Eigen::Matrix<double,n,1> dq;
    dq << 1,2,3,4,5,6;

    // 输入关节角加速度
    Eigen::Matrix<double,n,1> ddq;
    ddq << 0.01,0.05,0.1,0.2,0.4,0.05;

    // 计算齐次变换矩阵
    std::vector<Eigen::Matrix4d> T = Com_MDH_Trans(alpha, a, d, q);
    // 计算旋转矩阵
    std::vector<Eigen::Matrix3d> R= Ext_Rot_trans(n,T);
    // 计算平移矩阵
    std::vector<Eigen::Vector3d> P= Ext_Pos(n,T);

    // 计算内推所需旋转矩阵
    std::vector<Eigen::Matrix3d> R_n= Ext_Rot(n,T);

    // 底座连杆角速度，连杆角速度计算
    std::vector<Eigen::Vector3d> omega;
    omega.resize(n+1);
    for(int i = 0; i < n+1; i++)
    {
        omega[i] = Eigen::Vector3d::Zero();
    }
    // 底座连杆角加速度，连杆角加速度计算
    std::vector<Eigen::Vector3d> d_omega;
    d_omega.resize(n+1);
    for(int i = 0; i < n+1; i++)
    {
        d_omega[i] = Eigen::Vector3d::Zero();
    }
    // 线加速度计算
    std::vector<Eigen::Vector3d> d_v;
    d_v.resize(n+1);
    for(int i = 0; i < n+1; i++)
    {
        d_v[i] = Eigen::Vector3d::Zero();
    }
    // 质心线加速度计算
    std::vector<Eigen::Vector3d> d_v_c;
    d_v_c.resize(n+1);
    for(int i = 0; i < n+1; i++)
    {
        d_v_c[i] = Eigen::Vector3d::Zero();
    }
    // 连杆力计算
    std::vector<Eigen::Vector3d> F;
    F.resize(n+1);
    for(int i = 0; i < n+1; i++)
    {
        F[i] = Eigen::Vector3d::Zero();
    }
    // 连杆扭矩计算
    std::vector<Eigen::Vector3d> N;
    N.resize(n+1);
    for(int i = 0; i < n+1; i++)
    {
        N[i] = Eigen::Vector3d::Zero();
    }

    // 内推
    // 连杆总力计算
    std::vector<Eigen::Vector3d> f;
    f.resize(n+1);
    for(int i = 0; i < n+1; i++)
    {
        f[i] = Eigen::Vector3d::Zero();
    }
    // 连杆总力矩计算
    std::vector<Eigen::Vector3d> n_f;
    n_f.resize(n+1);
    for(int i = 0; i < n+1; i++)
    {
        n_f[i] = Eigen::Vector3d::Zero();
    }
    // 连杆力矩计算
    std::vector<double> tau(n+1, 0.0);

    // 底座角速度
    omega[0] = Eigen::Vector3d::Zero();
    // 底座角加速度
    d_omega[0] = Eigen::Vector3d::Zero();
    // 底座线加速度
    d_v[0] = Eigen::Vector3d(0.0, 0.0, 9.8);

    // 主要计算循环,RNEA外推计算，与pinocchio比较，外推结果全部正确
    for(int i = 0;i < n;i++)
    {
        // 连杆角速度
        omega[i+1] = R[i]*omega[i]+dq[i]*Z;
        // 连杆角加速度
        d_omega[i+1] = R[i]*d_omega[i] + (R[i]*omega[i]).cross(dq[i]*Z)+ddq[i]*Z;
        // 线加速度
        d_v[i+1] = R[i]*(d_omega[i].cross(P[i])+omega[i].cross(omega[i].cross(P[i]))+d_v[i]);
        // 质心线加速度
        d_v_c[i+1] = d_omega[i+1].cross(P_c[i])+omega[i+1].cross(omega[i+1].cross(P_c[i]))+d_v[i+1];
        // 连杆力
        F[i+1] = m[i]*d_v_c[i+1];
        // 连杆力矩
        N[i+1] = I[i]*d_omega[i+1]+omega[i+1].cross(I[i]*omega[i+1]);
    }


    // 主要计算循环，RNEA内推
    for (int i = n; i > 0 ; i--)
    {
        // 连杆总力
        f[i] = R_n[i] * f[i + 1] + F[i];
        // 连杆总力矩
        n_f[i] =N[i]+ R_n[i] * n_f[i + 1]+ P_c[i-1].cross(F[i])
                + P[i].cross(R_n[i] * f[i + 1]);
        //关节输出力矩
        tau[i] = n_f[i].dot(Z);
    }
    

    // 输出结果
    for (int i = 0; i < n; i++)
    {
        std::cout << "T[" << i << "] = \n" << T[i] << "\n\n";
        std::cout << "R[" << i << "] = \n" << R[i] << "\n\n";
        std::cout << "R_n[" << i << "] = \n" << R_n[i] << "\n\n";
    }
    for (int i = 0; i < n+1; i++)
    {
        std::cout << "omega[" << i << "] = \n" << omega[i] << "\n\n";
        std::cout << "d_omega[" << i << "] = \n" << d_omega[i] << "\n\n";
        std::cout << "d_v[" << i << "] = \n" << d_v[i] << "\n\n";
        std::cout << "d_v_c[" << i << "] = \n" << d_v_c[i] << "\n\n";
        std::cout << "F[" << i << "] = \n" << F[i] << "\n\n";
        std::cout << "N[" << i << "] = \n" << N[i] << "\n\n";
    }
    // 内推结果
    for (int i = 0; i < n+1; i++)
    {
        std::cout << "f[" << i << "] = \n" << f[i] << "\n\n";
        std::cout << "n_f[" << i << "] = \n" << n_f[i] << "\n\n";
    }
    // 最终计算关节力矩结果
    for (int i = 0; i < n+1; i++)
    {
        std::cout << "tau[" << i << "] = \n" << tau[i] << "\n\n";
    }

    return 0;
}