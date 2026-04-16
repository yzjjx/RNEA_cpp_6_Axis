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
        Eigen::Vector3d(3.48369999434000e-05/3.16699999486378,
                        -0.0253106639589312/3.16699999486378,
                        -0.291864385526951/3.16699999486378),
        Eigen::Vector3d(0.659449401868420/3.89505775503759,
                        -0.0651938216320210/3.89505775503759,
                        -0.271940703633802/3.89505775503759),
        Eigen::Vector3d(-0.00157764514450908/1.55161965449885,
                        0.153969960151856/1.55161965449885,   
                        -0.00930606042947241/1.55161965449885),
        Eigen::Vector3d(-4.70521423066045e-05/1.88199999547530,
                        -0.0205570880251570/1.88199999547530,
                        -0.134124493621368/1.88199999547530),
        Eigen::Vector3d(-0.00190221636650860/1.34621855234916,
                        0.0159457855937173/1.34621855234916,
                        -0.0281550840659444/1.34621855234916),
        Eigen::Vector3d(4.62815646042645e-05/0.633999997351584,
                        5.00861139672133e-05/0.633999997351584,
                        -0.0284368017495592/0.633999997351584)
                    };

    // 输入各个连杆质量，用于计算力
    const double m[n] = {3.16699999486378,3.89505775503759,1.55161965449885,1.8819999954753,1.34621855234916,0.633999997351584};

    // 输入各个连杆的惯性张量矩阵，用于计算力矩
    const Eigen::Matrix3d I[n] = {
        (Eigen::Matrix3d() <<
            0.058267910813839,  -3.715826953316560e-07, -1.05949175179955e-06,
        -3.715826953316560e-07, 0.0576103483714700,  -0.00456914016524626,
        -1.05949175179955e-06, -0.00456914016524626, 0.00607968319952077
        ).finished(),

        (Eigen::Matrix3d() <<
            0.0380304151034530,   0.0419599871875192,  0.0436322529474843,
            0.0419599871875192,   0.263486545009477,   -0.00383736352324302,
        0.0436322529474843,  -0.00383736352324302,  0.250878539307793
        ).finished(),

        (Eigen::Matrix3d() <<
           0.0359185029001311,   0.00357521327138579,  0.000314442104644629,
            0.00357521327138579,  0.00328728806178666,  0.00270560721315391,
           0.000314442104644629, 0.00270560721315391,  0.0354951529635439
        ).finished(),

        (Eigen::Matrix3d() <<
            0.0231030453535813,  -2.36395069266735e-06,  -2.48326590911455e-06,
        -2.36395069266735e-06, 0.0220785414360184,  -0.00290398198945597,
            -2.48326590911455e-06, -0.00290398198945597, 0.00301282626685543
        ).finished(),

        (Eigen::Matrix3d() <<
            0.00568553742802938, 1.20535594054042e-05, -4.97420578495685e-05,
       1.20535594054042e-05, 0.00444010179654207,   0.000717862262034322,
         -4.97420578495685e-05,  0.000717862262034322, 0.00277209865650963
        ).finished(),

        (Eigen::Matrix3d() <<
            0.00227249981806468, -1.38365624835237e-06,  2.97586698701961e-06,
        -1.38365624835237e-06, 0.00227800923976532,   3.82651246364623e-06,
            2.97586698701961e-06,  3.82651246364623e-06,  0.000447957334568561
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
        0.403112,
        -0.05,
        0.0,
        0.0;

    Eigen::Matrix<double, n, 1> d;
    d << 0.328,
        0.0,
        0.4,
        0.4,
        -0.136,
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
        // std::cout << "T[" << i << "] = \n" << T[i] << "\n\n";
        // std::cout << "R[" << i << "] = \n" << R[i] << "\n\n";
        // std::cout << "R_n[" << i << "] = \n" << R_n[i] << "\n\n";
    }
    for (int i = 0; i < n+1; i++)
    {
        std::cout << "omega[" << i << "] = \n" << omega[i] << "\n\n";
        // std::cout << "d_omega[" << i << "] = \n" << d_omega[i] << "\n\n";
        // std::cout << "d_v[" << i << "] = \n" << d_v[i] << "\n\n";
        // std::cout << "d_v_c[" << i << "] = \n" << d_v_c[i] << "\n\n";
        // std::cout << "F[" << i << "] = \n" << F[i] << "\n\n";
        // std::cout << "N[" << i << "] = \n" << N[i] << "\n\n";
    }
    // 内推结果
    // for (int i = 0; i < n+1; i++)
    // {
    //     std::cout << "f[" << i << "] = \n" << f[i] << "\n\n";
    //     std::cout << "n_f[" << i << "] = \n" << n_f[i] << "\n\n";
    // }
    // // 最终计算关节力矩结果
    // for (int i = 0; i < n+1; i++)
    // {
    //     std::cout << "tau[" << i << "] = \n" << tau[i] << "\n\n";
    // }

    return 0;
}