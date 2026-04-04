/*该代码用于pinocchio生成DH表和每一个齐次变换矩阵的具体值*/
#include "T_R_Mat_out.h"

// 该函数用于计算每一个齐次变换矩阵,输入MDH参数，返回T
std::vector<Eigen::Matrix4d> Com_MDH_Trans( 
    const Eigen::VectorXd& alpha,
    const Eigen::VectorXd& a,
    const Eigen::VectorXd& d,
    const Eigen::VectorXd& q)
{
    // 检查输入维度是否一致
    if (alpha.size() != a.size() || a.size() != d.size() || d.size() != q.size())
    {
        throw std::invalid_argument("MDH参数的维度必须一致");
    }

    int n = q.size();
    // 根据其次变换矩阵计算具体矩阵，用vector存储n个4*4矩阵
    std::vector<Eigen::Matrix4d> T(n);

    for(int i = 0;i<n;i++)
    {
        T[i] << cos(q(i)),                 -sin(q(i)),                   0,              a(i),
                sin(q(i))*cos(alpha(i)),    cos(q(i))*cos(alpha(i)),    -sin(alpha(i)), -sin(alpha(i))*d(i),
                sin(q(i))*sin(alpha(i)),    cos(q(i))*sin(alpha(i)),     cos(alpha(i)),  cos(alpha(i))*d(i),
                0,                          0,                           0,              1;
    }
    return T;
}

//该函数用于在齐次变换矩阵中返回旋转矩阵，输入T，返回Rot
std::vector<Eigen::Matrix3d> Ext_Rot(int n,const std::vector<Eigen::Matrix4d> & T)
{
    std::vector<Eigen::Matrix3d> R(n) ;
    for(int i = 0;i<n;i++)
    {
        R[i] = T[i].block<3,3>(0,0);
    }
    return R;
}

// int main()
// {
//     // 输入关节数量
//     const int n = 3;

//     // 在此编辑DH表，MDH
//     // alpha
//     Eigen::Matrix<double, n, 1> alpha;
//     alpha << 0, 0, 0;
//     // a
//     Eigen::Matrix<double, n, 1> a;
//     a << 0, 5, 10;
//     // d
//     Eigen::Matrix<double, n, 1> d;
//     d << 0, 0, 0;

//     // 输入关节角度（在实际计算中，关节角度需要接受外界未知量）
//     Eigen::Matrix<double, n, 1> q;
//     q << M_PI, M_PI , M_PI;

//     // 调用函数
//     std::vector<Eigen::Matrix4d> T = Com_MDH_Trans(alpha, a, d, q);

//     std::vector<Eigen::Matrix3d> R= Ext_Rot(n,T);

//     // 输出结果
//     for (int i = 0; i < n; i++)
//     {
//         std::cout << "T[" << i << "] = \n" << T[i] << "\n\n";
//         std::cout << "R[" << i << "] = \n" << R[i] << "\n\n";
//     }
//     return 0;
// }