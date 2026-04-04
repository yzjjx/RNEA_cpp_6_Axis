/*该函数用来测试scr的函数是否能在外部调用*/
#include "T_R_Mat_out.h"

int main()
{
    // 输入关节数量
    const int n = 3;

    // 在此编辑DH表，MDH
    // alpha
    Eigen::Matrix<double, n, 1> alpha;
    alpha << 0, 0, 0;
    // a
    Eigen::Matrix<double, n, 1> a;
    a << 0, 5, 10;
    // d
    Eigen::Matrix<double, n, 1> d;
    d << 0, 0, 0;

    // 输入关节角度（在实际计算中，关节角度需要接受外界未知量）
    Eigen::Matrix<double, n, 1> q;
    q << M_PI, M_PI , M_PI;

    // 调用函数
    std::vector<Eigen::Matrix4d> T = Com_MDH_Trans(alpha, a, d, q);

    std::vector<Eigen::Matrix3d> R= Ext_Rot(n,T);

    // 输出结果
    for (int i = 0; i < n; i++)
    {
        std::cout << "T[" << i << "] = \n" << T[i] << "\n\n";
        std::cout << "R[" << i << "] = \n" << R[i] << "\n\n";
    }
    return 0;
}