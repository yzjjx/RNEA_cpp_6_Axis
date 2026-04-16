/*该代码用于在FPGA进行综合*/
// #include <ap_fixed.h>      // 定点数
// #include <ap_int.h>        // 整数优化
#include <array>

const int n = 6;

//======================== 齐次变换矩阵 ========================
// 矩阵变换，DH矩阵、旋转矩阵等生成
// 该函数用于计算每一个齐次变换矩阵,输入MDH参数，返回T
// 一维数组展开，传输、计算效率高
std::array<std::array<float,16>,6> Com_MDH_Trans( 
    const std::array<float,6>  alpha,
    const std::array<float,6>  a,
    const std::array<float,6>  d,
    const std::array<float,6>  q,
    const std::array<float,6>  sinq,
    const std::array<float,6>  sin_alpha,
    const std::array<float,6>  cosq,
    const std::array<float,6>  cos_alpha
)
{
    std::array<std::array<float,16>,6> T;
    for(int i = 0;i<n;i++)
    {
        T[i] ={ cosq[i],                 -sinq[i],                   0,              a[i],
                sinq[i]*cos_alpha[i],    cosq[i]*cos_alpha[i],    -sin_alpha[i], -sin_alpha[i]*d[i],
                sinq[i]*sin_alpha[i],    cosq[i]*sin_alpha[i],     cos_alpha[i],  cos_alpha[i]*d[i],
                0,                          0,                           0,              1};
    }
    return T;
}

//该函数用于在齐次变换矩阵中返回旋转矩阵，输入T，返回Rot
std::array<std::array<float,9>,6> Ext_Rot(const std::array<std::array<float,16>,6>& T)
{
    std::array<std::array<float,9>,6> R;
    for(int i = 0;i<n;i++)
    {
        R[i] = {
            T[i][0], T[i][1], T[i][2],
            T[i][4], T[i][5], T[i][6],
            T[i][8], T[i][9], T[i][10]
        };
    }
    return R;
}

//该函数用于在齐次变换矩阵中返回旋转矩阵的转置，输入T，返回Rot_trans
std::array<std::array<float,9>,6> Ext_Rot_trans(const std::array<std::array<float,16>,6>& T)
{
    std::array<std::array<float,9>,6> R_Tran;
    for(int i = 0;i<n;i++)
    {
        R_Tran[i] = {
            T[i][0], T[i][4], T[i][8],
            T[i][1], T[i][5], T[i][9],
            T[i][2], T[i][6], T[i][10]
        };
    }
    return R_Tran;
}

//该函数用于返回位置矩阵P
std::array<std::array<float,3>,6> Ext_Pos(const std::array<std::array<float,16>,6>& T)
{
    std::array<std::array<float,3>,6> P;
    for(int i = 0;i<n;i++)
    {
        P[i] = {
            T[i][3],
            T[i][7],
            T[i][11]
        };
    }
    return P;
}

//======================== RNEA主要力矩计算 ========================


//======================== RNEA摩擦力计算 ========================