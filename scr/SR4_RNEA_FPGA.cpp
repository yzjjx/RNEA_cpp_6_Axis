/*该代码用于在FPGA进行综合*/
// #include <ap_fixed.h>      // 定点数
// #include <ap_int.h>        // 整数优化
#include <array>

typedef double data_t;

// 定义常量
const int n = 6;

static const data_t Z[3] = {0.0, 0.0, 1.0};

// 输入各个连杆质量，用于计算力
static const data_t m[n] = {3.16699999486378,
                            3.89505775503759,
                            1.55161965449885,
                            1.88199999547530,
                            1.34621855234916,
                            0.633999997351584};

// 输入各个连杆质心坐标向量，用于计算质心线加速度
static const data_t P_c[n][3] = {
                    {3.48369999434000e-05/m[0],
                    -0.0253106639589312/m[0],
                    -0.291864385526951/m[0]},

                    {0.659449401868420/m[1],
                    -0.0651938216320210/m[1],
                    -0.271940703633802/m[1]},

                    {-0.00157764514450908/m[2],
                    0.153969960151856/m[2],   
                    -0.00930606042947241/m[2]},

                    {-4.70521423066045e-05/m[3],
                    -0.0205570880251570/m[3],
                    -0.134124493621368/m[3]},

                    {-0.00190221636650860/m[4],
                    0.0159457855937173/m[4],
                    -0.0281550840659444/m[4]},
                    
                    {4.62815646042645e-05/m[5],
                    5.00861139672133e-05/m[5],
                    -0.0284368017495592/m[5]}
};

// 输入各个连杆的惯性张量矩阵，用于计算力矩
static const data_t I[n][9] = {
    {
        0.058267910813839,      -3.715826953316560e-07, -1.05949175179955e-06,
        -3.715826953316560e-07, 0.0576103483714700,     -0.00456914016524626,
        -1.05949175179955e-06,  -0.00456914016524626,   0.00607968319952077
    },

    {
        0.0380304151034530,   0.0419599871875192,  0.0436322529474843,
        0.0419599871875192,   0.263486545009477,   -0.00383736352324302,
        0.0436322529474843,  -0.00383736352324302,  0.250878539307793
    },

    {
        0.0359185029001311,   0.00357521327138579,  0.000314442104644629,
        0.00357521327138579,  0.00328728806178666,  0.00270560721315391,
        0.000314442104644629, 0.00270560721315391,  0.0354951529635439
    },

    {
        0.0231030453535813,     -2.36395069266735e-06,  -2.48326590911455e-06,
        -2.36395069266735e-06,  0.0220785414360184,     -0.00290398198945597,
        -2.48326590911455e-06,  -0.00290398198945597,   0.00301282626685543
    },

    {
        0.00568553742802938,    1.20535594054042e-05, -4.97420578495685e-05,
        1.20535594054042e-05,   0.00444010179654207,   0.000717862262034322,
        -4.97420578495685e-05,  0.000717862262034322,  0.00277209865650963
    },

    {
        0.00227249981806468,    -1.38365624835237e-06,  2.97586698701961e-06,
        -1.38365624835237e-06,  0.00227800923976532,    3.82651246364623e-06,
        2.97586698701961e-06,   3.82651246364623e-06,   0.000447957334568561
    }
};

// 在此编辑DH表，MDH
// MDH 参数
static const data_t sin_alpha[n] = {
     0.0f,
    -1.0f,
     0.0f,
    -1.0f,
     1.0f,
    -1.0f
};

static const data_t cos_alpha[n] = {
     1.0f,
     0.0f,
    -1.0f,
     0.0f,
     0.0f,
     0.0f
};

static const data_t a[n] = {
    0.0,
    0.0,
    0.403112,
    -0.05,
    0.0,
    0.0
};

static const data_t d[n] = {
    0.328,
    0.0,
    0.4,
    0.4,
    -0.136,
    0.1035
};

//===============摩擦参数===============
// 关节非线性摩擦参数
static const data_t Fc[n] = {39.3156017014124,
                            5.74376743911912,
                            11.0817930270772,
                            1.30308256888117,
                            1.33226719773913,
                            1.57685226136080};

static const data_t Fs[n] = {-2.53722774557035,
                            -2.07595046391569,
                            0.155945016604755,
                            -0.432048786756766,
                            -0.299954735443126,
                            -1.12558099217137};

static const data_t vs[n] = {6159.98349836929,
                            -0.00542827529961853,
                            -4.99999961405425,
                            0.000166612161014179,
                            0.000335859662883711,
                            0.00125116525892426};

static const data_t Fv[n] = {5.67202325533792,
                            9.50541644780893,
                            3.62367344786894,
                            2.01161766835989,
                            1.77753031609100,
                            1.84035495310709};

static const data_t delta_s[n] = {0.145914867459757,
                                0.272713039894872,
                                0.335189700243761,
                                1.99565622087150,
                                1.99730531138284,
                                0.323686309612415};

//======================== 齐次变换矩阵 ========================
// 矩阵变换，DH矩阵、旋转矩阵等生成
// 该函数用于计算每一个齐次变换矩阵,输入MDH参数，返回T
// 一维数组展开，传输、计算效率高，data_t T[6][16]
// HLS下稳妥的写法是输入数组加输出数组
void Com_MDH_Trans( 
    const data_t  a[n],
    const data_t  d[n],
    const data_t  q[n],
    const data_t  sinq[n],
    const data_t  sin_alpha[n],
    const data_t  cosq[n],
    const data_t  cos_alpha[n],
    data_t T[n][16]
)
{
    for(int i = 0;i<n;i++)
    {
         T[i][0] = cosq[i];                 T[i][1] = -sinq[i];                 T[i][2] = 0;                T[i][3] = a[i];
         T[i][4] = sinq[i]*cos_alpha[i];    T[i][5] = cosq[i]*cos_alpha[i];     T[i][6] = -sin_alpha[i];    T[i][7] = -sin_alpha[i]*d[i];
         T[i][8] = sinq[i]*sin_alpha[i];    T[i][9] = cosq[i]*sin_alpha[i];     T[i][10] = cos_alpha[i];    T[i][11] = cos_alpha[i]*d[i];
         T[i][12] =0;                       T[i][13] = 0;                       T[i][14] = 0;               T[i][15] = 1;
    }
}

//该函数用于在齐次变换矩阵中返回旋转矩阵，输入T，返回Rot
void Ext_Rot(const data_t T[n][16],data_t R[n+1][9])
{
    for(int i = 0;i<n;i++)
    {
        R[i][0] =  T[i][0];   R[i][1] = T[i][1];  R[i][2] = T[i][2];
        R[i][3] =  T[i][4];   R[i][4] = T[i][5];  R[i][5] = T[i][6];
        R[i][6] =  T[i][8];   R[i][7] = T[i][9];  R[i][8] = T[i][10];
    };
    // 为了防止内推动越界
    R[n][0] =  0;   R[n][1] = 0;  R[n][2] = 0;
    R[n][3] =  0;   R[n][4] = 0;  R[n][5] = 0;
    R[n][6] =  0;   R[n][7] = 0;  R[n][8] = 0;
}



//该函数用于在齐次变换矩阵中返回旋转矩阵的转置，输入T，返回Rot_trans
void Ext_Rot_trans(const data_t T[n][16],data_t R_Tran[n][9])
{
    for(int i = 0;i<n;i++)
    {
        R_Tran[i][0] =  T[i][0];   R_Tran[i][1] = T[i][4];  R_Tran[i][2] = T[i][8];
        R_Tran[i][3] =  T[i][1];   R_Tran[i][4] = T[i][5];  R_Tran[i][5] = T[i][9];
        R_Tran[i][6] =  T[i][2];   R_Tran[i][7] = T[i][6];  R_Tran[i][8] = T[i][10];
    };
}

//该函数用于返回位置矩阵P
void Ext_Pos(const data_t T[n][16],data_t P[n+1][3])
{
    for(int i = 0;i<n;i++)
    {
        P[i][0] =  T[i][3];  
        P[i][1] =  T[i][7]; 
        P[i][2] =  T[i][11];  
    };
    //防止越界
    P[n][0] =  0;  
    P[n][1] =  0; 
    P[n][2] =  0;  
}

// ====================== 静态数组的矩阵运算 ========================
// 静态数组的矩阵乘法运算(9元素旋转矩阵*3元素向量)
void mat_9x3_vec(
    const data_t A[9],
    const data_t x[3],
    data_t y[3]
)
{
    for (int i = 0; i < 3; i++)
    {
        y[i] = 0.0;
        for (int k = 0; k < 3; k++)
        {
            y[i] += A[i * 3 + k] * x[k];
        }
    }
}
// 叉乘计算
void cross_3x1(
    const data_t a[3],
    const data_t b[3],
    data_t c[3]
)
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}


//======================== RNEA主要力矩计算 ========================
// 主要计算循环,RNEA外推计算
void SR4_rnea_hls(
    const data_t q[n],
    const data_t dq[n],
    const data_t ddq[n],
    const data_t sinq[n],
    const data_t cosq[n],
    data_t omega[n+1][3],
    data_t d_omega[n+1][3],
    data_t d_v[n+1][3],
    data_t d_v_c[n+1][3],
    data_t F[n+1][3],
    data_t N[n+1][3],
    data_t f[n+2][3],
    data_t n_f[n+2][3],
    data_t tau[n+2]
)
{
    // 初始化
    data_t T[n][16];
    data_t R[n+1][9];
    data_t R_T[n][9];
    data_t P[n+1][3];

    // 计算齐次变换矩阵
    Com_MDH_Trans(a,d,q,sinq,sin_alpha,cosq,cos_alpha,T);
    // 计算旋转矩阵
    Ext_Rot_trans(T,R_T);
    // 计算平移矩阵
    Ext_Pos(T,P);
    // 计算内推所需旋转矩阵
    Ext_Rot(T,R);

    // 初始化结果矩阵
    // data_t omega[n+1][3];   // 多开一行，omega[0]作为基座角速度
    // 基座角速度 = 0
    omega[0][0] = 0.0;
    omega[0][1] = 0.0;
    omega[0][2] = 0.0;
    // 基座角加速度为0
    d_omega[0][0] = 0.0;
    d_omega[0][1] = 0.0;
    d_omega[0][2] = 0.0;
    // 基座线加速度为0
    d_v[0][0] = 0.0;
    d_v[0][1] = 0.0;
    d_v[0][2] = 9.8;
    // 质心线加速度为0
    d_v_c[0][0] = 0.0;
    d_v_c[0][1] = 0.0;
    d_v_c[0][2] = 0.0;
    // 连杆力为0
    F[0][0] = 0.0;
    F[0][1] = 0.0;
    F[0][2] = 0.0;
    // 连杆力矩为0
    N[0][0] = 0.0;
    N[0][1] = 0.0;
    N[0][2] = 0.0;
    // //    data_t f[n+1][3],
    // data_t n_f[n+1][3],
    // data_t tau[n+1][3]
    f[n+1][0] = 0.0;
    f[n+1][1] = 0.0;
    f[n+1][2] = 0.0;

    n_f[n+1][0] = 0.0;
    n_f[n+1][1] = 0.0;
    n_f[n+1][2] = 0.0;

    tau[n+1] = 0.0;


    for(int i = 0;i < n;i++)
    {
        // 连杆角速度计算
        data_t tmp[3];
        // tmp = R_T[i] * omega[i]
        mat_9x3_vec(R_T[i], omega[i], tmp);
        // omega[i+1] = tmp + Z * dq[i]
        omega[i+1][0] = tmp[0];
        omega[i+1][1] = tmp[1];
        omega[i+1][2] = tmp[2] + dq[i];

        // 连杆角加速度计算
        data_t d_omega_tmp[3];
        data_t d_omega_2[3];

        mat_9x3_vec(R_T[i],d_omega[i],d_omega_tmp);
        d_omega_2[0] = 0;
        d_omega_2[1] = 0;
        d_omega_2[2] = dq[i];

        data_t cross_1[3];
        cross_3x1(tmp,d_omega_2,cross_1);

        d_omega[i+1][0] = d_omega_tmp[0]+cross_1[0];
        d_omega[i+1][1] = d_omega_tmp[1]+cross_1[1];
        d_omega[i+1][2] = d_omega_tmp[2] +cross_1[2]+ddq[i];

        // 线加速度计算
        // d_v[i+1] = R[i]*(d_omega[i].cross(P[i])+omega[i].cross(omega[i].cross(P[i]))+d_v[i]);
        //cross_dv_1 = d_omega[i].cross(P[i])
        data_t cross_dv_1[3];
        cross_3x1(d_omega[i],P[i],cross_dv_1);
        //cross_dv_2 = omega[i].cross(P[i])
        data_t cross_dv_2[3];
        cross_3x1(omega[i],P[i],cross_dv_2);
        //cross_dv_3 = omega[i].cross(omega[i].cross(P[i]))
        data_t cross_dv_3[3];
        cross_3x1(omega[i],cross_dv_2,cross_dv_3);
        // add_dv = d_omega[i].cross(P[i])+omega[i].cross(omega[i].cross(P[i]))+d_v[i]
        data_t add_dv[3];
        add_dv[0] = cross_dv_1[0]+cross_dv_3[0]+d_v[i][0];
        add_dv[1] = cross_dv_1[1]+cross_dv_3[1]+d_v[i][1];
        add_dv[2] = cross_dv_1[2]+cross_dv_3[2]+d_v[i][2];
        // 最终结果计算
        mat_9x3_vec(R_T[i],add_dv,d_v[i+1]);

        // 质心线加速度
        // cross_dvc_1 = d_omega[i+1].cross(P_c[i])
        data_t cross_dvc_1[3];
        cross_3x1(d_omega[i+1],P_c[i],cross_dvc_1);
        //cross_dvc_2 = omega[i+1].cross(P_c[i])
        data_t cross_dvc_2[3];
        cross_3x1(omega[i+1],P_c[i],cross_dvc_2);
        // cross_dvc_3 = omega[i+1].cross(omega[i+1].cross(P_c[i]))
        data_t cross_dvc_3[3];
        cross_3x1(omega[i+1],cross_dvc_2,cross_dvc_3);
        // d_v_c
        d_v_c[i+1][0] = cross_dvc_1[0]+cross_dvc_3[0]+d_v[i+1][0];
        d_v_c[i+1][1] = cross_dvc_1[1]+cross_dvc_3[1]+d_v[i+1][1];
        d_v_c[i+1][2] = cross_dvc_1[2]+cross_dvc_3[2]+d_v[i+1][2];

        // 连杆力
        F[i+1][0]=m[i]*d_v_c[i+1][0];
        F[i+1][1]=m[i]*d_v_c[i+1][1];
        F[i+1][2]=m[i]*d_v_c[i+1][2];

        //连杆力矩
        // N[i+1] = I[i]*d_omega[i+1]+omega[i+1].cross(I[i]*omega[i+1]);
        data_t mat_1[3];
        mat_9x3_vec(I[i],d_omega[i+1],mat_1);
        //mat_2 = I[i]*omega[i+1]
        data_t mat_2[3];
        mat_9x3_vec(I[i],omega[i+1],mat_2);
        //cross_N_1 = omega[i+1].cross(I[i]*omega[i+1])
        data_t cross_N_1[3];
        cross_3x1(omega[i+1],mat_2,cross_N_1);
        //N
        N[i+1][0] = mat_1[0]+cross_N_1[0];
        N[i+1][1] = mat_1[1]+cross_N_1[1];
        N[i+1][2] = mat_1[2]+cross_N_1[2];


        // 连杆角速度
        // omega[i+1][3] = R[i]*omega[i]+dq[i]*Z;
        // // 连杆角加速度
        // d_omega[i+1] = R[i]*d_omega[i] + (R[i]*omega[i]).cross(dq[i]*Z)+ddq[i]*Z;
        // // 线加速度
        // d_v[i+1] = R[i]*(d_omega[i].cross(P[i])+omega[i].cross(omega[i].cross(P[i]))+d_v[i]);
        // // 质心线加速度
        // d_v_c[i+1] = d_omega[i+1].cross(P_c[i])+omega[i+1].cross(omega[i+1].cross(P_c[i]))+d_v[i+1];
        // // 连杆力
        // F[i+1] = m[i]*d_v_c[i+1];
        // // 连杆力矩
        // N[i+1] = I[i]*d_omega[i+1]+omega[i+1].cross(I[i]*omega[i+1]);
    }

    // 主要计算循环，RNEA内推
    for (int i = n; i > 0 ; i--)
    {
        // 连杆总力
        data_t mat_f_1[3];
        mat_9x3_vec(R[i],f[i+1],mat_f_1);
        f[i][0] = mat_f_1[0]+F[i][0];
        f[i][1] = mat_f_1[1]+F[i][1];
        f[i][2] = mat_f_1[2]+F[i][2];

        // 连杆总力矩
        //mat_n_f_1 = R_n[i] * n_f[i + 1]
        data_t mat_n_f_1[3];
        mat_9x3_vec(R[i],n_f[i+1],mat_n_f_1);
        //cross_n_f_1 = P_c[i-1].cross(F[i])
        data_t cross_n_f_1[3];
        cross_3x1(P_c[i-1],F[i],cross_n_f_1);
        //cross_n_f_2 = P[i].cross(R_n[i] * f[i + 1]);
        data_t cross_n_f_2[3];
        cross_3x1(P[i],mat_f_1,cross_n_f_2);
        //n_f
        n_f[i][0] = N[i][0]+mat_n_f_1[0]+cross_n_f_1[0]+cross_n_f_2[0];
        n_f[i][1] = N[i][1]+mat_n_f_1[1]+cross_n_f_1[1]+cross_n_f_2[1];
        n_f[i][2] = N[i][2]+mat_n_f_1[2]+cross_n_f_1[2]+cross_n_f_2[2];

        // 关节力矩
        tau[i] = n_f[i][2];


        // // 连杆总力
        // f[i] = R_n[i] * f[i + 1] + F[i];
        // // 连杆总力矩
        // n_f[i] =N[i]+ R_n[i] * n_f[i + 1]+ P_c[i-1].cross(F[i])
        //         + P[i].cross(R_n[i] * f[i + 1]);
        // //关节输出力矩
        // tau[i] = n_f[i].dot(Z);
    }
}


// //======================== RNEA摩擦力计算 ========================
// std::vector<double>  Com_tau_f(const int n,Eigen::Matrix<double,6,1> dq)
// {
//     // 计算摩擦参数,需要输入dq，输出六个关节的摩擦力矩
//     std::vector<double> g_q(n+1, 0.0);
//     std::vector<double> vis_q(n+1, 0.0);
//     std::vector<double> tau_f(n+1, 0.0);
//     for (int i=0;i<n;i++)
//     {
//         g_q[i] =  (Fc[i]+(Fs[i]-Fc[i])*std::exp(-pow(abs(dq[i]/vs[i]),delta_s[i])))*tanh(dq[i]);
//         vis_q[i] = Fv[i]*dq[i];
//         tau_f[i] = g_q[i]+vis_q[i];
//     }
//     return tau_f;
// }
