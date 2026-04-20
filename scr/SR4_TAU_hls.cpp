/*=============================================================
 * 文件名: total_tau_hls.cpp
 * 说明:
 *  1) 保留 float
 *  2) 面向 Vitis HLS 重构
 *  3) 删除中间 T[6][16]，直接生成 R / RT / P
 *  4) 前向递推 / 后向递推 / 摩擦模型 分离
 *  5) 顶层函数 total_tau_hls 可直接作为 HLS top
 *
 * 输出:
 *   tau_out[0..5] = 6 个关节总力矩
 *
 * 备注:
 *   - q 输入保留，用于接口兼容；当前实现中未直接使用 q，
 *     因为旋转关系已由 sinq / cosq 给出
 *   - 若你后续要进一步降资源，优先优化 friction_model()
 *=============================================================*/

#include <cmath>

#ifdef __SYNTHESIS__
#include <hls_math.h>
#endif

typedef float data_t;
const int DOF = 6;

//==============================================================
// 常量参数
//==============================================================
static const data_t m[DOF] = {
    3.16699999486378f,
    3.89505775503759f,
    1.55161965449885f,
    1.88199999547530f,
    1.34621855234916f,
    0.633999997351584f
};

// 质心坐标（已除质量）
static const data_t P_c[DOF][3] = {
    { 3.48369999434000e-05f / m[0], -0.0253106639589312f / m[0], -0.291864385526951f / m[0] },
    { 0.659449401868420f   / m[1], -0.0651938216320210f / m[1], -0.271940703633802f / m[1] },
    {-0.00157764514450908f / m[2],  0.153969960151856f  / m[2], -0.00930606042947241f / m[2]},
    {-4.70521423066045e-05f/ m[3], -0.0205570880251570f / m[3], -0.134124493621368f / m[3]},
    {-0.00190221636650860f / m[4],  0.0159457855937173f / m[4], -0.0281550840659444f / m[4]},
    { 4.62815646042645e-05f/ m[5],  5.00861139672133e-05f / m[5], -0.0284368017495592f / m[5]}
};

// 惯性张量（按 3x3 行优先展开）
static const data_t I[DOF][9] = {
    {
        0.058267910813839f,      -3.715826953316560e-07f, -1.05949175179955e-06f,
       -3.715826953316560e-07f,  0.0576103483714700f,     -0.00456914016524626f,
       -1.05949175179955e-06f,  -0.00456914016524626f,    0.00607968319952077f
    },
    {
        0.0380304151034530f,  0.0419599871875192f,   0.0436322529474843f,
        0.0419599871875192f,  0.263486545009477f,   -0.00383736352324302f,
        0.0436322529474843f, -0.00383736352324302f,  0.250878539307793f
    },
    {
        0.0359185029001311f,   0.00357521327138579f, 0.000314442104644629f,
        0.00357521327138579f,  0.00328728806178666f, 0.00270560721315391f,
        0.000314442104644629f, 0.00270560721315391f, 0.0354951529635439f
    },
    {
        0.0231030453535813f,    -2.36395069266735e-06f, -2.48326590911455e-06f,
       -2.36395069266735e-06f,   0.0220785414360184f,    -0.00290398198945597f,
       -2.48326590911455e-06f,  -0.00290398198945597f,   0.00301282626685543f
    },
    {
        0.00568553742802938f,   1.20535594054042e-05f, -4.97420578495685e-05f,
        1.20535594054042e-05f,  0.00444010179654207f,  0.000717862262034322f,
       -4.97420578495685e-05f,  0.000717862262034322f, 0.00277209865650963f
    },
    {
        0.00227249981806468f,  -1.38365624835237e-06f,  2.97586698701961e-06f,
       -1.38365624835237e-06f,  0.00227800923976532f,   3.82651246364623e-06f,
        2.97586698701961e-06f,  3.82651246364623e-06f,  0.000447957334568561f
    }
};

// MDH 参数
static const data_t sin_alpha[DOF] = {
     0.0f, -1.0f,  0.0f, -1.0f,  1.0f, -1.0f
};

static const data_t cos_alpha[DOF] = {
     1.0f,  0.0f, -1.0f,  0.0f,  0.0f,  0.0f
};

static const data_t a[DOF] = {
    0.0f, 0.0f, 0.403112f, -0.05f, 0.0f, 0.0f
};

static const data_t d[DOF] = {
    0.328f, 0.0f, 0.4f, 0.4f, -0.136f, 0.1035f
};

// 非线性摩擦参数
static const data_t Fc[DOF] = {
    39.3156017014124f,
    5.74376743911912f,
    11.0817930270772f,
    1.30308256888117f,
    1.33226719773913f,
    1.57685226136080f
};

static const data_t Fs[DOF] = {
   -2.53722774557035f,
   -2.07595046391569f,
    0.155945016604755f,
   -0.432048786756766f,
   -0.299954735443126f,
   -1.12558099217137f
};

static const data_t vs[DOF] = {
    6159.98349836929f,
   -0.00542827529961853f,
   -4.99999961405425f,
    0.000166612161014179f,
    0.000335859662883711f,
    0.00125116525892426f
};

static const data_t Fv[DOF] = {
    5.67202325533792f,
    9.50541644780893f,
    3.62367344786894f,
    2.01161766835989f,
    1.77753031609100f,
    1.84035495310709f
};

static const data_t delta_s[DOF] = {
    0.145914867459757f,
    0.272713039894872f,
    0.335189700243761f,
    1.99565622087150f,
    1.99730531138284f,
    0.323686309612415f
};

//==============================================================
// HLS / 普通C++ 兼容数学函数封装
//==============================================================
static inline data_t math_abs(data_t x)
{
#pragma HLS INLINE
#ifdef __SYNTHESIS__
    return hls::fabsf(x);
#else
    return std::fabs(x);
#endif
}

static inline data_t math_exp(data_t x)
{
#pragma HLS INLINE
#ifdef __SYNTHESIS__
    return hls::expf(x);
#else
    return std::exp(x);
#endif
}

static inline data_t math_pow(data_t x, data_t y)
{
#pragma HLS INLINE
#ifdef __SYNTHESIS__
    return hls::powf(x, y);
#else
    return std::pow(x, y);
#endif
}

static inline data_t math_tanh(data_t x)
{
#pragma HLS INLINE
#ifdef __SYNTHESIS__
    return hls::tanhf(x);
#else
    return std::tanh(x);
#endif
}

//==============================================================
// 基础计算函数
//==============================================================
static inline void cross_3x1(
    const data_t a_vec[3],
    const data_t b_vec[3],
    data_t c_vec[3]
)
{
#pragma HLS INLINE
    c_vec[0] = a_vec[1] * b_vec[2] - a_vec[2] * b_vec[1];
    c_vec[1] = a_vec[2] * b_vec[0] - a_vec[0] * b_vec[2];
    c_vec[2] = a_vec[0] * b_vec[1] - a_vec[1] * b_vec[0];
}

static inline void mat_3x3_vec(
    const data_t A[9],
    const data_t x[3],
    data_t y[3]
)
{
#pragma HLS INLINE
    for (int i = 0; i < 3; i++)
    {
#pragma HLS UNROLL
        y[i] = 0.0f;
        for (int k = 0; k < 3; k++)
        {
#pragma HLS UNROLL
            y[i] += A[i * 3 + k] * x[k];
        }
    }
}

//==============================================================
// 直接构造 R / RT / P，不再生成 T[6][16]
//==============================================================
static void build_rot_pos(
    const data_t sinq[DOF],
    const data_t cosq[DOF],
    data_t R[DOF + 1][9],
    data_t RT[DOF][9],
    data_t P[DOF + 1][3]
)
{
#pragma HLS INLINE off

build_rot_pos_loop:
    for (int i = 0; i < DOF; i++)
    {
#pragma HLS PIPELINE

        // R
        R[i][0] = cosq[i];
        R[i][1] = -sinq[i];
        R[i][2] = 0.0f;

        R[i][3] = sinq[i] * cos_alpha[i];
        R[i][4] = cosq[i] * cos_alpha[i];
        R[i][5] = -sin_alpha[i];

        R[i][6] = sinq[i] * sin_alpha[i];
        R[i][7] = cosq[i] * sin_alpha[i];
        R[i][8] = cos_alpha[i];

        // RT = R^T
        RT[i][0] = R[i][0];
        RT[i][1] = R[i][3];
        RT[i][2] = R[i][6];

        RT[i][3] = R[i][1];
        RT[i][4] = R[i][4];
        RT[i][5] = R[i][7];

        RT[i][6] = R[i][2];
        RT[i][7] = R[i][5];
        RT[i][8] = R[i][8];

        // P
        P[i][0] = a[i];
        P[i][1] = -sin_alpha[i] * d[i];
        P[i][2] =  cos_alpha[i] * d[i];
    }

sentinel_R_loop:
    for (int j = 0; j < 9; j++)
    {
#pragma HLS UNROLL
        R[DOF][j] = 0.0f;
    }

sentinel_P_loop:
    for (int j = 0; j < 3; j++)
    {
#pragma HLS UNROLL
        P[DOF][j] = 0.0f;
    }
}

//==============================================================
// 前向递推
//==============================================================
static void forward_rnea(
    const data_t dq[DOF],
    const data_t ddq[DOF],
    const data_t RT[DOF][9],
    const data_t P[DOF + 1][3],
    data_t omega[DOF + 1][3],
    data_t domega[DOF + 1][3],
    data_t dv[DOF + 1][3],
    data_t dvc[DOF + 1][3],
    data_t F[DOF + 1][3],
    data_t N[DOF + 1][3]
)
{
#pragma HLS INLINE off

    // 基座初始化
    omega[0][0] = 0.0f; omega[0][1] = 0.0f; omega[0][2] = 0.0f;
    domega[0][0] = 0.0f; domega[0][1] = 0.0f; domega[0][2] = 0.0f;
    dv[0][0] = 0.0f; dv[0][1] = 0.0f; dv[0][2] = 9.8f;

forward_loop:
    for (int i = 0; i < DOF; i++)
    {
#pragma HLS PIPELINE

        data_t tmp_w[3];
        data_t tmp_dw[3];
        data_t z_dq[3] = {0.0f, 0.0f, dq[i]};

        data_t c1[3];
        data_t c2[3];
        data_t c3[3];
        data_t addv[3];

        data_t I_dw[3];
        data_t I_w[3];

        // omega[i+1] = RT[i] * omega[i] + [0 0 dq]
        mat_3x3_vec(RT[i], omega[i], tmp_w);
        omega[i+1][0] = tmp_w[0];
        omega[i+1][1] = tmp_w[1];
        omega[i+1][2] = tmp_w[2] + dq[i];

        // domega[i+1]
        mat_3x3_vec(RT[i], domega[i], tmp_dw);
        cross_3x1(tmp_w, z_dq, c1);

        domega[i+1][0] = tmp_dw[0] + c1[0];
        domega[i+1][1] = tmp_dw[1] + c1[1];
        domega[i+1][2] = tmp_dw[2] + c1[2] + ddq[i];

        // dv[i+1] = RT[i] * (domega[i]xP[i] + omega[i]x(omega[i]xP[i]) + dv[i])
        cross_3x1(domega[i], P[i], c1);
        cross_3x1(omega[i], P[i], c2);
        cross_3x1(omega[i], c2, c3);

        addv[0] = c1[0] + c3[0] + dv[i][0];
        addv[1] = c1[1] + c3[1] + dv[i][1];
        addv[2] = c1[2] + c3[2] + dv[i][2];

        mat_3x3_vec(RT[i], addv, dv[i+1]);

        // dvc[i+1] = domega[i+1]xPc[i] + omega[i+1]x(omega[i+1]xPc[i]) + dv[i+1]
        cross_3x1(domega[i+1], P_c[i], c1);
        cross_3x1(omega[i+1], P_c[i], c2);
        cross_3x1(omega[i+1], c2, c3);

        dvc[i+1][0] = c1[0] + c3[0] + dv[i+1][0];
        dvc[i+1][1] = c1[1] + c3[1] + dv[i+1][1];
        dvc[i+1][2] = c1[2] + c3[2] + dv[i+1][2];

        // F[i+1] = m[i] * dvc[i+1]
        F[i+1][0] = m[i] * dvc[i+1][0];
        F[i+1][1] = m[i] * dvc[i+1][1];
        F[i+1][2] = m[i] * dvc[i+1][2];

        // N[i+1] = I[i]*domega[i+1] + omega[i+1] x (I[i]*omega[i+1])
        mat_3x3_vec(I[i], domega[i+1], I_dw);
        mat_3x3_vec(I[i], omega[i+1], I_w);
        cross_3x1(omega[i+1], I_w, c1);

        N[i+1][0] = I_dw[0] + c1[0];
        N[i+1][1] = I_dw[1] + c1[1];
        N[i+1][2] = I_dw[2] + c1[2];
    }
}

//==============================================================
// 后向递推
//==============================================================
static void backward_rnea(
    const data_t R[DOF + 1][9],
    const data_t P[DOF + 1][3],
    const data_t F[DOF + 1][3],
    const data_t N[DOF + 1][3],
    data_t tau_dyn[DOF]
)
{
#pragma HLS INLINE off

    data_t f[DOF + 2][3];
    data_t nf[DOF + 2][3];

init_terminal_loop:
    for (int j = 0; j < 3; j++)
    {
#pragma HLS UNROLL
        f[DOF + 1][j]  = 0.0f;
        nf[DOF + 1][j] = 0.0f;
    }

backward_loop:
    for (int i = DOF; i > 0; i--)
    {
#pragma HLS PIPELINE

        data_t Rf[3];
        data_t Rnf[3];
        data_t c1[3];
        data_t c2[3];

        // f[i] = R[i] * f[i+1] + F[i]
        mat_3x3_vec(R[i], f[i+1], Rf);
        f[i][0] = Rf[0] + F[i][0];
        f[i][1] = Rf[1] + F[i][1];
        f[i][2] = Rf[2] + F[i][2];

        // nf[i] = N[i] + R[i]*nf[i+1] + Pc[i-1]xF[i] + P[i]x(R[i]*f[i+1])
        mat_3x3_vec(R[i], nf[i+1], Rnf);
        cross_3x1(P_c[i - 1], F[i], c1);
        cross_3x1(P[i], Rf, c2);

        nf[i][0] = N[i][0] + Rnf[0] + c1[0] + c2[0];
        nf[i][1] = N[i][1] + Rnf[1] + c1[1] + c2[1];
        nf[i][2] = N[i][2] + Rnf[2] + c1[2] + c2[2];

        // 关节力矩输出到 tau_dyn[0..5]
        tau_dyn[i - 1] = nf[i][2];
    }
}

//==============================================================
// 摩擦力矩
//==============================================================
static void friction_model(
    const data_t dq[DOF],
    data_t tau_fric[DOF]
)
{
#pragma HLS INLINE off

friction_loop:
    for (int i = 0; i < DOF; i++)
    {
#pragma HLS PIPELINE

        data_t ratio = dq[i] / vs[i];
        data_t x_abs = math_abs(ratio);
        data_t pw    = math_pow(x_abs, delta_s[i]);
        data_t e_val = math_exp(-pw);
        data_t g_q   = (Fc[i] + (Fs[i] - Fc[i]) * e_val) * math_tanh(dq[i]);
        data_t vis_q = Fv[i] * dq[i];

        tau_fric[i] = g_q + vis_q;
    }
}

//==============================================================
// 顶层函数：总力矩 = 动力学力矩 + 摩擦力矩
// 输出 tau_out[0..5] = tau1..tau6
//==============================================================
extern "C" {
void total_tau_hls(
    const data_t q[DOF],
    const data_t dq[DOF],
    const data_t ddq[DOF],
    const data_t sinq[DOF],
    const data_t cosq[DOF],
    data_t tau_out[DOF]
)
{
#pragma HLS INTERFACE m_axi     port=q       offset=slave bundle=gmem0 depth=6
#pragma HLS INTERFACE m_axi     port=dq      offset=slave bundle=gmem1 depth=6
#pragma HLS INTERFACE m_axi     port=ddq     offset=slave bundle=gmem2 depth=6
#pragma HLS INTERFACE m_axi     port=sinq    offset=slave bundle=gmem3 depth=6
#pragma HLS INTERFACE m_axi     port=cosq    offset=slave bundle=gmem4 depth=6
#pragma HLS INTERFACE m_axi     port=tau_out offset=slave bundle=gmem5 depth=6
#pragma HLS INTERFACE s_axilite port=return  bundle=control

#pragma HLS ARRAY_PARTITION variable=P_c complete dim=2
#pragma HLS ARRAY_PARTITION variable=I   complete dim=2

    // q 当前保留接口兼容
    (void)q;

    data_t R[DOF + 1][9];
    data_t RT[DOF][9];
    data_t P[DOF + 1][3];

    data_t omega[DOF + 1][3];
    data_t domega[DOF + 1][3];
    data_t dv[DOF + 1][3];
    data_t dvc[DOF + 1][3];
    data_t F[DOF + 1][3];
    data_t N[DOF + 1][3];

    data_t tau_dyn[DOF];
    data_t tau_fric[DOF];

    build_rot_pos(sinq, cosq, R, RT, P);
    forward_rnea(dq, ddq, RT, P, omega, domega, dv, dvc, F, N);
    backward_rnea(R, P, F, N, tau_dyn);
    friction_model(dq, tau_fric);

combine_loop:
    for (int i = 0; i < DOF; i++)
    {
#pragma HLS PIPELINE
        tau_out[i] = tau_dyn[i] + tau_fric[i];
    }
}
}