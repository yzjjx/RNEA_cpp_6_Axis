#include <math.h>

// ============================================================
// SR4 六轴机器人 RNEA + 摩擦项 HLS 可综合版本
// 说明：
// 1) 不使用 Eigen / std::vector / 动态内存
// 2) 输入 sinq/cosq 由 ARM 侧提前计算后送入 FPGA
// 3) 顶层接口采用 BRAM 端口，便于先在 HLS 中综合验证
// 4) 当前数据类型使用 float，后续可再改 ap_fixed
// ============================================================

#define DOF 6

typedef float data_t;

static const data_t Z_AXIS[3] = {0.0f, 0.0f, 1.0f};
static const data_t G_BASE[3] = {0.0f, 0.0f, 9.8f};
static const data_t ZERO3[3]  = {0.0f, 0.0f, 0.0f};

// ---------------------- 机器人几何参数 ----------------------
static const data_t A_MDH[DOF] = {
    0.0f,
    0.0f,
    0.403112f,
   -0.05f,
    0.0f,
    0.0f
};

static const data_t D_MDH[DOF] = {
    0.328f,
    0.0f,
    0.4f,
    0.4f,
   -0.136f,
    0.1035f
};

// alpha = {0, -pi/2, pi, -pi/2, pi/2, -pi/2}
static const data_t SIN_ALPHA[DOF] = {
     0.0f,
    -1.0f,
     0.0f,
    -1.0f,
     1.0f,
    -1.0f
};

static const data_t COS_ALPHA[DOF] = {
     1.0f,
     0.0f,
    -1.0f,
     0.0f,
     0.0f,
     0.0f
};

// ---------------------- 质量参数 ----------------------
static const data_t MASS[DOF] = {
    3.166999995f,
    3.895057755f,
    1.551619654f,
    1.881999995f,
    1.346218552f,
    0.633999997f
};

// 质心位置（已化成 x/m, y/m, z/m ）
static const data_t PC[DOF][3] = {
    {  1.099684558e-05f, -7.991376771e-03f, -9.215161387e-02f },
    {  1.692913172e-01f, -1.673745341e-02f, -6.981649952e-02f },
    { -1.016772584e-03f,  9.923658479e-02f, -5.998880991e-03f },
    { -2.500113832e-05f, -1.092299045e-02f, -7.126700413e-02f },
    { -1.412996009e-03f,  1.184482510e-02f, -2.091496854e-02f },
    {  7.299931274e-05f,  7.900018205e-05f, -4.485299995e-02f }
};

// 惯性张量
static const data_t INERTIA[DOF][3][3] = {
    {
        { 0.0582679108f, -3.71582695e-07f, -1.05949175e-06f },
        { -3.71582695e-07f, 0.0576103484f, -0.00456914017f },
        { -1.05949175e-06f, -0.00456914017f, 0.00607968320f }
    },
    {
        { 0.0380304151f, 0.0419599872f, 0.0436322529f },
        { 0.0419599872f, 0.2634865450f, -0.00383736352f },
        { 0.0436322529f, -0.00383736352f, 0.2508785393f }
    },
    {
        { 0.0359185029f, 0.00357521327f, 0.000314442105f },
        { 0.00357521327f, 0.00328728806f, 0.00270560721f },
        { 0.000314442105f, 0.00270560721f, 0.0354951530f }
    },
    {
        { 0.0231030454f, -2.36395069e-06f, -2.48326591e-06f },
        { -2.36395069e-06f, 0.0220785414f, -0.00290398199f },
        { -2.48326591e-06f, -0.00290398199f, 0.00301282627f }
    },
    {
        { 0.00568553743f, 1.20535594e-05f, -4.97420578e-05f },
        { 1.20535594e-05f, 0.00444010180f, 0.000717862262f },
        { -4.97420578e-05f, 0.000717862262f, 0.00277209866f }
    },
    {
        { 0.00227249982f, -1.38365625e-06f, 2.97586699e-06f },
        { -1.38365625e-06f, 0.00227800924f, 3.82651246e-06f },
        { 2.97586699e-06f, 3.82651246e-06f, 0.000447957335f }
    }
};

// ---------------------- 摩擦参数 ----------------------
static const data_t FC[DOF] = {
    39.3156017f, 5.74376744f, 11.0817930f,
    1.30308257f, 1.33226720f, 1.57685226f
};

static const data_t FS[DOF] = {
   -2.53722775f, -2.07595046f,  0.155945017f,
   -0.432048787f, -0.299954735f, -1.12558099f
};

static const data_t VS[DOF] = {
    6159.98340f, -0.00542827556f, -4.99999952f,
    0.000166612153f, 0.000335859659f, 0.00125116529f
};

static const data_t FV[DOF] = {
    5.67202330f, 9.50541687f, 3.62367344f,
    2.01161766f, 1.77753031f, 1.84035492f
};

static const data_t DELTA_S[DOF] = {
    0.145914868f, 0.272713035f, 0.335189700f,
    1.99565625f,  1.99730527f,  0.323686302f
};

// ============================================================
// 基础向量/矩阵运算
// ============================================================
static void vec_copy(const data_t a[3], data_t b[3]) {
#pragma HLS INLINE
    b[0] = a[0];
    b[1] = a[1];
    b[2] = a[2];
}

static void vec_add3(const data_t a[3], const data_t b[3], data_t c[3]) {
#pragma HLS INLINE
    c[0] = a[0] + b[0];
    c[1] = a[1] + b[1];
    c[2] = a[2] + b[2];
}

static void vec_add4(const data_t a[3], const data_t b[3], const data_t c0[3], const data_t d[3], data_t out[3]) {
#pragma HLS INLINE
    out[0] = a[0] + b[0] + c0[0] + d[0];
    out[1] = a[1] + b[1] + c0[1] + d[1];
    out[2] = a[2] + b[2] + c0[2] + d[2];
}

static void vec_scale3(const data_t a[3], data_t s, data_t b[3]) {
#pragma HLS INLINE
    b[0] = a[0] * s;
    b[1] = a[1] * s;
    b[2] = a[2] * s;
}

static void cross3(const data_t a[3], const data_t b[3], data_t c[3]) {
#pragma HLS INLINE
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

static data_t dot3(const data_t a[3], const data_t b[3]) {
#pragma HLS INLINE
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void matvec3(const data_t M[3][3], const data_t x[3], data_t y[3]) {
#pragma HLS INLINE
    y[0] = M[0][0] * x[0] + M[0][1] * x[1] + M[0][2] * x[2];
    y[1] = M[1][0] * x[0] + M[1][1] * x[1] + M[1][2] * x[2];
    y[2] = M[2][0] * x[0] + M[2][1] * x[1] + M[2][2] * x[2];
}

static void build_rot_and_pos(
    data_t sq, data_t cq,
    data_t sa, data_t ca,
    data_t a,  data_t d,
    data_t R[3][3],
    data_t RT[3][3],
    data_t P[3]) {
#pragma HLS INLINE
    // MDH 齐次变换中的旋转块
    R[0][0] = cq;       R[0][1] = -sq;      R[0][2] = 0.0f;
    R[1][0] = sq * ca;  R[1][1] = cq * ca;  R[1][2] = -sa;
    R[2][0] = sq * sa;  R[2][1] = cq * sa;  R[2][2] = ca;

    // 转置，用于外推
    RT[0][0] = R[0][0]; RT[0][1] = R[1][0]; RT[0][2] = R[2][0];
    RT[1][0] = R[0][1]; RT[1][1] = R[1][1]; RT[1][2] = R[2][1];
    RT[2][0] = R[0][2]; RT[2][1] = R[1][2]; RT[2][2] = R[2][2];

    // 位置向量
    P[0] = a;
    P[1] = -sa * d;
    P[2] =  ca * d;
}

static data_t friction_one_joint(data_t dq, int i) {
#pragma HLS INLINE
    data_t ratio = fabsf(dq / VS[i]);
    data_t pw    = powf(ratio, DELTA_S[i]);
    data_t expo  = expf(-pw);
    data_t gq    = (FC[i] + (FS[i] - FC[i]) * expo) * tanhf(dq);
    data_t vis   = FV[i] * dq;
    return gq + vis;
}

// ============================================================
// 顶层函数
// q    : 关节角
// dq   : 关节角速度
// ddq  : 关节角加速度
// sinq : ARM 预先算好的 sin(q)
// cosq : ARM 预先算好的 cos(q)
// tau_out : 输出总力矩（RNEA + 摩擦）
// ============================================================
extern "C" {
void SR4_rnea_hls(
    const data_t q[DOF],
    const data_t dq[DOF],
    const data_t ddq[DOF],
    const data_t sinq[DOF],
    const data_t cosq[DOF],
    data_t tau_out[DOF]) {


    // 存储每级旋转/位移以及外推后的力和力矩
    data_t Rm[DOF][3][3];
    data_t Rt[DOF][3][3];
    data_t P[DOF][3];
    data_t F[DOF][3];
    data_t N[DOF][3];

    data_t w_prev[3]  = {0.0f, 0.0f, 0.0f};
    data_t dw_prev[3] = {0.0f, 0.0f, 0.0f};
    data_t dv_prev[3] = {G_BASE[0], G_BASE[1], G_BASE[2]};

    // ---------------------- 正向递推 ----------------------
forward_loop:
    for (int i = 0; i < DOF; i++) {
        data_t w_rot[3], dw_rot[3], dv_rot[3];
        data_t dqz[3], ddqz[3];
        data_t c1[3], c2[3], c3[3], sum_in[3];
        data_t w_now[3], dw_now[3], dv_now[3], dvc_now[3];
        data_t I_dw[3], I_w[3], wxIw[3];

        // 生成本级 R、R^T、P
        build_rot_and_pos(
            sinq[i], cosq[i],
            SIN_ALPHA[i], COS_ALPHA[i],
            A_MDH[i], D_MDH[i],
            Rm[i], Rt[i], P[i]
        );

        // w_now = Rt * w_prev + dq * Z
        matvec3(Rt[i], w_prev, w_rot);
        dqz[0] = 0.0f; dqz[1] = 0.0f; dqz[2] = dq[i];
        vec_add3(w_rot, dqz, w_now);

        // dw_now = Rt * dw_prev + cross(Rt*w_prev, dq*Z) + ddq*Z
        matvec3(Rt[i], dw_prev, dw_rot);
        ddqz[0] = 0.0f; ddqz[1] = 0.0f; ddqz[2] = ddq[i];
        cross3(w_rot, dqz, c1);
        vec_add4(dw_rot, c1, ddqz, ZERO3, dw_now);

        // dv_now = Rt * ( cross(dw_prev,P) + cross(w_prev, cross(w_prev,P)) + dv_prev )
        cross3(dw_prev, P[i], c1);
        cross3(w_prev, P[i], c2);
        cross3(w_prev, c2, c3);
        vec_add4(c1, c3, dv_prev, ZERO3, sum_in);
        matvec3(Rt[i], sum_in, dv_now);

        // dvc_now = cross(dw_now,Pc) + cross(w_now, cross(w_now,Pc)) + dv_now
        cross3(dw_now, PC[i], c1);
        cross3(w_now, PC[i], c2);
        cross3(w_now, c2, c3);
        vec_add4(c1, c3, dv_now, ZERO3, dvc_now);

        // F = m * dvc
        vec_scale3(dvc_now, MASS[i], F[i]);

        // N = I * dw + w x (I*w)
        matvec3(INERTIA[i], dw_now, I_dw);
        matvec3(INERTIA[i], w_now, I_w);
        cross3(w_now, I_w, wxIw);
        vec_add3(I_dw, wxIw, N[i]);

        // 更新上一节状态
        vec_copy(w_now,  w_prev);
        vec_copy(dw_now, dw_prev);
        vec_copy(dv_now, dv_prev);
    }

    // ---------------------- 反向递推 ----------------------
    data_t f_next[3] = {0.0f, 0.0f, 0.0f};
    data_t n_next[3] = {0.0f, 0.0f, 0.0f};

#pragma HLS ARRAY_PARTITION variable=f_next complete dim=1
#pragma HLS ARRAY_PARTITION variable=n_next complete dim=1

backward_loop:
    for (int i = DOF - 1; i >= 0; i--) {
#pragma HLS PIPELINE II=1
        data_t Rf[3], Rn[3], pcf[3], pRf[3];
        data_t f_now[3], n_now[3];
        data_t tau_rnea, tau_fric;

#pragma HLS ARRAY_PARTITION variable=Rf complete dim=1
#pragma HLS ARRAY_PARTITION variable=Rn complete dim=1
#pragma HLS ARRAY_PARTITION variable=pcf complete dim=1
#pragma HLS ARRAY_PARTITION variable=pRf complete dim=1
#pragma HLS ARRAY_PARTITION variable=f_now complete dim=1
#pragma HLS ARRAY_PARTITION variable=n_now complete dim=1

        // f_now = R * f_next + F
        matvec3(Rm[i], f_next, Rf);
        vec_add3(Rf, F[i], f_now);

        // n_now = N + R*n_next + Pc x F + P x (R*f_next)
        matvec3(Rm[i], n_next, Rn);
        cross3(PC[i], F[i], pcf);
        cross3(P[i], Rf, pRf);
        n_now[0] = N[i][0] + Rn[0] + pcf[0] + pRf[0];
        n_now[1] = N[i][1] + Rn[1] + pcf[1] + pRf[1];
        n_now[2] = N[i][2] + Rn[2] + pcf[2] + pRf[2];

        tau_rnea = dot3(n_now, Z_AXIS);
        tau_fric = friction_one_joint(dq[i], i);
        tau_out[i] = tau_rnea + tau_fric;

        vec_copy(f_now, f_next);
        vec_copy(n_now, n_next);
    }

    // q 在该版本中未参与计算本体变换（因为 sinq/cosq 已外部输入），
    // 这里保留 q 端口是为了接口一致性，避免后续你要切回 FPGA 内部算三角函数时再改顶层。
    (void)q;
}
}