#ifndef SR4_TAU_HLD
#define SR4_TAU_HLD

//=============================================================
// 文件名: total_tau_hls.h
// 说明:
//   这是 total_tau_hls.cpp 对应的头文件
//=============================================================

typedef float data_t;
const int DOF = 6;

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================
// 顶层 HLS 接口函数
// 输入:
//   q[6]    : 关节角（当前保留接口兼容）
//   dq[6]   : 关节角速度
//   ddq[6]  : 关节角加速度
//   sinq[6] : 各关节角正弦
//   cosq[6] : 各关节角余弦
//
// 输出:
//   tau_out[6] : 6个关节总力矩
//=============================================================
void total_tau_hls(
    const data_t q[DOF],
    const data_t dq[DOF],
    const data_t ddq[DOF],
    const data_t sinq[DOF],
    const data_t cosq[DOF],
    data_t tau_out[DOF]
);

#ifdef __cplusplus
}
#endif

#endif