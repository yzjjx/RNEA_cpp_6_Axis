#ifndef SR4_RNEA_HLS_H
#define SR4_RNEA_HLS_H

// ============================================================
// SR4 六轴机器人 RNEA + 摩擦项 HLS 头文件
// 对应源文件：SR4_rnea_hls.cpp
// ============================================================

#define DOF 6

typedef float data_t;

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// 顶层函数声明
// q       : 关节角
// dq      : 关节角速度
// ddq     : 关节角加速度
// sinq    : ARM 预先计算好的 sin(q)
// cosq    : ARM 预先计算好的 cos(q)
// tau_out : 输出总力矩（RNEA + 摩擦）
// ============================================================
void SR4_rnea_hls(
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

#endif // SR4_RNEA_HLS_H