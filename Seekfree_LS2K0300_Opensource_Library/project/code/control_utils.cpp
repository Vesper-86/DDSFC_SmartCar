#include "control_utils.hpp"
#include <cstddef>

/*
 * ============================================================================
 * 文件名称: control_utils.cpp
 * 文件用途: PID 与模糊 PD 的实现
 *
 * 说明:
 * - PID 用于左右轮速度闭环。
 * - 模糊 PD 用于根据循迹误差动态调整方向控制的 KP / KD。
 * ============================================================================
 */

/* ========================= PID 区域 ========================= */
void pid_init(pid_controller_t &pid, float kp, float ki, float kd, float i_limit, float out_limit)
{
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.integral = 0.0f;
    pid.last_error = 0.0f;
    pid.i_limit = i_limit;
    pid.out_limit = out_limit;
}

float pid_update(pid_controller_t &pid, float target, float measure, float dt_s)
{
    /* 当前误差 = 目标值 - 实际反馈值 */
    const float error = target - measure;

    /* 微分项表示误差变化速度，用于抑制快速摆动 */
    const float derivative = (dt_s > 1e-6f) ? (error - pid.last_error) / dt_s : 0.0f;

    /* 积分项累加并做限幅，防止积分饱和 */
    pid.integral += error * dt_s;
    pid.integral = clampf(pid.integral, -pid.i_limit, pid.i_limit);

    /* PID 总输出，并做最终限幅 */
    float out = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
    out = clampf(out, -pid.out_limit, pid.out_limit);

    pid.last_error = error;
    return out;
}

void pid_reset(pid_controller_t &pid)
{
    pid.integral = 0.0f;
    pid.last_error = 0.0f;
}

/* ========================= 模糊 PD 区域 ========================= */
static float g_error_max = 120.0f;   /* 误差归一化上限 */
static float g_derr_max = 50.0f;     /* 误差变化率归一化上限 */
static float g_kp_fuzzy = 2.0f;      /* KP 模糊修正最大量 */
static float g_kd_fuzzy = 10.0f;     /* KD 模糊修正最大量 */
static float g_kp_base = 8.0f;       /* KP 基础值 */
static float g_kd_base = 16.0f;      /* KD 基础值 */

/* 误差模糊论域：只关心误差绝对值大小 */
static const float g_error_domain[7] = {0.0f, 3.5f, 4.0f, 4.5f, 5.0f, 5.5f, 6.0f};

/* 误差变化率论域：保留正负方向，用于描述收敛还是发散 */
static const float g_derr_domain[7] = {-3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f};

/* KP 规则表：误差越大，通常需要更强的比例项 */
static const float g_kp_rule[7][7] = {
    {0,0,0,0,0,0,0},
    {1,1,1,1,1,1,1},
    {1,1,1,2,3,3,3},
    {2,2,3,3,3,4,4},
    {3,3,4,4,4,5,5},
    {4,5,5,5,5,5,6},
    {6,6,6,6,6,6,6}
};

/* KD 规则表：误差变化越剧烈，KD 越大，用于增加阻尼 */
static const float g_kd_rule[7][7] = {
    {6,5,4,4,4,3,2},
    {6,5,4,4,4,3,2},
    {6,5,4,3,3,2,2},
    {5,4,3,3,2,2,1},
    {4,3,3,3,2,1,0},
    {3,3,3,2,2,1,0},
    {3,3,3,2,2,1,0}
};

/*
 * 线性三角隶属函数。
 * 输入一个值 value，输出它落在哪两个相邻模糊区间以及对应隶属度。
 */
static void membership(float value, int index_pair[2], float degree_pair[2], const float *domain)
{
    if (value <= domain[0]) {
        index_pair[0] = 0;
        index_pair[1] = 1;
        degree_pair[0] = 1.0f;
        degree_pair[1] = 0.0f;
        return;
    }

    if (value >= domain[6]) {
        index_pair[0] = 5;
        index_pair[1] = 6;
        degree_pair[0] = 0.0f;
        degree_pair[1] = 1.0f;
        return;
    }

    for (int i = 0; i < 6; ++i) {
        if (domain[i] < value && value <= domain[i + 1]) {
            index_pair[0] = i;
            index_pair[1] = i + 1;
            degree_pair[0] = (domain[i + 1] - value) / (domain[i + 1] - domain[i]);
            degree_pair[1] = (value - domain[i]) / (domain[i + 1] - domain[i]);
            return;
        }
    }

    /* 理论上不会走到这里，仅做保护 */
    index_pair[0] = 0;
    index_pair[1] = 1;
    degree_pair[0] = 1.0f;
    degree_pair[1] = 0.0f;
}

void fuzzy_pd_set_base(float kp_base, float kd_base)
{
    g_kp_base = kp_base;
    g_kd_base = kd_base;
}

void fuzzy_pd_set_limit(float err_max, float derr_max, float kp_fuzzy, float kd_fuzzy)
{
    g_error_max = err_max;
    g_derr_max = derr_max;
    g_kp_fuzzy = kp_fuzzy;
    g_kd_fuzzy = kd_fuzzy;
}

void fuzzy_pd_update(float error, float &kp, float &kd)
{
    static float last_error = 0.0f;

    /* 误差变化率 = 本轮误差 - 上轮误差 */
    float derr = error - last_error;

    /* 把真实误差映射到模糊论域 */
    float error_mapped = (error / g_error_max) * 6.0f;
    float derr_mapped = (derr / g_derr_max) * 3.0f;

    /* 误差表只关心绝对值大小，所以负误差取绝对值 */
    if (error_mapped < 0.0f) {
        error_mapped = -error_mapped;
        derr_mapped = -derr_mapped;
    }

    /* 若误差符号翻转，通常说明车辆正在左右摆动，增强阻尼 */
    if (last_error * error < 0.0f) {
        derr_mapped = 3.0f;
    }

    int error_index_pair[2];
    int derr_index_pair[2];
    float error_degree_pair[2];
    float derr_degree_pair[2];
    float kp_rule_sum = 0.0f;
    float kd_rule_sum = 0.0f;

    membership(error_mapped, error_index_pair, error_degree_pair, g_error_domain);
    membership(derr_mapped, derr_index_pair, derr_degree_pair, g_derr_domain);

    /* 四邻域加权平均模糊推理 */
    kp_rule_sum += error_degree_pair[0] * derr_degree_pair[0] * g_kp_rule[error_index_pair[0]][derr_index_pair[0]];
    kp_rule_sum += error_degree_pair[0] * derr_degree_pair[1] * g_kp_rule[error_index_pair[0]][derr_index_pair[1]];
    kp_rule_sum += error_degree_pair[1] * derr_degree_pair[0] * g_kp_rule[error_index_pair[1]][derr_index_pair[0]];
    kp_rule_sum += error_degree_pair[1] * derr_degree_pair[1] * g_kp_rule[error_index_pair[1]][derr_index_pair[1]];

    kd_rule_sum += error_degree_pair[0] * derr_degree_pair[0] * g_kd_rule[error_index_pair[0]][derr_index_pair[0]];
    kd_rule_sum += error_degree_pair[0] * derr_degree_pair[1] * g_kd_rule[error_index_pair[0]][derr_index_pair[1]];
    kd_rule_sum += error_degree_pair[1] * derr_degree_pair[0] * g_kd_rule[error_index_pair[1]][derr_index_pair[0]];
    kd_rule_sum += error_degree_pair[1] * derr_degree_pair[1] * g_kd_rule[error_index_pair[1]][derr_index_pair[1]];

    /* 最终输出 = 基础值 + 归一化后的模糊修正量 */
    kp = g_kp_base + (kp_rule_sum / 6.0f) * g_kp_fuzzy;
    kd = g_kd_base + (kd_rule_sum / 6.0f) * g_kd_fuzzy;

    last_error = error;
}
