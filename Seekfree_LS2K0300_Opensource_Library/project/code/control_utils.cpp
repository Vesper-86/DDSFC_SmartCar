#include "control_utils.hpp"
#include <cstddef>
#include <cmath>

/*
 * ============================================================================
 * 文件名称: control_utils.cpp
 * 文件用途: PID 与模糊 PD 的实现（增强版）
 *
 * 改动重点:
 * 1. 速度环增加微分滤波和积分分离，减少突变与积分堆积。
 * 2. 模糊 PD 在原规则表基础上增加振荡抑制与大误差增强。
 * ============================================================================
 */

namespace
{
struct pid_shadow_t
{
    const pid_controller_t *key;
    float derivative_filtered;
};

static pid_shadow_t g_pid_shadow[4] = {
    {nullptr, 0.0f}, {nullptr, 0.0f}, {nullptr, 0.0f}, {nullptr, 0.0f}
};

static pid_shadow_t &shadow_of(const pid_controller_t *pid)
{
    for (auto &slot : g_pid_shadow) {
        if (slot.key == pid) {
            return slot;
        }
    }
    for (auto &slot : g_pid_shadow) {
        if (slot.key == nullptr) {
            slot.key = pid;
            slot.derivative_filtered = 0.0f;
            return slot;
        }
    }
    g_pid_shadow[0].key = pid;
    g_pid_shadow[0].derivative_filtered = 0.0f;
    return g_pid_shadow[0];
}

static void shadow_reset(const pid_controller_t *pid)
{
    for (auto &slot : g_pid_shadow) {
        if (slot.key == pid) {
            slot.derivative_filtered = 0.0f;
            return;
        }
    }
}
} // namespace

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
    shadow_reset(&pid);
}

float pid_update(pid_controller_t &pid, float target, float measure, float dt_s)
{
    if (dt_s <= 1e-6f) {
        dt_s = 1e-3f;
    }

    const float error = target - measure;
    const float raw_derivative = (error - pid.last_error) / dt_s;

    pid_shadow_t &shadow = shadow_of(&pid);
    shadow.derivative_filtered =
        shadow.derivative_filtered * (1.0f - SPEED_PID_D_FILTER_ALPHA) +
        raw_derivative * SPEED_PID_D_FILTER_ALPHA;

    const float integral_enable_band =
        SPEED_PID_INTEGRAL_ENABLE_ERR_MIN + 0.25f * ((target >= 0.0f) ? target : -target);

    bool allow_integral = (std::fabs(error) <= integral_enable_band);

    float tentative_integral = pid.integral;
    if (allow_integral) {
        tentative_integral += error * dt_s;
        tentative_integral = clampf(tentative_integral, -pid.i_limit, pid.i_limit);
    } else {
        tentative_integral *= (1.0f - SPEED_PID_INTEGRAL_RELEASE_GAIN);
    }

    float out = pid.kp * error + pid.ki * tentative_integral + pid.kd * shadow.derivative_filtered;

    const float out_clamped = clampf(out, -pid.out_limit, pid.out_limit);

    const bool saturating_same_direction =
        (out != out_clamped) && ((error > 0.0f && out > 0.0f) || (error < 0.0f && out < 0.0f));

    if (!saturating_same_direction) {
        pid.integral = tentative_integral;
    } else {
        pid.integral *= (1.0f - SPEED_PID_INTEGRAL_RELEASE_GAIN);
    }

    pid.last_error = error;
    return out_clamped;
}

void pid_reset(pid_controller_t &pid)
{
    pid.integral = 0.0f;
    pid.last_error = 0.0f;
    shadow_reset(&pid);
}

/* ========================= 模糊 PD 区域 ========================= */

static float g_error_max = 120.0f;
static float g_derr_max = 50.0f;

static float g_kp_fuzzy = 2.0f;
static float g_kd_fuzzy = 10.0f;

static float g_kp_base = 8.0f;
static float g_kd_base = 16.0f;

static const float g_error_domain[7] = {0.0f, 3.5f, 4.0f, 4.5f, 5.0f, 5.5f, 6.0f};
static const float g_derr_domain[7] = {-3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f};

static const float g_kp_rule[7][7] = {
    {0,0,0,0,0,0,0},
    {1,1,1,1,1,1,1},
    {1,1,1,2,3,3,3},
    {2,2,3,3,3,4,4},
    {3,3,4,4,4,5,5},
    {4,5,5,5,5,5,6},
    {6,6,6,6,6,6,6}
};

static const float g_kd_rule[7][7] = {
    {6,5,4,4,4,3,2},
    {6,5,4,4,4,3,2},
    {6,5,4,3,3,2,2},
    {5,4,3,3,2,2,1},
    {4,3,3,3,2,1,0},
    {3,3,3,2,2,1,0},
    {3,3,3,2,2,1,0}
};

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
    static float derr_filtered = 0.0f;

    const float raw_derr = error - last_error;
    derr_filtered = derr_filtered * 0.65f + raw_derr * 0.35f;

    float error_mapped = (error / g_error_max) * 6.0f;
    float derr_mapped = (derr_filtered / g_derr_max) * 3.0f;

    if (error_mapped < 0.0f) {
        error_mapped = -error_mapped;
        derr_mapped = -derr_mapped;
    }

    if (last_error * error < 0.0f) {
        derr_mapped = 3.0f;
    }

    derr_mapped = clampf(derr_mapped, -3.0f, 3.0f);
    error_mapped = clampf(error_mapped, 0.0f, 6.0f);

    int error_index_pair[2];
    int derr_index_pair[2];
    float error_degree_pair[2];
    float derr_degree_pair[2];

    membership(error_mapped, error_index_pair, error_degree_pair, g_error_domain);
    membership(derr_mapped, derr_index_pair, derr_degree_pair, g_derr_domain);

    float kp_rule_sum = 0.0f;
    float kd_rule_sum = 0.0f;

    kp_rule_sum += error_degree_pair[0] * derr_degree_pair[0] * g_kp_rule[error_index_pair[0]][derr_index_pair[0]];
    kp_rule_sum += error_degree_pair[0] * derr_degree_pair[1] * g_kp_rule[error_index_pair[0]][derr_index_pair[1]];
    kp_rule_sum += error_degree_pair[1] * derr_degree_pair[0] * g_kp_rule[error_index_pair[1]][derr_index_pair[0]];
    kp_rule_sum += error_degree_pair[1] * derr_degree_pair[1] * g_kp_rule[error_index_pair[1]][derr_index_pair[1]];

    kd_rule_sum += error_degree_pair[0] * derr_degree_pair[0] * g_kd_rule[error_index_pair[0]][derr_index_pair[0]];
    kd_rule_sum += error_degree_pair[0] * derr_degree_pair[1] * g_kd_rule[error_index_pair[0]][derr_index_pair[1]];
    kd_rule_sum += error_degree_pair[1] * derr_degree_pair[0] * g_kd_rule[error_index_pair[1]][derr_index_pair[0]];
    kd_rule_sum += error_degree_pair[1] * derr_degree_pair[1] * g_kd_rule[error_index_pair[1]][derr_index_pair[1]];

    const float error_ratio = clampf(std::fabs(error) / (0.75f * g_error_max + 1e-6f), 0.0f, 1.0f);
    const float oscillation_boost = (last_error * error < 0.0f) ? 1.0f : 0.0f;

    kp = g_kp_base + (kp_rule_sum / 6.0f) * g_kp_fuzzy + 0.35f * g_kp_fuzzy * error_ratio;
    kd = g_kd_base + (kd_rule_sum / 6.0f) * g_kd_fuzzy +
         (0.25f * error_ratio + 0.35f * oscillation_boost) * g_kd_fuzzy;

    kp = clampf(kp, 0.5f * g_kp_base, g_kp_base + 1.5f * g_kp_fuzzy + 6.0f);
    kd = clampf(kd, 0.5f * g_kd_base, g_kd_base + 1.5f * g_kd_fuzzy + 16.0f);

    last_error = error;
}
