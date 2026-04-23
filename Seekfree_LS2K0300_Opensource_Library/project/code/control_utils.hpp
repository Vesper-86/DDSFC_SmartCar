#pragma once

#include "app_types.hpp"

/*
 * PID 相关函数
 * ---------------------------------------------------------------------------
 * pid_init()   : 初始化 PID 参数和内部状态
 * pid_update() : 输入目标值、反馈值和周期，输出本轮控制量
 * pid_reset()  : 清零积分和上一轮误差
 * ---------------------------------------------------------------------------
 */
void pid_init(pid_controller_t &pid, float kp, float ki, float kd, float i_limit, float out_limit);
float pid_update(pid_controller_t &pid, float target, float measure, float dt_s);
void pid_reset(pid_controller_t &pid);

/*
 * 模糊 PD 相关函数
 * ---------------------------------------------------------------------------
 * fuzzy_pd_set_base()  : 设置基础 KP / KD
 * fuzzy_pd_set_limit() : 设置误差归一化范围和模糊修正量
 * fuzzy_pd_update()    : 根据当前误差在线计算新的 KP / KD
 * ---------------------------------------------------------------------------
 */
void fuzzy_pd_set_base(float kp_base, float kd_base);
void fuzzy_pd_set_limit(float err_max, float derr_max, float kp_fuzzy, float kd_fuzzy);
void fuzzy_pd_update(float error, float &kp, float &kd);
