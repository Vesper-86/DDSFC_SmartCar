#pragma once

#include "zf_common_headfile.hpp"
#include "app_types.hpp"
#include "app_config.hpp"

/*
 * ============================================================================
 * 文件名称: seekfree_hal.hpp
 * 文件用途: 逐飞开源库硬件适配层
 *
 * 当前只保留两部分：
 * 1. MotorDriver8701 : DRV8701 双电机输出
 * 2. IpsStatusView   : IPS200 状态显示
 *
 * 已删除:
 * - 蜂鸣器适配
 * - debug 串口适配
 * ============================================================================
 */

class MotorDriver8701
{
public:
    MotorDriver8701();
    ~MotorDriver8701();

    /* 初始化 PWM 设备并清零输出。 */
    int init(motor_state_t &motor_state);

    /* 关闭时清零输出。 */
    void deinit();

    /*
     * 设置左右轮逻辑命令。
     * 说明:
     * - 输入是统一的逻辑命令范围 [-10000, 10000]
     * - 函数内部会自动做方向判断、死区处理和 duty 映射
     */
    void set_cmd(int left_command, int right_command, motor_state_t &motor_state);

    /* 快速停车。 */
    void stop(motor_state_t &motor_state);

    /* 堵转检测。 */
    void check_block(motor_state_t &motor_state, uint64_t now_ms);

private:
    zf_driver_gpio dir_left_;     /* 左电机方向引脚 */
    zf_driver_gpio dir_right_;    /* 右电机方向引脚 */
    zf_driver_pwm pwm_left_;      /* 左电机 PWM 设备 */
    zf_driver_pwm pwm_right_;     /* 右电机 PWM 设备 */

    pwm_info pwm_left_info_;      /* 左 PWM 设备信息，含 duty_max */
    pwm_info pwm_right_info_;     /* 右 PWM 设备信息，含 duty_max */
};

class IpsStatusView
{
public:
    IpsStatusView();

    /* 初始化 IPS200。 */
    int init();

    /* 清屏。 */
    void clear();

    /* 显示启动界面。 */
    void show_boot(const char *title, const char *line2, const char *line3);

    /* 显示三行状态字符串。 */
    void show_status(const char *line1, const char *line2, const char *line3);

    /* 显示灰度图像 + 三行状态字符串，图像区和文字区分离，避免覆盖。 */
    void show_image_status(const frame_t &frame, const char *line1, const char *line2, const char *line3);

private:
    zf_device_ips200 ips_;        /* 逐飞 IPS200 设备对象 */
};
