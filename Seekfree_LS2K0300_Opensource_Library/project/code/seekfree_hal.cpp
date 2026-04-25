#include "seekfree_hal.hpp"
#include <cstring>

/*
 * ============================================================================
 * 文件名称: seekfree_hal.cpp
 * 文件用途: 逐飞硬件适配层实现
 * ============================================================================
 */

/* ========================= MotorDriver8701 ========================= */
MotorDriver8701::MotorDriver8701()
    : dir_left_(SMARTCAR_DIR_LEFT, O_RDWR),
      dir_right_(SMARTCAR_DIR_RIGHT, O_RDWR),
      pwm_left_(SMARTCAR_PWM_LEFT),
      pwm_right_(SMARTCAR_PWM_RIGHT)
{
    std::memset(&pwm_left_info_, 0, sizeof(pwm_left_info_));
    std::memset(&pwm_right_info_, 0, sizeof(pwm_right_info_));
}

MotorDriver8701::~MotorDriver8701()
{
}

int MotorDriver8701::init(motor_state_t &motor_state)
{
    std::memset(&motor_state, 0, sizeof(motor_state));

    pwm_left_.get_dev_info(&pwm_left_info_);
    pwm_right_.get_dev_info(&pwm_right_info_);

    pwm_left_.set_duty(0);
    pwm_right_.set_duty(0);
    return 0;
}

void MotorDriver8701::deinit()
{
    pwm_left_.set_duty(0);
    pwm_right_.set_duty(0);
}

/* 对小命令做死区清零，降低低速抖动。 */
static int apply_deadzone(int command)
{
    if (command > -MOTOR_DEADZONE_DUTY && command < MOTOR_DEADZONE_DUTY) {
        return 0;
    }
    return command;
}

void MotorDriver8701::set_cmd(int left_command, int right_command, motor_state_t &motor_state)
{
    left_command = clampi(left_command, MOTOR_CMD_MIN, MOTOR_CMD_MAX);
    right_command = clampi(right_command, MOTOR_CMD_MIN, MOTOR_CMD_MAX);

    left_command = apply_deadzone(left_command);
    right_command = apply_deadzone(right_command);

    motor_state.left_cmd = left_command;
    motor_state.right_cmd = right_command;

    const int left_abs_command = (left_command >= 0) ? left_command : -left_command;
    const int right_abs_command = (right_command >= 0) ? right_command : -right_command;

    dir_left_.set_level((left_command >= 0) ? 1 : 0);
    dir_right_.set_level((right_command >= 0) ? 1 : 0);

    const int left_duty =
        (int)((long long)left_abs_command * (long long)pwm_left_info_.duty_max / (long long)MOTOR_CMD_MAX);
    const int right_duty =
        (int)((long long)right_abs_command * (long long)pwm_right_info_.duty_max / (long long)MOTOR_CMD_MAX);

    pwm_left_.set_duty(left_duty);
    pwm_right_.set_duty(right_duty);
}

void MotorDriver8701::stop(motor_state_t &motor_state)
{
    set_cmd(0, 0, motor_state);
    motor_state.left_target_rps = 0.0f;
    motor_state.right_target_rps = 0.0f;
}

void MotorDriver8701::check_block(motor_state_t &motor_state, uint64_t now_ms)
{
    const float target_average = (motor_state.left_target_rps + motor_state.right_target_rps) * 0.5f;
    const float speed_average = (motor_state.left_speed_rps + motor_state.right_speed_rps) * 0.5f;
    const float abs_target = (target_average >= 0.0f) ? target_average : -target_average;
    const float abs_speed = (speed_average >= 0.0f) ? speed_average : -speed_average;

    if (abs_target > 3.0f && abs_speed < MOTOR_BLOCK_SPEED_EPS) {
        if (motor_state.blocked_since_ms == 0) {
            motor_state.blocked_since_ms = now_ms;
        } else if ((now_ms - motor_state.blocked_since_ms) >= MOTOR_BLOCK_TIME_MS) {
            motor_state.blocked = true;
        }
    } else {
        motor_state.blocked_since_ms = 0;
        motor_state.blocked = false;
    }
}

/* ========================= IpsStatusView ========================= */
IpsStatusView::IpsStatusView()
{
}

int IpsStatusView::init()
{
    ips_.init(FB_PATH);
    return 0;
}

void IpsStatusView::clear()
{
    ips_.clear();
}

void IpsStatusView::show_boot(const char *title, const char *line2, const char *line3)
{
    ips_.full(RGB565_BLACK);
    ips_.show_string(0, 0, (char *)title);
    ips_.show_string(0, 16, (char *)line2);
    ips_.show_string(0, 32, (char *)line3);
}

void IpsStatusView::show_status(const char *line1, const char *line2, const char *line3)
{
    ips_.full(RGB565_BLACK);
    ips_.show_string(0, 0, (char *)line1);
    ips_.show_string(0, 16, (char *)line2);
    ips_.show_string(0, 32, (char *)line3);
}

/*
 * show_image_status()
 * ---------------------------------------------------------------------------
 * IPS200 同时显示图像和状态文字：
 * - 图像固定显示在上半区；
 * - 文字固定显示在下半区；
 * - 每次只清理文字区，不整屏 full，避免把刚显示的图像擦掉。
 */
void IpsStatusView::show_image_status(const frame_t &frame,
                                      const char *line1,
                                      const char *line2,
                                      const char *line3)
{
#if IPS_IMAGE_ENABLE
    if (frame.valid && frame.gray != nullptr && frame.width > 0 && frame.height > 0)
    {
        ips_.show_gray_image(IPS_IMAGE_X,
                             IPS_IMAGE_Y,
                             frame.gray,
                             static_cast<uint16>(frame.width),
                             static_cast<uint16>(frame.height),
                             IPS_IMAGE_W,
                             IPS_IMAGE_H,
                             IPS_IMAGE_THRESHOLD);
    }

    for (uint16 y = IPS_TEXT_Y; y < (IPS_TEXT_Y + IPS_TEXT_AREA_H); ++y)
    {
        for (uint16 x = IPS_TEXT_X; x < (IPS_TEXT_X + IPS_TEXT_AREA_W); ++x)
        {
            ips_.draw_point(x, y, RGB565_BLACK);
        }
    }

    ips_.show_string(IPS_TEXT_X, IPS_TEXT_Y, (char *)line1);
    ips_.show_string(IPS_TEXT_X, IPS_TEXT_Y + IPS_TEXT_LINE_H, (char *)line2);
    ips_.show_string(IPS_TEXT_X, IPS_TEXT_Y + 2 * IPS_TEXT_LINE_H, (char *)line3);
#else
    show_status(line1, line2, line3);
#endif
}
