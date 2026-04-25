#include "seekfree_hal.hpp"
#include "line_track.hpp"
#include <cstring>

#ifndef RGB565_RED
#define RGB565_RED 0xF800
#endif
#ifndef RGB565_GREEN
#define RGB565_GREEN 0x07E0
#endif
#ifndef RGB565_BLUE
#define RGB565_BLUE 0x001F
#endif

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
namespace
{
static int ips_compute_scan_y(int scan_index, int image_height)
{
    const int roi_height = TRACK_ROI_Y1 - TRACK_ROI_Y0;
    int scan_step = (roi_height > 0) ? (roi_height / TRACK_SCAN_LINES) : 1;
    if (scan_step <= 0)
    {
        scan_step = 1;
    }

    int y = TRACK_ROI_Y1 - 1 - scan_index * scan_step;
    if (y < 0) y = 0;
    if (y >= image_height) y = image_height - 1;
    return y;
}

static void ips_draw_safe_point(zf_device_ips200 &ips, int x, int y, uint16 color)
{
    if (x < 0 || x >= IPS_IMAGE_W || y < 0 || y >= IPS_IMAGE_H)
    {
        return;
    }
    ips.draw_point(static_cast<uint16>(x + IPS_IMAGE_X), static_cast<uint16>(y + IPS_IMAGE_Y), color);
}

static void ips_draw_cross(zf_device_ips200 &ips, int x, int y, uint16 color)
{
#if TRACK_OVERLAY_ENABLE
    const int half = TRACK_OVERLAY_POINT_HALF_SIZE;
    for (int dy = -half; dy <= half; ++dy)
    {
        ips_draw_safe_point(ips, x, y + dy, color);
    }
    for (int dx = -half; dx <= half; ++dx)
    {
        ips_draw_safe_point(ips, x + dx, y, color);
    }
#else
    (void)ips; (void)x; (void)y; (void)color;
#endif
}

static void ips_draw_curve(zf_device_ips200 &ips, const int *sample_y, const int *sample_x, int sample_count, uint16 color)
{
#if TRACK_OVERLAY_ENABLE
    if (sample_count <= 0)
    {
        return;
    }

    for (int i = 0; i < sample_count - 1; ++i)
    {
        const int y0 = sample_y[i];
        const int y1 = sample_y[i + 1];
        const int x0 = sample_x[i];
        const int x1 = sample_x[i + 1];
        const int dy = y1 - y0;

        if (dy == 0)
        {
            ips_draw_safe_point(ips, x0, y0, color);
            continue;
        }

        const int y_begin = (y0 < y1) ? y0 : y1;
        const int y_end = (y0 < y1) ? y1 : y0;
        for (int y = y_begin; y <= y_end; ++y)
        {
            const float t = static_cast<float>(y - y0) / static_cast<float>(dy);
            const int x = static_cast<int>(x0 + (x1 - x0) * t + ((x1 >= x0) ? 0.5f : -0.5f));
            ips_draw_safe_point(ips, x, y, color);
        }
    }
#else
    (void)ips; (void)sample_y; (void)sample_x; (void)sample_count; (void)color;
#endif
}
}

void IpsStatusView::show_image_status(const frame_t &frame,
                                      const track_result_t &track,
                                      const char *line1,
                                      const char *line2,
                                      const char *line3)
{
#if IPS_IMAGE_ENABLE
    if (frame.valid && frame.gray != nullptr && frame.width > 0 && frame.height > 0)
    {
#if IPS_SHOW_BINARY_IMAGE
        const uint8_t *ips_image_src = line_track_get_binary_image();
        if (ips_image_src == nullptr)
        {
            ips_image_src = frame.gray;
        }
#else
        const uint8_t *ips_image_src = frame.gray;
#endif
        ips_.show_gray_image(IPS_IMAGE_X,
                             IPS_IMAGE_Y,
                             ips_image_src,
                             static_cast<uint16>(frame.width),
                             static_cast<uint16>(frame.height),
                             IPS_IMAGE_W,
                             IPS_IMAGE_H,
                             IPS_IMAGE_THRESHOLD);

#if TRACK_OVERLAY_ENABLE
        if (track.valid)
        {
            int sample_y[CAM_HEIGHT] = {0};
            int sample_left[CAM_HEIGHT] = {0};
            int sample_center[CAM_HEIGHT] = {0};
            int sample_right[CAM_HEIGHT] = {0};
            int sample_count = 0;

            const int draw_y0 = clampi(TRACK_ROI_Y0, 0, frame.height - 1);
            const int draw_y1 = clampi(TRACK_ROI_Y1, draw_y0 + 1, frame.height);
            for (int y = draw_y0; y < draw_y1 && y < CAM_HEIGHT; ++y)
            {
                sample_y[sample_count] = y;
                sample_left[sample_count] = clampi(track.full_left[y], 0, frame.width - 1);
                sample_center[sample_count] = clampi(track.full_center[y], 0, frame.width - 1);
                sample_right[sample_count] = clampi(track.full_right[y], 0, frame.width - 1);
                ++sample_count;
            }

            ips_draw_curve(ips_, sample_y, sample_left, sample_count, RGB565_RED);
            ips_draw_curve(ips_, sample_y, sample_center, sample_count, RGB565_GREEN);
            ips_draw_curve(ips_, sample_y, sample_right, sample_count, RGB565_BLUE);

            for (int i = 0; i < sample_count; i += 8)
            {
                ips_draw_cross(ips_, sample_left[i], sample_y[i], RGB565_RED);
                ips_draw_cross(ips_, sample_center[i], sample_y[i], RGB565_GREEN);
                ips_draw_cross(ips_, sample_right[i], sample_y[i], RGB565_BLUE);
            }
        }
#endif
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
    (void)track;
    show_status(line1, line2, line3);
#endif
}
