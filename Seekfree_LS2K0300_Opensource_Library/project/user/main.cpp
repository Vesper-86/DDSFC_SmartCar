#include "zf_common_headfile.hpp"
#include "zf_driver_encoder.hpp"
#include "app_types.hpp"
#include "app_config.hpp"
#include "uvc_camera.hpp"
#include "vision_uart.hpp"
#include "line_track.hpp"
#include "control_utils.hpp"
#include "seekfree_hal.hpp"

#include <csignal>
#include <cstdio>
#include <cstring>

/*
 * ============================================================================
 * 文件名称: main.cpp
 * 文件用途: 三轮智能车统一调度入口（低速稳定安全版）
 *
 * 当前版本特点:
 * 1. 支持无编码器开环和有编码器闭环两种模式切换
 * 2. 默认配置低速安全版，适合调试和初学
 * 3. 保留完整TCP图传
 * ============================================================================
 */

/* 调试打印功能开关 */
#ifndef MOTOR_DEBUG_PRINT_ENABLE
#define MOTOR_DEBUG_PRINT_ENABLE 1
#endif

#ifndef MOTOR_DEBUG_PRINT_PERIOD_MS
#define MOTOR_DEBUG_PRINT_PERIOD_MS 100
#endif

/* 全局运行标志，用于响应 Ctrl+C 信号 */
static volatile int g_app_running = 1;

/* 编码器设备对象 */
static zf_driver_encoder g_encoder_left(ENCODER_LEFT_PATH);
static zf_driver_encoder g_encoder_right(ENCODER_RIGHT_PATH);

/* TCP 客户端对象 */
static zf_driver_tcp_client g_tcp_client;

/* TCP图传缓存 */
static uint8 g_tcp_image_copy[TCP_IMAGE_HEIGHT][TCP_IMAGE_WIDTH];
static volatile int g_tcp_send_guard = 0;
static int g_tcp_enabled = 0;

/* XY边界缓存 */
static uint8 g_xy_x1_boundary[BOUNDARY_NUM], g_xy_x2_boundary[BOUNDARY_NUM], g_xy_x3_boundary[BOUNDARY_NUM];
static uint8 g_xy_y1_boundary[BOUNDARY_NUM], g_xy_y2_boundary[BOUNDARY_NUM], g_xy_y3_boundary[BOUNDARY_NUM];
static uint8 g_x1_boundary[TCP_IMAGE_HEIGHT], g_x2_boundary[TCP_IMAGE_HEIGHT], g_x3_boundary[TCP_IMAGE_HEIGHT];

/* 电机/编码器调试变量 */
static int16 g_dbg_left_count = 0;
static int16 g_dbg_right_count = 0;
static float g_dbg_left_raw_rps = 0.0f;
static float g_dbg_right_raw_rps = 0.0f;

/*
 * 函数名称: on_sigint
 * 功能描述: 响应 Ctrl+C 信号
 */
static void on_sigint(int)
{
    g_app_running = 0;
}

/*
 * 函数名称: tcp_send_wrap
 * 功能描述: TCP发送包装函数
 */
static uint32 tcp_send_wrap(const uint8 *buf, uint32 len)
{
    return g_tcp_client.send_data(buf, len);
}

/*
 * 函数名称: tcp_read_wrap
 * 功能描述: TCP接收包装函数
 */
static uint32 tcp_read_wrap(uint8 *buf, uint32 len)
{
    return g_tcp_client.read_data(buf, len);
}

/*
 * 函数名称: assistant_camera_config
 * 功能描述: 配置逐飞助手相机和边界
 */
static void assistant_camera_config(void)
{
    std::memset(g_tcp_image_copy, 0, sizeof(g_tcp_image_copy));

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        g_tcp_image_copy[0],
        TCP_IMAGE_WIDTH,
        TCP_IMAGE_HEIGHT);

    std::memset(g_xy_x1_boundary, 0, sizeof(g_xy_x1_boundary));
    std::memset(g_xy_x2_boundary, 0, sizeof(g_xy_x2_boundary));
    std::memset(g_xy_x3_boundary, 0, sizeof(g_xy_x3_boundary));
    std::memset(g_xy_y1_boundary, 0, sizeof(g_xy_y1_boundary));
    std::memset(g_xy_y2_boundary, 0, sizeof(g_xy_y2_boundary));
    std::memset(g_xy_y3_boundary, 0, sizeof(g_xy_y3_boundary));

    seekfree_assistant_camera_boundary_config(
        XY_BOUNDARY,
        1,
        g_xy_x1_boundary, g_xy_x2_boundary, g_xy_x3_boundary,
        g_xy_y1_boundary, g_xy_y2_boundary, g_xy_y3_boundary);
}

/*
 * 函数名称: init_tcp_transfer
 * 功能描述: 初始化TCP图传
 */
static int init_tcp_transfer()
{
#if TCP_ASSISTANT_ENABLE
    if (g_tcp_client.init(TCP_SERVER_IP, TCP_SERVER_PORT) == 0)
    {
        std::printf("tcp_client ok\r\n");
        seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);
        assistant_camera_config();
        g_tcp_enabled = 1;
        return 0;
    }

    std::printf("tcp_client error, tcp gray stream disabled\r\n");
    g_tcp_enabled = 0;
    return -1;
#else
    g_tcp_enabled = 0;
    return -1;
#endif
}

/*
 * 函数名称: tcp_update_boundaries
 * 功能描述: 更新TCP图传边界点
 */
static void tcp_update_boundaries(const frame_t &frame, const track_result_t &track)
{
    if (frame.width != TCP_IMAGE_WIDTH || frame.height != TCP_IMAGE_HEIGHT)
    {
        return;
    }

    int count = 0;
    const int y0 = clampi(TRACK_ROI_Y0, 0, TCP_IMAGE_HEIGHT - 1);
    const int y1 = clampi(TRACK_ROI_Y1, y0 + 1, TCP_IMAGE_HEIGHT);

    for (int y = y1 - 1; y >= y0 && count < BOUNDARY_NUM; --y)
    {
        const int left = clampi(track.full_left[y], 0, TCP_IMAGE_WIDTH - 1);
        const int center = clampi(track.full_center[y], 0, TCP_IMAGE_WIDTH - 1);
        const int right = clampi(track.full_right[y], 0, TCP_IMAGE_WIDTH - 1);

        g_xy_x1_boundary[count] = static_cast<uint8>(left);
        g_xy_y1_boundary[count] = static_cast<uint8>(y);
        g_xy_x2_boundary[count] = static_cast<uint8>(center);
        g_xy_y2_boundary[count] = static_cast<uint8>(y);
        g_xy_x3_boundary[count] = static_cast<uint8>(right);
        g_xy_y3_boundary[count] = static_cast<uint8>(y);
        ++count;
    }

    if (count <= 0)
    {
        count = 1;
        g_xy_x1_boundary[0] = 0;
        g_xy_y1_boundary[0] = static_cast<uint8>(y0);
        g_xy_x2_boundary[0] = TCP_IMAGE_WIDTH / 2;
        g_xy_y2_boundary[0] = static_cast<uint8>(y0);
        g_xy_x3_boundary[0] = TCP_IMAGE_WIDTH - 1;
        g_xy_y3_boundary[0] = static_cast<uint8>(y0);
    }

    seekfree_assistant_camera_boundary_config(
        XY_BOUNDARY,
        count,
        g_xy_x1_boundary, g_xy_x2_boundary, g_xy_x3_boundary,
        g_xy_y1_boundary, g_xy_y2_boundary, g_xy_y3_boundary);
}

/*
 * 函数名称: tcp_send_gray_frame
 * 功能描述: 发送一帧图像到逐飞助手
 */
static void tcp_send_gray_frame(const frame_t &frame, const track_result_t &track)
{
    if (!g_tcp_enabled || !frame.valid || frame.gray == nullptr)
    {
        return;
    }

    if (g_tcp_send_guard)
    {
        return;
    }

    g_tcp_send_guard = 1;

#if TCP_SHOW_BINARY_IMAGE
    const uint8 *tcp_image_src = line_track_get_binary_image();
    if (tcp_image_src == nullptr)
    {
        tcp_image_src = frame.gray;
    }
#else
    const uint8 *tcp_image_src = frame.gray;
#endif

    std::memcpy(g_tcp_image_copy[0], tcp_image_src,
                (size_t)TCP_IMAGE_WIDTH * (size_t)TCP_IMAGE_HEIGHT);

    tcp_update_boundaries(frame, track);
    seekfree_assistant_camera_send();

    g_tcp_send_guard = 0;
}

/*
 * 函数名称: tcp_poll_assistant
 * 功能描述: 轮询逐飞助手数据
 */
static void tcp_poll_assistant()
{
    if (!g_tcp_enabled)
    {
        return;
    }
    seekfree_assistant_data_analysis();
}

/*
 * 函数名称: ramp_limit_int
 * 功能描述: 斜坡限幅函数，避免电机命令跳变过大
 */
static int ramp_limit_int(int target, int current, int step)
{
    if (target > current + step)
        return current + step;
    if (target < current - step)
        return current - step;
    return target;
}

/*
 * 函数名称: init_encoder_feedback
 * 功能描述: 初始化编码器反馈
 */
static void init_encoder_feedback(motor_state_t &motor_state)
{
    g_encoder_left.clear_count();
    g_encoder_right.clear_count();

    g_dbg_left_count = 0;
    g_dbg_right_count = 0;
    g_dbg_left_raw_rps = 0.0f;
    g_dbg_right_raw_rps = 0.0f;

    motor_state.left_speed_rps = 0.0f;
    motor_state.right_speed_rps = 0.0f;
}

/*
 * 函数名称: update_encoder_feedback
 * 功能描述: 更新编码器反馈数据
 */
static void update_encoder_feedback(motor_state_t &motor_state)
{
    const float dt_s = CONTROL_PERIOD_MS / 1000.0f;

    const int16 left_count = g_encoder_left.get_count();
    const int16 right_count = g_encoder_right.get_count();

    g_dbg_left_count = left_count;
    g_dbg_right_count = right_count;

    g_encoder_left.clear_count();
    g_encoder_right.clear_count();

    float left_rps = 0.0f;
    float right_rps = 0.0f;

    if (ENCODER_COUNTS_PER_WHEEL_REV > 1e-6f)
    {
        left_rps = ((float)left_count / ENCODER_COUNTS_PER_WHEEL_REV) / dt_s;
        right_rps = ((float)right_count / ENCODER_COUNTS_PER_WHEEL_REV) / dt_s;
    }

    g_dbg_left_raw_rps = left_rps;
    g_dbg_right_raw_rps = right_rps;

    left_rps *= ENCODER_LEFT_SIGN;
    right_rps *= ENCODER_RIGHT_SIGN;

    motor_state.left_speed_rps =
        motor_state.left_speed_rps * (1.0f - ENCODER_SPEED_FILTER_ALPHA) +
        left_rps * ENCODER_SPEED_FILTER_ALPHA;

    motor_state.right_speed_rps =
        motor_state.right_speed_rps * (1.0f - ENCODER_SPEED_FILTER_ALPHA) +
        right_rps * ENCODER_SPEED_FILTER_ALPHA;
}

/*
 * 函数名称: debug_print_motor_info
 * 功能描述: 打印电机/编码器调试信息
 */
static void debug_print_motor_info(const app_context_t &app)
{
#if MOTOR_DEBUG_PRINT_ENABLE
    std::printf(
        "MDBG LC=%d RC=%d RAW_L=%.3f RAW_R=%.3f "
        "LS=%.3f RS=%.3f LT=%.3f RT=%.3f "
        "CMD_L=%d CMD_R=%d ERR=%.1f VALID=%d LOST=%d BLK=%d\r\n",
        (int)g_dbg_left_count,
        (int)g_dbg_right_count,
        g_dbg_left_raw_rps,
        g_dbg_right_raw_rps,
        app.motor.left_speed_rps,
        app.motor.right_speed_rps,
        app.motor.left_target_rps,
        app.motor.right_target_rps,
        app.motor.left_cmd,
        app.motor.right_cmd,
        app.track.error_filtered,
        app.track.valid ? 1 : 0,
        app.track.lost ? 1 : 0,
        app.motor.blocked ? 1 : 0);

    std::fflush(stdout);
#else
    (void)app;
#endif
}

/*
 * 函数名称: run_tracking_cycle
 * 功能描述: 运行一次循迹处理
 */
static void run_tracking_cycle(UvcCamera &camera, app_context_t &app)
{
    if (camera.capture(app.frame) == 0)
    {
        camera.yuyv_to_gray(app.frame);
        line_track_process(app.frame, app.track);
    }
    else
    {
        app.frame.valid = false;
        app.track.valid = false;
        app.track.lost = true;
    }
}

/*
 * 函数名称: update_motion_control
 * 功能描述: 运动控制核心函数
 */
static void update_motion_control(app_context_t &app,
                                  pid_controller_t &left_speed_pid,
                                  pid_controller_t &right_speed_pid,
                                  MotorDriver8701 &motor_driver)
{
    float speed_scale = 1.0f;
    float adaptive_kp = STEER_PD_KP_BASE;
    float adaptive_kd = STEER_PD_KD_BASE;

    static float last_track_error = 0.0f;
    static int lost_count = 0;
    static int last_left_cmd = 0;
    static int last_right_cmd = 0;

    if (app.motor.blocked)
    {
        pid_reset(left_speed_pid);
        pid_reset(right_speed_pid);
        motor_driver.stop(app.motor);
        last_left_cmd = 0;
        last_right_cmd = 0;
        last_track_error = 0.0f;
        return;
    }

    const bool track_ok = app.track.valid &&
                         ((app.now_ms - app.track.timestamp_ms) <= CAMERA_TIMEOUT_MS);

    if (track_ok)
    {
        lost_count = 0;

        fuzzy_pd_set_base(STEER_PD_KP_BASE, STEER_PD_KD_BASE);
        fuzzy_pd_set_limit(STEER_PD_ERR_MAX, STEER_PD_DERR_MAX,
                           STEER_PD_KP_FUZZY, STEER_PD_KD_FUZZY);
        fuzzy_pd_update(app.track.error_filtered, adaptive_kp, adaptive_kd);

        const float error_delta = app.track.error_filtered - last_track_error;
        float differential_output = adaptive_kp * app.track.error_filtered + adaptive_kd * error_delta;
        differential_output = clampf(differential_output, -DIFF_OUT_LIMIT, DIFF_OUT_LIMIT);

        speed_scale = line_track_compute_speed_scale(app.track);

#if ENCODER_FEEDBACK_ENABLE
        const float base_speed = MOTOR_BASE_SPEED * speed_scale;
        app.motor.left_target_rps = clampf(base_speed - differential_output, 0.0f, MOTOR_TURN_SPEED_LIMIT) / 100.0f;
        app.motor.right_target_rps = clampf(base_speed + differential_output, 0.0f, MOTOR_TURN_SPEED_LIMIT) / 100.0f;

        int left_output_command = (int)pid_update(left_speed_pid, app.motor.left_target_rps,
                                               app.motor.left_speed_rps, CONTROL_PERIOD_MS / 1000.0f);
        int right_output_command = (int)pid_update(right_speed_pid, app.motor.right_target_rps,
                                                app.motor.right_speed_rps, CONTROL_PERIOD_MS / 1000.0f);
#else
        const int base_pwm = (int)(OPEN_LOOP_BASE_PWM * speed_scale);
        const int diff_pwm = (int)(differential_output);

        int left_output_command = base_pwm - diff_pwm;
        int right_output_command = base_pwm + diff_pwm;

        app.motor.left_target_rps = 0.0f;
        app.motor.right_target_rps = 0.0f;
#endif

        last_track_error = app.track.error_filtered;
    }
    else
    {
        ++lost_count;
        if (lost_count >= TRACK_LOST_STOP_COUNT)
        {
            app.motor.left_target_rps = 0.0f;
            app.motor.right_target_rps = 0.0f;
            pid_reset(left_speed_pid);
            pid_reset(right_speed_pid);
            motor_driver.stop(app.motor);
            last_left_cmd = 0;
            last_right_cmd = 0;
            last_track_error = 0.0f;
            return;
        }
        else
        {
            last_left_cmd = (int)(last_left_cmd * 0.85f);
            last_right_cmd = (int)(last_right_cmd * 0.85f);
            int left_output_command = last_left_cmd;
            int right_output_command = last_right_cmd;

            left_output_command = clampi(left_output_command, -MOTOR_FINAL_OUTPUT_LIMIT, MOTOR_FINAL_OUTPUT_LIMIT);
            right_output_command = clampi(right_output_command, -MOTOR_FINAL_OUTPUT_LIMIT, MOTOR_FINAL_OUTPUT_LIMIT);

            motor_driver.set_cmd(left_output_command, right_output_command, app.motor);
            return;
        }
    }

    left_output_command = ramp_limit_int(left_output_command, last_left_cmd, MOTOR_CMD_RAMP_STEP);
    right_output_command = ramp_limit_int(right_output_command, last_right_cmd, MOTOR_CMD_RAMP_STEP);

    left_output_command = clampi(left_output_command, -MOTOR_FINAL_OUTPUT_LIMIT, MOTOR_FINAL_OUTPUT_LIMIT);
    right_output_command = clampi(right_output_command, -MOTOR_FINAL_OUTPUT_LIMIT, MOTOR_FINAL_OUTPUT_LIMIT);

    last_left_cmd = left_output_command;
    last_right_cmd = right_output_command;

    motor_driver.set_cmd(left_output_command, right_output_command, app.motor);
}

int main()
{
    std::signal(SIGINT, on_sigint);

    app_context_t app;
    std::memset(&app, 0, sizeof(app));

    UvcCamera camera;
    VisionUart vision_uart;
    MotorDriver8701 motor_driver;
    pid_controller_t left_speed_pid;
    pid_controller_t right_speed_pid;

    uint64_t last_control_tick_ms = 0;
    uint64_t last_track_tick_ms = 0;
    uint64_t last_uart_tick_ms = 0;
    uint64_t last_display_tick_ms = 0;
    uint64_t last_tcp_tick_ms = 0;
    uint64_t last_tcp_poll_tick_ms = 0;
    uint64_t last_debug_tick_ms = 0;

    line_track_init(app.track);
    app.system_state = SYS_READY;
    app.now_ms = app_millis();

    (void)init_tcp_transfer();

    if (camera.init(app.frame) != 0)
    {
        std::printf("camera init error\r\n");
        return -1;
    }

    if (motor_driver.init(app.motor) != 0)
    {
        std::printf("motor init error\r\n");
        return -1;
    }

#if VISION_UART_ENABLE
    if (vision_uart.init() != 0)
    {
        std::printf("vision uart init warning\r\n");
    }
#endif

    init_encoder_feedback(app.motor);

    pid_init(left_speed_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
             SPEED_PID_I_LIMIT, SPEED_PID_OUT_LIMIT);
    pid_init(right_speed_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
             SPEED_PID_I_LIMIT, SPEED_PID_OUT_LIMIT);

    std::printf("%s start, version=%s, encoder=%d\r\n",
               APP_NAME, APP_VERSION, ENCODER_FEEDBACK_ENABLE);

    while (g_app_running)
    {
        app.now_ms = app_millis();
        app.system_state = app.motor.blocked ? SYS_ERROR : SYS_RUNNING;

        if ((app.now_ms - last_track_tick_ms) >= TRACK_PERIOD_MS)
        {
            last_track_tick_ms = app.now_ms;
            run_tracking_cycle(camera, app);
        }

#if VISION_UART_ENABLE
        if ((app.now_ms - last_uart_tick_ms) >= UART_PERIOD_MS)
        {
            last_uart_tick_ms = app.now_ms;
            vision_uart.poll(app.recog);
        }
#endif

        if ((app.now_ms - last_control_tick_ms) >= CONTROL_PERIOD_MS)
        {
            last_control_tick_ms = app.now_ms;
#if ENCODER_FEEDBACK_ENABLE
            update_encoder_feedback(app.motor);
            motor_driver.check_block(app.motor, app.now_ms);
#else
            /* 无编码器时，仍读取编码器用于调试显示 */
            update_encoder_feedback(app.motor);
#endif
            update_motion_control(app, left_speed_pid, right_speed_pid, motor_driver);
        }

        if ((app.now_ms - last_debug_tick_ms) >= MOTOR_DEBUG_PRINT_PERIOD_MS)
        {
            last_debug_tick_ms = app.now_ms;
            debug_print_motor_info(app);
        }

        if ((app.now_ms - last_tcp_tick_ms) >= TCP_SEND_PERIOD_MS)
        {
            last_tcp_tick_ms = app.now_ms;
            tcp_send_gray_frame(app.frame, app.track);
        }

        if ((app.now_ms - last_tcp_poll_tick_ms) >= TCP_ASSISTANT_POLL_PERIOD_MS)
        {
            last_tcp_poll_tick_ms = app.now_ms;
            tcp_poll_assistant();
        }

        system_delay_ms(1);
    }

    motor_driver.stop(app.motor);
    camera.deinit(app.frame);
    vision_uart.deinit();
    motor_driver.deinit();
    return 0;
}
