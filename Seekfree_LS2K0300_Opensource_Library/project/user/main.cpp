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
 * 文件用途: 三轮智能车（无舵机）统一调度入口
 *
 * 核心链路:
 * 1. 相机采集灰度图
 * 2. 循迹算法输出中心偏差
 * 3. 模糊 PD 计算左右轮差速量
 * 4. 左右轮速度 PID 闭环输出电机命令
 * 5. 可选显示识别/状态信息，可选发送灰度图给逐飞助手
 *
 * 车辆结构说明:
 * - 本工程适用于“前/后双驱 + 第三轮随动”的无舵机差速车。
 * - 不使用舵机，方向完全由左右轮目标速度差实现。
 * ============================================================================
 */

static volatile int g_app_running = 1;

/* 左右编码器对象。 */
static zf_driver_encoder g_encoder_left(ENCODER_LEFT_PATH);
static zf_driver_encoder g_encoder_right(ENCODER_RIGHT_PATH);

/* TCP 图传客户端。 */
static zf_driver_tcp_client g_tcp_client;

/* 图传使用的灰度图拷贝缓存。 */
static uint8 g_image_copy[CAM_HEIGHT][CAM_WIDTH];

/* 防止重复发送的保护标志。 */
static volatile int g_tcp_send_guard = 0;

/* 当前是否成功启用 TCP 图传。 */
static int g_tcp_enabled = 0;

/* 以下边界数组用于逐飞助手的叠加显示。 */
static uint8 g_xy_x1_boundary[BOUNDARY_NUM], g_xy_x2_boundary[BOUNDARY_NUM], g_xy_x3_boundary[BOUNDARY_NUM];
static uint8 g_xy_y1_boundary[BOUNDARY_NUM], g_xy_y2_boundary[BOUNDARY_NUM], g_xy_y3_boundary[BOUNDARY_NUM];
static uint8 g_x1_boundary[CAM_HEIGHT], g_x2_boundary[CAM_HEIGHT], g_x3_boundary[CAM_HEIGHT];
static uint8 g_y1_boundary[CAM_WIDTH], g_y2_boundary[CAM_WIDTH], g_y3_boundary[CAM_WIDTH];

/* Ctrl+C 退出时置位。 */
static void on_sigint(int)
{
    g_app_running = 0;
}

/* TCP 发送包装函数，交给逐飞助手接口调用。 */
static uint32 tcp_send_wrap(const uint8 *buf, uint32 len)
{
    return g_tcp_client.send_data(buf, len);
}

/* TCP 接收包装函数，交给逐飞助手接口调用。 */
static uint32 tcp_read_wrap(uint8 *buf, uint32 len)
{
    return g_tcp_client.read_data(buf, len);
}

/*
 * assistant_camera_config()
 * ---------------------------------------------------------------------------
 * 配置逐飞助手的图像显示方式。
 */
static void assistant_camera_config(void)
{
#if (0 == INCLUDE_BOUNDARY_TYPE)
    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        g_image_copy[0],
        CAM_WIDTH,
        CAM_HEIGHT);
#elif (1 == INCLUDE_BOUNDARY_TYPE)
    for (int32 i = 0; i < CAM_HEIGHT; i++)
    {
        g_x1_boundary[i] = 50 - (50 - 20) * i / CAM_HEIGHT;
        g_x2_boundary[i] = CAM_WIDTH / 2;
        g_x3_boundary[i] = 70 + (148 - 70) * i / CAM_HEIGHT;
    }

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        g_image_copy[0],
        CAM_WIDTH,
        CAM_HEIGHT);
    seekfree_assistant_camera_boundary_config(
        X_BOUNDARY,
        CAM_HEIGHT,
        g_x1_boundary, g_x2_boundary, g_x3_boundary,
        NULL, NULL, NULL);
#elif (2 == INCLUDE_BOUNDARY_TYPE)
    for (int32 i = 0; i < CAM_WIDTH; i++)
    {
        g_y1_boundary[i] = 50 - (50 - 20) * i / CAM_HEIGHT;
        g_y2_boundary[i] = CAM_WIDTH / 2;
        g_y3_boundary[i] = 78 + (78 - 58) * i / CAM_HEIGHT;
    }

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        g_image_copy[0],
        CAM_WIDTH,
        CAM_HEIGHT);
    seekfree_assistant_camera_boundary_config(
        Y_BOUNDARY,
        CAM_WIDTH,
        NULL, NULL, NULL,
        g_y1_boundary, g_y2_boundary, g_y3_boundary);
#elif (4 == INCLUDE_BOUNDARY_TYPE)
    for (int32 i = 0; i < CAM_HEIGHT; i++)
    {
        g_x1_boundary[i] = 70 - (70 - 20) * i / CAM_HEIGHT;
        g_x2_boundary[i] = CAM_WIDTH / 2;
        g_x3_boundary[i] = 80 + (159 - 80) * i / CAM_HEIGHT;
    }

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        NULL,
        CAM_WIDTH,
        CAM_HEIGHT);
    seekfree_assistant_camera_boundary_config(
        X_BOUNDARY,
        CAM_HEIGHT,
        g_x1_boundary, g_x2_boundary, g_x3_boundary,
        NULL, NULL, NULL);
#else
    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        g_image_copy[0],
        CAM_WIDTH,
        CAM_HEIGHT);
#endif

    (void)g_xy_x1_boundary;
    (void)g_xy_x2_boundary;
    (void)g_xy_x3_boundary;
    (void)g_xy_y1_boundary;
    (void)g_xy_y2_boundary;
    (void)g_xy_y3_boundary;
}

/* 初始化 TCP 图传。 */
static int init_tcp_transfer(void)
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

/* 发送灰度图到逐飞助手。 */
static void tcp_send_gray_frame(const frame_t &frame)
{
    if (!g_tcp_enabled || !frame.valid || frame.gray == nullptr)
    {
        return;
    }

    if (g_tcp_send_guard)
    {
        return;
    }

    if (frame.width != CAM_WIDTH || frame.height != CAM_HEIGHT)
    {
        return;
    }

    g_tcp_send_guard = 1;
    std::memcpy(g_image_copy[0], frame.gray, (size_t)CAM_WIDTH * (size_t)CAM_HEIGHT);
    seekfree_assistant_camera_send();
    g_tcp_send_guard = 0;
}

/* 轮询逐飞助手控制指令。 */
static void tcp_poll_assistant(void)
{
    if (!g_tcp_enabled)
    {
        return;
    }
    seekfree_assistant_data_analysis();
}

/* 对电机命令做斜坡限制，防止突变。 */
static int ramp_limit_int(int target, int current, int step)
{
    if (target > current + step)
        return current + step;
    if (target < current - step)
        return current - step;
    return target;
}

/* 初始化编码器反馈。 */
static void init_encoder_feedback(motor_state_t &motor_state)
{
    g_encoder_left.clear_count();
    g_encoder_right.clear_count();

    motor_state.left_speed_rps = 0.0f;
    motor_state.right_speed_rps = 0.0f;
}

/*
 * update_encoder_feedback()
 * ---------------------------------------------------------------------------
 * 读取左右轮编码器，并计算轮速反馈值。
 */
static void update_encoder_feedback(motor_state_t &motor_state)
{
    const float dt_s = CONTROL_PERIOD_MS / 1000.0f;

    const int16 left_count = g_encoder_left.get_count();
    const int16 right_count = g_encoder_right.get_count();

    g_encoder_left.clear_count();
    g_encoder_right.clear_count();

    float left_rps = 0.0f;
    float right_rps = 0.0f;

    if (ENCODER_COUNTS_PER_WHEEL_REV > 1e-6f)
    {
        left_rps = ((float)left_count / ENCODER_COUNTS_PER_WHEEL_REV) / dt_s;
        right_rps = ((float)right_count / ENCODER_COUNTS_PER_WHEEL_REV) / dt_s;
    }

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
 * run_tracking_cycle()
 * ---------------------------------------------------------------------------
 * 完成一次图像采集与循迹处理。
 */
static void run_tracking_cycle(UvcCamera &camera, app_context_t &app)
{
    static int first_valid_frame_printed = 0;

    if (camera.capture(app.frame) == 0)
    {
        camera.yuyv_to_gray(app.frame);
        line_track_process(app.frame, app.track);

        if (!first_valid_frame_printed && app.frame.valid && app.frame.gray != nullptr)
        {
            std::printf("first valid frame ok: %d x %d\r\n", app.frame.width, app.frame.height);
            first_valid_frame_printed = 1;
        }
    }
    else
    {
        app.frame.valid = false;
        app.track.valid = false;
        app.track.lost = true;
    }
}

/*
 * update_motion_control()
 * ---------------------------------------------------------------------------
 * 统一控制逻辑:
 * 1. 从循迹误差生成差速量 differential_output
 * 2. 得到左右轮目标速度 left_target_rps / right_target_rps
 * 3. 左右轮速度 PID 输出最终电机命令
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

    const bool track_ok =
        app.track.valid &&
        ((app.now_ms - app.track.timestamp_ms) <= CAMERA_TIMEOUT_MS);

    if (track_ok)
    {
        lost_count = 0;

        fuzzy_pd_set_base(STEER_PD_KP_BASE, STEER_PD_KD_BASE);
        fuzzy_pd_set_limit(STEER_PD_ERR_MAX,
                           STEER_PD_DERR_MAX,
                           STEER_PD_KP_FUZZY,
                           STEER_PD_KD_FUZZY);
        fuzzy_pd_update(app.track.error_filtered, adaptive_kp, adaptive_kd);

        const float error_delta = app.track.error_filtered - last_track_error;
        float differential_output =
            adaptive_kp * app.track.error_filtered + adaptive_kd * error_delta;
        differential_output = clampf(differential_output, -DIFF_OUT_LIMIT, DIFF_OUT_LIMIT);

        speed_scale = line_track_compute_speed_scale(app.track);
        const float base_speed = MOTOR_BASE_SPEED * speed_scale;

        app.motor.left_target_rps =
            clampf(base_speed - differential_output, 0.0f, MOTOR_TURN_SPEED_LIMIT) / 100.0f;
        app.motor.right_target_rps =
            clampf(base_speed + differential_output, 0.0f, MOTOR_TURN_SPEED_LIMIT) / 100.0f;

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
            app.motor.left_target_rps *= 0.85f;
            app.motor.right_target_rps *= 0.85f;
        }
    }

    int left_output_command = (int)pid_update(left_speed_pid,
                                              app.motor.left_target_rps,
                                              app.motor.left_speed_rps,
                                              CONTROL_PERIOD_MS / 1000.0f);

    int right_output_command = (int)pid_update(right_speed_pid,
                                               app.motor.right_target_rps,
                                               app.motor.right_speed_rps,
                                               CONTROL_PERIOD_MS / 1000.0f);

    left_output_command = ramp_limit_int(left_output_command, last_left_cmd, MOTOR_CMD_RAMP_STEP);
    right_output_command = ramp_limit_int(right_output_command, last_right_cmd, MOTOR_CMD_RAMP_STEP);

    last_left_cmd = left_output_command;
    last_right_cmd = right_output_command;

    motor_driver.set_cmd(left_output_command, right_output_command, app.motor);
}

/* 把系统状态转换成短字符串，供屏幕显示。 */
static const char *system_state_name(system_state_e state)
{
    switch (state)
    {
    case SYS_READY:   return "READY";
    case SYS_RUNNING: return "RUN";
    case SYS_ERROR:   return "ERR";
    default:          return "UNK";
    }
}

/*
 * update_display_view()
 * ---------------------------------------------------------------------------
 * 用三行短字符串显示当前主状态。
 */
static void update_display_view(IpsStatusView &display, const app_context_t &app)
{
#if DISPLAY_ENABLE
    char line1[64];
    char line2[64];
    char line3[64];

    std::snprintf(line1,
                  sizeof(line1),
                  "%s T:%c E:%5.1f",
                  system_state_name(app.system_state),
                  app.track.valid ? 'Y' : 'N',
                  app.track.error_filtered);

    std::snprintf(line2,
                  sizeof(line2),
                  "L:%4.2f/%4.2f R:%4.2f/%4.2f",
                  app.motor.left_speed_rps,
                  app.motor.left_target_rps,
                  app.motor.right_speed_rps,
                  app.motor.right_target_rps);

    std::snprintf(line3,
                  sizeof(line3),
                  "CLS:%u ST:%u SC:%0.2f",
                  (unsigned)app.recog.cls,
                  (unsigned)app.recog.stable_cls,
                  app.recog.score);

    display.show_status(line1, line2, line3);
#else
    (void)display;
    (void)app;
#endif
}

int main()
{
    std::signal(SIGINT, on_sigint);

    app_context_t app;
    std::memset(&app, 0, sizeof(app));

    UvcCamera camera;
    VisionUart vision_uart;
    MotorDriver8701 motor_driver;
    IpsStatusView display_view;
    pid_controller_t left_speed_pid;
    pid_controller_t right_speed_pid;

    uint64_t last_control_tick_ms = 0;
    uint64_t last_track_tick_ms = 0;
    uint64_t last_uart_tick_ms = 0;
    uint64_t last_display_tick_ms = 0;
    uint64_t last_tcp_tick_ms = 0;
    uint64_t last_tcp_poll_tick_ms = 0;

    line_track_init(app.track);
    app.system_state = SYS_READY;
    app.now_ms = app_millis();

    if (display_view.init() == 0)
    {
        display_view.show_boot(APP_NAME, "three-wheel no-servo", APP_VERSION);
    }

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

    std::printf("%s start, version = %s\r\n", APP_NAME, APP_VERSION);

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
            update_encoder_feedback(app.motor);
            motor_driver.check_block(app.motor, app.now_ms);
            update_motion_control(app, left_speed_pid, right_speed_pid, motor_driver);
        }

        if ((app.now_ms - last_display_tick_ms) >= DISPLAY_PERIOD_MS)
        {
            last_display_tick_ms = app.now_ms;
            update_display_view(display_view, app);
        }

        if ((app.now_ms - last_tcp_tick_ms) >= TCP_SEND_PERIOD_MS)
        {
            last_tcp_tick_ms = app.now_ms;
            tcp_send_gray_frame(app.frame);
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
    display_view.clear();
    return 0;
}
