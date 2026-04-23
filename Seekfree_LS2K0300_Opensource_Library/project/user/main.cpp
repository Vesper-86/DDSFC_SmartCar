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
 * 文件用途: 低速稳定安全版主程序入口（TCP图像传输版）
 *
 * 当前版本特点:
 * 1. 保留“摄像头 -> 循迹 -> 控制 -> 电机”主链路。
 * 2. 删除 IPS200 屏幕显示输出。
 * 3. 仿照逐飞官方 TCP 历程，增加灰度图像 TCP 传输。
 * 4. 默认仅传灰度图，不传边界。
 * 5. 识别串口仍只显示结果，不触发任务动作。
 * ============================================================================
 */

/* ========================= TCP 配置 =========================
 * 修改为你电脑运行逐飞助手时的本机 IP 和端口
 * 逐飞助手选择 TCP Server，端口保持一致。
 */
#define TCP_SERVER_IP "10.190.147.185"
#define TCP_SERVER_PORT 8086

/* 可选边界类型：
 * 0 = 仅灰度图
 * 1 = X_BOUNDARY + 灰度图
 * 2 = Y_BOUNDARY + 灰度图
 * 4 = 仅 X_BOUNDARY
 */
#define INCLUDE_BOUNDARY_TYPE 0
#define BOUNDARY_NUM (CAM_HEIGHT * 4 / 2)

/* 主循环运行标志。收到 Ctrl+C 时会被置 0。 */
static volatile int g_app_running = 1;

/* 真实编码器设备对象。 */
static zf_driver_encoder g_encoder_left(ENCODER_LEFT_PATH);
static zf_driver_encoder g_encoder_right(ENCODER_RIGHT_PATH);

/* TCP 客户端对象。 */
static zf_driver_tcp_client g_tcp_client;

/* TCP 图像发送缓存。只传灰度图。 */
static uint8 g_image_copy[CAM_HEIGHT][CAM_WIDTH];

/* 下面这些边界数组默认不会启用，仅为保持与逐飞助手标准配置兼容。 */
static uint8 g_xy_x1_boundary[BOUNDARY_NUM], g_xy_x2_boundary[BOUNDARY_NUM], g_xy_x3_boundary[BOUNDARY_NUM];
static uint8 g_xy_y1_boundary[BOUNDARY_NUM], g_xy_y2_boundary[BOUNDARY_NUM], g_xy_y3_boundary[BOUNDARY_NUM];
static uint8 g_x1_boundary[CAM_HEIGHT], g_x2_boundary[CAM_HEIGHT], g_x3_boundary[CAM_HEIGHT];
static uint8 g_y1_boundary[CAM_WIDTH], g_y2_boundary[CAM_WIDTH], g_y3_boundary[CAM_WIDTH];

/* SIGINT 信号处理函数。 */
static void on_sigint(int)
{
    g_app_running = 0;
}

/* TCP 发送包装函数，适配逐飞助手接口。 */
static uint32 tcp_send_wrap(const uint8 *buf, uint32 len)
{
    return g_tcp_client.send_data(buf, len);
}

/* TCP 接收包装函数，适配逐飞助手接口。 */
static uint32 tcp_read_wrap(uint8 *buf, uint32 len)
{
    return g_tcp_client.read_data(buf, len);
}

/* 标准逐飞助手相机配置：默认只发灰度图，不改你的主流程。 */
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

/* 初始化 TCP 与逐飞助手图像通道。 */
static int init_tcp_transfer()
{
    if (g_tcp_client.init(TCP_SERVER_IP, TCP_SERVER_PORT) == 0)
    {
        std::printf("tcp_client ok\r\n");
    }
    else
    {
        std::printf("tcp_client error\r\n");
        return -1;
    }

    /* 初始化逐飞助手通信接口 */
    seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);

    /* 按逐飞助手标准方式配置图传；默认仅灰度图 */
    assistant_camera_config();

    return 0;
}

/* 发送一帧灰度图像到逐飞助手。 */
static void tcp_send_gray_frame(const frame_t &frame)
{
    if (!frame.valid || frame.gray == nullptr)
    {
        return;
    }

    std::memcpy(g_image_copy[0], frame.gray, (size_t)frame.width * (size_t)frame.height);
    seekfree_assistant_camera_send();
}

/* 保持逐飞助手下行协议可用（在线调试/参数等）。 */
static void tcp_poll_assistant(void)
{
    seekfree_assistant_data_analysis();
}

/* 整型斜坡限幅，防止电机命令一帧内跳变过大。 */
static int ramp_limit_int(int target, int current, int step)
{
    if (target > current + step)
        return current + step;
    if (target < current - step)
        return current - step;
    return target;
}

/*
 * init_encoder_feedback()
 * ---------------------------------------------------------------------------
 * 清零编码器累计值，初始化速度反馈状态。
 */
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
 * 每个控制周期读取一次编码器计数，并换算为 rps。
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
 * 完成一次摄像头采集、灰度化和循迹计算。
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
 * update_motion_control()
 * ---------------------------------------------------------------------------
 * 这是运动控制核心：
 * 1. 根据循迹误差得到左右差速。
 * 2. 根据偏差大小自动降速。
 * 3. 连续丢线后停车。
 * 4. 用左右轮 PID 生成最终电机输出命令。
 * 5. 对最终命令做斜坡限幅，降低低速抖动。
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
            last_track_error = 0.0f;
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
    uint64_t last_tcp_tick_ms = 0;

    line_track_init(app.track);
    app.system_state = SYS_READY;
    app.now_ms = app_millis();

    /* 初始化外设 */
    if (init_tcp_transfer() != 0)
    {
        return -1;
    }

    (void)motor_driver.init(app.motor);
    if (camera.init(app.frame) != 0)
    {
        std::printf("camera init error\r\n");
        return -1;
    }
    (void)vision_uart.init();
    init_encoder_feedback(app.motor);

    pid_init(left_speed_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
             SPEED_PID_I_LIMIT, SPEED_PID_OUT_LIMIT);
    pid_init(right_speed_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
             SPEED_PID_I_LIMIT, SPEED_PID_OUT_LIMIT);

    std::printf("%s start, tcp gray stream enabled\r\n", APP_NAME);

    while (g_app_running)
    {
        app.now_ms = app_millis();
        app.system_state = app.motor.blocked ? SYS_ERROR : SYS_RUNNING;

        if ((app.now_ms - last_track_tick_ms) >= TRACK_PERIOD_MS)
        {
            last_track_tick_ms = app.now_ms;
            run_tracking_cycle(camera, app);
        }

        if ((app.now_ms - last_uart_tick_ms) >= UART_PERIOD_MS)
        {
            last_uart_tick_ms = app.now_ms;
            vision_uart.poll(app.recog);
        }

        if ((app.now_ms - last_control_tick_ms) >= CONTROL_PERIOD_MS)
        {
            last_control_tick_ms = app.now_ms;
            update_encoder_feedback(app.motor);
            motor_driver.check_block(app.motor, app.now_ms);
            update_motion_control(app, left_speed_pid, right_speed_pid, motor_driver);
        }

        /*
         * TCP 图像发送：
         * 不必每 1ms 都发，跟随较高频率周期发即可。
         * 这里用 20ms，约 50fps 上限；实际受摄像头与网络影响。
         */
        if ((app.now_ms - last_tcp_tick_ms) >= 20)
        {
            last_tcp_tick_ms = app.now_ms;
            tcp_send_gray_frame(app.frame);
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
