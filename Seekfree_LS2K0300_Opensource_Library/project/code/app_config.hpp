#pragma once

/*
 * ============================================================================
 * 文件名称: app_config.hpp
 * 文件用途: 本工程全部“可调宏参数”的统一入口
 *
 * 使用建议:
 * 1. 现场调参时，优先只改这个文件。
 * 2. 不要把阈值、速度、周期常量散落到各个 .cpp 文件里。
 * 3. 每个宏前都保留了用途说明，方便后续比赛前快速查找。
 * ============================================================================
 */

/* -------------------------
 * 工程基本信息
 * ------------------------- */
#define APP_NAME "DDSFC_ThreeWheel_NoServo_Integrated"
#define APP_VERSION "3.0.0"

/* -------------------------
 * 功能开关
 * ------------------------- */
#define DISPLAY_ENABLE 1
#define VISION_UART_ENABLE 1
#define TCP_ASSISTANT_ENABLE 1

/* -------------------------
 * 任务调度周期，单位: ms
 * ------------------------- */
#define CONTROL_PERIOD_MS 10
#define TRACK_PERIOD_MS 20
#define UART_PERIOD_MS 10
#define DISPLAY_PERIOD_MS 100
#define TCP_SEND_PERIOD_MS 120
#define TCP_ASSISTANT_POLL_PERIOD_MS 5

/* -------------------------
 * 数据有效期，单位: ms
 * ------------------------- */
#define CAMERA_TIMEOUT_MS 120
#define RECOG_TIMEOUT_MS 300

/* -------------------------
 * UVC 摄像头参数
 * ------------------------- */
#define CAM_DEV_PATH "/dev/video0"
#define CAM_WIDTH 320
#define CAM_HEIGHT 240
#define CAM_FPS 60

/* -------------------------
 * 图传参数
 * -------------------------
 * 说明:
 * - 这里只做逐飞助手的灰度图传。
 * - 如果不需要图传，直接把 TCP_ASSISTANT_ENABLE 改为 0。
 * ------------------------- */
#define TCP_SERVER_IP "10.190.147.185"
#define TCP_SERVER_PORT 8086

/* -------------------------
 * 图传叠加边界类型
 * -------------------------
 * 0: 只发灰度图
 * 1: 发 X_BOUNDARY
 * 2: 发 Y_BOUNDARY
 * 4: 只发边界不发底图
 * ------------------------- */
#define INCLUDE_BOUNDARY_TYPE 0
#define BOUNDARY_NUM (CAM_HEIGHT * 2)

/* -------------------------
 * 循迹算法参数
 * ------------------------- */
#define TRACK_SCAN_LINES 6
#define TRACK_ROI_Y0 130
#define TRACK_ROI_Y1 235

/* 保底阈值。实际运行时，line_track.cpp 会基于 ROI Otsu 做自适应修正。 */
#define TRACK_BIN_THRESHOLD 100

/* 保留原有宏，兼容旧代码含义。 */
#define TRACK_MIN_VALID_POINTS 20
#define TRACK_CENTER_X (CAM_WIDTH / 2)
#define TRACK_ERROR_FILTER_ALPHA 0.20f

/* 根据偏差大小自动降速的两个阈值 */
#define TRACK_SLOWDOWN_ERROR 35.0f
#define TRACK_BRAKE_ERROR 60.0f

/* 图像鲁棒性参数 */
#define TRACK_OTSU_SAMPLE_STRIDE 2
#define TRACK_LOCAL_SEARCH_MARGIN 40
#define TRACK_SEED_SEARCH_HALF_WIDTH 72
#define TRACK_MIN_LANE_WIDTH 26
#define TRACK_MAX_LANE_WIDTH 220
#define TRACK_EDGE_GRAD_MIN 8
#define TRACK_WIDTH_FILTER_ALPHA 0.25f
#define TRACK_CURVATURE_FILTER_ALPHA 0.25f
#define TRACK_VALID_RATIO_MIN 0.34f

/* 速度缩放补偿参数 */
#define TRACK_SPEED_CURVATURE_SLOWDOWN 18.0f
#define TRACK_SPEED_QUALITY_FLOOR 0.45f

/* -------------------------
 * 电机控制参数
 * -------------------------
 * 说明:
 * - 本车为“三轮无舵机”结构，方向控制通过左右驱动轮差速实现。
 * - 第三轮通常为万向轮/随动轮，不参与主动转向控制。
 * ------------------------- */
#define MOTOR_CMD_MAX 10000
#define MOTOR_CMD_MIN (-10000)
#define MOTOR_BASE_SPEED 240.0f
#define MOTOR_TURN_SPEED_LIMIT 360.0f
#define MOTOR_START_SPEED 0.0f
#define MOTOR_DEADZONE_DUTY 380

/* 堵转判定参数 */
#define MOTOR_BLOCK_SPEED_EPS 0.15f
#define MOTOR_BLOCK_TIME_MS 250

/* -------------------------
 * 左右轮速度环 PID 参数
 * ------------------------- */
#define SPEED_PID_KP 1.60f
#define SPEED_PID_KI 0.10f
#define SPEED_PID_KD 0.08f
#define SPEED_PID_I_LIMIT 150.0f
#define SPEED_PID_OUT_LIMIT 7000.0f

/* PID 内部增强参数 */
#define SPEED_PID_D_FILTER_ALPHA 0.35f
#define SPEED_PID_INTEGRAL_ENABLE_ERR_MIN 0.60f
#define SPEED_PID_INTEGRAL_RELEASE_GAIN 0.35f

/* -------------------------
 * 方向控制（模糊 PD）参数
 * -------------------------
 * 说明:
 * - 输入是图像循迹误差。
 * - 输出是左右轮差速量 differential_output。
 * ------------------------- */
#define STEER_PD_KP_BASE 5.5f
#define STEER_PD_KD_BASE 10.0f
#define STEER_PD_KP_FUZZY 1.0f
#define STEER_PD_KD_FUZZY 5.0f
#define STEER_PD_ERR_MAX 120.0f
#define STEER_PD_DERR_MAX 40.0f
#define DIFF_OUT_LIMIT 120.0f

/* -------------------------
 * 识别串口协议参数
 * 协议格式: AA 55 cls score checksum 0D
 * ------------------------- */
#define RECOG_CONFIRM_FRAMES 3
#define RECOG_SCORE_MIN 0.50f
#define RECOG_CONFIRM_DECAY 1

#define UART_RECOG_DEV "/dev/ttyS1"
#define UART_BAUD_RECOG 115200
#define UART_FRAME_HEAD1 0xAA
#define UART_FRAME_HEAD2 0x55
#define UART_FRAME_TAIL 0x0D

/* -------------------------
 * 逐飞库接口绑定
 * ------------------------- */
#define SMARTCAR_PWM_LEFT ZF_PWM_MOTOR_1
#define SMARTCAR_DIR_LEFT ZF_GPIO_MOTOR_1
#define SMARTCAR_PWM_RIGHT ZF_PWM_MOTOR_2
#define SMARTCAR_DIR_RIGHT ZF_GPIO_MOTOR_2

/* -------------------------
 * 编码器参数
 * ------------------------- */
#define ENCODER_LEFT_PATH ZF_ENCODER_DIR_1
#define ENCODER_RIGHT_PATH ZF_ENCODER_DIR_2
#define ENCODER_LEFT_SIGN 1.0f
#define ENCODER_RIGHT_SIGN 1.0f
#define ENCODER_SPEED_FILTER_ALPHA 0.35f
#define ENCODER_COUNTS_PER_WHEEL_REV 1000.0f

/* -------------------------
 * 电机输出平滑参数
 * ------------------------- */
#define MOTOR_CMD_RAMP_STEP 350

/* -------------------------
 * 丢线保护参数
 * ------------------------- */
#define TRACK_LOST_STOP_COUNT 3
