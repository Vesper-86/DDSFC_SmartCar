#pragma once

/*
 * ============================================================================
 * 文件名称: app_config.hpp
 * 文件用途: 本工程全部可调宏参数的统一入口
 *
 * 当前版本重点:
 * 1. 保留基础循迹 + 差速控制主链；
 * 2. 增加十字识别 / 环岛识别 / 补线状态机；
 * 3. 为十字 / 环岛补上“超时退出”保护；
 * 4. 为普通 / 十字 / 环岛分别配置独立目标速度。
 * ============================================================================
 */

/* -------------------------
 * 工程基本信息
 * ------------------------- */
#define APP_NAME "DDSFC_ThreeWheel_NoServo_CrossRing"
#define APP_VERSION "4.1.0"

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
 * ------------------------- */
#define TCP_SERVER_IP "10.190.147.185"
#define TCP_SERVER_PORT 8086

/* -------------------------
 * 图传叠加边界类型
 * 0: 只发灰度图
 * 1: 发 X_BOUNDARY
 * 2: 发 Y_BOUNDARY
 * 4: 只发边界不发底图
 * ------------------------- */
#define INCLUDE_BOUNDARY_TYPE 0
#define BOUNDARY_NUM (CAM_HEIGHT * 2)

/* -------------------------
 * 基础循迹参数
 * ------------------------- */
#define TRACK_SCAN_LINES 6
#define TRACK_ROI_Y0 130
#define TRACK_ROI_Y1 235
#define TRACK_TOP_LINE 8
#define TRACK_SEARCH_FINISH_LINE 30

/* 自适应阈值基础值与限幅。 */
#define TRACK_BIN_THRESHOLD 100
#define TRACK_BINARY_MIN 40
#define TRACK_BINARY_MAX 180
#define TRACK_OTSU_SAMPLE_STRIDE 2

/* 图像中心与误差滤波。 */
#define TRACK_CENTER_X (CAM_WIDTH / 2)
#define TRACK_ERROR_FILTER_ALPHA 0.20f
#define TRACK_CURVATURE_FILTER_ALPHA 0.25f
#define TRACK_VALID_RATIO_MIN 0.34f

/* 边界搜索与赛道宽度约束。 */
#define TRACK_LOCAL_SEARCH_MARGIN 40
#define TRACK_SEED_SEARCH_HALF_WIDTH 72
#define TRACK_MIN_SEARCH_HALF 18
#define TRACK_MIN_LANE_WIDTH 26
#define TRACK_MAX_LANE_WIDTH 220
#define TRACK_DEFAULT_WIDTH 70
#define TRACK_EDGE_GRAD_MIN 8
#define TRACK_WIDTH_FILTER_ALPHA 0.25f

/* 中线输出平滑。 */
#define TRACK_SMOOTH_ALPHA_NUM 8
#define TRACK_SMOOTH_ALPHA_DEN 10

/* 根据偏差大小自动降速。 */
#define TRACK_SLOWDOWN_ERROR 35.0f
#define TRACK_BRAKE_ERROR 60.0f
#define TRACK_SPEED_CURVATURE_SLOWDOWN 18.0f
#define TRACK_SPEED_QUALITY_FLOOR 0.45f

/* 元素状态机参数。 */
#define TRACK_COUNTER_MAX 10
#define TRACK_CROSS_ENTER_COUNT 2
#define TRACK_RING_ENTER_COUNT 2
#define TRACK_CROSS_HOLD_FRAMES 8
#define TRACK_RING_HOLD_FRAMES 12

/*
 * 元素区超时退出保护。
 * 说明:
 * - TRACK_PERIOD_MS 默认是 20ms。
 * - 例如 45 帧大约等于 900ms。
 * - 若元素区卡住，超过这个时间会强制退回 NORMAL。
 */
#define TRACK_CROSS_TIMEOUT_FRAMES 45
#define TRACK_RING_TIMEOUT_FRAMES 90
#define TRACK_STATE_REENTER_COOLDOWN_FRAMES 6

/* -------------------------
 * 十字 / 环岛 图像判定阈值
 * ------------------------- */
#define TRACK_CROSS_BOTH_LOST_MID_MIN 6
#define TRACK_CROSS_WIDTH_DELTA_MIN 12
#define TRACK_RING_SIDE_LOST_MID_MIN 10
#define TRACK_RING_OTHER_SIDE_LOST_MID_MAX 4
#define TRACK_RING_BOTH_LOST_MID_MAX 5
#define TRACK_CORNER_SLOPE_MIN 6
#define TRACK_CORNER_WIDTH_JUMP_MIN 10

/* -------------------------
 * 电机控制参数
 * ------------------------- */
#define MOTOR_CMD_MAX 10000
#define MOTOR_CMD_MIN (-10000)

/*
 * 三档目标速度。
 * 说明:
 * - NORMAL: 普通直道 / 弯道的基础目标速度。
 * - CROSS : 十字区主动降一档，避免冲飞。
 * - RING  : 环岛再降一档，优先保证可控性。
 */
#define MOTOR_BASE_SPEED_NORMAL 240.0f
#define MOTOR_BASE_SPEED_CROSS  180.0f
#define MOTOR_BASE_SPEED_RING   150.0f

/* 兼容旧代码中对 MOTOR_BASE_SPEED 的引用。 */
#define MOTOR_BASE_SPEED MOTOR_BASE_SPEED_NORMAL

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

#define SPEED_PID_D_FILTER_ALPHA 0.35f
#define SPEED_PID_INTEGRAL_ENABLE_ERR_MIN 0.60f
#define SPEED_PID_INTEGRAL_RELEASE_GAIN 0.35f

/* -------------------------
 * 方向控制（模糊 PD）参数
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
