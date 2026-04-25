#pragma once

/*
 * ============================================================================
 * 文件名称: app_config.hpp
 * 文件用途: 本工程全部可调宏参数的统一入口
 *
 * 当前版本重点:
 * 1. 修正 UVC 实际输出 160x120 与工程原 320x240 不一致的问题；
 * 2. TCP 图传按真实 160x120 发送，降低带宽并避免花屏 / buffer full；
 * 3. IPS200 上半区显示图像，下半区显示数据，避免图像与状态文字互相覆盖。
 * ============================================================================
 */

/* -------------------------
 * 工程基本信息
 * ------------------------- */
#define APP_NAME "DDSFC_ThreeWheel_NoServo_CrossRing"
#define APP_VERSION "4.3.0_image_track_core"

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
#define TCP_SEND_PERIOD_MS 150
#define TCP_ASSISTANT_POLL_PERIOD_MS 5

/* -------------------------
 * 数据有效期，单位: ms
 * ------------------------- */
#define CAMERA_TIMEOUT_MS 120
#define RECOG_TIMEOUT_MS 300

/* -------------------------
 * UVC 摄像头参数
 * -------------------------
 * 说明：当前逐飞 UVC 驱动头文件里定义的实际输出是 UVC_WIDTH x UVC_HEIGHT，
 * 也就是 160 x 120。这里固定为 160x120，和当前驱动输出保持一致，避免再出现 320x240 越界拷贝。
 */
#define CAM_DEV_PATH "/dev/video0"
#define CAM_WIDTH 160
#define CAM_HEIGHT 120
#define CAM_FPS 180

/* -------------------------
 * 图传参数
 * ------------------------- */
#define TCP_SERVER_IP "10.124.199.185"
#define TCP_SERVER_PORT 8086

/*
 * TCP 图传分辨率。
 * 逐飞助手收到的宽高必须和发送缓存真实尺寸一致。
 */
#define TCP_IMAGE_WIDTH  CAM_WIDTH
#define TCP_IMAGE_HEIGHT CAM_HEIGHT

/* -------------------------
 * IPS200 图像显示参数
 * -------------------------
 * 屏幕有效横向 240 像素，160x120 灰度图放在上方；
 * 状态文字从 y=128 开始显示，避免覆盖图像。
 */
#define IPS_IMAGE_ENABLE 1
#define IPS_IMAGE_X 0
#define IPS_IMAGE_Y 0
#define IPS_IMAGE_W 160
#define IPS_IMAGE_H 120
#define IPS_TEXT_X 0
#define IPS_TEXT_Y 128
#define IPS_TEXT_LINE_H 16
#define IPS_TEXT_AREA_W 240
#define IPS_TEXT_AREA_H 56
#define IPS_IMAGE_THRESHOLD 0

/* -------------------------
 * 图传叠加边界类型
 * 0: 只发灰度图
 * 1: 发 X_BOUNDARY
 * 2: 发 Y_BOUNDARY
 * 4: 只发边界不发底图
 * ------------------------- */
#define INCLUDE_BOUNDARY_TYPE 1
#define BOUNDARY_NUM (TCP_IMAGE_HEIGHT * 2)

/* TCP 与 IPS 叠加显示开关。 */
#define TRACK_OVERLAY_ENABLE 1
#define TRACK_OVERLAY_POINT_HALF_SIZE 1

/* -------------------------
 * 基础循迹参数（160x120 版）
 * ------------------------- */
#define TRACK_SCAN_LINES 6
#define TRACK_ROI_Y0 55
#define TRACK_ROI_Y1 116
#define TRACK_TOP_LINE 4
#define TRACK_SEARCH_FINISH_LINE 15

/* 自适应阈值基础值与限幅。 */
#define TRACK_BIN_THRESHOLD 100
#define TRACK_BINARY_MIN 40
#define TRACK_BINARY_MAX 180
#define TRACK_OTSU_SAMPLE_STRIDE 2
#define TRACK_BINARY_DENOISE_ENABLE 1
#define TRACK_MIN_WHITE_COLUMN_LEN 10
#define TRACK_SEARCH_STOP_MARGIN 8
#define TRACK_MIN_SEARCH_STOP_LINE 30

/* 图像中心与误差滤波。 */
#define TRACK_CENTER_X (CAM_WIDTH / 2)
#define TRACK_ERROR_FILTER_ALPHA 0.20f
#define TRACK_CURVATURE_FILTER_ALPHA 0.25f
#define TRACK_VALID_RATIO_MIN 0.34f

/* 边界搜索与赛道宽度约束。 */
#define TRACK_LOCAL_SEARCH_MARGIN 22
#define TRACK_SEED_SEARCH_HALF_WIDTH 36
#define TRACK_MIN_SEARCH_HALF 10
#define TRACK_MIN_LANE_WIDTH 13
#define TRACK_MAX_LANE_WIDTH 110
#define TRACK_DEFAULT_WIDTH 35
#define TRACK_EDGE_GRAD_MIN 8
#define TRACK_WIDTH_FILTER_ALPHA 0.25f

/* 中线输出平滑。 */
#define TRACK_SMOOTH_ALPHA_NUM 8
#define TRACK_SMOOTH_ALPHA_DEN 10

/* 根据偏差大小自动降速。 */
#define TRACK_SLOWDOWN_ERROR 18.0f
#define TRACK_BRAKE_ERROR 30.0f
#define TRACK_SPEED_CURVATURE_SLOWDOWN 9.0f
#define TRACK_SPEED_QUALITY_FLOOR 0.45f

/* 元素区速度限幅。 */
#define TRACK_CROSS_SPEED_SCALE 0.72f
#define TRACK_RING_SPEED_SCALE 0.60f

/* -------------------------
 * 十字 / 环岛 图像判定阈值（160x120 版）
 * ------------------------- */
#define TRACK_CROSS_BOTH_LOST_MID_MIN 6
#define TRACK_CROSS_WIDTH_DELTA_MIN 6
#define TRACK_RING_SIDE_LOST_MID_MIN 10
#define TRACK_RING_OTHER_SIDE_LOST_MID_MAX 4
#define TRACK_RING_BOTH_LOST_MID_MAX 5
#define TRACK_CORNER_SLOPE_MIN 3
#define TRACK_CORNER_WIDTH_JUMP_MIN 5

/* 元素状态机参数。 */
#define TRACK_COUNTER_MAX 10
#define TRACK_CROSS_ENTER_COUNT 2
#define TRACK_RING_ENTER_COUNT 2
#define TRACK_CROSS_HOLD_FRAMES 8
#define TRACK_RING_HOLD_FRAMES 12
#define TRACK_CROSS_TIMEOUT_FRAMES 45
#define TRACK_RING_TIMEOUT_FRAMES 90
#define TRACK_STATE_REENTER_COOLDOWN_FRAMES 6

/* -------------------------
 * 电机控制参数
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
#define STEER_PD_ERR_MAX 60.0f
#define STEER_PD_DERR_MAX 20.0f
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
