#pragma once

/*
 * ============================================================================
 * 文件名称: app_config.hpp
 * 文件用途: 本工程全部可调宏参数的统一入口
 *
 * 当前版本特点:
 * 1. 支持无编码器开环和编码器闭环两种模式
 * 2. 默认配置为低速稳定安全版
 * ============================================================================
 */

#define TCP_IMAGE_HEIGHT 120

#define IPS_IMAGE_X 0
#define IPS_IMAGE_Y 0
#define IPS_IMAGE_W TCP_IMAGE_WIDTH
#define IPS_IMAGE_H TCP_IMAGE_HEIGHT

/* -------------------------
 * 工程基本信息
 * ------------------------- */
#define APP_NAME                        "DDSFC_SmartCar_SeekfreePort_Lite"
#define APP_VERSION                     "3.0.0_stable"/* 应用程序名称 */
#define APP_VERSION "4.5.0_Stable_LowSpeed"           /* 版本号 */

/* -------------------------
 * 功能开关
 * ------------------------- */
#define DISPLAY_ENABLE 0          /* 1=启用IPS200显示 0=禁用 */
#define VISION_UART_ENABLE 0      /* 1=启用视觉识别串口 0=禁用 */
#define TCP_ASSISTANT_ENABLE 1    /* 1=启用逐飞助手TCP图传 0=禁用 */
#define ENCODER_FEEDBACK_ENABLE 0 /* 0=无编码器开环 1=有编码器闭环 */

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
 * 摄像头方向修正
 * -------------------------
 * 你的当前安装状态是上下反了，所以默认开启 CAMERA_FLIP_VERTICAL。
 * 若后续把摄像头重新正装，只需要改为 0。
 *
 * 说明:
 * - CAMERA_FLIP_VERTICAL   : 上下翻转，修正“近处在画面上方”的问题；
 * - CAMERA_FLIP_HORIZONTAL : 左右镜像，若左右方向反了再开启；
 * - CAMERA_ROTATE_180      : 同时做上下 + 左右，相当于旋转 180 度。
 */
#define CAMERA_FLIP_VERTICAL 0
#define CAMERA_FLIP_HORIZONTAL 0
#define CAMERA_ROTATE_180 1

/* -------------------------
 * 图传参数
 * ------------------------- */
#define TCP_SERVER_IP "10.124.199.185"
#define TCP_SERVER_PORT 8086

/*
 * TCP 图传分辨率。
 * 逐飞助手收到的宽高必须和发送缓存真实尺寸一致。
 */
#define TCP_IMAGE_WIDTH CAM_WIDTH
#define IPS_TEXT_X 0          /* 文字起始X坐标 */
#define IPS_TEXT_Y 128        /* 文字起始Y坐标 */
#define IPS_TEXT_LINE_H 16    /* 每行文字高度 */
#define IPS_TEXT_AREA_W 240   /* 文字区域宽度 */
#define IPS_TEXT_AREA_H 56    /* 文字区域高度 */
#define IPS_IMAGE_THRESHOLD 0 /* 图像二值化阈值(0=用原图) */

/* -------------------------
 * 图传叠加边界类型
 * -------------------------
 * 0: 只发灰度图
 * 1: 发X_BOUNDARY，整行显示边界
 * 2: 发Y_BOUNDARY
 * 3: 发XY_BOUNDARY，只显示ROI内真实边界点(推荐)
 * 4: 只发边界不发底图
 */
#define INCLUDE_BOUNDARY_TYPE 3



#define BOUNDARY_NUM (TCP_IMAGE_HEIGHT * 2) /* 边界点数量 */

/* 叠加显示配置 */
#define TRACK_OVERLAY_ENABLE 1          /* 1=在图像上叠加边线 */
#define TRACK_OVERLAY_POINT_HALF_SIZE 1 /* 叠加点大小 */

/* 调试图像源选择 */
#define TCP_SHOW_BINARY_IMAGE 1 /* 0=显示灰度图 1=显示二值图 */
#define IPS_SHOW_BINARY_IMAGE 1 /* 0=显示灰度图 1=显示二值图 */

/* -------------------------
 * 基础循迹参数（160x120版）
 * ------------------------- */
#define TRACK_SCAN_LINES 6          /* 扫描线数量 */
#define TRACK_ROI_Y0 35             /* 感兴趣区域起始行 */
#define TRACK_ROI_Y1 100            /* 感兴趣区域结束行 */
#define TRACK_TOP_LINE 4            /* 顶部起始扫描线 */
#define TRACK_SEARCH_FINISH_LINE 15 /* 底部结束扫描线 */

/* 自适应阈值 */
#define TRACK_BIN_THRESHOLD 100       /* 二值化基础阈值 */
#define TRACK_BINARY_MIN 40           /* 最小阈值限幅 */
#define TRACK_BINARY_MAX 180          /* 最大阈值限幅 */
#define TRACK_OTSU_SAMPLE_STRIDE 2    /* 大津法采样步长 */
#define TRACK_BINARY_DENOISE_ENABLE 1 /* 1=启用去噪 */
#define TRACK_MIN_WHITE_COLUMN_LEN 6  /* 最小白色列长度 */
#define TRACK_SEED_ROW_LOOKUP 34      /* 种子行位置 */
#define TRACK_SEARCH_STOP_MARGIN 8    /* 搜索停止边界 */
#define TRACK_MIN_SEARCH_STOP_LINE 30 /* 最小搜索停止行 */

/* 循迹误差和滤波 */
#define TRACK_CENTER_X (CAM_WIDTH / 2)     /* 中心参考X坐标 */
#define TRACK_ERROR_FILTER_ALPHA 0.12f     /* 误差滤波系数(越小越平滑) */
#define TRACK_CURVATURE_FILTER_ALPHA 0.25f /* 曲率滤波系数 */
#define TRACK_VALID_RATIO_MIN 0.20f        /* 最小有效线比例 */

/* 边界搜索参数 */
#define TRACK_LOCAL_SEARCH_MARGIN 45    /* 局部搜索范围 */
#define TRACK_SEED_SEARCH_HALF_WIDTH 50 /* 种子搜索半宽 */
#define TRACK_MIN_SEARCH_HALF 16        /* 最小搜索半宽 */
#define TRACK_MIN_LANE_WIDTH 22         /* 最小赛道宽度 */
#define TRACK_MAX_LANE_WIDTH 135        /* 最大赛道宽度 */
#define TRACK_DEFAULT_WIDTH 70          /* 默认赛道宽度 */
#define TRACK_EDGE_GRAD_MIN 4           /* 边界梯度最小值 */
#define TRACK_WIDTH_FILTER_ALPHA 0.12f  /* 赛道宽度滤波系数 */

/* 中线平滑 */
#define TRACK_SMOOTH_ALPHA_NUM 8  /* 平滑分子 */
#define TRACK_SMOOTH_ALPHA_DEN 10 /* 平滑分母 */

/* 自动降速参数 */
#define TRACK_SLOWDOWN_ERROR 18.0f          /* 开始降速的误差阈值 */
#define TRACK_BRAKE_ERROR 30.0f             /* 开始刹车的误差阈值 */
#define TRACK_SPEED_CURVATURE_SLOWDOWN 9.0f /* 曲率降速阈值 */
#define TRACK_SPEED_QUALITY_FLOOR 0.45f     /* 质量系数下限 */

/* 元素区速度缩放 */
#define TRACK_CROSS_SPEED_SCALE 0.72f /* 十字区速度比例 */
#define TRACK_RING_SPEED_SCALE 0.60f  /* 环岛区速度比例 */

/* -------------------------
 * 十字/环岛图像判定阈值（160x120版）
 * ------------------------- */
#define TRACK_CROSS_BOTH_LOST_MID_MIN 6      /* 十字双边丢失最小行数 */
#define TRACK_CROSS_WIDTH_DELTA_MIN 6        /* 十字宽度跳变最小值 */
#define TRACK_RING_SIDE_LOST_MID_MIN 10      /* 环岛单边丢失最小行数 */
#define TRACK_RING_OTHER_SIDE_LOST_MID_MAX 4 /* 环岛另一边最大丢失行数 */
#define TRACK_RING_BOTH_LOST_MID_MAX 5       /* 环岛双边最大丢失行数 */
#define TRACK_CORNER_SLOPE_MIN 3             /* 拐角斜率最小值 */
#define TRACK_CORNER_WIDTH_JUMP_MIN 5        /* 拐角宽度跳变最小值 */

/* 元素状态机参数 */
#define TRACK_COUNTER_MAX 10                  /* 计数器最大值 */
#define TRACK_CROSS_ENTER_COUNT 2             /* 进入十字需要的帧数 */
#define TRACK_RING_ENTER_COUNT 2              /* 进入环岛需要的帧数 */
#define TRACK_CROSS_HOLD_FRAMES 8             /* 十字保持帧数 */
#define TRACK_RING_HOLD_FRAMES 12             /* 环岛保持帧数 */
#define TRACK_CROSS_TIMEOUT_FRAMES 45         /* 十字超时帧数 */
#define TRACK_RING_TIMEOUT_FRAMES 90          /* 环岛超时帧数 */
#define TRACK_STATE_REENTER_COOLDOWN_FRAMES 6 /* 状态重入冷却帧数 */

/* -------------------------
 * 电机控制参数
 * ------------------------- */
#define MOTOR_CMD_MAX 2000    /* 电机命令最大值 */
#define MOTOR_CMD_MIN (-2000) /* 电机命令最小值 */

/* 两种模式的速度配置 */
#if ENCODER_FEEDBACK_ENABLE
#define MOTOR_BASE_SPEED 80.0f        /* 编码器版本：基础速度(rps*100) */
#define MOTOR_TURN_SPEED_LIMIT 120.0f /* 编码器版本：转向限幅 */
#else
#define OPEN_LOOP_BASE_PWM 500   /* 【低速版】无编码器：基础PWM(0-2000) */
#define OPEN_LOOP_TURN_LIMIT 700 /* 【低速版】无编码器：转向PWM限幅 */
#endif

#define MOTOR_START_SPEED 0.0f  /* 起步速度 */
#define MOTOR_DEADZONE_DUTY 380 /* 电机死区PWM */

/* 堵转判定参数 */
#define MOTOR_BLOCK_SPEED_EPS 0.15f /* 堵转速度阈值 */
#define MOTOR_BLOCK_TIME_MS 250     /* 堵转判定时间(ms) */

/* -------------------------
 * 左右轮速度环PID参数
 * -------------------------
 * 说明：仅ENCODER_FEEDBACK_ENABLE=1时有效
 */
#define SPEED_PID_KP 1.60f                      /* 比例系数P */
#define SPEED_PID_KI 0.10f                      /* 积分系数I */
#define SPEED_PID_KD 0.08f                      /* 微分系数D */
#define SPEED_PID_I_LIMIT 150.0f                /* 积分项限幅 */
#define SPEED_PID_OUT_LIMIT 2000.0f             /* PID输出限幅 */
#define SPEED_PID_D_FILTER_ALPHA 0.35f          /* 微分项滤波系数 */
#define SPEED_PID_INTEGRAL_ENABLE_ERR_MIN 0.60f /* 积分启用的最小误差 */
#define SPEED_PID_INTEGRAL_RELEASE_GAIN 0.35f   /* 积分释放系数 */

/* -------------------------
 * 方向控制（模糊PD）参数
 * -------------------------
 * 说明：控制左右轮差速，两种模式通用
 */
#define STEER_PD_KP_BASE 4.5f   /* 【低速版】基础P系数(原值5.5，降低更稳) */
#define STEER_PD_KD_BASE 8.0f   /* 【低速版】基础D系数(原值10.0，降低更稳) */
#define STEER_PD_KP_FUZZY 1.0f  /* P的模糊调整范围 */
#define STEER_PD_KD_FUZZY 5.0f  /* D的模糊调整范围 */
#define STEER_PD_ERR_MAX 60.0f  /* 误差最大归一化值 */
#define STEER_PD_DERR_MAX 20.0f /* 误差变化率归一化值 */
#define DIFF_OUT_LIMIT 100.0f   /* 【低速版】差速输出限幅(原值120) */

/* -------------------------
 * 识别串口协议参数
 * 协议格式: AA 55 cls score checksum 0D
 * ------------------------- */
#define RECOG_CONFIRM_FRAMES 3 /* 识别确认帧数 */
#define RECOG_SCORE_MIN 0.50f  /* 最小置信度 */
#define RECOG_CONFIRM_DECAY 1  /* 确认衰减系数 */

#define UART_RECOG_DEV "/dev/ttyS1" /* 识别串口设备 */
#define UART_BAUD_RECOG 115200      /* 串口波特率 */
#define UART_FRAME_HEAD1 0xAA       /* 帧头1 */
#define UART_FRAME_HEAD2 0x55       /* 帧头2 */
#define UART_FRAME_TAIL 0x0D        /* 帧尾 */

/* -------------------------
 * 逐飞库接口绑定
 * ------------------------- */
#define SMARTCAR_PWM_LEFT ZF_PWM_MOTOR_1   /* 左电机PWM设备 */
#define SMARTCAR_DIR_LEFT ZF_GPIO_MOTOR_1  /* 左电机方向GPIO */
#define SMARTCAR_PWM_RIGHT ZF_PWM_MOTOR_2  /* 右电机PWM设备 */
#define SMARTCAR_DIR_RIGHT ZF_GPIO_MOTOR_2 /* 右电机方向GPIO */

/* -------------------------
 * 编码器参数
 * ------------------------- */
#define ENCODER_LEFT_PATH ZF_ENCODER_DIR_1   /* 左编码器设备路径 */
#define ENCODER_RIGHT_PATH ZF_ENCODER_DIR_2  /* 右编码器设备路径 */
#define ENCODER_LEFT_SIGN (-1.0f)            /* 左编码器方向符号(正转+) */
#define ENCODER_RIGHT_SIGN (-1.0f)           /* 右编码器方向符号(正转+) */
#define ENCODER_SPEED_FILTER_ALPHA 0.35f     /* 速度滤波系数 */
#define ENCODER_COUNTS_PER_WHEEL_REV 1000.0f /* 车轮一圈的编码器脉冲数 */

/* -------------------------
 * 电机输出平滑参数
 * ------------------------- */
#define MOTOR_CMD_RAMP_STEP 120       /* 【低速版】斜坡限幅步长(原值150，更小更缓) */
#define MOTOR_FINAL_OUTPUT_LIMIT 1500 /* 【低速版】最终输出限幅(原值2000) */

/* -------------------------
 * 丢线保护参数
 * ------------------------- */
#define TRACK_LOST_STOP_COUNT 3 /* 连续丢线N帧后停车 */
