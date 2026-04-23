#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/time.h>
#include "app_config.hpp"

/*
 * ============================================================================
 * 文件名称: app_types.hpp
 * 文件用途: 定义主程序需要共享的数据结构和基础工具函数
 *
 * 设计原则:
 * - 每个模块尽量围绕 app_context_t 交换数据。
 * - 减少到处散落的全局变量。
 * - 让主循环的数据流更清楚：摄像头 -> 循迹 -> 控制 -> 显示。
 * ============================================================================
 */

/* 系统运行状态。 */
enum system_state_e {
    SYS_READY = 0,   /* 已初始化完成，等待进入主循环 */
    SYS_RUNNING,     /* 正常运行中 */
    SYS_ERROR        /* 发生堵转等异常 */
};

/* 通用 PID 控制器结构体。
 * 说明:
 * - kp/ki/kd      : 比例、积分、微分系数
 * - integral      : 当前积分累计值
 * - last_error    : 上一周期误差
 * - i_limit       : 积分限幅
 * - out_limit     : 输出限幅
 */
struct pid_controller_t {
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float i_limit;
    float out_limit;
};

/* 一帧图像的数据容器。 */
struct frame_t {
    int width;                /* 图像宽度 */
    int height;               /* 图像高度 */
    uint8_t *yuyv;            /* 原始 YUYV 数据缓冲区 */
    uint8_t *gray;            /* 灰度图缓冲区，仅保留亮度通道 */
    uint64_t timestamp_ms;    /* 最近一次成功采集时间戳 */
    bool valid;               /* 当前帧是否有效 */
};

/* 循迹算法输出结果。 */
struct track_result_t {
    bool valid;                              /* 本次循迹结果是否可信 */
    bool lost;                               /* 是否丢线 */
    int left_edge[TRACK_SCAN_LINES];         /* 每条扫描线找到的左边界 */
    int right_edge[TRACK_SCAN_LINES];        /* 每条扫描线找到的右边界 */
    int center[TRACK_SCAN_LINES];            /* 每条扫描线计算出的中心 */
    float center_avg;                        /* 多扫描线中心平均值 */
    float error;                             /* 原始偏差 = 图像中心 - 赛道中心 */
    float error_filtered;                    /* 低通滤波后的偏差 */
    uint64_t timestamp_ms;                   /* 结果更新时间戳 */
};

/* 串口识别结果。当前版本仅做显示与保留，不触发动作。 */
struct recognition_t {
    bool valid;               /* 最近是否解析到有效识别帧 */
    uint8_t cls;              /* 当前瞬时类别 */
    float score;              /* 当前瞬时置信度 */
    uint8_t stable_cls;       /* 连续确认后的稳定类别 */
    uint8_t stable_count;     /* 已连续确认的帧数 */
    uint64_t timestamp_ms;    /* 最近一次识别更新时间 */
};

/* 电机运行状态。 */
struct motor_state_t {
    float left_speed_rps;     /* 左轮实际速度（当前版本可由模拟值提供） */
    float right_speed_rps;    /* 右轮实际速度 */
    float left_target_rps;    /* 左轮目标速度 */
    float right_target_rps;   /* 右轮目标速度 */
    int left_cmd;             /* 左轮最终输出命令 */
    int right_cmd;            /* 右轮最终输出命令 */
    bool blocked;             /* 是否判定堵转 */
    uint64_t blocked_since_ms;/* 首次满足堵转条件的时间 */
};

/* 主循环统一上下文。
 * 所有主要模块都围绕它读写状态。 */
struct app_context_t {
    frame_t frame;                  /* 摄像头帧 */
    track_result_t track;           /* 循迹输出 */
    recognition_t recog;            /* 识别结果 */
    motor_state_t motor;            /* 电机状态 */
    system_state_e system_state;    /* 当前系统状态 */
    uint64_t now_ms;                /* 本轮循环时间戳 */
};

/* 获取当前毫秒时间戳。 */
static inline uint64_t app_millis()
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000ULL);
}

/* 浮点限幅。 */
static inline float clampf(float x, float min_v, float max_v)
{
    return (x < min_v) ? min_v : ((x > max_v) ? max_v : x);
}

/* 整型限幅。 */
static inline int clampi(int x, int min_v, int max_v)
{
    return (x < min_v) ? min_v : ((x > max_v) ? max_v : x);
}
