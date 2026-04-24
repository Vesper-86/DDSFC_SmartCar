#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/time.h>
#include "app_config.hpp"

/*
 * ============================================================================
 * 文件名称: app_types.hpp
 * 文件用途: 定义主程序共享的数据结构和基础工具函数
 *
 * 说明:
 * - 当前版本已经把“十字 / 环岛”图像状态纳入 track_result_t。
 * - 主循环仍然只围绕一个 app_context_t 交换数据，方便后续继续扩展。
 * ============================================================================
 */

/* 系统运行状态。 */
enum system_state_e {
    SYS_READY = 0,
    SYS_RUNNING,
    SYS_ERROR
};

/* 图像元素判定结果。 */
enum track_element_e {
    TRACK_ELEMENT_NONE = 0,
    TRACK_ELEMENT_STRAIGHT,
    TRACK_ELEMENT_CROSS,
    TRACK_ELEMENT_RING_LEFT,
    TRACK_ELEMENT_RING_RIGHT
};

/* 图像状态机。 */
enum track_state_e {
    TRACK_STATE_NORMAL = 0,
    TRACK_STATE_CROSS,
    TRACK_STATE_RING_LEFT,
    TRACK_STATE_RING_RIGHT
};

/* 角点信息。 */
struct track_point_t {
    int x;
    int y;
    bool valid;
};

/* 通用 PID 控制器结构体。 */
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
    int width;
    int height;
    uint8_t *yuyv;
    uint8_t *gray;
    uint64_t timestamp_ms;
    bool valid;
};

/* 循迹算法输出结果。
 *
 * 说明:
 * - left_edge / right_edge / center 仍保留“给控制器看的抽样扫描线结果”。
 * - element / state / ring_flag / cross_flag 用于上层理解当前图像处于什么元素区。
 */
struct track_result_t {
    bool valid;
    bool lost;

    int left_edge[TRACK_SCAN_LINES];
    int right_edge[TRACK_SCAN_LINES];
    int center[TRACK_SCAN_LINES];

    float center_avg;
    float error;
    float error_filtered;
    float curvature_hint;
    float quality_hint;
    uint64_t timestamp_ms;

    uint8_t element;
    uint8_t state;
    uint8_t state_hold;
    uint16_t state_elapsed_frames;  /* 当前状态已持续多少帧 */
    bool timeout_exit;              /* 本帧是否刚发生超时退回 */
    bool cross_flag;
    uint8_t ring_flag;              /* 0: 无 1: 左环 2: 右环 */

    int lane_width_base;
    int lane_width_far;

    uint8_t left_lost_total;
    uint8_t right_lost_total;
    uint8_t both_lost_total;
    uint8_t left_lost_mid;
    uint8_t right_lost_mid;
    uint8_t both_lost_mid;

    track_point_t left_up;
    track_point_t right_up;
    track_point_t left_down;
    track_point_t right_down;
};

/* 串口识别结果。 */
struct recognition_t {
    bool valid;
    uint8_t cls;
    float score;
    uint8_t stable_cls;
    uint8_t stable_count;
    uint64_t timestamp_ms;
};

/* 电机运行状态。 */
struct motor_state_t {
    float left_speed_rps;
    float right_speed_rps;
    float left_target_rps;
    float right_target_rps;
    int left_cmd;
    int right_cmd;
    bool blocked;
    uint64_t blocked_since_ms;
};

/* 主循环统一上下文。 */
struct app_context_t {
    frame_t frame;
    track_result_t track;
    recognition_t recog;
    motor_state_t motor;
    system_state_e system_state;
    uint64_t now_ms;
};

static inline uint64_t app_millis()
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000ULL);
}

static inline float clampf(float x, float min_v, float max_v)
{
    return (x < min_v) ? min_v : ((x > max_v) ? max_v : x);
}

static inline int clampi(int x, int min_v, int max_v)
{
    return (x < min_v) ? min_v : ((x > max_v) ? max_v : x);
}
