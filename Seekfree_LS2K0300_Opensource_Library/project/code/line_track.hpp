#pragma once

#include "app_types.hpp"

/* 初始化循迹结果结构体。 */
void line_track_init(track_result_t &track_result);

/*
 * 输入一帧灰度图，输出：
 * 1. 基础循迹误差
 * 2. 十字 / 环岛图像判定
 * 3. 元素区补线后的中心线结果
 */
void line_track_process(const frame_t &frame, track_result_t &track_result);

/* 根据偏差、曲率、图像质量和元素区状态返回速度缩放系数。 */
float line_track_compute_speed_scale(const track_result_t &track_result);

/* 返回当前元素名称，便于显示调试。 */
const char *line_track_element_name(uint8_t element);
const char *line_track_state_name(uint8_t state);
