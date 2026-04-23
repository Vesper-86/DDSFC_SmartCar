#pragma once

#include "app_types.hpp"

/*
 * line_track_init()
 * ---------------------------------------------------------------------------
 * 初始化循迹结果结构体，清空历史数据。
 */
void line_track_init(track_result_t &track_result);

/*
 * line_track_process()
 * ---------------------------------------------------------------------------
 * 输入一帧灰度图，在 ROI 区域内做多扫描线边界搜索，输出赛道中心偏差。
 */
void line_track_process(const frame_t &frame, track_result_t &track_result);

/*
 * line_track_compute_speed_scale()
 * ---------------------------------------------------------------------------
 * 根据偏差大小返回一个速度缩放系数。
 * 偏差越大，返回值越小，用于自动减速。
 */
float line_track_compute_speed_scale(const track_result_t &track_result);
