#pragma once

#include "app_types.hpp"

/*
 * ============================================================================
 * 类名称: VisionUart
 * 文件用途: 解析视觉识别模块通过串口发来的结果帧
 *
 * 当前协议:
 *   AA 55 cls score checksum 0D
 *
 * 字段说明:
 * - cls      : 类别编号
 * - score    : 置信度百分数，例如 85 表示 0.85
 * - checksum : 对 cls 和 score 做异或校验
 *
 * 当前版本只“接收、解析、显示”识别结果，不再触发任务动作。
 * ============================================================================
 */
class VisionUart
{
public:
    VisionUart();
    ~VisionUart();

    int init();
    void deinit();
    void poll(recognition_t &recognition);

private:
    int fd_;                    /* 串口文件描述符 */
    uint8_t rx_buf_[256];       /* 接收缓存 */
    size_t rx_len_;             /* 当前缓存有效长度 */
    uint8_t last_cls_;          /* 上一帧类别，用于稳定确认 */
    uint8_t confirm_cnt_;       /* 连续确认计数 */

    void handle_frame(const uint8_t *frame_bytes, recognition_t &recognition);
};
