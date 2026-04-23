#pragma once

#include "app_types.hpp"

/*
 * ============================================================================
 * 类名称: UvcCamera
 * 文件用途: 封装标准 UVC 摄像头的 V4L2 采集流程
 *
 * 设计说明:
 * - 继续使用 Linux 用户态 V4L2 接口，便于复用你现有的摄像头方案。
 * - 与逐飞库并不冲突，因为 UVC 摄像头本身就是 Linux 设备。
 *
 * 主要成员函数:
 * - init()         : 打开摄像头并分配图像缓存
 * - deinit()       : 关闭摄像头并释放缓存
 * - capture()      : 采集一帧 YUYV 图像
 * - yuyv_to_gray() : 抽取亮度通道，生成灰度图
 * ============================================================================
 */
class UvcCamera
{
public:
    UvcCamera();
    ~UvcCamera();

    int init(frame_t &frame);
    void deinit(frame_t &frame);

    int capture(frame_t &frame);
    void yuyv_to_gray(frame_t &frame);

private:
    int fd_;    /* 摄像头设备文件描述符 */
};
