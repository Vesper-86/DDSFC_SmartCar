#pragma once

#include "zf_common_headfile.hpp"
#include "app_types.hpp"
#include "app_config.hpp"

/*
 * ============================================================================
 * 类名称: UvcCamera
 * 文件用途: 对逐飞 zf_device_uvc 做一层轻量封装
 *
 * 设计目标:
 * 1. 保留原来的 UvcCamera 类接口，不改主程序调用方式。
 * 2. capture() 成功后，frame.gray 中已经是可直接使用的灰度图。
 * 3. yuyv_to_gray() 保留为空操作，方便兼容旧主程序调用风格。
 * ============================================================================
 */
class UvcCamera
{
public:
    UvcCamera();
    ~UvcCamera();

    /*
     * init()
     * ------------------------------------------------------------------------
     * 初始化逐飞 UVC 设备，并为 frame 分配灰度图缓存。
     */
    int init(frame_t &frame);

    /* 释放缓存并关闭对象。 */
    void deinit(frame_t &frame);

    /*
     * capture()
     * ------------------------------------------------------------------------
     * 从逐飞 UVC 设备获取最新一帧灰度图。
     * 返回 0 表示成功，负数表示失败。
     */
    int capture(frame_t &frame);

    /*
     * yuyv_to_gray()
     * ------------------------------------------------------------------------
     * 当前版本为空操作。
     * 因为 zf_device_uvc 已直接提供灰度图接口。
     */
    void yuyv_to_gray(frame_t &frame);

private:
    zf_device_uvc uvc_dev_;   /* 逐飞 UVC 设备对象 */
    uint8_t *gray_shadow_;    /* 灰度影子缓存，防止底层缓冲被下一帧覆盖 */
    int gray_ready_;          /* 当前灰度图是否准备完成 */
};
