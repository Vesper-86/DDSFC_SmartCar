#pragma once

#include "zf_common_headfile.hpp"
#include "app_types.hpp"
#include "app_config.hpp"

/*
 * 说明：
 * 1. 保留你原来的 UvcCamera 类接口，不改主程序调用方式。
 * 2. 类内部不再自己直接走 Linux read() 取图，而是改为封装逐飞官方 zf_device_uvc。
 * 3. capture() 成功后，frame.gray 中已经是可直接使用的灰度图，yuyv_to_gray() 变成空操作。
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
    zf_device_uvc uvc_dev_;
    uint8_t *gray_shadow_;
    int gray_ready_;
};
