#include "uvc_camera.hpp"

#include <cstdlib>
#include <cstring>
#include <cstdio>

UvcCamera::UvcCamera()
    : gray_shadow_(nullptr), gray_ready_(0)
{
}

UvcCamera::~UvcCamera()
{
}

int UvcCamera::init(frame_t &frame)
{
    if (uvc_dev_.init(UVC_PATH) < 0)
    {
        frame.yuyv = nullptr;
        frame.gray = nullptr;
        frame.valid = false;
        return -1;
    }

    frame.width = CAM_WIDTH;
    frame.height = CAM_HEIGHT;
    frame.yuyv = static_cast<uint8_t *>(std::malloc((size_t)CAM_WIDTH * (size_t)CAM_HEIGHT * 2U));
    frame.gray = static_cast<uint8_t *>(std::malloc((size_t)CAM_WIDTH * (size_t)CAM_HEIGHT));
    gray_shadow_ = static_cast<uint8_t *>(std::malloc((size_t)CAM_WIDTH * (size_t)CAM_HEIGHT));
    frame.valid = false;
    gray_ready_ = 0;

    if (frame.yuyv == nullptr || frame.gray == nullptr || gray_shadow_ == nullptr)
    {
        if (frame.yuyv) std::free(frame.yuyv);
        if (frame.gray) std::free(frame.gray);
        if (gray_shadow_) std::free(gray_shadow_);
        frame.yuyv = nullptr;
        frame.gray = nullptr;
        gray_shadow_ = nullptr;
        frame.valid = false;
        return -2;
    }

    std::printf("uvc seekfree wrapper ready, frame = %d x %d\r\n", frame.width, frame.height);
    return 0;
}

void UvcCamera::deinit(frame_t &frame)
{
    if (frame.yuyv) std::free(frame.yuyv);
    if (frame.gray) std::free(frame.gray);
    if (gray_shadow_) std::free(gray_shadow_);

    frame.yuyv = nullptr;
    frame.gray = nullptr;
    gray_shadow_ = nullptr;
    frame.valid = false;
    gray_ready_ = 0;
}

int UvcCamera::capture(frame_t &frame)
{
    gray_ready_ = 0;

    if (frame.gray == nullptr || gray_shadow_ == nullptr)
    {
        frame.valid = false;
        return -1;
    }

    if (uvc_dev_.wait_image_refresh() < 0)
    {
        frame.valid = false;
        return -2;
    }

    uint8 *gray_ptr = uvc_dev_.get_gray_image_ptr();
    if (gray_ptr == nullptr)
    {
        frame.valid = false;
        return -3;
    }

    /*
     * 当前实现仍假设底层灰度缓冲区尺寸与 CAM_WIDTH/CAM_HEIGHT 一致。
     * 若后续接入不同分辨率摄像头，需要在这里增加尺寸查询与缩放/裁剪适配。
     */
    std::memcpy(gray_shadow_, gray_ptr, (size_t)CAM_WIDTH * (size_t)CAM_HEIGHT);
    std::memcpy(frame.gray, gray_shadow_, (size_t)CAM_WIDTH * (size_t)CAM_HEIGHT);

    frame.timestamp_ms = app_millis();
    frame.valid = true;
    gray_ready_ = 1;
    return 0;
}

void UvcCamera::yuyv_to_gray(frame_t &frame)
{
    (void)frame;
    /*
     * 逐飞官方 zf_device_uvc 已经直接给出灰度图。
     * 为了保持你原主程序调用方式不变，这里不再重复转换。
     */
}
