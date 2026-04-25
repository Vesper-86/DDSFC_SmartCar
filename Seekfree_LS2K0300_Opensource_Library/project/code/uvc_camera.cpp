#include "uvc_camera.hpp"

#include <cstdlib>
#include <cstring>
#include <cstdio>

namespace
{
/*
 * apply_camera_orientation()
 * ---------------------------------------------------------------------------
 * 对 UVC 灰度图做方向修正。
 *
 * 为什么放在摄像头封装层：
 * - 后面的 line_track、TCP 图传、IPS 显示全部共用 frame.gray；
 * - 在这里修正一次，后续所有模块看到的都是同一个正确方向；
 * - 避免在寻线、显示、控制里各自翻转导致不一致。
 */
static void apply_camera_orientation(const uint8_t *src, uint8_t *dst, int width, int height)
{
    if (src == nullptr || dst == nullptr || width <= 0 || height <= 0)
    {
        return;
    }

#if CAMERA_ROTATE_180
    for (int y = 0; y < height; ++y)
    {
        const int src_y = height - 1 - y;
        for (int x = 0; x < width; ++x)
        {
            const int src_x = width - 1 - x;
            dst[y * width + x] = src[src_y * width + src_x];
        }
    }
#elif CAMERA_FLIP_VERTICAL && CAMERA_FLIP_HORIZONTAL
    for (int y = 0; y < height; ++y)
    {
        const int src_y = height - 1 - y;
        for (int x = 0; x < width; ++x)
        {
            const int src_x = width - 1 - x;
            dst[y * width + x] = src[src_y * width + src_x];
        }
    }
#elif CAMERA_FLIP_VERTICAL
    for (int y = 0; y < height; ++y)
    {
        const int src_y = height - 1 - y;
        std::memcpy(&dst[y * width],
                    &src[src_y * width],
                    static_cast<size_t>(width));
    }
#elif CAMERA_FLIP_HORIZONTAL
    for (int y = 0; y < height; ++y)
    {
        const uint8_t *src_row = &src[y * width];
        uint8_t *dst_row = &dst[y * width];
        for (int x = 0; x < width; ++x)
        {
            dst_row[x] = src_row[width - 1 - x];
        }
    }
#else
    std::memcpy(dst, src, static_cast<size_t>(width) * static_cast<size_t>(height));
#endif
}
} // namespace


UvcCamera::UvcCamera()
    : gray_shadow_(nullptr), gray_ready_(0)
{
}

UvcCamera::~UvcCamera()
{
}

int UvcCamera::init(frame_t &frame)
{
    if (uvc_dev_.init(CAM_DEV_PATH) < 0)
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
    frame.timestamp_ms = 0;
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

    std::printf("uvc seekfree wrapper ready, frame = %d x %d, dev = %s\r\n",
                frame.width,
                frame.height,
                CAM_DEV_PATH);
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
     * 说明:
     * - 当前实现假设底层灰度图分辨率与 CAM_WIDTH / CAM_HEIGHT 一致。
     * - 先把底层灰度图复制到 shadow，再根据 app_config.hpp 的方向宏修正到 frame.gray。
     * - 这样 line_track / TCP 图传 / IPS 显示使用的都是修正后的统一图像。
     */
    std::memcpy(gray_shadow_, gray_ptr, (size_t)CAM_WIDTH * (size_t)CAM_HEIGHT);
    apply_camera_orientation(gray_shadow_, frame.gray, CAM_WIDTH, CAM_HEIGHT);

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
     * 为了保持旧主程序调用方式不变，这里不再重复转换。
     */
}
