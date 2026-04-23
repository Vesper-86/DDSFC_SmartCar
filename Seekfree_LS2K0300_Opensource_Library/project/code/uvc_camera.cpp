#include "uvc_camera.hpp"
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
#include <cstring>

/*
 * ============================================================================
 * 文件名称: uvc_camera.cpp
 * 文件用途: UVC 摄像头采集实现
 *
 * 当前实现特点:
 * - 使用 read() 直接取帧，结构简单，容易读懂。
 * - 如果以后需要更高帧率，可以再升级为 mmap + buffer queue。
 * ============================================================================
 */

UvcCamera::UvcCamera() : fd_(-1)
{
}

UvcCamera::~UvcCamera()
{
}

int UvcCamera::init(frame_t &frame)
{
    fd_ = open(CAM_DEV_PATH, O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
        frame.yuyv = nullptr;
        frame.gray = nullptr;
        frame.valid = false;
        return -1;
    }

    /* 配置图像格式为 YUYV */
    v4l2_format format_config;
    std::memset(&format_config, 0, sizeof(format_config));
    format_config.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format_config.fmt.pix.width = CAM_WIDTH;
    format_config.fmt.pix.height = CAM_HEIGHT;
    format_config.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    format_config.fmt.pix.field = V4L2_FIELD_NONE;
    (void)ioctl(fd_, VIDIOC_S_FMT, &format_config);

    frame.width = CAM_WIDTH;
    frame.height = CAM_HEIGHT;
    frame.yuyv = static_cast<uint8_t *>(std::malloc((size_t)CAM_WIDTH * CAM_HEIGHT * 2U));
    frame.gray = static_cast<uint8_t *>(std::malloc((size_t)CAM_WIDTH * CAM_HEIGHT));
    frame.valid = false;

    if (frame.yuyv == nullptr || frame.gray == nullptr) {
        if (frame.yuyv) std::free(frame.yuyv);
        if (frame.gray) std::free(frame.gray);
        frame.yuyv = nullptr;
        frame.gray = nullptr;
        close(fd_);
        fd_ = -1;
        return -2;
    }

    return 0;
}

void UvcCamera::deinit(frame_t &frame)
{
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }

    if (frame.yuyv) std::free(frame.yuyv);
    if (frame.gray) std::free(frame.gray);
    frame.yuyv = nullptr;
    frame.gray = nullptr;
    frame.valid = false;
}

int UvcCamera::capture(frame_t &frame)
{
    if (fd_ < 0 || frame.yuyv == nullptr) {
        frame.valid = false;
        return -1;
    }

    const ssize_t expected_bytes = (ssize_t)frame.width * frame.height * 2;
    const ssize_t actual_bytes = read(fd_, frame.yuyv, (size_t)expected_bytes);
    if (actual_bytes != expected_bytes) {
        frame.valid = false;
        return -2;
    }

    frame.timestamp_ms = app_millis();
    frame.valid = true;
    return 0;
}

void UvcCamera::yuyv_to_gray(frame_t &frame)
{
    if (!frame.valid || frame.gray == nullptr || frame.yuyv == nullptr) {
        return;
    }

    /*
     * YUYV 每 4 字节表示 2 个像素：Y0 U Y1 V。
     * 循迹只需要亮度，所以直接取 Y0 和 Y1 即可得到灰度图。
     */
    const int pixel_count = frame.width * frame.height;
    for (int source_index = 0, gray_index = 0; gray_index < pixel_count; source_index += 4, gray_index += 2) {
        frame.gray[gray_index] = frame.yuyv[source_index];
        frame.gray[gray_index + 1] = frame.yuyv[source_index + 2];
    }
}
