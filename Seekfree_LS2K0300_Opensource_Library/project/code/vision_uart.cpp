#include "vision_uart.hpp"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

/*
 * ============================================================================
 * 文件名称: vision_uart.cpp
 * 文件用途: 识别串口协议解析实现（增强版）
 *
 * 改动重点:
 * 1. 解析器在尾字节/校验错误时做更稳的重同步，而不是直接盲目吃掉 6 字节。
 * 2. 低置信度帧不再直接重置稳定结果，只做衰减处理。
 * 3. 识别超时后自动失效，避免使用陈旧 stable_cls。
 * ============================================================================
 */

namespace
{
static constexpr size_t kFrameLen = 6;

static speed_t baud_to_flag(int baud)
{
    switch (baud) {
    case 115200: return B115200;
    case 57600:  return B57600;
    default:     return B115200;
    }
}

static uint8_t checksum_xor(const uint8_t *buffer, size_t length)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; ++i) {
        checksum ^= buffer[i];
    }
    return checksum;
}

static bool is_valid_frame(const uint8_t *frame_bytes)
{
    if (frame_bytes[0] != UART_FRAME_HEAD1 ||
        frame_bytes[1] != UART_FRAME_HEAD2 ||
        frame_bytes[5] != UART_FRAME_TAIL) {
        return false;
    }

    return checksum_xor(&frame_bytes[2], 2) == frame_bytes[4];
}
} // namespace

VisionUart::VisionUart()
    : fd_(-1), rx_len_(0), last_cls_(0), confirm_cnt_(0)
{
    std::memset(rx_buf_, 0, sizeof(rx_buf_));
}

VisionUart::~VisionUart()
{
    deinit();
}

int VisionUart::init()
{
    fd_ = open(UART_RECOG_DEV, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        return -1;
    }

    termios serial_config;
    std::memset(&serial_config, 0, sizeof(serial_config));
    tcgetattr(fd_, &serial_config);

    cfmakeraw(&serial_config);
    cfsetispeed(&serial_config, baud_to_flag(UART_BAUD_RECOG));
    cfsetospeed(&serial_config, baud_to_flag(UART_BAUD_RECOG));
    serial_config.c_cflag |= (CLOCAL | CREAD);

    tcsetattr(fd_, TCSANOW, &serial_config);
    return 0;
}

void VisionUart::deinit()
{
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }

    rx_len_ = 0;
    last_cls_ = 0;
    confirm_cnt_ = 0;
}

void VisionUart::handle_frame(const uint8_t *frame_bytes, recognition_t &recognition)
{
    if (!is_valid_frame(frame_bytes)) {
        return;
    }

    const uint8_t cls = frame_bytes[2];
    const float score = static_cast<float>(frame_bytes[3]) / 100.0f;
    const uint64_t now = app_millis();

    recognition.valid = true;
    recognition.cls = cls;
    recognition.score = score;
    recognition.timestamp_ms = now;

    if (score < RECOG_SCORE_MIN) {
        if (confirm_cnt_ > RECOG_CONFIRM_DECAY) {
            confirm_cnt_ -= RECOG_CONFIRM_DECAY;
        } else {
            confirm_cnt_ = 0;
        }
        recognition.stable_count = confirm_cnt_;
        return;
    }

    if (cls == last_cls_) {
        if (confirm_cnt_ < 255) {
            ++confirm_cnt_;
        }
    } else {
        last_cls_ = cls;
        confirm_cnt_ = 1;
    }

    recognition.stable_count = confirm_cnt_;
    if (confirm_cnt_ >= RECOG_CONFIRM_FRAMES) {
        recognition.stable_cls = cls;
    }
}

void VisionUart::poll(recognition_t &recognition)
{
    const uint64_t now = app_millis();
    if (recognition.valid && (now - recognition.timestamp_ms > RECOG_TIMEOUT_MS)) {
        recognition.valid = false;
        recognition.stable_count = 0;
    }

    if (fd_ < 0) {
        return;
    }

    uint8_t read_buffer[64];
    const ssize_t read_size = read(fd_, read_buffer, sizeof(read_buffer));
    if (read_size <= 0) {
        return;
    }

    if (rx_len_ + static_cast<size_t>(read_size) > sizeof(rx_buf_)) {
        rx_len_ = 0;
    }

    std::memcpy(&rx_buf_[rx_len_], read_buffer, static_cast<size_t>(read_size));
    rx_len_ += static_cast<size_t>(read_size);

    while (rx_len_ >= kFrameLen) {
        size_t frame_head_pos = 0;
        while (frame_head_pos + 1 < rx_len_ &&
               !(rx_buf_[frame_head_pos] == UART_FRAME_HEAD1 &&
                 rx_buf_[frame_head_pos + 1] == UART_FRAME_HEAD2)) {
            ++frame_head_pos;
        }

        if (frame_head_pos > 0) {
            std::memmove(rx_buf_, rx_buf_ + frame_head_pos, rx_len_ - frame_head_pos);
            rx_len_ -= frame_head_pos;
            if (rx_len_ < kFrameLen) {
                break;
            }
        }

        if (rx_buf_[5] != UART_FRAME_TAIL) {
            std::memmove(rx_buf_, rx_buf_ + 1, rx_len_ - 1);
            --rx_len_;
            continue;
        }

        if (!is_valid_frame(rx_buf_)) {
            std::memmove(rx_buf_, rx_buf_ + 1, rx_len_ - 1);
            --rx_len_;
            continue;
        }

        handle_frame(rx_buf_, recognition);
        std::memmove(rx_buf_, rx_buf_ + kFrameLen, rx_len_ - kFrameLen);
        rx_len_ -= kFrameLen;
    }
}
