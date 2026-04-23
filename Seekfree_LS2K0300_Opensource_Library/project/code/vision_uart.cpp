#include "vision_uart.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

/*
 * ============================================================================
 * 文件名称: vision_uart.cpp
 * 文件用途: 识别串口协议解析实现
 * ============================================================================
 */

static speed_t baud_to_flag(int baud)
{
    switch (baud) {
        case 115200: return B115200;
        case 57600:  return B57600;
        default:     return B115200;
    }
}

/* 对输入字节做逐字节异或校验。 */
static uint8_t checksum_xor(const uint8_t *buffer, size_t length)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; ++i) {
        checksum ^= buffer[i];
    }
    return checksum;
}

VisionUart::VisionUart()
    : fd_(-1), rx_len_(0), last_cls_(0), confirm_cnt_(0)
{
    std::memset(rx_buf_, 0, sizeof(rx_buf_));
}

VisionUart::~VisionUart()
{
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
    if (frame_bytes[0] != UART_FRAME_HEAD1 || frame_bytes[1] != UART_FRAME_HEAD2 || frame_bytes[5] != UART_FRAME_TAIL) {
        return;
    }

    if (checksum_xor(&frame_bytes[2], 2) != frame_bytes[4]) {
        return;
    }

    const uint8_t cls = frame_bytes[2];
    const float score = frame_bytes[3] / 100.0f;

    recognition.valid = true;
    recognition.cls = cls;
    recognition.score = score;
    recognition.timestamp_ms = app_millis();

    /* 连续帧确认，减少偶发误识别抖动 */
    if (cls == last_cls_ && score >= RECOG_SCORE_MIN) {
        if (confirm_cnt_ < 255) {
            ++confirm_cnt_;
        }
    } else {
        last_cls_ = cls;
        confirm_cnt_ = 1;
    }

    if (confirm_cnt_ >= RECOG_CONFIRM_FRAMES) {
        recognition.stable_cls = cls;
        recognition.stable_count = confirm_cnt_;
    }
}

void VisionUart::poll(recognition_t &recognition)
{
    if (fd_ < 0) {
        return;
    }

    uint8_t read_buffer[64];
    const ssize_t read_size = read(fd_, read_buffer, sizeof(read_buffer));
    if (read_size <= 0) {
        return;
    }

    /* 先把新收到的数据追加到接收缓存 */
    if (rx_len_ + (size_t)read_size > sizeof(rx_buf_)) {
        rx_len_ = 0;
    }

    std::memcpy(&rx_buf_[rx_len_], read_buffer, (size_t)read_size);
    rx_len_ += (size_t)read_size;

    /* 循环解析缓存中的完整协议帧 */
    while (rx_len_ >= 6) {
        size_t frame_head_pos = 0;

        while (frame_head_pos + 1 < rx_len_ &&
              !(rx_buf_[frame_head_pos] == UART_FRAME_HEAD1 && rx_buf_[frame_head_pos + 1] == UART_FRAME_HEAD2)) {
            ++frame_head_pos;
        }

        /* 抛弃帧头之前的无效字节 */
        if (frame_head_pos > 0) {
            std::memmove(rx_buf_, rx_buf_ + frame_head_pos, rx_len_ - frame_head_pos);
            rx_len_ -= frame_head_pos;
            if (rx_len_ < 6) {
                break;
            }
        }

        handle_frame(rx_buf_, recognition);
        std::memmove(rx_buf_, rx_buf_ + 6, rx_len_ - 6);
        rx_len_ -= 6;
    }
}
