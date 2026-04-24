#include "zf_common_headfile.hpp"
#include "image.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <cstring>

using namespace cv;

#define SERVER_IP        "10.190.147.15"
#define PORT             8086
#define BOUNDARY_NUM     120

zf_driver_tcp_client tcp_client_dev;
uint8_t image_copy[UVC_HEIGHT][UVC_WIDTH];

uint8 xy_x1_boundary[BOUNDARY_NUM];
uint8 xy_x2_boundary[BOUNDARY_NUM];
uint8 xy_x3_boundary[BOUNDARY_NUM];
uint8 xy_y1_boundary[BOUNDARY_NUM];
uint8 xy_y2_boundary[BOUNDARY_NUM];
uint8 xy_y3_boundary[BOUNDARY_NUM];

extern uint8_t base_image[UVC_HEIGHT][UVC_WIDTH];
extern uint8_t image[UVC_HEIGHT][UVC_WIDTH];
extern uint8_t left_line_list[UVC_HEIGHT];
extern uint8_t right_line_list[UVC_HEIGHT];
extern uint8_t mid_line_list[UVC_HEIGHT];
extern uint8_t final_mid_line;
extern track_context_t g_track_ctx;

uint32 tcp_send_wrap(const uint8 *buf, uint32 len) { return tcp_client_dev.send_data(buf, len); }
uint32 tcp_read_wrap(uint8 *buf, uint32 len) { return tcp_client_dev.read_data(buf, len); }

static void binary_preprocess(const Mat &gray, Mat &bin)
{
    threshold(gray, bin, 0, 255, THRESH_BINARY | THRESH_OTSU);

    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(bin, bin, MORPH_OPEN, kernel);
    morphologyEx(bin, bin, MORPH_CLOSE, kernel);
    medianBlur(bin, bin, 3);
    flip(bin, bin, 0);
}

void show_edge_mid(void)
{
    memset(xy_x1_boundary, 0, sizeof(xy_x1_boundary));
    memset(xy_y1_boundary, 0, sizeof(xy_y1_boundary));
    memset(xy_x2_boundary, 0, sizeof(xy_x2_boundary));
    memset(xy_y2_boundary, 0, sizeof(xy_y2_boundary));
    memset(xy_x3_boundary, 0, sizeof(xy_x3_boundary));
    memset(xy_y3_boundary, 0, sizeof(xy_y3_boundary));

    int show_num = (BOUNDARY_NUM < UVC_HEIGHT) ? BOUNDARY_NUM : UVC_HEIGHT;

    for (int i = 0; i < show_num; ++i)
    {
        int y = UVC_HEIGHT - 1 - i;

        xy_x1_boundary[i] = left_line_list[y];
        xy_y1_boundary[i] = y;
        xy_x2_boundary[i] = right_line_list[y];
        xy_y2_boundary[i] = y;
        xy_x3_boundary[i] = mid_line_list[y];
        xy_y3_boundary[i] = y;
    }

    seekfree_assistant_camera_boundary_config(
        XY_BOUNDARY,
        50,
        xy_x1_boundary,
        xy_x2_boundary,
        xy_x3_boundary,
        xy_y1_boundary,
        xy_y2_boundary,
        xy_y3_boundary
    );
}

int main(int, char **)
{
    if (tcp_client_dev.init(SERVER_IP, PORT) != 0)
    {
        printf("tcp client init failed\r\n");
        return -1;
    }

    seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);
    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        image_copy[0],
        UVC_WIDTH,
        UVC_HEIGHT
    );

    zf_device_uvc uvc_obj;
    if (uvc_obj.init("/dev/video0") != 0)
    {
        printf("uvc init error\r\n");
        return -1;
    }

    while (1)
    {
        if (uvc_obj.wait_image_refresh() != 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        uint8_t *gray_ptr = uvc_obj.get_gray_image_ptr();
        Mat gray(Size(UVC_WIDTH, UVC_HEIGHT), CV_8UC1, gray_ptr);
        Mat bin;

        for (int y = 0; y < UVC_HEIGHT; ++y)
        {
            memcpy(base_image[y], gray.ptr(y), UVC_WIDTH);
        }

        binary_preprocess(gray, bin);

        for (int y = 0; y < UVC_HEIGHT; ++y)
        {
            memcpy(image[y], bin.ptr(y), UVC_WIDTH);
        }

        track_process_frame();

        memcpy(image_copy[0], image[0], UVC_WIDTH * UVC_HEIGHT);
        show_edge_mid();
        seekfree_assistant_camera_send();

        // 这里把 final_mid_line / g_track_ctx.state / g_track_ctx.element 发给下位机控制即可
        // 例如：servo_error = (int)final_mid_line - (UVC_WIDTH / 2);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}
