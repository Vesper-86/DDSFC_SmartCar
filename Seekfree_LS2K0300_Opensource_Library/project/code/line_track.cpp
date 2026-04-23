#include "line_track.hpp"
#include <cstring>

/*
 * ============================================================================
 * 文件名称: line_track.cpp
 * 文件用途: 多扫描线循迹核心实现
 *
 * 算法流程:
 * 1. 在 ROI 区域内选择若干条水平扫描线。
 * 2. 每条扫描线分别从左到右、从右到左寻找边界。
 * 3. 若左右边界都有效，则得到该扫描线的中心点。
 * 4. 对所有有效扫描线中心求平均，得到整体赛道中心。
 * 5. 图像中心减赛道中心，即为转向误差。
 * ============================================================================
 */

void line_track_init(track_result_t &track_result)
{
    std::memset(&track_result, 0, sizeof(track_result));
    track_result.lost = true;
}

/*
 * 从左向右寻找“亮 -> 暗”的跳变点，作为左边界。
 */
static int find_left_edge(const uint8_t *gray_row, int image_width)
{
    for (int x = 1; x < image_width - 1; ++x) {
        if (gray_row[x - 1] > TRACK_BIN_THRESHOLD && gray_row[x] <= TRACK_BIN_THRESHOLD) {
            return x;
        }
    }
    return -1;
}

/*
 * 从右向左寻找“亮 -> 暗”的跳变点，作为右边界。
 */
static int find_right_edge(const uint8_t *gray_row, int image_width)
{
    for (int x = image_width - 2; x > 1; --x) {
        if (gray_row[x + 1] > TRACK_BIN_THRESHOLD && gray_row[x] <= TRACK_BIN_THRESHOLD) {
            return x;
        }
    }
    return -1;
}

void line_track_process(const frame_t &frame, track_result_t &track_result)
{
    if (!frame.valid || frame.gray == nullptr) {
        track_result.valid = false;
        track_result.lost = true;
        return;
    }

    const int image_width = frame.width;
    const int image_height = frame.height;
    const int roi_height = TRACK_ROI_Y1 - TRACK_ROI_Y0;

    int scan_step = (roi_height > 0) ? (roi_height / TRACK_SCAN_LINES) : 1;
    if (scan_step <= 0) {
        scan_step = 1;
    }

    int valid_line_count = 0;
    int center_sum = 0;

    for (int scan_index = 0; scan_index < TRACK_SCAN_LINES; ++scan_index) {
        int y = TRACK_ROI_Y1 - 1 - scan_index * scan_step;
        if (y < 0) y = 0;
        if (y >= image_height) y = image_height - 1;

        const uint8_t *gray_row = &frame.gray[y * image_width];
        const int left_edge = find_left_edge(gray_row, image_width);
        const int right_edge = find_right_edge(gray_row, image_width);

        track_result.left_edge[scan_index] = left_edge;
        track_result.right_edge[scan_index] = right_edge;
        track_result.center[scan_index] = -1;

        if (left_edge >= 0 && right_edge >= 0 && (right_edge - left_edge) > TRACK_MIN_VALID_POINTS) {
            track_result.center[scan_index] = (left_edge + right_edge) / 2;
            center_sum += track_result.center[scan_index];
            ++valid_line_count;
        }
    }

    /* 低速稳定安全版要求至少 3 条有效扫描线，避免单条误检带偏车体。 */
    if (valid_line_count < 3) {
        track_result.valid = false;
        track_result.lost = true;
        return;
    }

    track_result.center_avg = (float)center_sum / (float)valid_line_count;
    track_result.error = (float)TRACK_CENTER_X - track_result.center_avg;

    /* 一阶低通滤波，减少单帧噪声造成的方向抖动 */
    track_result.error_filtered =
        track_result.error_filtered * (1.0f - TRACK_ERROR_FILTER_ALPHA) +
        track_result.error * TRACK_ERROR_FILTER_ALPHA;

    track_result.timestamp_ms = frame.timestamp_ms;
    track_result.valid = true;
    track_result.lost = false;
}

float line_track_compute_speed_scale(const track_result_t &track_result)
{
    const float abs_error = (track_result.error_filtered >= 0.0f)
                          ? track_result.error_filtered
                          : -track_result.error_filtered;

    if (abs_error >= TRACK_BRAKE_ERROR) {
        return 0.25f;
    }
    if (abs_error >= TRACK_SLOWDOWN_ERROR) {
        return 0.55f;
    }
    return 1.0f;
}
