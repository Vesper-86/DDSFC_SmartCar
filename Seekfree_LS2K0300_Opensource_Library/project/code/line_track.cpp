#include "line_track.hpp"

#include <array>
#include <cmath>
#include <cstring>

/*
 * ============================================================================
 * 文件名称: line_track.cpp
 * 文件用途: 多扫描线循迹核心实现（增强版）
 *
 * 相比基础版的改动:
 * 1. 使用 ROI Otsu 阈值作为全局基准，替代纯固定阈值。
 * 2. 由“整行暴力找边”改为“种子点 + 局部窗口连续跟踪”。
 * 3. 支持单边丢失恢复，使用历史赛道宽度补全中心线。
 * 4. 增加曲率与有效率估计，为速度调度提供额外依据。
 * ============================================================================
 */

namespace
{
float g_lane_width_est = 72.0f;
float g_curvature_hint = 0.0f;
float g_quality_hint = 0.0f;
float g_center_hint = static_cast<float>(TRACK_CENTER_X);

inline float absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

inline int clamp_int(int x, int lo, int hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static int compute_roi_otsu_threshold(const frame_t &frame)
{
    if (frame.gray == nullptr || frame.width <= 0 || frame.height <= 0) {
        return TRACK_BIN_THRESHOLD;
    }

    std::array<int, 256> hist{};
    const int y0 = clamp_int(TRACK_ROI_Y0, 0, frame.height - 1);
    const int y1 = clamp_int(TRACK_ROI_Y1, 0, frame.height);
    const int stride = (TRACK_OTSU_SAMPLE_STRIDE > 0) ? TRACK_OTSU_SAMPLE_STRIDE : 1;

    int sample_count = 0;
    for (int y = y0; y < y1; y += stride) {
        const uint8_t *row = &frame.gray[y * frame.width];
        for (int x = 0; x < frame.width; x += stride) {
            ++hist[row[x]];
            ++sample_count;
        }
    }

    if (sample_count <= 0) {
        return TRACK_BIN_THRESHOLD;
    }

    double total_sum = 0.0;
    for (int i = 0; i < 256; ++i) {
        total_sum += static_cast<double>(i * hist[i]);
    }

    double bg_sum = 0.0;
    int bg_count = 0;
    double best_var = -1.0;
    int best_th = TRACK_BIN_THRESHOLD;

    for (int th = 0; th < 256; ++th) {
        bg_count += hist[th];
        if (bg_count == 0) {
            continue;
        }

        const int fg_count = sample_count - bg_count;
        if (fg_count == 0) {
            break;
        }

        bg_sum += static_cast<double>(th * hist[th]);
        const double bg_mean = bg_sum / static_cast<double>(bg_count);
        const double fg_mean = (total_sum - bg_sum) / static_cast<double>(fg_count);
        const double mean_diff = bg_mean - fg_mean;
        const double between_var =
            static_cast<double>(bg_count) * static_cast<double>(fg_count) * mean_diff * mean_diff;

        if (between_var > best_var) {
            best_var = between_var;
            best_th = th;
        }
    }

    return clamp_int(best_th, 40, 180);
}

static int find_dark_seed(const uint8_t *row, int width, int threshold)
{
    const int mid = width / 2;
    const int left_bound = clamp_int(mid - TRACK_SEED_SEARCH_HALF_WIDTH, 0, width - 1);
    const int right_bound = clamp_int(mid + TRACK_SEED_SEARCH_HALF_WIDTH, 0, width - 1);

    int best_x = -1;
    int best_dist = width + 1;

    for (int x = left_bound; x <= right_bound; ++x) {
        if (row[x] <= threshold) {
            const int dist = (x >= mid) ? (x - mid) : (mid - x);
            if (dist < best_dist) {
                best_dist = dist;
                best_x = x;
            }
        }
    }

    if (best_x >= 0) {
        return best_x;
    }

    uint8_t darkest = 255;
    for (int x = left_bound; x <= right_bound; ++x) {
        if (row[x] < darkest) {
            darkest = row[x];
            best_x = x;
        }
    }

    return best_x;
}

static int search_left_edge(const uint8_t *row, int width, int threshold, int center_guess, int search_margin)
{
    const int x0 = clamp_int(center_guess - search_margin, 1, width - 2);
    const int x1 = clamp_int(center_guess, 1, width - 2);

    for (int x = x1; x >= x0; --x) {
        const int grad = static_cast<int>(row[x - 1]) - static_cast<int>(row[x]);
        if (row[x - 1] > threshold && row[x] <= threshold && grad >= TRACK_EDGE_GRAD_MIN) {
            return x;
        }
    }
    return -1;
}

static int search_right_edge(const uint8_t *row, int width, int threshold, int center_guess, int search_margin)
{
    const int x0 = clamp_int(center_guess, 1, width - 2);
    const int x1 = clamp_int(center_guess + search_margin, 1, width - 2);

    for (int x = x0; x <= x1; ++x) {
        const int grad = static_cast<int>(row[x + 1]) - static_cast<int>(row[x]);
        if (row[x + 1] > threshold && row[x] <= threshold && grad >= TRACK_EDGE_GRAD_MIN) {
            return x;
        }
    }
    return -1;
}

static bool lane_width_is_reasonable(int width_px)
{
    return width_px >= TRACK_MIN_LANE_WIDTH && width_px <= TRACK_MAX_LANE_WIDTH;
}
} // namespace

void line_track_init(track_result_t &track_result)
{
    std::memset(&track_result, 0, sizeof(track_result));
    for (int i = 0; i < TRACK_SCAN_LINES; ++i) {
        track_result.left_edge[i] = -1;
        track_result.right_edge[i] = -1;
        track_result.center[i] = -1;
    }
    track_result.lost = true;
    g_lane_width_est = 72.0f;
    g_curvature_hint = 0.0f;
    g_quality_hint = 0.0f;
    g_center_hint = static_cast<float>(TRACK_CENTER_X);
}

void line_track_process(const frame_t &frame, track_result_t &track_result)
{
    for (int i = 0; i < TRACK_SCAN_LINES; ++i) {
        track_result.left_edge[i] = -1;
        track_result.right_edge[i] = -1;
        track_result.center[i] = -1;
    }

    if (!frame.valid || frame.gray == nullptr || frame.width <= 8 || frame.height <= 8) {
        track_result.valid = false;
        track_result.lost = true;
        return;
    }

    const int image_width = frame.width;
    const int image_height = frame.height;
    const int y0 = clamp_int(TRACK_ROI_Y0, 0, image_height - 1);
    const int y1 = clamp_int(TRACK_ROI_Y1, y0 + 1, image_height);
    const int roi_height = y1 - y0;
    int scan_step = (roi_height > 0) ? (roi_height / TRACK_SCAN_LINES) : 1;
    if (scan_step <= 0) {
        scan_step = 1;
    }

    const int otsu_threshold = compute_roi_otsu_threshold(frame);

    int valid_line_count = 0;
    float weighted_center_sum = 0.0f;
    float weight_sum = 0.0f;
    int prev_center = clamp_int(static_cast<int>(g_center_hint + 0.5f), 0, image_width - 1);
    bool prev_center_valid = false;

    float top_center_sum = 0.0f;
    float bottom_center_sum = 0.0f;
    int top_center_count = 0;
    int bottom_center_count = 0;

    for (int scan_index = 0; scan_index < TRACK_SCAN_LINES; ++scan_index) {
        int y = y1 - 1 - scan_index * scan_step;
        y = clamp_int(y, 0, image_height - 1);
        const uint8_t *row = &frame.gray[y * image_width];

        int local_threshold = otsu_threshold;
        const int seed = find_dark_seed(row, image_width, local_threshold);
        if (scan_index == 0 && seed >= 0) {
            prev_center = seed;
        }

        int center_guess = prev_center_valid ? prev_center : prev_center;
        center_guess = clamp_int(center_guess, 1, image_width - 2);

        const int margin =
            (scan_index == 0) ? TRACK_SEED_SEARCH_HALF_WIDTH : TRACK_LOCAL_SEARCH_MARGIN;

        int left_edge = search_left_edge(row, image_width, local_threshold, center_guess, margin);
        int right_edge = search_right_edge(row, image_width, local_threshold, center_guess, margin);

        int center = -1;
        bool recovered_single_side = false;

        if (left_edge >= 0 && right_edge >= 0) {
            const int width_px = right_edge - left_edge;
            if (lane_width_is_reasonable(width_px)) {
                center = (left_edge + right_edge) / 2;
                g_lane_width_est =
                    g_lane_width_est * (1.0f - TRACK_WIDTH_FILTER_ALPHA) +
                    static_cast<float>(width_px) * TRACK_WIDTH_FILTER_ALPHA;
            } else {
                left_edge = -1;
                right_edge = -1;
            }
        }

        if (center < 0 && g_lane_width_est >= static_cast<float>(TRACK_MIN_LANE_WIDTH)) {
            const int half_width = clamp_int(static_cast<int>(g_lane_width_est * 0.5f + 0.5f),
                                             TRACK_MIN_LANE_WIDTH / 2,
                                             TRACK_MAX_LANE_WIDTH / 2);
            if (left_edge >= 0 && right_edge < 0) {
                right_edge = clamp_int(left_edge + 2 * half_width, 0, image_width - 1);
                center = (left_edge + right_edge) / 2;
                recovered_single_side = true;
            } else if (right_edge >= 0 && left_edge < 0) {
                left_edge = clamp_int(right_edge - 2 * half_width, 0, image_width - 1);
                center = (left_edge + right_edge) / 2;
                recovered_single_side = true;
            }
        }

        if (center >= 0) {
            track_result.left_edge[scan_index] = left_edge;
            track_result.right_edge[scan_index] = right_edge;
            track_result.center[scan_index] = center;
            prev_center = center;
            prev_center_valid = true;

            const float weight = static_cast<float>(TRACK_SCAN_LINES - scan_index);
            weighted_center_sum += static_cast<float>(center) * weight;
            weight_sum += weight;
            ++valid_line_count;

            if (scan_index < TRACK_SCAN_LINES / 2) {
                bottom_center_sum += static_cast<float>(center);
                ++bottom_center_count;
            } else {
                top_center_sum += static_cast<float>(center);
                ++top_center_count;
            }

            if (recovered_single_side) {
                weighted_center_sum -= 0.20f * weight * (static_cast<float>(center) - g_center_hint);
            }
        }
    }

    const float valid_ratio = static_cast<float>(valid_line_count) / static_cast<float>(TRACK_SCAN_LINES);
    g_quality_hint = 0.80f * g_quality_hint + 0.20f * valid_ratio;

    if (valid_line_count < 3 || valid_ratio < TRACK_VALID_RATIO_MIN || weight_sum <= 0.0f) {
        track_result.valid = false;
        track_result.lost = true;
        return;
    }

    track_result.center_avg = weighted_center_sum / weight_sum;
    g_center_hint = g_center_hint * (1.0f - TRACK_ERROR_FILTER_ALPHA) +
                    track_result.center_avg * TRACK_ERROR_FILTER_ALPHA;

    track_result.error = static_cast<float>(TRACK_CENTER_X) - track_result.center_avg;
    track_result.error_filtered =
        track_result.error_filtered * (1.0f - TRACK_ERROR_FILTER_ALPHA) +
        track_result.error * TRACK_ERROR_FILTER_ALPHA;

    float curvature_raw = 0.0f;
    if (bottom_center_count > 0 && top_center_count > 0) {
        const float bottom_mean = bottom_center_sum / static_cast<float>(bottom_center_count);
        const float top_mean = top_center_sum / static_cast<float>(top_center_count);
        curvature_raw = top_mean - bottom_mean;
    }
    g_curvature_hint = g_curvature_hint * (1.0f - TRACK_CURVATURE_FILTER_ALPHA) +
                       curvature_raw * TRACK_CURVATURE_FILTER_ALPHA;

    track_result.timestamp_ms = frame.timestamp_ms;
    track_result.valid = true;
    track_result.lost = false;
}

float line_track_compute_speed_scale(const track_result_t &track_result)
{
    const float abs_error = absf(track_result.error_filtered);
    const float abs_curvature = absf(g_curvature_hint);

    float error_scale = 1.0f;
    if (abs_error >= TRACK_BRAKE_ERROR) {
        error_scale = 0.28f;
    } else if (abs_error >= TRACK_SLOWDOWN_ERROR) {
        const float t = (abs_error - TRACK_SLOWDOWN_ERROR) /
                        (TRACK_BRAKE_ERROR - TRACK_SLOWDOWN_ERROR + 1e-6f);
        error_scale = 0.62f - 0.34f * clampf(t, 0.0f, 1.0f);
    }

    const float curvature_penalty =
        clampf(abs_curvature / TRACK_SPEED_CURVATURE_SLOWDOWN, 0.0f, 1.0f) * 0.28f;
    const float quality_scale =
        TRACK_SPEED_QUALITY_FLOOR + (1.0f - TRACK_SPEED_QUALITY_FLOOR) * clampf(g_quality_hint, 0.0f, 1.0f);

    float final_scale = error_scale * (1.0f - curvature_penalty) * quality_scale;
    if (!track_result.valid || track_result.lost) {
        final_scale = 0.20f;
    }

    return clampf(final_scale, 0.20f, 1.00f);
}
