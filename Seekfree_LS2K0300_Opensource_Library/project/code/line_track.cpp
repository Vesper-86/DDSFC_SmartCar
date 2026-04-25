#include "line_track.hpp"

#include <array>
#include <cstring>

/*
 * ============================================================================
 * 文件名称: line_track.cpp
 * 文件用途: 以 image_track 为骨干的增强寻线实现
 *
 * 核心思路:
 * 1. 灰度图 -> ROI Otsu 自适应二值化，白色为可行赛道，黑色为边界/背景；
 * 2. 从底部最长白列寻找种子点；
 * 3. 由种子点逐行局部搜索左右边界；
 * 4. 单边/双边丢失时使用历史赛道宽度补线；
 * 5. 统计十字/环岛特征，进入元素状态后做针对性补线；
 * 6. 输出逐行边界和抽样边界，供控制、TCP 图传和 IPS 叠加显示使用。
 * ============================================================================
 */

namespace
{
static uint8_t g_binary[CAM_HEIGHT][CAM_WIDTH];
static uint8_t g_binary_tmp[CAM_HEIGHT][CAM_WIDTH];
static uint8_t g_white_column[CAM_WIDTH];

struct internal_track_context_t
{
    uint8_t element;
    uint8_t state;
    uint8_t state_hold;
    uint8_t cross_counter;
    uint8_t ring_left_counter;
    uint8_t ring_right_counter;
    uint8_t reenter_cooldown;
    uint16_t state_elapsed_frames;
    bool timeout_exit;

    int seed_column;
    int search_stop_line;
    int search_top_y;
    int road_width_base;
    int road_width_far;
    int lane_width_est;
    int last_mid_line;

    uint8_t left_lost_total;
    uint8_t right_lost_total;
    uint8_t both_lost_total;
    uint8_t left_lost_mid;
    uint8_t right_lost_mid;
    uint8_t both_lost_mid;

    track_point_t left_up;
    track_point_t right_up;
    track_point_t left_down;
    track_point_t right_down;
};

static internal_track_context_t g_ctx;
static float g_error_filtered = 0.0f;
static float g_curvature_hint = 0.0f;
static float g_quality_hint = 0.0f;

static inline int clip_int(int v, int low, int high)
{
    if (v < low) return low;
    if (v > high) return high;
    return v;
}

static inline int abs_int(int v)
{
    return (v >= 0) ? v : -v;
}

static inline float abs_float(float v)
{
    return (v >= 0.0f) ? v : -v;
}

static int compute_roi_otsu_threshold(const frame_t &frame)
{
    std::array<int, 256> hist{};
    const int y0 = clip_int(TRACK_ROI_Y0, 0, frame.height - 1);
    const int y1 = clip_int(TRACK_ROI_Y1, y0 + 1, frame.height);
    const int stride = (TRACK_OTSU_SAMPLE_STRIDE > 0) ? TRACK_OTSU_SAMPLE_STRIDE : 1;

    int sample_count = 0;
    for (int y = y0; y < y1; y += stride)
    {
        const uint8_t *row = &frame.gray[y * frame.width];
        for (int x = 0; x < frame.width; x += stride)
        {
            ++hist[row[x]];
            ++sample_count;
        }
    }

    if (sample_count <= 0)
    {
        return TRACK_BIN_THRESHOLD;
    }

    double total_sum = 0.0;
    for (int i = 0; i < 256; ++i)
    {
        total_sum += static_cast<double>(i * hist[i]);
    }

    double bg_sum = 0.0;
    int bg_count = 0;
    double best_var = -1.0;
    int best_th = TRACK_BIN_THRESHOLD;

    for (int th = 0; th < 256; ++th)
    {
        bg_count += hist[th];
        if (bg_count == 0)
        {
            continue;
        }

        const int fg_count = sample_count - bg_count;
        if (fg_count == 0)
        {
            break;
        }

        bg_sum += static_cast<double>(th * hist[th]);
        const double bg_mean = bg_sum / static_cast<double>(bg_count);
        const double fg_mean = (total_sum - bg_sum) / static_cast<double>(fg_count);
        const double diff = bg_mean - fg_mean;
        const double between_var = static_cast<double>(bg_count) * static_cast<double>(fg_count) * diff * diff;

        if (between_var > best_var)
        {
            best_var = between_var;
            best_th = th;
        }
    }

    return clip_int(best_th, TRACK_BINARY_MIN, TRACK_BINARY_MAX);
}

static void binary_preprocess(const frame_t &frame)
{
    const int threshold = compute_roi_otsu_threshold(frame);

    for (int y = 0; y < frame.height; ++y)
    {
        const uint8_t *src = &frame.gray[y * frame.width];
        for (int x = 0; x < frame.width; ++x)
        {
            g_binary[y][x] = (src[x] >= threshold) ? 255 : 0;
        }
    }

#if TRACK_BINARY_DENOISE_ENABLE
    std::memcpy(g_binary_tmp, g_binary, sizeof(g_binary));
    for (int y = 1; y < frame.height - 1; ++y)
    {
        for (int x = 1; x < frame.width - 1; ++x)
        {
            int white_count = 0;
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dx = -1; dx <= 1; ++dx)
                {
                    if (g_binary_tmp[y + dy][x + dx] == 255)
                    {
                        ++white_count;
                    }
                }
            }
            g_binary[y][x] = (white_count >= 5) ? 255 : 0;
        }
    }
#endif
}

static void reset_frame_result(track_result_t &track)
{
    track.valid = false;
    track.lost = true;
    track.center_avg = static_cast<float>(g_ctx.last_mid_line);
    track.error = 0.0f;
    track.curvature_hint = 0.0f;
    track.quality_hint = 0.0f;
    track.timeout_exit = false;

    for (int i = 0; i < TRACK_SCAN_LINES; ++i)
    {
        track.left_edge[i] = -1;
        track.right_edge[i] = -1;
        track.center[i] = -1;
    }

    const int default_left = clip_int(g_ctx.last_mid_line - g_ctx.lane_width_est / 2, 0, CAM_WIDTH - 1);
    const int default_right = clip_int(g_ctx.last_mid_line + g_ctx.lane_width_est / 2, 0, CAM_WIDTH - 1);
    for (int y = 0; y < CAM_HEIGHT; ++y)
    {
        track.full_left[y] = default_left;
        track.full_right[y] = default_right;
        track.full_center[y] = g_ctx.last_mid_line;
        track.full_left_lost[y] = 1;
        track.full_right_lost[y] = 1;
    }

    g_ctx.left_lost_total = 0;
    g_ctx.right_lost_total = 0;
    g_ctx.both_lost_total = 0;
    g_ctx.left_lost_mid = 0;
    g_ctx.right_lost_mid = 0;
    g_ctx.both_lost_mid = 0;
    g_ctx.left_up = {0, 0, false};
    g_ctx.right_up = {0, 0, false};
    g_ctx.left_down = {0, 0, false};
    g_ctx.right_down = {0, 0, false};
    g_ctx.road_width_base = g_ctx.lane_width_est;
    g_ctx.road_width_far = g_ctx.lane_width_est;
    g_ctx.seed_column = g_ctx.last_mid_line;
    g_ctx.search_stop_line = TRACK_MIN_SEARCH_STOP_LINE;
    g_ctx.search_top_y = CAM_HEIGHT - TRACK_MIN_SEARCH_STOP_LINE;
}

static int average_width(const track_result_t &track, int y_start, int y_end)
{
    int sum = 0;
    int cnt = 0;

    y_start = clip_int(y_start, 0, CAM_HEIGHT - 1);
    y_end = clip_int(y_end, 0, CAM_HEIGHT - 1);
    if (y_start > y_end)
    {
        const int t = y_start;
        y_start = y_end;
        y_end = t;
    }

    for (int y = y_start; y <= y_end; ++y)
    {
        if (!track.full_left_lost[y] && !track.full_right_lost[y])
        {
            const int width = track.full_right[y] - track.full_left[y];
            if (width >= TRACK_MIN_LANE_WIDTH && width <= TRACK_MAX_LANE_WIDTH)
            {
                sum += width;
                ++cnt;
            }
        }
    }

    if (cnt <= 0)
    {
        return g_ctx.lane_width_est;
    }
    return sum / cnt;
}


struct white_run_t
{
    int left;
    int right;
    int center;
    int width;
    bool valid;
};

static white_run_t make_invalid_run()
{
    white_run_t run;
    run.left = 0;
    run.right = 0;
    run.center = 0;
    run.width = 0;
    run.valid = false;
    return run;
}

static white_run_t make_run(int left, int right)
{
    white_run_t run;
    run.left = left;
    run.right = right;
    run.center = (left + right) / 2;
    run.width = right - left + 1;
    run.valid = (right >= left);
    return run;
}

/*
 * find_best_white_run_on_row()
 * ---------------------------------------------------------------------------
 * 在指定行寻找“可信白色连通段”。
 *
 * 为什么不用原来的“从 ref_mid 左右找 0/255 跳变”：
 * - 你的二值图里赛道是大块白色区域，左右边界不一定刚好落在 ref_mid 附近；
 * - 底部有车头遮挡时，原算法容易找不到右边界，然后退化成默认补线；
 * - 这里改成先找整行白色连通段，再选最接近上一次中线的那一段，稳定性更好。
 */
static white_run_t find_best_white_run_on_row(int y, int ref_mid, int width)
{
    white_run_t best = make_invalid_run();
    int best_score = 0x7fffffff;

    y = clip_int(y, 0, CAM_HEIGHT - 1);
    ref_mid = clip_int(ref_mid, 0, width - 1);

    int x = 1;
    while (x < width - 1)
    {
        while (x < width - 1 && g_binary[y][x] == 0)
        {
            ++x;
        }

        const int run_left = x;
        while (x < width - 1 && g_binary[y][x] == 255)
        {
            ++x;
        }
        const int run_right = x - 1;

        if (run_right >= run_left)
        {
            const int run_width = run_right - run_left + 1;
            const int run_center = (run_left + run_right) / 2;

            /*
             * 过滤太窄的噪声白段。
             * 对于特别宽的十字/环岛开口，不直接丢弃，但会通过分数惩罚，避免普通状态下乱跳。
             */
            if (run_width >= TRACK_MIN_LANE_WIDTH)
            {
                const bool contains_ref = (ref_mid >= run_left && ref_mid <= run_right);
                const int center_dist = abs_int(run_center - ref_mid);
                const int width_dist = abs_int(run_width - g_ctx.lane_width_est);
                const int touch_edge_penalty =
                    (run_left <= 2 || run_right >= width - 3) ? TRACK_DEFAULT_WIDTH : 0;

                int score = center_dist * 4 + width_dist + touch_edge_penalty;
                if (contains_ref)
                {
                    score -= TRACK_DEFAULT_WIDTH;
                }

                /*
                 * 太宽的白段可能是十字或大弯，也可能是反光/背景。
                 * 不禁止，但降低优先级。
                 */
                if (run_width > TRACK_MAX_LANE_WIDTH)
                {
                    score += (run_width - TRACK_MAX_LANE_WIDTH) * 2;
                }

                if (score < best_score)
                {
                    best_score = score;
                    best = make_run(run_left, run_right);
                }
            }
        }

        ++x;
    }

    return best;
}

/*
 * find_seed_run_from_roi()
 * ---------------------------------------------------------------------------
 * 从 ROI 底部往上找第一条可靠白色连通段作为起点。
 * 这比“最长白列”更适合当前 160x120 画面，因为底部车体遮挡会破坏白列统计。
 */
static white_run_t find_seed_run_from_roi(const frame_t &frame)
{
    const int roi_top = clip_int(TRACK_ROI_Y0, 0, frame.height - 1);
    const int roi_bottom = clip_int(TRACK_ROI_Y1 - 1, roi_top, frame.height - 1);
    const int center_ref = clip_int(g_ctx.last_mid_line, 5, frame.width - 6);

    white_run_t best = make_invalid_run();
    int best_y = roi_bottom;
    int best_score = 0x7fffffff;

    /*
     * 只在 ROI 下半部分找种子，优先使用近处赛道；
     * 但不直接使用最底部，避免车头遮挡造成误判。
     */
    const int seed_top = clip_int(roi_bottom - TRACK_SEED_ROW_LOOKUP, roi_top, roi_bottom);

    for (int y = roi_bottom; y >= seed_top; --y)
    {
        white_run_t run = find_best_white_run_on_row(y, center_ref, frame.width);
        if (!run.valid)
        {
            continue;
        }

        const int center_dist = abs_int(run.center - center_ref);
        const int width_dist = abs_int(run.width - g_ctx.lane_width_est);
        const int bottom_bonus = (roi_bottom - y);
        const int score = center_dist * 5 + width_dist + bottom_bonus;

        if (score < best_score)
        {
            best_score = score;
            best = run;
            best_y = y;
        }
    }

    if (best.valid)
    {
        g_ctx.seed_column = best.center;
        g_ctx.search_top_y = roi_top;
        g_ctx.search_stop_line = roi_bottom - roi_top + 1;
        (void)best_y;
        return best;
    }

    /*
     * 如果 ROI 下半部分没有找到可信白段，用上一次中线 + 参考宽度兜底。
     */
    const int half = clip_int(g_ctx.lane_width_est / 2, TRACK_MIN_LANE_WIDTH / 2, TRACK_MAX_LANE_WIDTH / 2);
    g_ctx.seed_column = center_ref;
    g_ctx.search_top_y = roi_top;
    g_ctx.search_stop_line = roi_bottom - roi_top + 1;
    return make_run(clip_int(center_ref - half, 1, frame.width - 2),
                    clip_int(center_ref + half, 1, frame.width - 2));
}

static void find_longest_white_column(const frame_t &frame)
{
    std::memset(g_white_column, 0, sizeof(g_white_column));

    const int roi_top = clip_int(TRACK_ROI_Y0, 0, frame.height - 1);
    const int roi_bottom = clip_int(TRACK_ROI_Y1 - 1, roi_top, frame.height - 1);

    /*
     * 保留 white_column 统计，便于后续调试；
     * 但真实种子改为“ROI 近处白色连通段”，不再单纯依赖最长白列。
     */
    const int center_ref = clip_int(g_ctx.last_mid_line, 5, frame.width - 6);
    const int start_column = clip_int(center_ref - TRACK_SEED_SEARCH_HALF_WIDTH, 5, frame.width - 6);
    const int end_column = clip_int(center_ref + TRACK_SEED_SEARCH_HALF_WIDTH, 5, frame.width - 6);

    for (int x = start_column; x <= end_column; ++x)
    {
        int len = 0;
        for (int y = roi_bottom; y >= roi_top; --y)
        {
            if (g_binary[y][x] == 255)
            {
                ++len;
            }
            else
            {
                break;
            }
        }
        g_white_column[x] = static_cast<uint8_t>(clip_int(len, 0, 255));
    }

    white_run_t seed = find_seed_run_from_roi(frame);
    if (seed.valid)
    {
        g_ctx.seed_column = seed.center;
        /*
         * 种子行宽可信时，温和更新宽度估计，避免三条线挤在一起。
         */
        if (seed.width >= TRACK_MIN_LANE_WIDTH && seed.width <= TRACK_MAX_LANE_WIDTH)
        {
            g_ctx.lane_width_est = clip_int((g_ctx.lane_width_est * 7 + seed.width * 3) / 10,
                                            TRACK_MIN_LANE_WIDTH,
                                            TRACK_MAX_LANE_WIDTH);
        }
    }
}

static void search_edges_from_seed(track_result_t &track, const frame_t &frame)
{
    const int roi_top = clip_int(TRACK_ROI_Y0, 0, frame.height - 1);
    const int roi_bottom = clip_int(TRACK_ROI_Y1 - 1, roi_top, frame.height - 1);

    int ref_mid = clip_int(g_ctx.seed_column, 2, frame.width - 3);
    int ref_width = clip_int(g_ctx.lane_width_est, TRACK_MIN_LANE_WIDTH, TRACK_MAX_LANE_WIDTH);

    /*
     * 从近处向远处逐行追踪白色连通段：
     * - 找到真实白段：左右边界直接取该段左右端点；
     * - 没找到白段：用上一行中心 + 历史宽度兜底；
     * - 不再直接退化到 0 / 159，避免出现整幅图贴边线。
     */
    for (int y = roi_bottom; y >= roi_top; --y)
    {
        white_run_t run = find_best_white_run_on_row(y, ref_mid, frame.width);

        bool left_lost = false;
        bool right_lost = false;
        int left_found = 0;
        int right_found = 0;

        if (run.valid)
        {
            int run_width = run.width;

            /*
             * 如果白段过宽，说明可能在十字/环岛/大弯开口区。
             * 这时仍然保留真实边界，但宽度估计不立即跟随到过大值。
             */
            left_found = clip_int(run.left, 1, frame.width - 2);
            right_found = clip_int(run.right, left_found + 1, frame.width - 1);

            if (run_width > TRACK_MAX_LANE_WIDTH)
            {
                /*
                 * 白段过宽时，不再把显示边线截成“默认宽度”。
                 * 你现在的赛道近处白区本来就可能很宽，截断会导致右边线飘在路面中间。
                 * 这里保留真实左右端点，只是不拿这个过宽值去快速更新 lane_width_est。
                 */
                left_lost = false;
                right_lost = false;
            }
            else if (run_width < TRACK_MIN_LANE_WIDTH)
            {
                const int half = clip_int(ref_width / 2, TRACK_MIN_LANE_WIDTH / 2, TRACK_MAX_LANE_WIDTH / 2);
                left_found = clip_int(ref_mid - half, 1, frame.width - 2);
                right_found = clip_int(ref_mid + half, left_found + 1, frame.width - 1);
                left_lost = true;
                right_lost = true;
            }
            else
            {
                const int new_width = right_found - left_found;
                ref_width = clip_int((ref_width * 7 + new_width * 3) / 10,
                                     TRACK_MIN_LANE_WIDTH,
                                     TRACK_MAX_LANE_WIDTH);
                g_ctx.lane_width_est = clip_int((g_ctx.lane_width_est * 8 + new_width * 2) / 10,
                                                TRACK_MIN_LANE_WIDTH,
                                                TRACK_MAX_LANE_WIDTH);
            }
        }
        else
        {
            const int half = clip_int(ref_width / 2, TRACK_MIN_LANE_WIDTH / 2, TRACK_MAX_LANE_WIDTH / 2);
            left_found = clip_int(ref_mid - half, 1, frame.width - 2);
            right_found = clip_int(ref_mid + half, left_found + 1, frame.width - 1);
            left_lost = true;
            right_lost = true;
        }

        const int width_found = right_found - left_found;
        if (width_found < TRACK_MIN_LANE_WIDTH || width_found > TRACK_MAX_LANE_WIDTH || left_found >= right_found)
        {
            const int half = clip_int(ref_width / 2, TRACK_MIN_LANE_WIDTH / 2, TRACK_MAX_LANE_WIDTH / 2);
            left_found = clip_int(ref_mid - half, 1, frame.width - 2);
            right_found = clip_int(ref_mid + half, left_found + 1, frame.width - 1);
            left_lost = true;
            right_lost = true;
        }

        track.full_left[y] = left_found;
        track.full_right[y] = right_found;
        track.full_center[y] = (left_found + right_found) / 2;
        track.full_left_lost[y] = left_lost ? 1 : 0;
        track.full_right_lost[y] = right_lost ? 1 : 0;

        /*
         * 中心点平滑追踪：真实白段中心权重大，兜底补线中心权重小。
         */
        const int detected_mid = track.full_center[y];
        if (!left_lost || !right_lost)
        {
            ref_mid = clip_int((ref_mid * 3 + detected_mid * 7) / 10, 1, frame.width - 2);
        }
        else
        {
            ref_mid = clip_int((ref_mid * 8 + detected_mid * 2) / 10, 1, frame.width - 2);
        }
    }

    /*
     * ROI 以外的行只用于显示，不参与控制。
     */
    for (int y = roi_top - 1; y >= 0; --y)
    {
        track.full_left[y] = track.full_left[roi_top];
        track.full_right[y] = track.full_right[roi_top];
        track.full_center[y] = track.full_center[roi_top];
        track.full_left_lost[y] = 1;
        track.full_right_lost[y] = 1;
    }

    for (int y = roi_bottom + 1; y < frame.height; ++y)
    {
        track.full_left[y] = track.full_left[roi_bottom];
        track.full_right[y] = track.full_right[roi_bottom];
        track.full_center[y] = track.full_center[roi_bottom];
        track.full_left_lost[y] = 1;
        track.full_right_lost[y] = 1;
    }
}

static void collect_statistics(track_result_t &track)
{
    const int y_mid_start = CAM_HEIGHT / 3;
    const int y_mid_end = CAM_HEIGHT * 3 / 4;

    for (int y = TRACK_ROI_Y0; y < TRACK_ROI_Y1 && y < CAM_HEIGHT; ++y)
    {
        if (track.full_left_lost[y]) ++g_ctx.left_lost_total;
        if (track.full_right_lost[y]) ++g_ctx.right_lost_total;
        if (track.full_left_lost[y] && track.full_right_lost[y]) ++g_ctx.both_lost_total;

        if (y >= y_mid_start && y <= y_mid_end)
        {
            if (track.full_left_lost[y]) ++g_ctx.left_lost_mid;
            if (track.full_right_lost[y]) ++g_ctx.right_lost_mid;
            if (track.full_left_lost[y] && track.full_right_lost[y]) ++g_ctx.both_lost_mid;
        }
    }

    const int roi_top = clip_int(TRACK_ROI_Y0, 0, CAM_HEIGHT - 1);
    const int roi_bottom = clip_int(TRACK_ROI_Y1 - 1, roi_top, CAM_HEIGHT - 1);
    g_ctx.road_width_base = average_width(track, roi_bottom - 18, roi_bottom - 2);
    g_ctx.road_width_far = average_width(track, roi_top, (roi_top + roi_bottom) / 2 + 8);
}

static void locate_corners_by_boundary_shape(const track_result_t &track)
{
    const int mid_top = clip_int(TRACK_ROI_Y0, 0, CAM_HEIGHT - 1);
    const int mid_bottom = clip_int(TRACK_ROI_Y1 - 1, mid_top, CAM_HEIGHT - 1);

    for (int y = mid_bottom - 1; y >= mid_top + 2; --y)
    {
        const int dl = track.full_left[y] - track.full_left[y - 2];
        const int dr = track.full_right[y] - track.full_right[y - 2];
        const int w_now = track.full_right[y] - track.full_left[y];
        const int w_pre = track.full_right[y - 2] - track.full_left[y - 2];

        if (!g_ctx.left_down.valid && y > CAM_HEIGHT / 2)
        {
            if (!track.full_left_lost[y] && !track.full_left_lost[y - 2] && abs_int(dl) >= TRACK_CORNER_SLOPE_MIN)
            {
                g_ctx.left_down = {track.full_left[y], y, true};
            }
        }

        if (!g_ctx.right_down.valid && y > CAM_HEIGHT / 2)
        {
            if (!track.full_right_lost[y] && !track.full_right_lost[y - 2] && abs_int(dr) >= TRACK_CORNER_SLOPE_MIN)
            {
                g_ctx.right_down = {track.full_right[y], y, true};
            }
        }

        if (!g_ctx.left_up.valid && y < mid_bottom)
        {
            if (!track.full_left_lost[y] && (w_now - w_pre) >= TRACK_CORNER_WIDTH_JUMP_MIN)
            {
                g_ctx.left_up = {track.full_left[y], y, true};
            }
        }

        if (!g_ctx.right_up.valid && y < mid_bottom)
        {
            if (!track.full_right_lost[y] && (w_now - w_pre) >= TRACK_CORNER_WIDTH_JUMP_MIN)
            {
                g_ctx.right_up = {track.full_right[y], y, true};
            }
        }
    }
}

static bool judge_straight(void)
{
    return g_ctx.road_width_far <= (g_ctx.road_width_base + 10) &&
           g_ctx.left_lost_mid <= 3 &&
           g_ctx.right_lost_mid <= 3;
}

static bool judge_cross(void)
{
    return g_ctx.both_lost_mid >= TRACK_CROSS_BOTH_LOST_MID_MIN &&
           g_ctx.road_width_far > (g_ctx.road_width_base + TRACK_CROSS_WIDTH_DELTA_MIN) &&
           g_ctx.left_up.valid &&
           g_ctx.right_up.valid;
}

static uint8_t judge_ring(void)
{
    if (g_ctx.left_lost_mid >= TRACK_RING_SIDE_LOST_MID_MIN &&
        g_ctx.right_lost_mid <= TRACK_RING_OTHER_SIDE_LOST_MID_MAX &&
        g_ctx.right_down.valid &&
        g_ctx.both_lost_mid <= TRACK_RING_BOTH_LOST_MID_MAX)
    {
        return 1;
    }

    if (g_ctx.right_lost_mid >= TRACK_RING_SIDE_LOST_MID_MIN &&
        g_ctx.left_lost_mid <= TRACK_RING_OTHER_SIDE_LOST_MID_MAX &&
        g_ctx.left_down.valid &&
        g_ctx.both_lost_mid <= TRACK_RING_BOTH_LOST_MID_MAX)
    {
        return 2;
    }

    return 0;
}

static void apply_cross_repair(track_result_t &track)
{
    int width_ref = clip_int(g_ctx.road_width_base, TRACK_MIN_LANE_WIDTH, TRACK_MAX_LANE_WIDTH);
    int valid_mid = g_ctx.last_mid_line;

    for (int y = clip_int(TRACK_ROI_Y1 - 1, 0, CAM_HEIGHT - 1); y >= TRACK_ROI_Y0; --y)
    {
        if (!track.full_left_lost[y] && !track.full_right_lost[y])
        {
            valid_mid = (track.full_left[y] + track.full_right[y]) / 2;
            width_ref = clip_int((width_ref * 7 + (track.full_right[y] - track.full_left[y]) * 3) / 10,
                                 TRACK_MIN_LANE_WIDTH,
                                 TRACK_MAX_LANE_WIDTH);
        }
        else if (track.full_left_lost[y] && track.full_right_lost[y])
        {
            track.full_left[y] = clip_int(valid_mid - width_ref / 2, 0, CAM_WIDTH - 2);
            track.full_right[y] = clip_int(valid_mid + width_ref / 2, track.full_left[y] + 1, CAM_WIDTH - 1);
        }
        else if (track.full_left_lost[y])
        {
            track.full_left[y] = clip_int(track.full_right[y] - width_ref, 0, CAM_WIDTH - 2);
        }
        else if (track.full_right_lost[y])
        {
            track.full_right[y] = clip_int(track.full_left[y] + width_ref, track.full_left[y] + 1, CAM_WIDTH - 1);
        }
        track.full_center[y] = (track.full_left[y] + track.full_right[y]) / 2;
    }
}

static void extend_right_line(track_result_t &track, int start_y, int end_y)
{
    start_y = clip_int(start_y, 0, CAM_HEIGHT - 1);
    end_y = clip_int(end_y, 0, CAM_HEIGHT - 1);
    if (start_y > end_y)
    {
        const int t = start_y;
        start_y = end_y;
        end_y = t;
    }

    const int ref0 = clip_int(end_y, 1, CAM_HEIGHT - 2);
    const int ref1 = clip_int(end_y - 4, 0, CAM_HEIGHT - 2);
    const int x0 = track.full_right[ref0];
    const int x1 = track.full_right[ref1];
    int dy = ref0 - ref1;
    if (dy == 0) dy = 1;
    const int k_num = x0 - x1;

    for (int y = start_y; y <= end_y; ++y)
    {
        track.full_right[y] = clip_int(x0 + (y - ref0) * k_num / dy, 1, CAM_WIDTH - 1);
        track.full_right_lost[y] = 0;
    }
}

static void extend_left_line(track_result_t &track, int start_y, int end_y)
{
    start_y = clip_int(start_y, 0, CAM_HEIGHT - 1);
    end_y = clip_int(end_y, 0, CAM_HEIGHT - 1);
    if (start_y > end_y)
    {
        const int t = start_y;
        start_y = end_y;
        end_y = t;
    }

    const int ref0 = clip_int(end_y, 1, CAM_HEIGHT - 2);
    const int ref1 = clip_int(end_y - 4, 0, CAM_HEIGHT - 2);
    const int x0 = track.full_left[ref0];
    const int x1 = track.full_left[ref1];
    int dy = ref0 - ref1;
    if (dy == 0) dy = 1;
    const int k_num = x0 - x1;

    for (int y = start_y; y <= end_y; ++y)
    {
        track.full_left[y] = clip_int(x0 + (y - ref0) * k_num / dy, 0, CAM_WIDTH - 2);
        track.full_left_lost[y] = 0;
    }
}

static void apply_ring_left_repair(track_result_t &track)
{
    const int width_ref = clip_int(g_ctx.road_width_base, TRACK_MIN_LANE_WIDTH, TRACK_MAX_LANE_WIDTH);
    const int top = g_ctx.right_down.valid ? g_ctx.right_down.y : (CAM_HEIGHT / 2);
    extend_right_line(track, TRACK_ROI_Y0, top);

    for (int y = clip_int(TRACK_ROI_Y1 - 1, 0, CAM_HEIGHT - 1); y >= TRACK_ROI_Y0; --y)
    {
        if (track.full_left_lost[y])
        {
            int x = track.full_right[y] - width_ref;
            if (y > 2 && !track.full_left_lost[y - 1] && !track.full_left_lost[y - 2])
            {
                x += track.full_left[y - 1] - track.full_left[y - 2];
            }
            track.full_left[y] = clip_int(x, 0, CAM_WIDTH - 2);
        }
        track.full_center[y] = (track.full_left[y] + track.full_right[y]) / 2;
    }
}

static void apply_ring_right_repair(track_result_t &track)
{
    const int width_ref = clip_int(g_ctx.road_width_base, TRACK_MIN_LANE_WIDTH, TRACK_MAX_LANE_WIDTH);
    const int top = g_ctx.left_down.valid ? g_ctx.left_down.y : (CAM_HEIGHT / 2);
    extend_left_line(track, TRACK_ROI_Y0, top);

    for (int y = clip_int(TRACK_ROI_Y1 - 1, 0, CAM_HEIGHT - 1); y >= TRACK_ROI_Y0; --y)
    {
        if (track.full_right_lost[y])
        {
            int x = track.full_left[y] + width_ref;
            if (y > 2 && !track.full_right_lost[y - 1] && !track.full_right_lost[y - 2])
            {
                x += track.full_right[y - 1] - track.full_right[y - 2];
            }
            track.full_right[y] = clip_int(x, 1, CAM_WIDTH - 1);
        }
        track.full_center[y] = (track.full_left[y] + track.full_right[y]) / 2;
    }
}

static void leave_element_state(void)
{
    g_ctx.state = TRACK_STATE_NORMAL;
    g_ctx.state_hold = 0;
    g_ctx.state_elapsed_frames = 0;
    g_ctx.reenter_cooldown = TRACK_STATE_REENTER_COOLDOWN_FRAMES;
}

static void update_state_machine(track_result_t &track, bool cross_flag, uint8_t ring_flag, bool straight_flag)
{
    g_ctx.timeout_exit = false;

    if (cross_flag)
    {
        if (g_ctx.cross_counter < TRACK_COUNTER_MAX) ++g_ctx.cross_counter;
    }
    else if (g_ctx.cross_counter > 0)
    {
        --g_ctx.cross_counter;
    }

    if (ring_flag == 1)
    {
        if (g_ctx.ring_left_counter < TRACK_COUNTER_MAX) ++g_ctx.ring_left_counter;
    }
    else if (g_ctx.ring_left_counter > 0)
    {
        --g_ctx.ring_left_counter;
    }

    if (ring_flag == 2)
    {
        if (g_ctx.ring_right_counter < TRACK_COUNTER_MAX) ++g_ctx.ring_right_counter;
    }
    else if (g_ctx.ring_right_counter > 0)
    {
        --g_ctx.ring_right_counter;
    }

    if (g_ctx.reenter_cooldown > 0)
    {
        --g_ctx.reenter_cooldown;
    }

    if (g_ctx.state != TRACK_STATE_NORMAL)
    {
        ++g_ctx.state_elapsed_frames;
    }

    switch (g_ctx.state)
    {
    case TRACK_STATE_NORMAL:
        g_ctx.state_elapsed_frames = 0;
        if (g_ctx.reenter_cooldown == 0 && g_ctx.cross_counter >= TRACK_CROSS_ENTER_COUNT)
        {
            g_ctx.state = TRACK_STATE_CROSS;
            g_ctx.state_hold = TRACK_CROSS_HOLD_FRAMES;
            g_ctx.state_elapsed_frames = 1;
        }
        else if (g_ctx.reenter_cooldown == 0 && g_ctx.ring_left_counter >= TRACK_RING_ENTER_COUNT)
        {
            g_ctx.state = TRACK_STATE_RING_LEFT;
            g_ctx.state_hold = TRACK_RING_HOLD_FRAMES;
            g_ctx.state_elapsed_frames = 1;
        }
        else if (g_ctx.reenter_cooldown == 0 && g_ctx.ring_right_counter >= TRACK_RING_ENTER_COUNT)
        {
            g_ctx.state = TRACK_STATE_RING_RIGHT;
            g_ctx.state_hold = TRACK_RING_HOLD_FRAMES;
            g_ctx.state_elapsed_frames = 1;
        }
        break;

    case TRACK_STATE_CROSS:
        if (g_ctx.state_hold > 0) --g_ctx.state_hold;
        if (g_ctx.state_elapsed_frames >= TRACK_CROSS_TIMEOUT_FRAMES)
        {
            g_ctx.timeout_exit = true;
            leave_element_state();
        }
        else if (g_ctx.state_hold == 0 && g_ctx.cross_counter == 0)
        {
            leave_element_state();
        }
        break;

    case TRACK_STATE_RING_LEFT:
        if (g_ctx.state_hold > 0) --g_ctx.state_hold;
        if (g_ctx.state_elapsed_frames >= TRACK_RING_TIMEOUT_FRAMES)
        {
            g_ctx.timeout_exit = true;
            leave_element_state();
        }
        else if (g_ctx.state_hold == 0 && g_ctx.ring_left_counter == 0)
        {
            leave_element_state();
        }
        break;

    case TRACK_STATE_RING_RIGHT:
        if (g_ctx.state_hold > 0) --g_ctx.state_hold;
        if (g_ctx.state_elapsed_frames >= TRACK_RING_TIMEOUT_FRAMES)
        {
            g_ctx.timeout_exit = true;
            leave_element_state();
        }
        else if (g_ctx.state_hold == 0 && g_ctx.ring_right_counter == 0)
        {
            leave_element_state();
        }
        break;

    default:
        leave_element_state();
        break;
    }

    switch (g_ctx.state)
    {
    case TRACK_STATE_CROSS:
        g_ctx.element = TRACK_ELEMENT_CROSS;
        apply_cross_repair(track);
        break;
    case TRACK_STATE_RING_LEFT:
        g_ctx.element = TRACK_ELEMENT_RING_LEFT;
        apply_ring_left_repair(track);
        break;
    case TRACK_STATE_RING_RIGHT:
        g_ctx.element = TRACK_ELEMENT_RING_RIGHT;
        apply_ring_right_repair(track);
        break;
    default:
        g_ctx.element = straight_flag ? TRACK_ELEMENT_STRAIGHT : TRACK_ELEMENT_NONE;
        break;
    }
}

static int dynamic_mid_weight(int y)
{
    return 1 + (y * 17) / ((CAM_HEIGHT > 1) ? (CAM_HEIGHT - 1) : 1);
}

static int compute_weighted_mid(track_result_t &track)
{
    uint32_t weighted_sum = 0;
    uint32_t weight_sum = 0;

    const int y_start = clip_int(TRACK_SEARCH_FINISH_LINE, TRACK_ROI_Y0, CAM_HEIGHT - 2);
    const int y_end = clip_int(TRACK_ROI_Y1 - 1, y_start + 1, CAM_HEIGHT - 1);
    for (int y = y_end; y > y_start; --y)
    {
        const int weight = dynamic_mid_weight(y);
        weighted_sum += static_cast<uint32_t>(track.full_center[y] * weight);
        weight_sum += static_cast<uint32_t>(weight);
    }

    if (weight_sum == 0)
    {
        return g_ctx.last_mid_line;
    }

    const int raw_mid = static_cast<int>(weighted_sum / weight_sum);
    const int smooth_mid = (g_ctx.last_mid_line * (TRACK_SMOOTH_ALPHA_DEN - TRACK_SMOOTH_ALPHA_NUM) +
                            raw_mid * TRACK_SMOOTH_ALPHA_NUM) /
                           TRACK_SMOOTH_ALPHA_DEN;
    g_ctx.last_mid_line = clip_int(smooth_mid, 0, CAM_WIDTH - 1);
    return g_ctx.last_mid_line;
}

static void fill_sample_lines(track_result_t &track)
{
    const int roi_height = TRACK_ROI_Y1 - TRACK_ROI_Y0;
    int scan_step = (roi_height > 0) ? (roi_height / TRACK_SCAN_LINES) : 1;
    if (scan_step <= 0) scan_step = 1;

    for (int i = 0; i < TRACK_SCAN_LINES; ++i)
    {
        int y = TRACK_ROI_Y1 - 1 - i * scan_step;
        y = clip_int(y, 0, CAM_HEIGHT - 1);
        track.left_edge[i] = track.full_left[y];
        track.right_edge[i] = track.full_right[y];
        track.center[i] = track.full_center[y];
    }
}

static void finalize_result(track_result_t &track, const frame_t &frame)
{
    fill_sample_lines(track);

    const int weighted_mid = compute_weighted_mid(track);
    track.center_avg = static_cast<float>(weighted_mid);
    track.error = static_cast<float>(TRACK_CENTER_X) - track.center_avg;
    g_error_filtered = g_error_filtered * (1.0f - TRACK_ERROR_FILTER_ALPHA) + track.error * TRACK_ERROR_FILTER_ALPHA;
    track.error_filtered = g_error_filtered;

    int real_valid_rows = 0;
    int roi_rows = 0;
    const int y0 = clip_int(TRACK_ROI_Y0, 0, CAM_HEIGHT - 1);
    const int y1 = clip_int(TRACK_ROI_Y1, y0 + 1, CAM_HEIGHT);
    for (int y = y0; y < y1; ++y)
    {
        ++roi_rows;
        if (!(track.full_left_lost[y] && track.full_right_lost[y]))
        {
            ++real_valid_rows;
        }
    }

    const float valid_ratio = (roi_rows > 0) ? static_cast<float>(real_valid_rows) / static_cast<float>(roi_rows) : 0.0f;
    g_quality_hint = g_quality_hint * 0.80f + valid_ratio * 0.20f;
    track.quality_hint = g_quality_hint;

    const int near_y = clip_int(TRACK_ROI_Y1 - 8, TRACK_ROI_Y0, CAM_HEIGHT - 1);
    const int far_y = clip_int((TRACK_ROI_Y0 + TRACK_ROI_Y1) / 2, 0, CAM_HEIGHT - 1);
    const float curvature_raw = static_cast<float>(track.full_center[far_y] - track.full_center[near_y]);
    g_curvature_hint = g_curvature_hint * (1.0f - TRACK_CURVATURE_FILTER_ALPHA) +
                       curvature_raw * TRACK_CURVATURE_FILTER_ALPHA;
    track.curvature_hint = g_curvature_hint;

    track.valid = real_valid_rows >= 8 && valid_ratio >= (TRACK_VALID_RATIO_MIN * 0.55f);
    track.lost = !track.valid;
    track.timestamp_ms = frame.timestamp_ms;

    track.element = g_ctx.element;
    track.state = g_ctx.state;
    track.state_hold = g_ctx.state_hold;
    track.cross_flag = (g_ctx.cross_counter > 0);
    track.ring_flag = (g_ctx.ring_left_counter > 0) ? 1 : ((g_ctx.ring_right_counter > 0) ? 2 : 0);
    track.lane_width_base = g_ctx.road_width_base;
    track.lane_width_far = g_ctx.road_width_far;
    track.left_lost_total = g_ctx.left_lost_total;
    track.right_lost_total = g_ctx.right_lost_total;
    track.both_lost_total = g_ctx.both_lost_total;
    track.left_lost_mid = g_ctx.left_lost_mid;
    track.right_lost_mid = g_ctx.right_lost_mid;
    track.both_lost_mid = g_ctx.both_lost_mid;
    track.left_up = g_ctx.left_up;
    track.right_up = g_ctx.right_up;
    track.left_down = g_ctx.left_down;
    track.right_down = g_ctx.right_down;
    track.seed_column = g_ctx.seed_column;
    track.search_top_y = g_ctx.search_top_y;
    track.state_elapsed_frames = g_ctx.state_elapsed_frames;
    track.timeout_exit = g_ctx.timeout_exit;
}
}

void line_track_init(track_result_t &track_result)
{
    std::memset(&track_result, 0, sizeof(track_result));
    std::memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.state = TRACK_STATE_NORMAL;
    g_ctx.element = TRACK_ELEMENT_NONE;
    g_ctx.lane_width_est = TRACK_DEFAULT_WIDTH;
    g_ctx.road_width_base = TRACK_DEFAULT_WIDTH;
    g_ctx.road_width_far = TRACK_DEFAULT_WIDTH;
    g_ctx.last_mid_line = TRACK_CENTER_X;
    g_ctx.seed_column = TRACK_CENTER_X;
    g_ctx.search_top_y = TRACK_TOP_LINE;
    g_error_filtered = 0.0f;
    g_curvature_hint = 0.0f;
    g_quality_hint = 0.0f;
    reset_frame_result(track_result);
}

void line_track_process(const frame_t &frame, track_result_t &track_result)
{
    if (!frame.valid || frame.gray == nullptr || frame.width != CAM_WIDTH || frame.height != CAM_HEIGHT)
    {
        track_result.valid = false;
        track_result.lost = true;
        return;
    }

    reset_frame_result(track_result);
    binary_preprocess(frame);
    find_longest_white_column(frame);
    search_edges_from_seed(track_result, frame);
    collect_statistics(track_result);
    locate_corners_by_boundary_shape(track_result);

    const bool straight_flag = judge_straight();
    const bool cross_flag = judge_cross();
    const uint8_t ring_flag = judge_ring();

    update_state_machine(track_result, cross_flag, ring_flag, straight_flag);
    for (int y = 0; y < CAM_HEIGHT; ++y)
    {
        track_result.full_center[y] = (track_result.full_left[y] + track_result.full_right[y]) / 2;
    }
    finalize_result(track_result, frame);
}

float line_track_compute_speed_scale(const track_result_t &track_result)
{
    const float abs_error = abs_float(track_result.error_filtered);
    const float abs_curvature = abs_float(track_result.curvature_hint);

    float error_scale = 1.0f;
    if (abs_error >= TRACK_BRAKE_ERROR)
    {
        error_scale = 0.28f;
    }
    else if (abs_error >= TRACK_SLOWDOWN_ERROR)
    {
        const float t = (abs_error - TRACK_SLOWDOWN_ERROR) /
                        (TRACK_BRAKE_ERROR - TRACK_SLOWDOWN_ERROR + 1e-6f);
        error_scale = 0.62f - 0.34f * clampf(t, 0.0f, 1.0f);
    }

    const float curvature_penalty = clampf(abs_curvature / TRACK_SPEED_CURVATURE_SLOWDOWN, 0.0f, 1.0f) * 0.28f;
    const float quality_scale = TRACK_SPEED_QUALITY_FLOOR +
                                (1.0f - TRACK_SPEED_QUALITY_FLOOR) * clampf(track_result.quality_hint, 0.0f, 1.0f);

    float element_scale = 1.0f;
    if (track_result.state == TRACK_STATE_CROSS)
    {
        element_scale = TRACK_CROSS_SPEED_SCALE;
    }
    else if (track_result.state == TRACK_STATE_RING_LEFT || track_result.state == TRACK_STATE_RING_RIGHT)
    {
        element_scale = TRACK_RING_SPEED_SCALE;
    }

    float final_scale = error_scale * (1.0f - curvature_penalty) * quality_scale * element_scale;
    if (!track_result.valid || track_result.lost)
    {
        final_scale = 0.20f;
    }
    return clampf(final_scale, 0.20f, 1.00f);
}

const char *line_track_element_name(uint8_t element)
{
    switch (element)
    {
    case TRACK_ELEMENT_STRAIGHT:   return "STRAIGHT";
    case TRACK_ELEMENT_CROSS:      return "CROSS";
    case TRACK_ELEMENT_RING_LEFT:  return "RING_L";
    case TRACK_ELEMENT_RING_RIGHT: return "RING_R";
    default:                       return "NONE";
    }
}

const char *line_track_state_name(uint8_t state)
{
    switch (state)
    {
    case TRACK_STATE_CROSS:      return "CROSS";
    case TRACK_STATE_RING_LEFT:  return "RING_L";
    case TRACK_STATE_RING_RIGHT: return "RING_R";
    default:                     return "NORMAL";
    }
}

const uint8_t *line_track_get_binary_image()
{
    return &g_binary[0][0];
}
