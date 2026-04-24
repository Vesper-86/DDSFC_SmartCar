#include "line_track.hpp"

#include <array>
#include <cmath>
#include <cstring>

namespace
{
struct track_internal_context_t
{
    uint8_t element;
    uint8_t state;
    uint8_t state_hold;
    uint16_t state_elapsed_frames;
    uint8_t state_cooldown;
    bool timeout_exit;
    uint8_t cross_counter;
    uint8_t ring_left_counter;
    uint8_t ring_right_counter;

    int seed_column;
    int search_stop_line;
    int road_width_base;
    int road_width_far;

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

static uint8_t g_binary_image[CAM_HEIGHT][CAM_WIDTH];
static int16_t g_left_line[CAM_HEIGHT];
static int16_t g_right_line[CAM_HEIGHT];
static int16_t g_mid_line[CAM_HEIGHT];
static uint8_t g_left_lost[CAM_HEIGHT];
static uint8_t g_right_lost[CAM_HEIGHT];
static int16_t g_road_width[CAM_HEIGHT];
static uint8_t g_white_column[CAM_WIDTH];

static track_internal_context_t g_ctx = {};

static uint8_t g_cross_flag = 0;
static uint8_t g_ring_flag = 0;
static int g_final_mid_line = TRACK_CENTER_X;
static int g_last_mid_line = TRACK_CENTER_X;

static float g_lane_width_est = static_cast<float>(TRACK_DEFAULT_WIDTH);
static float g_curvature_hint = 0.0f;
static float g_quality_hint = 0.0f;
static float g_center_hint = static_cast<float>(TRACK_CENTER_X);

inline float absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

inline int clamp_int(int x, int lo, int hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static void clear_point(track_point_t &p)
{
    p.x = 0;
    p.y = 0;
    p.valid = false;
}

static void reset_internal_buffers(void)
{
    std::memset(g_left_lost, 0, sizeof(g_left_lost));
    std::memset(g_right_lost, 0, sizeof(g_right_lost));
    std::memset(g_road_width, 0, sizeof(g_road_width));
    std::memset(g_white_column, 0, sizeof(g_white_column));

    for (int y = 0; y < CAM_HEIGHT; ++y)
    {
        g_left_line[y] = 0;
        g_right_line[y] = CAM_WIDTH - 1;
        g_mid_line[y] = CAM_WIDTH / 2;
    }

    g_cross_flag = 0;
    g_ring_flag = 0;

    g_ctx.seed_column = TRACK_CENTER_X;
    g_ctx.search_stop_line = CAM_HEIGHT / 2;
    g_ctx.road_width_base = TRACK_DEFAULT_WIDTH;
    g_ctx.road_width_far = TRACK_DEFAULT_WIDTH;
    g_ctx.left_lost_total = 0;
    g_ctx.right_lost_total = 0;
    g_ctx.both_lost_total = 0;
    g_ctx.left_lost_mid = 0;
    g_ctx.right_lost_mid = 0;
    g_ctx.both_lost_mid = 0;
    clear_point(g_ctx.left_up);
    clear_point(g_ctx.right_up);
    clear_point(g_ctx.left_down);
    clear_point(g_ctx.right_down);
    g_ctx.timeout_exit = false;
}

static int compute_roi_otsu_threshold(const frame_t &frame)
{
    if (frame.gray == nullptr || frame.width <= 0 || frame.height <= 0) {
        return TRACK_BIN_THRESHOLD;
    }

    std::array<int, 256> hist{};
    const int y0 = clamp_int(TRACK_ROI_Y0, 0, frame.height - 1);
    const int y1 = clamp_int(TRACK_ROI_Y1, y0 + 1, frame.height);
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
        const double diff = bg_mean - fg_mean;
        const double between_var =
            static_cast<double>(bg_count) * static_cast<double>(fg_count) * diff * diff;

        if (between_var > best_var) {
            best_var = between_var;
            best_th = th;
        }
    }

    return clamp_int(best_th, TRACK_BINARY_MIN, TRACK_BINARY_MAX);
}

static void build_binary_image(const frame_t &frame, int threshold)
{
    for (int y = 0; y < CAM_HEIGHT; ++y)
    {
        const uint8_t *src = &frame.gray[y * frame.width];
        for (int x = 0; x < CAM_WIDTH; ++x)
        {
            /* 当前车型场景下，赛道主体通常比背景更暗，因此这里做“暗变白”。 */
            g_binary_image[y][x] = (src[x] <= threshold) ? 255 : 0;
        }
    }
}

static int average_width(int y_start, int y_end)
{
    int sum = 0;
    int count = 0;

    y_start = clamp_int(y_start, 0, CAM_HEIGHT - 1);
    y_end = clamp_int(y_end, 0, CAM_HEIGHT - 1);
    if (y_start > y_end)
    {
        const int tmp = y_start;
        y_start = y_end;
        y_end = tmp;
    }

    for (int y = y_start; y <= y_end; ++y)
    {
        if (!g_left_lost[y] && !g_right_lost[y])
        {
            const int width = static_cast<int>(g_right_line[y]) - static_cast<int>(g_left_line[y]);
            if (width > 10 && width < CAM_WIDTH)
            {
                sum += width;
                ++count;
            }
        }
    }

    if (count == 0)
    {
        return TRACK_DEFAULT_WIDTH;
    }
    return sum / count;
}

static void search_edges_from_seed(void)
{
    int ref_mid = clamp_int(g_ctx.seed_column, 2, CAM_WIDTH - 3);
    int ref_width = (g_ctx.road_width_base > 20) ? g_ctx.road_width_base : TRACK_DEFAULT_WIDTH;
    int top_line = CAM_HEIGHT - g_ctx.search_stop_line;
    top_line = clamp_int(top_line, TRACK_TOP_LINE, CAM_HEIGHT - 1);

    for (int y = CAM_HEIGHT - 1; y >= top_line; --y)
    {
        int half_search = ref_width / 2 + 8;
        if (half_search < TRACK_MIN_SEARCH_HALF) half_search = TRACK_MIN_SEARCH_HALF;
        if (half_search > (CAM_WIDTH / 2 - 2)) half_search = CAM_WIDTH / 2 - 2;

        const int left_start = clamp_int(ref_mid, 2, CAM_WIDTH - 3);
        const int left_limit = clamp_int(ref_mid - half_search, 2, CAM_WIDTH - 3);
        const int right_start = clamp_int(ref_mid, 2, CAM_WIDTH - 3);
        const int right_limit = clamp_int(ref_mid + half_search, 2, CAM_WIDTH - 3);

        int left_found = -1;
        int right_found = -1;

        for (int x = left_start; x >= left_limit; --x)
        {
            if (g_binary_image[y][x - 1] == 0 && g_binary_image[y][x] == 255)
            {
                left_found = x;
                break;
            }
        }

        for (int x = right_start; x <= right_limit; ++x)
        {
            if (g_binary_image[y][x] == 255 && g_binary_image[y][x + 1] == 0)
            {
                right_found = x;
                break;
            }
        }

        if (left_found < 0)
        {
            g_left_lost[y] = 1;
            left_found = clamp_int(ref_mid - ref_width / 2, 1, CAM_WIDTH - 2);
        }
        if (right_found < 0)
        {
            g_right_lost[y] = 1;
            right_found = clamp_int(ref_mid + ref_width / 2, 1, CAM_WIDTH - 2);
        }

        if (left_found >= right_found)
        {
            const int half_w = ref_width / 2;
            left_found = clamp_int(ref_mid - half_w, 1, CAM_WIDTH - 3);
            right_found = clamp_int(ref_mid + half_w, left_found + 2, CAM_WIDTH - 2);
            g_left_lost[y] = 1;
            g_right_lost[y] = 1;
        }

        g_left_line[y] = left_found;
        g_right_line[y] = right_found;
        g_road_width[y] = clamp_int(right_found - left_found, 0, CAM_WIDTH - 1);

        ref_mid = (left_found + right_found) >> 1;
        ref_width = clamp_int((ref_width * 7 + (right_found - left_found) * 3) / 10,
                              TRACK_MIN_LANE_WIDTH,
                              CAM_WIDTH - 8);
    }
}

static void longest_white_column(void)
{
    int best_len = 0;
    int best_col = CAM_WIDTH / 2;

    for (int x = 10; x <= CAM_WIDTH - 11; ++x)
    {
        int len = 0;
        for (int y = CAM_HEIGHT - 1; y >= 0; --y)
        {
            if (g_binary_image[y][x] == 255) {
                ++len;
            } else {
                break;
            }
        }

        g_white_column[x] = static_cast<uint8_t>(clamp_int(len, 0, 255));
        if (len > best_len)
        {
            best_len = len;
            best_col = x;
        }
    }

    if (best_len < 20)
    {
        best_len = CAM_HEIGHT / 2;
        best_col = g_last_mid_line;
    }

    g_ctx.seed_column = best_col;
    g_ctx.search_stop_line = clamp_int(best_len + 8, 40, CAM_HEIGHT - TRACK_TOP_LINE);
    g_ctx.road_width_base = average_width(CAM_HEIGHT - 15, CAM_HEIGHT - 1);
    if (g_ctx.road_width_base < TRACK_MIN_LANE_WIDTH)
    {
        g_ctx.road_width_base = TRACK_DEFAULT_WIDTH;
    }

    search_edges_from_seed();
}

static void locate_corners_by_boundary_shape(void)
{
    const int mid_top = CAM_HEIGHT / 3;
    const int mid_bottom = CAM_HEIGHT - 8;

    for (int y = CAM_HEIGHT - 2; y >= mid_top + 2; --y)
    {
        const int dl = static_cast<int>(g_left_line[y]) - static_cast<int>(g_left_line[y - 2]);
        const int dr = static_cast<int>(g_right_line[y]) - static_cast<int>(g_right_line[y - 2]);
        const int width_now = static_cast<int>(g_road_width[y]);
        const int width_pre = static_cast<int>(g_road_width[y - 2]);

        if (!g_ctx.left_down.valid && y > CAM_HEIGHT / 2)
        {
            if (!g_left_lost[y] && !g_left_lost[y - 2] && std::abs(dl) >= TRACK_CORNER_SLOPE_MIN)
            {
                g_ctx.left_down.x = g_left_line[y];
                g_ctx.left_down.y = y;
                g_ctx.left_down.valid = true;
            }
        }

        if (!g_ctx.right_down.valid && y > CAM_HEIGHT / 2)
        {
            if (!g_right_lost[y] && !g_right_lost[y - 2] && std::abs(dr) >= TRACK_CORNER_SLOPE_MIN)
            {
                g_ctx.right_down.x = g_right_line[y];
                g_ctx.right_down.y = y;
                g_ctx.right_down.valid = true;
            }
        }

        if (!g_ctx.left_up.valid && y < mid_bottom)
        {
            if (!g_left_lost[y] && (width_now - width_pre) >= TRACK_CORNER_WIDTH_JUMP_MIN)
            {
                g_ctx.left_up.x = g_left_line[y];
                g_ctx.left_up.y = y;
                g_ctx.left_up.valid = true;
            }
        }

        if (!g_ctx.right_up.valid && y < mid_bottom)
        {
            if (!g_right_lost[y] && (width_now - width_pre) >= TRACK_CORNER_WIDTH_JUMP_MIN)
            {
                g_ctx.right_up.x = g_right_line[y];
                g_ctx.right_up.y = y;
                g_ctx.right_up.valid = true;
            }
        }
    }
}

static void collect_statistics(void)
{
    const int y_mid_start = CAM_HEIGHT / 3;
    const int y_mid_end = CAM_HEIGHT * 3 / 4;

    for (int y = TRACK_TOP_LINE; y < CAM_HEIGHT; ++y)
    {
        if (g_left_lost[y]) ++g_ctx.left_lost_total;
        if (g_right_lost[y]) ++g_ctx.right_lost_total;
        if (g_left_lost[y] && g_right_lost[y]) ++g_ctx.both_lost_total;

        if (y >= y_mid_start && y <= y_mid_end)
        {
            if (g_left_lost[y]) ++g_ctx.left_lost_mid;
            if (g_right_lost[y]) ++g_ctx.right_lost_mid;
            if (g_left_lost[y] && g_right_lost[y]) ++g_ctx.both_lost_mid;
        }
    }

    g_ctx.road_width_base = average_width(CAM_HEIGHT - 18, CAM_HEIGHT - 2);
    g_ctx.road_width_far = average_width(CAM_HEIGHT / 3, CAM_HEIGHT / 2 + 8);
}

static void judge_cross_and_ring(void)
{
    g_cross_flag = 0;
    g_ring_flag = 0;

    if (g_ctx.both_lost_mid >= TRACK_CROSS_BOTH_LOST_MID_MIN &&
        g_ctx.road_width_far > (g_ctx.road_width_base + TRACK_CROSS_WIDTH_DELTA_MIN) &&
        g_ctx.left_up.valid &&
        g_ctx.right_up.valid)
    {
        g_cross_flag = 1;
    }

    if (g_ctx.left_lost_mid >= TRACK_RING_SIDE_LOST_MID_MIN &&
        g_ctx.right_lost_mid <= TRACK_RING_OTHER_SIDE_LOST_MID_MAX &&
        g_ctx.right_down.valid &&
        g_ctx.both_lost_mid <= TRACK_RING_BOTH_LOST_MID_MAX)
    {
        g_ring_flag = 1;
    }
    else if (g_ctx.right_lost_mid >= TRACK_RING_SIDE_LOST_MID_MIN &&
             g_ctx.left_lost_mid <= TRACK_RING_OTHER_SIDE_LOST_MID_MAX &&
             g_ctx.left_down.valid &&
             g_ctx.both_lost_mid <= TRACK_RING_BOTH_LOST_MID_MAX)
    {
        g_ring_flag = 2;
    }
}

static void left_line_lengthen(uint8_t start, uint8_t end)
{
    int s = clamp_int(start, 0, CAM_HEIGHT - 1);
    int e = clamp_int(end, 0, CAM_HEIGHT - 1);
    if (s > e)
    {
        const int tmp = s;
        s = e;
        e = tmp;
    }

    const int ref0 = clamp_int(e, 1, CAM_HEIGHT - 2);
    const int ref1 = clamp_int(e - 4, 0, CAM_HEIGHT - 2);
    const int x0 = g_left_line[ref0];
    const int x1 = g_left_line[ref1];
    int dy = ref0 - ref1;
    if (dy == 0) dy = 1;
    const int k_num = x0 - x1;

    for (int y = s; y <= e; ++y)
    {
        const int x = x0 + (y - ref0) * k_num / dy;
        g_left_line[y] = clamp_int(x, 0, CAM_WIDTH - 1);
    }
}

static void right_line_lengthen(uint8_t start, uint8_t end)
{
    int s = clamp_int(start, 0, CAM_HEIGHT - 1);
    int e = clamp_int(end, 0, CAM_HEIGHT - 1);
    if (s > e)
    {
        const int tmp = s;
        s = e;
        e = tmp;
    }

    const int ref0 = clamp_int(e, 1, CAM_HEIGHT - 2);
    const int ref1 = clamp_int(e - 4, 0, CAM_HEIGHT - 2);
    const int x0 = g_right_line[ref0];
    const int x1 = g_right_line[ref1];
    int dy = ref0 - ref1;
    if (dy == 0) dy = 1;
    const int k_num = x0 - x1;

    for (int y = s; y <= e; ++y)
    {
        const int x = x0 + (y - ref0) * k_num / dy;
        g_right_line[y] = clamp_int(x, 0, CAM_WIDTH - 1);
    }
}

static void process_cross_state(void)
{
    int width_ref = g_ctx.road_width_base;
    int valid_mid = g_last_mid_line;

    for (int y = CAM_HEIGHT - 1; y >= TRACK_TOP_LINE; --y)
    {
        if (!g_left_lost[y] && !g_right_lost[y])
        {
            valid_mid = (g_left_line[y] + g_right_line[y]) >> 1;
            width_ref = clamp_int((width_ref * 7 + g_road_width[y] * 3) / 10,
                                  TRACK_MIN_LANE_WIDTH,
                                  CAM_WIDTH - 8);
        }
        else if (g_left_lost[y] && g_right_lost[y])
        {
            g_left_line[y] = clamp_int(valid_mid - width_ref / 2, 0, CAM_WIDTH - 2);
            g_right_line[y] = clamp_int(valid_mid + width_ref / 2, 1, CAM_WIDTH - 1);
        }
        else if (g_left_lost[y])
        {
            g_left_line[y] = clamp_int(static_cast<int>(g_right_line[y]) - width_ref,
                                    0,
                                    CAM_WIDTH - 2);
        }
        else if (g_right_lost[y])
        {
            g_right_line[y] = clamp_int(static_cast<int>(g_left_line[y]) + width_ref,
                                     1,
                                     CAM_WIDTH - 1);
        }

        g_road_width[y] = clamp_int(static_cast<int>(g_right_line[y]) -
                                 static_cast<int>(g_left_line[y]),
                                 0,
                                 CAM_WIDTH - 1);
    }
}

static void process_ring_left_state(void)
{
    const int width_ref = (g_ctx.road_width_base > TRACK_MIN_LANE_WIDTH) ? g_ctx.road_width_base : TRACK_DEFAULT_WIDTH;
    const int top = g_ctx.right_down.valid ? g_ctx.right_down.y : (CAM_HEIGHT / 2);

    right_line_lengthen(static_cast<uint8_t>(TRACK_TOP_LINE), static_cast<uint8_t>(top));

    for (int y = CAM_HEIGHT - 1; y >= TRACK_TOP_LINE; --y)
    {
        if (g_left_lost[y])
        {
            int x = static_cast<int>(g_right_line[y]) - width_ref;
            if (y > 2 && !g_left_lost[y - 1] && !g_left_lost[y - 2])
            {
                const int trend = static_cast<int>(g_left_line[y - 1]) - static_cast<int>(g_left_line[y - 2]);
                x += trend;
            }
            g_left_line[y] = clamp_int(x, 0, CAM_WIDTH - 2);
        }
        g_road_width[y] = clamp_int(static_cast<int>(g_right_line[y]) -
                                 static_cast<int>(g_left_line[y]),
                                 0,
                                 CAM_WIDTH - 1);
    }
}

static void process_ring_right_state(void)
{
    const int width_ref = (g_ctx.road_width_base > TRACK_MIN_LANE_WIDTH) ? g_ctx.road_width_base : TRACK_DEFAULT_WIDTH;
    const int top = g_ctx.left_down.valid ? g_ctx.left_down.y : (CAM_HEIGHT / 2);

    left_line_lengthen(static_cast<uint8_t>(TRACK_TOP_LINE), static_cast<uint8_t>(top));

    for (int y = CAM_HEIGHT - 1; y >= TRACK_TOP_LINE; --y)
    {
        if (g_right_lost[y])
        {
            int x = static_cast<int>(g_left_line[y]) + width_ref;
            if (y > 2 && !g_right_lost[y - 1] && !g_right_lost[y - 2])
            {
                const int trend = static_cast<int>(g_right_line[y - 1]) - static_cast<int>(g_right_line[y - 2]);
                x += trend;
            }
            g_right_line[y] = clamp_int(x, 1, CAM_WIDTH - 1);
        }
        g_road_width[y] = clamp_int(static_cast<int>(g_right_line[y]) -
                                 static_cast<int>(g_left_line[y]),
                                 0,
                                 CAM_WIDTH - 1);
    }
}

static void switch_to_state(uint8_t new_state, uint8_t hold_frames)
{
    g_ctx.state = new_state;
    g_ctx.state_hold = hold_frames;
    g_ctx.state_elapsed_frames = 0;
    g_ctx.timeout_exit = false;
}

static void force_back_to_normal(bool timeout_exit)
{
    g_ctx.state = TRACK_STATE_NORMAL;
    g_ctx.state_hold = 0;
    g_ctx.state_elapsed_frames = 0;
    g_ctx.state_cooldown = TRACK_STATE_REENTER_COOLDOWN_FRAMES;
    g_ctx.timeout_exit = timeout_exit;
    g_ctx.cross_counter = 0;
    g_ctx.ring_left_counter = 0;
    g_ctx.ring_right_counter = 0;
}

static uint16_t state_timeout_limit(uint8_t state)
{
    switch (state)
    {
    case TRACK_STATE_CROSS:      return TRACK_CROSS_TIMEOUT_FRAMES;
    case TRACK_STATE_RING_LEFT:
    case TRACK_STATE_RING_RIGHT: return TRACK_RING_TIMEOUT_FRAMES;
    default:                     return 0;
    }
}

static void update_track_state(void)
{
    if (g_cross_flag) {
        if (g_ctx.cross_counter < TRACK_COUNTER_MAX) ++g_ctx.cross_counter;
    } else if (g_ctx.cross_counter > 0) {
        --g_ctx.cross_counter;
    }

    if (g_ring_flag == 1) {
        if (g_ctx.ring_left_counter < TRACK_COUNTER_MAX) ++g_ctx.ring_left_counter;
    } else if (g_ctx.ring_left_counter > 0) {
        --g_ctx.ring_left_counter;
    }

    if (g_ring_flag == 2) {
        if (g_ctx.ring_right_counter < TRACK_COUNTER_MAX) ++g_ctx.ring_right_counter;
    } else if (g_ctx.ring_right_counter > 0) {
        --g_ctx.ring_right_counter;
    }

    g_ctx.timeout_exit = false;
    if (g_ctx.state_cooldown > 0)
    {
        --g_ctx.state_cooldown;
    }

    if (g_ctx.state != TRACK_STATE_NORMAL)
    {
        ++g_ctx.state_elapsed_frames;
        const uint16_t timeout_limit = state_timeout_limit(g_ctx.state);
        if (timeout_limit > 0 && g_ctx.state_elapsed_frames >= timeout_limit)
        {
            force_back_to_normal(true);
        }
    }

    switch (g_ctx.state)
    {
    case TRACK_STATE_NORMAL:
        if (g_ctx.state_cooldown == 0)
        {
            if (g_ctx.cross_counter >= TRACK_CROSS_ENTER_COUNT)
            {
                switch_to_state(TRACK_STATE_CROSS, TRACK_CROSS_HOLD_FRAMES);
            }
            else if (g_ctx.ring_left_counter >= TRACK_RING_ENTER_COUNT)
            {
                switch_to_state(TRACK_STATE_RING_LEFT, TRACK_RING_HOLD_FRAMES);
            }
            else if (g_ctx.ring_right_counter >= TRACK_RING_ENTER_COUNT)
            {
                switch_to_state(TRACK_STATE_RING_RIGHT, TRACK_RING_HOLD_FRAMES);
            }
        }
        break;

    case TRACK_STATE_CROSS:
        if (g_ctx.state_hold > 0) --g_ctx.state_hold;
        if (g_ctx.state_hold == 0 && g_ctx.cross_counter == 0)
        {
            force_back_to_normal(false);
        }
        break;

    case TRACK_STATE_RING_LEFT:
        if (g_ctx.state_hold > 0) --g_ctx.state_hold;
        if (g_ctx.state_hold == 0 && g_ctx.ring_left_counter == 0)
        {
            force_back_to_normal(false);
        }
        break;

    case TRACK_STATE_RING_RIGHT:
        if (g_ctx.state_hold > 0) --g_ctx.state_hold;
        if (g_ctx.state_hold == 0 && g_ctx.ring_right_counter == 0)
        {
            force_back_to_normal(false);
        }
        break;

    default:
        force_back_to_normal(false);
        break;
    }

    switch (g_ctx.state)
    {
    case TRACK_STATE_CROSS:
        g_ctx.element = TRACK_ELEMENT_CROSS;
        process_cross_state();
        break;
    case TRACK_STATE_RING_LEFT:
        g_ctx.element = TRACK_ELEMENT_RING_LEFT;
        process_ring_left_state();
        break;
    case TRACK_STATE_RING_RIGHT:
        g_ctx.element = TRACK_ELEMENT_RING_RIGHT;
        process_ring_right_state();
        break;
    default:
        g_ctx.element = TRACK_ELEMENT_STRAIGHT;
        break;
    }
}

static void build_mid_line(void)
{
    for (int y = CAM_HEIGHT - 1; y >= TRACK_TOP_LINE; --y)
    {
        const int mid = (static_cast<int>(g_left_line[y]) + static_cast<int>(g_right_line[y])) >> 1;
        g_mid_line[y] = clamp_int(mid, 1, CAM_WIDTH - 2);
    }
}

static int compute_weighted_mid_line(void)
{
    uint32_t weighted_mid_sum = 0;
    uint32_t weight_sum = 0;

    for (int y = CAM_HEIGHT - 1; y > TRACK_SEARCH_FINISH_LINE; --y)
    {
        const int weight = y - TRACK_SEARCH_FINISH_LINE + 1;
        weighted_mid_sum += static_cast<uint32_t>(g_mid_line[y]) * static_cast<uint32_t>(weight);
        weight_sum += static_cast<uint32_t>(weight);
    }

    if (weight_sum == 0)
    {
        g_final_mid_line = g_last_mid_line;
        return g_final_mid_line;
    }

    const int mid_line_value = static_cast<int>(weighted_mid_sum / weight_sum);
    g_final_mid_line =
        (g_last_mid_line * (TRACK_SMOOTH_ALPHA_DEN - TRACK_SMOOTH_ALPHA_NUM) +
         mid_line_value * TRACK_SMOOTH_ALPHA_NUM) /
        TRACK_SMOOTH_ALPHA_DEN;
    g_last_mid_line = g_final_mid_line;
    return g_final_mid_line;
}

static void fill_track_samples(track_result_t &track_result)
{
    const int y0 = clamp_int(TRACK_ROI_Y0, 0, CAM_HEIGHT - 1);
    const int y1 = clamp_int(TRACK_ROI_Y1, y0 + 1, CAM_HEIGHT);
    const int roi_height = y1 - y0;
    int scan_step = (roi_height > 0) ? (roi_height / TRACK_SCAN_LINES) : 1;
    if (scan_step <= 0) scan_step = 1;

    int valid_line_count = 0;
    float weighted_center_sum = 0.0f;
    float weight_sum = 0.0f;
    float top_center_sum = 0.0f;
    float bottom_center_sum = 0.0f;
    int top_center_count = 0;
    int bottom_center_count = 0;

    for (int i = 0; i < TRACK_SCAN_LINES; ++i)
    {
        const int y = clamp_int(y1 - 1 - i * scan_step, 0, CAM_HEIGHT - 1);
        const bool row_valid = (y >= TRACK_TOP_LINE) &&
                               (g_road_width[y] >= TRACK_MIN_LANE_WIDTH) &&
                               (g_road_width[y] <= TRACK_MAX_LANE_WIDTH);

        if (row_valid)
        {
            track_result.left_edge[i] = static_cast<int>(g_left_line[y]);
            track_result.right_edge[i] = static_cast<int>(g_right_line[y]);
            track_result.center[i] = static_cast<int>(g_mid_line[y]);

            const float weight = static_cast<float>(TRACK_SCAN_LINES - i);
            weighted_center_sum += static_cast<float>(g_mid_line[y]) * weight;
            weight_sum += weight;
            ++valid_line_count;

            if (i < TRACK_SCAN_LINES / 2)
            {
                bottom_center_sum += static_cast<float>(g_mid_line[y]);
                ++bottom_center_count;
            }
            else
            {
                top_center_sum += static_cast<float>(g_mid_line[y]);
                ++top_center_count;
            }
        }
        else
        {
            track_result.left_edge[i] = -1;
            track_result.right_edge[i] = -1;
            track_result.center[i] = -1;
        }
    }

    const float valid_ratio = static_cast<float>(valid_line_count) / static_cast<float>(TRACK_SCAN_LINES);
    g_quality_hint = 0.80f * g_quality_hint + 0.20f * valid_ratio;

    if (valid_line_count < 3 || valid_ratio < TRACK_VALID_RATIO_MIN)
    {
        track_result.valid = false;
        track_result.lost = true;
        return;
    }

    track_result.valid = true;
    track_result.lost = false;

    if (weight_sum > 0.0f)
    {
        track_result.center_avg = weighted_center_sum / weight_sum;
    }
    else
    {
        track_result.center_avg = static_cast<float>(g_final_mid_line);
    }

    g_center_hint = g_center_hint * (1.0f - TRACK_ERROR_FILTER_ALPHA) +
                    track_result.center_avg * TRACK_ERROR_FILTER_ALPHA;

    track_result.error = static_cast<float>(TRACK_CENTER_X) - static_cast<float>(g_final_mid_line);
    track_result.error_filtered =
        track_result.error_filtered * (1.0f - TRACK_ERROR_FILTER_ALPHA) +
        track_result.error * TRACK_ERROR_FILTER_ALPHA;

    float curvature_raw = 0.0f;
    if (bottom_center_count > 0 && top_center_count > 0)
    {
        const float bottom_mean = bottom_center_sum / static_cast<float>(bottom_center_count);
        const float top_mean = top_center_sum / static_cast<float>(top_center_count);
        curvature_raw = top_mean - bottom_mean;
    }

    g_curvature_hint = g_curvature_hint * (1.0f - TRACK_CURVATURE_FILTER_ALPHA) +
                       curvature_raw * TRACK_CURVATURE_FILTER_ALPHA;

    track_result.curvature_hint = g_curvature_hint;
    track_result.quality_hint = g_quality_hint;
    track_result.lane_width_base = g_ctx.road_width_base;
    track_result.lane_width_far = g_ctx.road_width_far;
    track_result.left_lost_total = g_ctx.left_lost_total;
    track_result.right_lost_total = g_ctx.right_lost_total;
    track_result.both_lost_total = g_ctx.both_lost_total;
    track_result.left_lost_mid = g_ctx.left_lost_mid;
    track_result.right_lost_mid = g_ctx.right_lost_mid;
    track_result.both_lost_mid = g_ctx.both_lost_mid;
    track_result.left_up = g_ctx.left_up;
    track_result.right_up = g_ctx.right_up;
    track_result.left_down = g_ctx.left_down;
    track_result.right_down = g_ctx.right_down;
    track_result.cross_flag = (g_cross_flag != 0);
    track_result.ring_flag = g_ring_flag;
    track_result.element = g_ctx.element;
    track_result.state = g_ctx.state;
    track_result.state_hold = g_ctx.state_hold;
    track_result.state_elapsed_frames = g_ctx.state_elapsed_frames;
    track_result.timeout_exit = g_ctx.timeout_exit;
    g_lane_width_est = g_lane_width_est * (1.0f - TRACK_WIDTH_FILTER_ALPHA) +
                       static_cast<float>(g_ctx.road_width_base) * TRACK_WIDTH_FILTER_ALPHA;
}

static void export_invalid_result(track_result_t &track_result, uint64_t timestamp_ms)
{
    track_result.valid = false;
    track_result.lost = true;
    track_result.timestamp_ms = timestamp_ms;
    track_result.element = TRACK_ELEMENT_NONE;
    track_result.state = TRACK_STATE_NORMAL;
    track_result.state_hold = 0;
    track_result.state_elapsed_frames = 0;
    track_result.timeout_exit = false;
    track_result.cross_flag = false;
    track_result.ring_flag = 0;
    track_result.curvature_hint = g_curvature_hint;
    track_result.quality_hint = g_quality_hint;
    for (int i = 0; i < TRACK_SCAN_LINES; ++i)
    {
        track_result.left_edge[i] = -1;
        track_result.right_edge[i] = -1;
        track_result.center[i] = -1;
    }
}
} // namespace

void line_track_init(track_result_t &track_result)
{
    std::memset(&track_result, 0, sizeof(track_result));
    for (int i = 0; i < TRACK_SCAN_LINES; ++i)
    {
        track_result.left_edge[i] = -1;
        track_result.right_edge[i] = -1;
        track_result.center[i] = -1;
    }

    track_result.lost = true;
    track_result.element = TRACK_ELEMENT_NONE;
    track_result.state = TRACK_STATE_NORMAL;
    track_result.state_elapsed_frames = 0;
    track_result.timeout_exit = false;

    std::memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.element = TRACK_ELEMENT_NONE;
    g_ctx.state = TRACK_STATE_NORMAL;
    g_ctx.state_hold = 0;
    g_ctx.state_elapsed_frames = 0;
    g_ctx.state_cooldown = 0;
    g_ctx.timeout_exit = false;
    g_final_mid_line = TRACK_CENTER_X;
    g_last_mid_line = TRACK_CENTER_X;
    g_lane_width_est = static_cast<float>(TRACK_DEFAULT_WIDTH);
    g_curvature_hint = 0.0f;
    g_quality_hint = 0.0f;
    g_center_hint = static_cast<float>(TRACK_CENTER_X);
    reset_internal_buffers();
}

void line_track_process(const frame_t &frame, track_result_t &track_result)
{
    for (int i = 0; i < TRACK_SCAN_LINES; ++i)
    {
        track_result.left_edge[i] = -1;
        track_result.right_edge[i] = -1;
        track_result.center[i] = -1;
    }

    if (!frame.valid || frame.gray == nullptr || frame.width != CAM_WIDTH || frame.height != CAM_HEIGHT)
    {
        export_invalid_result(track_result, frame.timestamp_ms);
        return;
    }

    reset_internal_buffers();

    const int otsu_threshold = compute_roi_otsu_threshold(frame);
    build_binary_image(frame, otsu_threshold);
    longest_white_column();
    collect_statistics();
    locate_corners_by_boundary_shape();
    judge_cross_and_ring();
    update_track_state();
    build_mid_line();
    compute_weighted_mid_line();

    track_result.timestamp_ms = frame.timestamp_ms;
    fill_track_samples(track_result);
    if (!track_result.valid)
    {
        export_invalid_result(track_result, frame.timestamp_ms);
        return;
    }

    track_result.timestamp_ms = frame.timestamp_ms;
}

float line_track_state_base_speed(const track_result_t &track_result)
{
    switch (track_result.state)
    {
    case TRACK_STATE_CROSS:
        return MOTOR_BASE_SPEED_CROSS;
    case TRACK_STATE_RING_LEFT:
    case TRACK_STATE_RING_RIGHT:
        return MOTOR_BASE_SPEED_RING;
    default:
        return MOTOR_BASE_SPEED_NORMAL;
    }
}

float line_track_compute_speed_scale(const track_result_t &track_result)
{
    const float abs_error = absf(track_result.error_filtered);
    const float abs_curvature = absf(track_result.curvature_hint);

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
        TRACK_SPEED_QUALITY_FLOOR +
        (1.0f - TRACK_SPEED_QUALITY_FLOOR) * clampf(track_result.quality_hint, 0.0f, 1.0f);

    float final_scale = error_scale * (1.0f - curvature_penalty) * quality_scale;

    if (!track_result.valid || track_result.lost) {
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
