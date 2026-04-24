#include "zf_common_headfile.hpp"
#include "image.hpp"
#include <cstring>

uint8_t base_image[UVC_HEIGHT][UVC_WIDTH] = {0};
uint8_t image[UVC_HEIGHT][UVC_WIDTH] = {0};

uint8_t left_line_list[UVC_HEIGHT] = {0};
uint8_t right_line_list[UVC_HEIGHT] = {0};
uint8_t mid_line_list[UVC_HEIGHT] = {0};
uint8_t left_lost_list[UVC_HEIGHT] = {0};
uint8_t right_lost_list[UVC_HEIGHT] = {0};
uint8_t road_wide[UVC_HEIGHT] = {0};

static uint8_t white_column[UVC_WIDTH] = {0};
static uint8_t mid_weight_list[UVC_HEIGHT] = {
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    2,2,2,2,2,2,2,2,2,2,
    3,3,3,3,3,3,3,3,3,3,
    4,4,4,4,4,4,4,4,4,4,
    5,5,5,5,5,5,5,5,5,5,
    7,7,7,7,7,7,8,8,8,8,
    9,9,10,10,11,11,12,12,13,13,
    14,14,15,15,16,16,17,17,18,18
};

uint8_t left_jidian = 0;
uint8_t right_jidian = 0;
uint8_t left_lost_point = 0;
uint8_t right_lost_point = 0;
uint8_t both_lost_point = 0;
uint8_t search_stop_line = 0;

uint8_t straight_flag = 0;
uint8_t cross_flag = 0;
uint8_t ring_flag_0 = 0;
uint8_t right_down_find = 0;
uint8_t left_down_find = 0;
uint8_t left_up_find = 0;
uint8_t right_up_find = 0;

uint8_t final_mid_line = UVC_WIDTH / 2;
uint8_t last_mid_line = UVC_WIDTH / 2;
track_context_t g_track_ctx = {0};

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

static void reset_frame_buffers(void)
{
    memset(left_lost_list, 0, sizeof(left_lost_list));
    memset(right_lost_list, 0, sizeof(right_lost_list));
    memset(road_wide, 0, sizeof(road_wide));

    for (int y = 0; y < UVC_HEIGHT; ++y)
    {
        left_line_list[y] = 0;
        right_line_list[y] = UVC_WIDTH - 1;
        mid_line_list[y] = UVC_WIDTH / 2;
    }

    left_jidian = 0;
    right_jidian = 0;
    left_lost_point = 0;
    right_lost_point = 0;
    both_lost_point = 0;
    search_stop_line = 0;

    left_up_find = 0;
    right_up_find = 0;
    left_down_find = 0;
    right_down_find = 0;
    straight_flag = 0;
    cross_flag = 0;
    ring_flag_0 = 0;

    memset(&g_track_ctx.left_up, 0, sizeof(track_point_t));
    memset(&g_track_ctx.right_up, 0, sizeof(track_point_t));
    memset(&g_track_ctx.left_down, 0, sizeof(track_point_t));
    memset(&g_track_ctx.right_down, 0, sizeof(track_point_t));

    g_track_ctx.left_lost_total = 0;
    g_track_ctx.right_lost_total = 0;
    g_track_ctx.both_lost_total = 0;
    g_track_ctx.left_lost_mid = 0;
    g_track_ctx.right_lost_mid = 0;
    g_track_ctx.both_lost_mid = 0;
    g_track_ctx.road_width_base = TRACK_DEFAULT_WIDTH;
    g_track_ctx.road_width_far = TRACK_DEFAULT_WIDTH;
}

static int average_width(int y_start, int y_end)
{
    int sum = 0;
    int cnt = 0;

    y_start = clip_int(y_start, 0, UVC_HEIGHT - 1);
    y_end = clip_int(y_end, 0, UVC_HEIGHT - 1);

    if (y_start > y_end)
    {
        int t = y_start;
        y_start = y_end;
        y_end = t;
    }

    for (int y = y_start; y <= y_end; ++y)
    {
        if (!left_lost_list[y] && !right_lost_list[y])
        {
            int w = (int)right_line_list[y] - (int)left_line_list[y];
            if (w > 10 && w < UVC_WIDTH)
            {
                sum += w;
                cnt++;
            }
        }
    }

    if (cnt == 0) return TRACK_DEFAULT_WIDTH;
    return sum / cnt;
}

static void search_edges_from_seed(void)
{
    int seed_x = g_track_ctx.seed_column;
    int ref_mid = clip_int(seed_x, 2, UVC_WIDTH - 3);
    int ref_width = (g_track_ctx.road_width_base > 20) ? g_track_ctx.road_width_base : TRACK_DEFAULT_WIDTH;
    int top_line = UVC_HEIGHT - g_track_ctx.search_stop_line;
    top_line = clip_int(top_line, TRACK_TOP_LINE, UVC_HEIGHT - 1);

    for (int y = UVC_HEIGHT - 1; y >= top_line; --y)
    {
        int half_search = ref_width / 2 + 8;
        if (half_search < TRACK_MIN_SEARCH_HALF) half_search = TRACK_MIN_SEARCH_HALF;
        if (half_search > (UVC_WIDTH / 2 - 2)) half_search = UVC_WIDTH / 2 - 2;

        int left_start = clip_int(ref_mid, 2, UVC_WIDTH - 3);
        int left_limit = clip_int(ref_mid - half_search, 2, UVC_WIDTH - 3);
        int right_start = clip_int(ref_mid, 2, UVC_WIDTH - 3);
        int right_limit = clip_int(ref_mid + half_search, 2, UVC_WIDTH - 3);

        int left_found = -1;
        int right_found = -1;

        for (int x = left_start; x >= left_limit; --x)
        {
            if (image[y][x - 1] == 0 && image[y][x] == 255)
            {
                left_found = x;
                break;
            }
        }

        for (int x = right_start; x <= right_limit; ++x)
        {
            if (image[y][x] == 255 && image[y][x + 1] == 0)
            {
                right_found = x;
                break;
            }
        }

        if (left_found < 0)
        {
            left_lost_list[y] = 1;
            left_found = clip_int(ref_mid - ref_width / 2, 1, UVC_WIDTH - 2);
        }
        if (right_found < 0)
        {
            right_lost_list[y] = 1;
            right_found = clip_int(ref_mid + ref_width / 2, 1, UVC_WIDTH - 2);
        }

        if (left_found >= right_found)
        {
            int half_w = ref_width / 2;
            left_found = clip_int(ref_mid - half_w, 1, UVC_WIDTH - 3);
            right_found = clip_int(ref_mid + half_w, left_found + 2, UVC_WIDTH - 2);
            left_lost_list[y] = 1;
            right_lost_list[y] = 1;
        }

        left_line_list[y] = (uint8_t)left_found;
        right_line_list[y] = (uint8_t)right_found;
        road_wide[y] = (uint8_t)clip_int(right_found - left_found, 0, UVC_WIDTH - 1);

        if (!left_lost_list[y] && left_jidian == 0) left_jidian = (uint8_t)y;
        if (!right_lost_list[y] && right_jidian == 0) right_jidian = (uint8_t)y;

        ref_mid = (left_found + right_found) >> 1;
        ref_width = clip_int((ref_width * 7 + (right_found - left_found) * 3) / 10, 24, UVC_WIDTH - 8);
    }
}

void longest_white_column(void)
{
    reset_frame_buffers();
    memset(white_column, 0, sizeof(white_column));

    int start_column = 10;
    int end_column = UVC_WIDTH - 11;
    int best_len = 0;
    int best_col = UVC_WIDTH / 2;

    for (int x = start_column; x <= end_column; ++x)
    {
        int len = 0;
        for (int y = UVC_HEIGHT - 1; y >= 0; --y)
        {
            if (image[y][x] == 255) len++;
            else break;
        }
        white_column[x] = (uint8_t)clip_int(len, 0, 255);
        if (len > best_len)
        {
            best_len = len;
            best_col = x;
        }
    }

    if (best_len < 20)
    {
        best_len = UVC_HEIGHT / 2;
        best_col = last_mid_line;
    }

    g_track_ctx.seed_column = best_col;
    g_track_ctx.search_stop_line = clip_int(best_len + 8, 40, UVC_HEIGHT - TRACK_TOP_LINE);
    search_stop_line = (uint8_t)g_track_ctx.search_stop_line;

    g_track_ctx.road_width_base = average_width(UVC_HEIGHT - 15, UVC_HEIGHT - 1);
    if (g_track_ctx.road_width_base < 24) g_track_ctx.road_width_base = TRACK_DEFAULT_WIDTH;

    search_edges_from_seed();

    for (int y = UVC_HEIGHT - 1; y >= TRACK_TOP_LINE; --y)
    {
        if (left_lost_list[y]) left_lost_point++;
        if (right_lost_list[y]) right_lost_point++;
        if (left_lost_list[y] && right_lost_list[y]) both_lost_point++;
    }
}

static void locate_corners_by_boundary_shape(void)
{
    int mid_top = UVC_HEIGHT / 3;
    int mid_bottom = UVC_HEIGHT - 8;

    for (int y = UVC_HEIGHT - 2; y >= mid_top + 2; --y)
    {
        int dl = (int)left_line_list[y] - (int)left_line_list[y - 2];
        int dr = (int)right_line_list[y] - (int)right_line_list[y - 2];
        int w_now = (int)road_wide[y];
        int w_pre = (int)road_wide[y - 2];

        if (!g_track_ctx.left_down.valid && y > UVC_HEIGHT / 2)
        {
            if (!left_lost_list[y] && !left_lost_list[y - 2] && abs_int(dl) >= 6)
            {
                g_track_ctx.left_down.x = left_line_list[y];
                g_track_ctx.left_down.y = y;
                g_track_ctx.left_down.valid = 1;
            }
        }

        if (!g_track_ctx.right_down.valid && y > UVC_HEIGHT / 2)
        {
            if (!right_lost_list[y] && !right_lost_list[y - 2] && abs_int(dr) >= 6)
            {
                g_track_ctx.right_down.x = right_line_list[y];
                g_track_ctx.right_down.y = y;
                g_track_ctx.right_down.valid = 1;
            }
        }

        if (!g_track_ctx.left_up.valid && y < mid_bottom)
        {
            if (!left_lost_list[y] && (w_now - w_pre) >= 10)
            {
                g_track_ctx.left_up.x = left_line_list[y];
                g_track_ctx.left_up.y = y;
                g_track_ctx.left_up.valid = 1;
            }
        }

        if (!g_track_ctx.right_up.valid && y < mid_bottom)
        {
            if (!right_lost_list[y] && (w_now - w_pre) >= 10)
            {
                g_track_ctx.right_up.x = right_line_list[y];
                g_track_ctx.right_up.y = y;
                g_track_ctx.right_up.valid = 1;
            }
        }
    }

    left_up_find = g_track_ctx.left_up.valid ? (uint8_t)g_track_ctx.left_up.y : 0;
    right_up_find = g_track_ctx.right_up.valid ? (uint8_t)g_track_ctx.right_up.y : 0;
    left_down_find = g_track_ctx.left_down.valid ? (uint8_t)g_track_ctx.left_down.y : 0;
    right_down_find = g_track_ctx.right_down.valid ? (uint8_t)g_track_ctx.right_down.y : 0;
}

void fast_find_up_down_corner(void)
{
    locate_corners_by_boundary_shape();
}

static void collect_statistics(void)
{
    int y_mid_start = UVC_HEIGHT / 3;
    int y_mid_end = UVC_HEIGHT * 3 / 4;

    for (int y = TRACK_TOP_LINE; y < UVC_HEIGHT; ++y)
    {
        if (left_lost_list[y]) g_track_ctx.left_lost_total++;
        if (right_lost_list[y]) g_track_ctx.right_lost_total++;
        if (left_lost_list[y] && right_lost_list[y]) g_track_ctx.both_lost_total++;

        if (y >= y_mid_start && y <= y_mid_end)
        {
            if (left_lost_list[y]) g_track_ctx.left_lost_mid++;
            if (right_lost_list[y]) g_track_ctx.right_lost_mid++;
            if (left_lost_list[y] && right_lost_list[y]) g_track_ctx.both_lost_mid++;
        }
    }

    g_track_ctx.road_width_base = average_width(UVC_HEIGHT - 18, UVC_HEIGHT - 2);
    g_track_ctx.road_width_far = average_width(UVC_HEIGHT / 3, UVC_HEIGHT / 2 + 8);
}

void straight_judge(void)
{
    straight_flag = 0;
    if (g_track_ctx.road_width_far <= (g_track_ctx.road_width_base + 10) &&
        g_track_ctx.left_lost_mid <= 3 &&
        g_track_ctx.right_lost_mid <= 3 &&
        left_jidian >= (UVC_HEIGHT / 2) &&
        right_jidian >= (UVC_HEIGHT / 2))
    {
        straight_flag = 1;
    }
}

void cross_judge(void)
{
    cross_flag = 0;

    if (g_track_ctx.both_lost_mid >= 6 &&
        g_track_ctx.road_width_far > (g_track_ctx.road_width_base + 12) &&
        g_track_ctx.left_up.valid &&
        g_track_ctx.right_up.valid)
    {
        cross_flag = 1;
    }
}

void ring_judge_0(void)
{
    ring_flag_0 = 0;

    if (g_track_ctx.left_lost_mid >= 10 &&
        g_track_ctx.right_lost_mid <= 4 &&
        g_track_ctx.right_down.valid &&
        g_track_ctx.both_lost_mid <= 5)
    {
        ring_flag_0 = 1;
    }
    else if (g_track_ctx.right_lost_mid >= 10 &&
             g_track_ctx.left_lost_mid <= 4 &&
             g_track_ctx.left_down.valid &&
             g_track_ctx.both_lost_mid <= 5)
    {
        ring_flag_0 = 2;
    }
}

void left_add_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    int yy1 = y1;
    int yy2 = y2;
    int xx1 = x1;
    int xx2 = x2;

    yy1 = clip_int(yy1, 0, UVC_HEIGHT - 1);
    yy2 = clip_int(yy2, 0, UVC_HEIGHT - 1);
    xx1 = clip_int(xx1, 0, UVC_WIDTH - 1);
    xx2 = clip_int(xx2, 0, UVC_WIDTH - 1);

    if (yy1 == yy2)
    {
        left_line_list[yy1] = (uint8_t)xx1;
        return;
    }

    if (yy1 > yy2)
    {
        int t;
        t = yy1; yy1 = yy2; yy2 = t;
        t = xx1; xx1 = xx2; xx2 = t;
    }

    for (int y = yy1; y <= yy2; ++y)
    {
        int x = xx1 + (y - yy1) * (xx2 - xx1) / (yy2 - yy1);
        left_line_list[y] = (uint8_t)clip_int(x, 0, UVC_WIDTH - 1);
    }
}

void right_add_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    int yy1 = y1;
    int yy2 = y2;
    int xx1 = x1;
    int xx2 = x2;

    yy1 = clip_int(yy1, 0, UVC_HEIGHT - 1);
    yy2 = clip_int(yy2, 0, UVC_HEIGHT - 1);
    xx1 = clip_int(xx1, 0, UVC_WIDTH - 1);
    xx2 = clip_int(xx2, 0, UVC_WIDTH - 1);

    if (yy1 == yy2)
    {
        right_line_list[yy1] = (uint8_t)xx1;
        return;
    }

    if (yy1 > yy2)
    {
        int t;
        t = yy1; yy1 = yy2; yy2 = t;
        t = xx1; xx1 = xx2; xx2 = t;
    }

    for (int y = yy1; y <= yy2; ++y)
    {
        int x = xx1 + (y - yy1) * (xx2 - xx1) / (yy2 - yy1);
        right_line_list[y] = (uint8_t)clip_int(x, 0, UVC_WIDTH - 1);
    }
}

void left_line_lengthen(uint8_t start, uint8_t end)
{
    int s = clip_int(start, 0, UVC_HEIGHT - 1);
    int e = clip_int(end, 0, UVC_HEIGHT - 1);
    if (s > e)
    {
        int t = s;
        s = e;
        e = t;
    }

    int ref0 = clip_int(e, 1, UVC_HEIGHT - 2);
    int ref1 = clip_int(e - 4, 0, UVC_HEIGHT - 2);
    int x0 = left_line_list[ref0];
    int x1 = left_line_list[ref1];
    int dy = ref0 - ref1;
    if (dy == 0) dy = 1;
    int k_num = x0 - x1;

    for (int y = s; y <= e; ++y)
    {
        int x = x0 + (y - ref0) * k_num / dy;
        left_line_list[y] = (uint8_t)clip_int(x, 0, UVC_WIDTH - 1);
    }
}

void right_line_lengthen(uint8_t start, uint8_t end)
{
    int s = clip_int(start, 0, UVC_HEIGHT - 1);
    int e = clip_int(end, 0, UVC_HEIGHT - 1);
    if (s > e)
    {
        int t = s;
        s = e;
        e = t;
    }

    int ref0 = clip_int(e, 1, UVC_HEIGHT - 2);
    int ref1 = clip_int(e - 4, 0, UVC_HEIGHT - 2);
    int x0 = right_line_list[ref0];
    int x1 = right_line_list[ref1];
    int dy = ref0 - ref1;
    if (dy == 0) dy = 1;
    int k_num = x0 - x1;

    for (int y = s; y <= e; ++y)
    {
        int x = x0 + (y - ref0) * k_num / dy;
        right_line_list[y] = (uint8_t)clip_int(x, 0, UVC_WIDTH - 1);
    }
}

static void process_cross_state(void)
{
    int width_ref = g_track_ctx.road_width_base;
    int valid_mid = last_mid_line;

    for (int y = UVC_HEIGHT - 1; y >= TRACK_TOP_LINE; --y)
    {
        if (!left_lost_list[y] && !right_lost_list[y])
        {
            valid_mid = (left_line_list[y] + right_line_list[y]) >> 1;
            width_ref = clip_int((width_ref * 7 + road_wide[y] * 3) / 10, 24, UVC_WIDTH - 8);
        }
        else if (left_lost_list[y] && right_lost_list[y])
        {
            left_line_list[y] = (uint8_t)clip_int(valid_mid - width_ref / 2, 0, UVC_WIDTH - 2);
            right_line_list[y] = (uint8_t)clip_int(valid_mid + width_ref / 2, 1, UVC_WIDTH - 1);
        }
        else if (left_lost_list[y])
        {
            left_line_list[y] = (uint8_t)clip_int((int)right_line_list[y] - width_ref, 0, UVC_WIDTH - 2);
        }
        else if (right_lost_list[y])
        {
            right_line_list[y] = (uint8_t)clip_int((int)left_line_list[y] + width_ref, 1, UVC_WIDTH - 1);
        }
        road_wide[y] = (uint8_t)clip_int((int)right_line_list[y] - (int)left_line_list[y], 0, UVC_WIDTH - 1);
    }
}

static void process_ring_left_state(void)
{
    int width_ref = g_track_ctx.road_width_base;
    int top = g_track_ctx.right_down.valid ? g_track_ctx.right_down.y : (UVC_HEIGHT / 2);

    right_line_lengthen((uint8_t)TRACK_TOP_LINE, (uint8_t)top);

    for (int y = UVC_HEIGHT - 1; y >= TRACK_TOP_LINE; --y)
    {
        if (left_lost_list[y])
        {
            int x = (int)right_line_list[y] - width_ref;
            if (y > 2 && !left_lost_list[y - 1] && !left_lost_list[y - 2])
            {
                int trend = (int)left_line_list[y - 1] - (int)left_line_list[y - 2];
                x += trend;
            }
            left_line_list[y] = (uint8_t)clip_int(x, 0, UVC_WIDTH - 2);
        }
        road_wide[y] = (uint8_t)clip_int((int)right_line_list[y] - (int)left_line_list[y], 0, UVC_WIDTH - 1);
    }
}

static void process_ring_right_state(void)
{
    int width_ref = g_track_ctx.road_width_base;
    int top = g_track_ctx.left_down.valid ? g_track_ctx.left_down.y : (UVC_HEIGHT / 2);

    left_line_lengthen((uint8_t)TRACK_TOP_LINE, (uint8_t)top);

    for (int y = UVC_HEIGHT - 1; y >= TRACK_TOP_LINE; --y)
    {
        if (right_lost_list[y])
        {
            int x = (int)left_line_list[y] + width_ref;
            if (y > 2 && !right_lost_list[y - 1] && !right_lost_list[y - 2])
            {
                int trend = (int)right_line_list[y - 1] - (int)right_line_list[y - 2];
                x += trend;
            }
            right_line_list[y] = (uint8_t)clip_int(x, 1, UVC_WIDTH - 1);
        }
        road_wide[y] = (uint8_t)clip_int((int)right_line_list[y] - (int)left_line_list[y], 0, UVC_WIDTH - 1);
    }
}

static void update_track_state(void)
{
    if (cross_flag)
    {
        if (g_track_ctx.cross_counter < 10) g_track_ctx.cross_counter++;
    }
    else if (g_track_ctx.cross_counter > 0)
    {
        g_track_ctx.cross_counter--;
    }

    if (ring_flag_0 == 1)
    {
        if (g_track_ctx.ring_left_counter < 10) g_track_ctx.ring_left_counter++;
    }
    else if (g_track_ctx.ring_left_counter > 0)
    {
        g_track_ctx.ring_left_counter--;
    }

    if (ring_flag_0 == 2)
    {
        if (g_track_ctx.ring_right_counter < 10) g_track_ctx.ring_right_counter++;
    }
    else if (g_track_ctx.ring_right_counter > 0)
    {
        g_track_ctx.ring_right_counter--;
    }

    switch (g_track_ctx.state)
    {
        case TRACK_STATE_NORMAL:
            if (g_track_ctx.cross_counter >= 2)
            {
                g_track_ctx.state = TRACK_STATE_CROSS;
                g_track_ctx.state_hold = 8;
            }
            else if (g_track_ctx.ring_left_counter >= 2)
            {
                g_track_ctx.state = TRACK_STATE_RING_LEFT;
                g_track_ctx.state_hold = 12;
            }
            else if (g_track_ctx.ring_right_counter >= 2)
            {
                g_track_ctx.state = TRACK_STATE_RING_RIGHT;
                g_track_ctx.state_hold = 12;
            }
            break;

        case TRACK_STATE_CROSS:
            if (g_track_ctx.state_hold > 0) g_track_ctx.state_hold--;
            if (g_track_ctx.state_hold == 0 && g_track_ctx.cross_counter == 0)
            {
                g_track_ctx.state = TRACK_STATE_NORMAL;
            }
            break;

        case TRACK_STATE_RING_LEFT:
            if (g_track_ctx.state_hold > 0) g_track_ctx.state_hold--;
            if (g_track_ctx.state_hold == 0 && g_track_ctx.ring_left_counter == 0)
            {
                g_track_ctx.state = TRACK_STATE_NORMAL;
            }
            break;

        case TRACK_STATE_RING_RIGHT:
            if (g_track_ctx.state_hold > 0) g_track_ctx.state_hold--;
            if (g_track_ctx.state_hold == 0 && g_track_ctx.ring_right_counter == 0)
            {
                g_track_ctx.state = TRACK_STATE_NORMAL;
            }
            break;

        default:
            g_track_ctx.state = TRACK_STATE_NORMAL;
            break;
    }

    switch (g_track_ctx.state)
    {
        case TRACK_STATE_CROSS:
            g_track_ctx.element = TRACK_ELEMENT_CROSS;
            process_cross_state();
            break;
        case TRACK_STATE_RING_LEFT:
            g_track_ctx.element = TRACK_ELEMENT_RING_LEFT;
            process_ring_left_state();
            break;
        case TRACK_STATE_RING_RIGHT:
            g_track_ctx.element = TRACK_ELEMENT_RING_RIGHT;
            process_ring_right_state();
            break;
        default:
            g_track_ctx.element = straight_flag ? TRACK_ELEMENT_STRAIGHT : TRACK_ELEMENT_NONE;
            break;
    }
}

void mid_line(void)
{
    for (int y = UVC_HEIGHT - 1; y >= TRACK_TOP_LINE; --y)
    {
        int mid = ((int)left_line_list[y] + (int)right_line_list[y]) >> 1;
        mid_line_list[y] = (uint8_t)clip_int(mid, 1, UVC_WIDTH - 2);
    }
}

uint8_t find_mid_line_weight(void)
{
    uint32_t weight_midline_sum = 0;
    uint32_t weight_sum = 0;

    for (int y = UVC_HEIGHT - 1; y > search_finish_line; --y)
    {
        weight_midline_sum += (uint32_t)mid_line_list[y] * mid_weight_list[y];
        weight_sum += mid_weight_list[y];
    }

    if (weight_sum == 0)
    {
        final_mid_line = last_mid_line;
        return final_mid_line;
    }

    uint8_t mid_line_value = (uint8_t)(weight_midline_sum / weight_sum);
    final_mid_line = (uint8_t)((last_mid_line * (TRACK_SMOOTH_ALPHA_DEN - TRACK_SMOOTH_ALPHA_NUM) +
                               mid_line_value * TRACK_SMOOTH_ALPHA_NUM) /
                              TRACK_SMOOTH_ALPHA_DEN);
    last_mid_line = final_mid_line;
    return final_mid_line;
}

void track_process_frame(void)
{
    longest_white_column();
    collect_statistics();
    fast_find_up_down_corner();
    straight_judge();
    cross_judge();
    ring_judge_0();
    update_track_state();
    mid_line();
    find_mid_line_weight();
}
