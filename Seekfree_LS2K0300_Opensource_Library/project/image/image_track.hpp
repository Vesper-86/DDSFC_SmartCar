#ifndef CODE_IMAGE_HPP_
#define CODE_IMAGE_HPP_

#include "zf_common_headfile.hpp"

// -------------------- 参数区 --------------------
#define search_finish_line      30
#define TRACK_TOP_LINE          8
#define TRACK_MIN_SEARCH_HALF   18
#define TRACK_DEFAULT_WIDTH     70
#define TRACK_SMOOTH_ALPHA_NUM  8   // 0.8
#define TRACK_SMOOTH_ALPHA_DEN  10

typedef enum
{
    TRACK_ELEMENT_NONE = 0,
    TRACK_ELEMENT_STRAIGHT,
    TRACK_ELEMENT_CROSS,
    TRACK_ELEMENT_RING_LEFT,
    TRACK_ELEMENT_RING_RIGHT,
} track_element_e;

typedef enum
{
    TRACK_STATE_NORMAL = 0,
    TRACK_STATE_CROSS,
    TRACK_STATE_RING_LEFT,
    TRACK_STATE_RING_RIGHT,
} track_state_e;

typedef struct
{
    int x;
    int y;
    uint8_t valid;
} track_point_t;

typedef struct
{
    uint8_t element;
    uint8_t state;
    uint8_t state_hold;
    uint8_t cross_counter;
    uint8_t ring_left_counter;
    uint8_t ring_right_counter;
    uint8_t stable_counter;

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
} track_context_t;

extern uint8_t base_image[UVC_HEIGHT][UVC_WIDTH];
extern uint8_t image[UVC_HEIGHT][UVC_WIDTH];

extern uint8_t left_line_list[UVC_HEIGHT];
extern uint8_t right_line_list[UVC_HEIGHT];
extern uint8_t mid_line_list[UVC_HEIGHT];
extern uint8_t left_lost_list[UVC_HEIGHT];
extern uint8_t right_lost_list[UVC_HEIGHT];
extern uint8_t road_wide[UVC_HEIGHT];

extern uint8_t left_jidian;
extern uint8_t right_jidian;
extern uint8_t final_mid_line;
extern uint8_t last_mid_line;
extern uint8_t search_stop_line;

extern uint8_t straight_flag;
extern uint8_t cross_flag;
extern uint8_t ring_flag_0;
extern uint8_t left_lost_point;
extern uint8_t right_lost_point;
extern uint8_t both_lost_point;

extern uint8_t right_down_find;
extern uint8_t left_down_find;
extern uint8_t left_up_find;
extern uint8_t right_up_find;

extern track_context_t g_track_ctx;

#ifdef __cplusplus
extern "C" {
#endif

void track_process_frame(void);
void longest_white_column(void);
void mid_line(void);
uint8_t find_mid_line_weight(void);
void straight_judge(void);
void cross_judge(void);
void ring_judge_0(void);
void fast_find_up_down_corner(void);

void left_add_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void right_add_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void left_line_lengthen(uint8_t start, uint8_t end);
void right_line_lengthen(uint8_t start, uint8_t end);

#ifdef __cplusplus
}
#endif

#endif
