#ifndef __SONY_MICRO_DISPLAY_H__
#define __SONY_MICRO_DISPLAY_H__
#include "typedef.h"

#define DISP_NUMBER_MAX                5
#define MICRO_DISPLAY_BLINK_TASK_CYCLE 5  // for vTaskDelayUntil using
#define ACTUAL_TICK_DELAY_COUNT        40 // actual delay we need

// format RGB888
#define COLOR_WHITE        0xFFFFFF
#define COLOR_RED          0xFF0000
#define COLOR_GREEN        0x00FF00
#define COLOR_BLUE         0x0000FF

#define DEFAULT_BRIGHTNESS 255.0f

typedef enum {
    ANIMATE_STATE_STOP = 0,
    ANIMATE_STATE_START,
    ANIMATE_STATE_PLAY,
} sony_animation_state_e;

typedef enum {
    SONY_TASK_CMD_DEFAULT = 0,
    SONY_TASK_CMD_ENABLE,
    SONY_TASK_CMD_DISABLE,
    SONY_TASK_CMD_FIXATION_MODE,
    SONY_TASK_CMD_SHOW,
    SONY_TASK_CMD_DISC,
    SONY_TASK_CMD_FULL,
    SONY_TASK_CMD_CHANGE_POSITION,
    SONY_TASK_CMD_CHANGE_SIZE,
    SONY_TASK_CMD_GET_INFO,
    SONY_TASK_CMD_CHANGE_SPEED,
    SONY_TASK_CMD_CHANGE_BRIGHTNESS,
} sony_task_cmd_e;

typedef enum {
    SONY_PATTERN_RECTANGLE = 0,
    SONY_PATTERN_BLINK,
    SONY_PATTERN_OPEN,
    SONY_PATTERN_CLOSE,
    SONY_PATTERN_OK,
    SONY_PATTERN_1_PS,
    SONY_PATTERN_1_AS,
    SONY_PATTERN_2_PS,
    SONY_PATTERN_2_AS,
    // Keep PATTERN_FULL and PATTERN_MAX as the last entries.
    SONY_PATTERN_FULL,
    SONY_PATTERN_MAX,
} sony_pattern_e;

typedef enum {
    SONY_SIZE_SMALL = 0,
    SONY_SIZE_MEDIUM,
    SONY_SIZE_LARGE,
} sony_size_e;

typedef enum {
    SONY_COLOR_WHITE = 0,
    SONY_COLOR_RED,
    SONY_COLOR_GREEN,
    SONY_COLOR_BLUE,
} sony_color_e;

typedef struct
{
    sony_task_cmd_e sony_cmd;
    INT32S          x;
    INT32S          y;
    INT16U          w_size;
    INT16U          h_size;
    sony_pattern_e  pattern;
    INT8U           color;
    float           speed;
    INT16U          brightness;
} sony_task_message_t;

void   sony_task_runner(void);
INT16U sony_check_color(INT8U color_idx);
void   draw_pixel(INT16U* src, INT32S x, INT32S y, INT16U color);
void   sony_init_base_buff(sony_task_message_t msg);
void   sony_stop_animation(void);
INT8U  sony_change_transition_speed(float speed);
#endif // __SONY_MICRO_DISPLAY_H__
