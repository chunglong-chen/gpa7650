/**
 * @file    asps_switch_task.h
 * @brief   AS/PS mode switching task interface definitions.
 * @ingroup asps_switcher
 * @details
 * This header file defines macros, data structures, and interfaces
 * for controlling AS/PS mode switching using a DC motor and GPIO signals.
 *
 * It includes motor control macros, timeout configurations,
 * command types, and message structures used by the ASPS switch task.
 */
#ifndef __ASPS_SWITCH_TASK_H__
#define __ASPS_SWITCH_TASK_H__
#include "gpio_init.h"

#define DC_MOTOR_FORWARD                       \
    {                                          \
        gpio_set_value(ASPS_CTRL1, GPIO_HIGH); \
        gpio_set_value(ASPS_CTRL2, GPIO_LOW);  \
    }

#define DC_MOTOR_REVERSE                       \
    {                                          \
        gpio_set_value(ASPS_CTRL1, GPIO_LOW);  \
        gpio_set_value(ASPS_CTRL2, GPIO_HIGH); \
    }

#define DC_MOTOR_BRAKE                         \
    {                                          \
        gpio_set_value(ASPS_CTRL1, GPIO_HIGH); \
        gpio_set_value(ASPS_CTRL2, GPIO_HIGH); \
    }

#define ASPS_LOOP_DELAY_MS        1
#define ASPS_TOTAL_TIMEOUT_CNT    5000   // 5 seconds (1ms per loop)
#define ASPS_HIGH_TIMEOUT_CNT     1280  // keep original time length
#define ASPS_STABLE_LOW_CNT       5      // unchanged (still 5 consecutive reads)


// --- AS_CHK history (bit-packed ring buffer) ---
#define GPIO_HIST_LEN             1280U
#define GPIO_HIST_BYTES           (GPIO_HIST_LEN / 8U)   // 1280 bits = 160 bytes
typedef enum {
    ASPS_TASK_CMD_UNKOWN_MODE = 0,
    ASPS_TASK_CMD_SET_AS_MODE,
    ASPS_TASK_CMD_SET_PS_MODE,
    ASPS_TASK_CMD_GET_MODE,
    ASPS_TASK_CMD_GET_PI_GPIO,
    ASPS_TASK_CMD_GET_GPIO_STATUS,
    ASPS_TASK_CMD_GET_ALL,
} asps_task_cmd_e;
typedef struct {
    INT8U          mode;
    INT32S         index;
} asps_task_message_t;

void asps_switch_task(void);
static volatile uint8_t  g_aschk_hist[GPIO_HIST_BYTES];
static volatile uint16_t g_aschk_hist_idx;
#endif // __ASPS_SWITCH_TASK_H__
