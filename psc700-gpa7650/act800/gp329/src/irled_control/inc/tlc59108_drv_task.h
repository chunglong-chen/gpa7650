/**
 * @file    tlc59108_drv_task.h
 * @brief   TLC59108 IR LED driver and task interface definitions.
 * @ingroup pupil_led_controller
 * @details
 * This header file defines the data types, command types,
 * register-related macros, and public interfaces for the
 * TLC59108 IR LED control module.
 *
 * It provides the definitions required for IR LED initialization,
 * LED current control, register access, and task-based command handling.
 */
#ifndef __TLC59108_DRV_H__
#define __TLC59108_DRV_H__

#include "typedef.h"

#define TLC59108_I2C_CLK_RATE 200
#define TLC59108_SLAVE_ADDR   0x90
#define TLC59108_MAX_LED_NUM  8 // 0~7
#define MIIS_MAX_LED_NUM      4 // 1~4

#define TLC59108_MIN_LED_CURR 0
#define TLC59108_MAX_LED_CURR 255

#define TLC59108_REG_MODE1    0x00
#define TLC59108_REG_MODE2    0x01

// REG PWM0 to PWM7 is 0x02 to 0x09 for LED0 to LED7
#define TLC59108_REG_PWM(x)  (0x02 + (x))
#define TLC59108_REG_GRPPWM  0x0A
#define TLC59108_REG_GRPFREQ 0x0B

// REG LEDOUT0 0x0C for LED0~3
// REG LEDOUT1 0x0D for LED4~7
#define TLC59108_REG_LEDOUT(x)     (0x0C + (x >> 2))

#define TLC59108_REG_ALLCALLADR    0x11
#define TLC59108_REG_IREF          0x12
#define TLC59108_REG_EFLAG         0x13

#define LDRx_CTRL_OFF              0x0                            /* LED driver off */
#define LDRx_CTRL_ON               (1 << 0)                       /* LED driver fully on */
#define LDRx_CTRL_PWM              (1 << 1)                       /* LED driver brightness can be controlled by PWMx */
#define LDRx_CTRL_PWM_GRPPWM       (LDRx_CTRL_ON | LDRx_CTRL_PWM) /* LED driver brightness can be controlled by PWMx and GRPPWM*/

#define CONVERT_TO_TLC59108_NUM(x) (TLC59108_MAX_LED_NUM - (x)) // MiiS use 1,2,3,4 mapping to Led7,6,5,4

typedef enum {
    LED_TASK_CMD_DEFAULT = 0,
    LED_TASK_CMD_SET_IRLED_ON,
    LED_TASK_CMD_SET_IRLED_OFF,
    LED_TASK_CMD_SET_IR_CURRENT,
    LED_TASK_CMD_GET_IR_CURRENT,
    LED_TASK_CMD_MIIS_SET_IR_CURRENT,
    LED_TASK_CMD_MIIS_GET_IR_CURRENT,
} led_task_cmd_e;

typedef struct {
    led_task_cmd_e ir_cmd;
    INT8U          led_no;
    INT16U         led_cur;
    INT32S         index;
} led_task_message_t;

void   irled_init(void);
void   irled_task_runner(void);
INT32S irled_get_cur(INT8U);
INT32S set_irled_on(INT8U, INT16U);
INT32S set_irled_off(INT8U);
INT32S irled_write_reg(INT8U, INT8U);
INT32S irled_read_reg(INT8U, INT8U*);

#endif
