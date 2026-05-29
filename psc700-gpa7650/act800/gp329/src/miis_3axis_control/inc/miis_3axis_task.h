/**
 * @file    miis_3axis_task.h
 * @brief   MIIS 3-axis motor control interface definitions.
 * @ingroup three_axis_controller
 * @details
 * This header file defines command codes, axis masks, status macros,
 * data structures, and public interfaces for the MIIS 3-axis motor
 * control module.
 *
 * It provides the definitions required for movement control, status query,
 * zero-point calibration, center positioning, reset control, and
 * task-based command handling.
 */
#ifndef __MIIS_3AXIS_H__
#define __MIIS_3AXIS_H__

#include "typedef.h"
#include "gpio_init.h"
#include "drv_l1_gpio.h"

#define TCA9535A_I2C_CLK_RATE 200
#define TCA9535A_SLAVE_ADDR   0x88

// i2c commands
#define MIIS_3X_FW_VERSION       0x00
#define MIIS_3X_HOME             0x01
#define MIIS_3X_MOVE_ABS         0x02
#define MIIS_3X_MOVE_REL         0x03
#define SET_TORQUE               0x04
#define MIIS_3X_STATUS           0x05
#define MIIS_3X_POS              0x06
#define MIIS_3X_SET_ZERO         0x07
#define MIIS_3X_SET_SPEED        0x08
#define MIIS_3X_GET_SPEED        0x09
#define MIIS_3X_SET_DIR_LIM      0x0A
#define MIIS_3X_SAVE_SPEED       0x0B
#define MIIS_3X_SAVE_DIR_LIM     0x0C
#define MIIS_3X_SET_NO_TRESPASS  0x0D
#define MIIS_3X_SAVE_NO_TRESPASS 0x0E
#define MIIS_3X_LOCKOFF          0xFD //
#define MIIS_3X_RESET            0xFE //
#define MIIS_3X_MOVE_CENTER      0xFF // command to move home then move x&y to center

#define MIIS_3X_X_AXIS           0x01
#define MIIS_3X_Y_AXIS           0x02
#define MIIS_3X_Z_AXIS           0x04
#define MIIS_3X_MOTOR_MASK       0x07

#define MIIS_3X_IS_BUSY(a)       (a & 0x1) // bit0
#define MIIS_3X_AT_NEAR_LIM(a)   (a & 0x2) // bit1
#define MIIS_3X_AT_FAR_LIM(a)    (a & 0x4) // bit2

#define MIIS_3X_CENTER_AXIS      MIIS_3X_X_AXIS | MIIS_3X_Y_AXIS
#define MIIS_3X_X_CENTER         10000
#define MIIS_3X_Y_CENTER         20000

// typedef enum
// {
//     MIIS_3X_FW_VERSION = 0,
//     MIIS_3X_HOME,
//     MIIS_3X_MOVE_ABS,
//     MIIS_3X_MOVE_REL,
//     MIIS_3X_RESERVED,
//     MIIS_3X_STATUS,
//     MIIS_3X_POS,
//     MIIS_3X_SET_ZERO,
//     MIIS_3X_SET_SPEED,
//     MIIS_3X_GET_SPEED,
//     MIIS_3X_SET_DIR_LIM,
//     MIIS_3X_SAVE_SPEED,
//     MIIS_3X_SAVE_DIR_LIM,
//     MIIS_3X_SET_NO_TRESPASS,
//     MIIS_3X_SAVE_NO_TRESPASS,

// } miis_3axis_cmd_e;

typedef struct {
    CHAR   axes;
    INT32S x;
    INT32S y;
    INT32S z;
} miis_3axis_data_t;

typedef struct {
    INT8S x_dir;
    INT8S x_lim;
    INT8S y_dir;
    INT8S y_lim;
    INT8S z_dir;
    INT8S z_lim;
} miis_3axis_dir_lim_t;

typedef struct
{
    // miis_3axis_cmd_e cmd;
    INT8U             cmd;
    INT32S          index;
    miis_3axis_data_t data;
    miis_3axis_dir_lim_t dirlim;
} miis_3axis_message_t;

void   miis_3axis_show_i2c_reg_info(void);
void   miis_3axis_reset_i2c_bus(void);
INT32S miis_3axis_write_reg(INT8U* data, INT8U size);
INT32S miis_3axis_read_reg(INT8U* data, INT8U size);
INT32S miis_3axis_wait(miis_3axis_data_t data);
INT32S miis_3axis_get_fw_version(const char* char_seq);
INT32S miis_3axis_move_home(miis_3axis_data_t data);
INT32S miis_3axis_move(INT8U cmd, miis_3axis_data_t data);
INT32S miis_3axis_get_pos(const char* seq, miis_3axis_data_t data);
INT32S miis_3axis_set_zero(miis_3axis_data_t data);
INT32S miis_3axis_move_center(void);

miis_3axis_data_t miis_3axis_get_status(miis_3axis_data_t data);

void miis_3axis_init(void);
void miis_3axis_reset(void);
void miis_3axis_task_runner(void);

#endif //__MIIS_3AXIS_H__
