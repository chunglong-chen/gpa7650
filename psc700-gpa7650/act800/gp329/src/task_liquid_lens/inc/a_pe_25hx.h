/**
 * @file    a_pe_25hx.h
 * @brief   Liquid Lens (A_PE_25HX) control interface definitions.
 * @ingroup liquid_lens_controller
 * @details
 * This header file defines the interface, data structures, and command
 * types for controlling the A_PE_25HX liquid lens module.
 *
 * It provides:
 * - Task message definitions for liquid lens control
 * - Command enumeration used by the task dispatcher
 * - External task entry function
 *
 * The module supports:
 * - Optical power (diopter) control
 * - Voltage (VRMS) control
 * - Mode switching (OP mode / voltage mode)
 * - Status and diagnostic queries
 *
 * The control is performed through a message queue mechanism,
 * enabling asynchronous command processing.
 */
#ifndef __A_PE_25HX_H__
#define __A_PE_25HX_H__

#include "typedef.h"

#define A_PE_25HX_I2C_CLK_RATE 200
#define A_PE_25HX_SLAVE_ADDR   0x30 // 0x30 for write; 0x31 for read

#define MIN_VRMS               24.0
#define MAX_VRMS               70.0
#define CODE_TO_VRMS(N)        (float)((N * 0.001) + MIN_VRMS)
#define VRMS_TO_CODE(V)        ((V - MIN_VRMS) / 0.001)

// Liquid Lens Diopter
#define A_PE_25HX_DIOPTER_MAX 10.0
#define A_PE_25HX_DIOPTER_MIN (-5.0)

// Eye Diopter
#define DIOPTER_MAX  17
#define DIOPTER_HOME 0
#define DIOPTER_MIN  (-17)

/* The focus value is a 16 bit integer corresponding to the following Vrms value:
Code 0x0000 = 24Vrms
…
Code 0xB3B0 = 70Vrms
Vrms = N x 0.001 + 24 (in Volts)
where N = code from 0x 0000 to 0x B3B0
Please note that the voltage will be adjusted to the closest voltage step allowed by Maxim MAX14574 IC (44mV resolution step). Maxim MAX14574 has a 10-bit resolution
*/
#define REG_FOCUS   0x0000

#define REG_VERSION 0x0004 // Software version
#define REG_TEMP    0x000A // Temperature (float value)
#define REG_OPWR    0x001E // Optical power value when liquid lens is controlled using Optical Power and not voltage (refer to Control mode register 0x620)
#define REG_V_TEMP  0x0028 // This bit, if set, indicates V-temp feature is running
#define REG_V_SPEED 0x0080 // This bit, if set, indicates V-speed feature is running
// B0 This bit, if set, indicates sweep 1 is enabled
// B1 This bit, if set, indicates sweep 2 is enabled
#define REG_V_SWEEP                        0x0600
#define REG_V_SWEEP1_INIT_V                0x0601
#define REG_V_SWEEP1_DELTA_V               0x0602
#define REG_V_SWEEP1_FINAL_V               0x0603
#define REG_V_SWEEP1_DELTA_V_MS            0x0604

#define REG_V_SWEEP2_INIT_V                0x0605
#define REG_V_SWEEP2_DELTA_V               0x0606
#define REG_V_SWEEP2_FINAL_V               0x0607
#define REG_V_SWEEP2_DELTA_V_MS            0x0608
#define REG_V_SWEEP_DELAY_BETWEEN_SWEEP_MS 0x0609 // Delay in ms between sweep 1 and 2 with 100 μs precision (float)

#define REG_STATUS                         0x0618
#define REG_FAIL                           0x0619
#define REG_TEMP_SENSE                     0x061A
#define REG_USERMODE                       0x061B
#define REG_COMMAND                        0x061C
#define REG_DRIVERCONF                     0x061D
#define REG_OIS_LSB                        0x061E
#define REG_LLV                            0x061F

// If this register equals one, liquid lens is controlled using Optical Power using register 0x001E.
// For other values it is controlled using voltage using register 0x0000 (default mode).
#define REG_CONTROL_MODE 0x0620

typedef struct
{
    INT8S  eye_diopter; // Eye Diopter (D)
    float  ll_diopter;  // Liquid Lens Diopter (D)
    double voltage;     // Liquid Lens Voltage (V)

} lens_data_t;

typedef enum {
    A_PE_25HX_TASK_CMD_DEFAULT = 0,
    A_PE_25HX_TASK_CMD_GET_VERSION,
    A_PE_25HX_TASK_CMD_SET_VOLTAGE,
    A_PE_25HX_TASK_CMD_GET_VOLTAGE,
    A_PE_25HX_TASK_CMD_GET_OPTICAL_POWER,
    A_PE_25HX_TASK_CMD_HOME,
    A_PE_25HX_TASK_CMD_GOTO_ABS, // move to absolute
    A_PE_25HX_TASK_CMD_GOTO_REL, // move to relative
    A_PE_25HX_TASK_CMD_GET_POSITION,
    A_PE_25HX_TASK_CMD_OP_MODE_CTRL,
    A_PE_25HX_TASK_CMD_V_TEMP_CTRL,
    A_PE_25HX_TASK_CMD_V_SPEED_CTRL,
    A_PE_25HX_TASK_CMD_GET_OP_MODE_STATUS,
    A_PE_25HX_TASK_CMD_GET_V_TEMP_STATUS,
    A_PE_25HX_TASK_CMD_GET_V_SPEED_STATUS,
    A_PE_25HX_TASK_CMD_AS_MODE,
    A_PE_25HX_TASK_CMD_GET_TEMPERATURE,
    A_PE_25HX_TASK_CMD_GET_STATUS,
    A_PE_25HX_TASK_CMD_GET_FAIL,
} a_pe_25hx_task_cmd_e;

typedef struct
{
    a_pe_25hx_task_cmd_e ll_cmd;
    double      voltage; // Use double type for higher precision
    INT8S       diopter;
    INT8U       enable;
    INT32U      diopter_hex;
    INT32U      index;
} a_pe_25hx_task_message_t;

void a_pe_25hx_task_runner(void);
#endif //__A_PE_25HX_H__
