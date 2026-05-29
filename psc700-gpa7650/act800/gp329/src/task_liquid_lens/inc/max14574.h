#ifndef __MAX14574_H__
#define __MAX14574_H__

#include "typedef.h"

#define MAX14574_I2C_CLK_RATE 200
#define MAX14574_SLAVE_ADDR   0xEE // 0xEE for write; 0xEF for read

/**
 *      Liquid Lens Circuit
 *           |
 *           |  MAX14574
 *           |
 * V2   o----|TM2
 *      |    |
 *      Rt   |
 *      |    |
 * V1   o----|TM1
 *      |    |____________
 *      R1(82KΩ)
 *      |
 *      Ground
 *
 *      V1/V2 = R1/(Rt+R1)
 *      Rt = R1/(V1/V2)-R1
 *      Note: V1/V2 will be read from MAX14574_REG_TEMPSENSE
 */

#define MIIS_RESISTOR    (float)(82 * 1000) // MiiS use 82KΩ

#define RESISTOR_TEMP(d) (MIIS_RESISTOR / d - MIIS_RESISTOR) // Calculate Rt,d = val /255.0; val is read from MAX14574_REG_TEMPSENSE
// T(K)=T(°C)+273.15
#define TEMPERATRUE_K1        (25 + 273.15)    // K1 = 25(°C)+273.15
#define RESISTOR_K1           (100 * 1000)     // R1 = 100KΩ at 25(°C)
#define THERMISTOR_B_CONSTANT (4330)           // B25/50 = 4330
#define LN_R1                 log(RESISTOR_K1) // ln(100K)

// Equation w/o resistor
#define EQU1_S(T)   (float)(2.536e-5 * pow(T, 2) - 2.347e-3 * T + 9.256e-1)
#define EQU1_V0D(T) (float)(-8.548e-4 * pow(T, 2) - 8.229e-3 * T + 4.556 * 10)

// Equation with 5KΩ resistor from Spec
#define EQU2_S(T)       (float)(2.539e-5 * pow(T, 2) - 2.283e-3 * T + 9.401e-1)
#define EQU2_V0D(T)     (float)(-7.453e-4 * pow(T, 2) - 9.007e-3 * T + 4.785 * 10)
#define V_TO_VRMS(V)    (float)((V - 24.4) / 0.0442)  // 0.0442V per step
#define VRMS_TO_V(VRMS) (float)(VRMS * 0.0442 + 24.4)

// Read-Only Register
#define MAX14574_REG_STATUS    0x00
#define STATUS_LL_TH           (1 << 4)
#define STATUS_BST_FAIL        (1 << 2)

#define MAX14574_REG_FAIL      0x01
#define FAIL_OP                (1 << 6)
#define FAIL_SH                (1 << 5)
#define FAIL_COM_FAIL          (1 << 4)
#define FAIL_LL4_FAIL          (1 << 3)
#define FAIL_LL3_FAIL          (1 << 2)
#define FAIL_LL2_FAIL          (1 << 1)
#define FAIL_LL1_FAIL          (1 << 0)

#define MAX14574_REG_TEMPSENSE 0x02

// Read/Write Register
#define MAX14574_REG_USERMODE   0x03
#define USERMODE_ACTIVE         (1 << 1)
#define USERMODE_SM             (1 << 0) // Sleep Mode

#define MAX14574_REG_OIS_LSB    0x04
#define OIS_LSB_MASK            0x0003
#define MAX14574_REG_LLV1       0x05
#define MAX14574_REG_LLV2       0x06
#define MAX14574_REG_LLV3       0x07
#define MAX14574_REG_LLV4       0x08

#define MAX14574_REG_COMMAND    0x09
#define COMMAND_UPD_OUT         (1 << 1)
#define COMMAND_CHK_FAIL        (1 << 0)

#define MAX14574_REG_DRIVERCONF 0x0A
#define TS_INT                  (1 << 7) // Internal temp-sensor enable bit
#define TS_EXT                  (1 << 6) // External temp-sensor enable bit

/* Definition of min and max voltage : 24.4V and 70V */
#define MAX14574_MIN_VOLT 0    // 0x000
#define MAX14574_MAX_VOLT 1023 // 0x3FF (10bits)

#define DIOPTER_MAX       17
#define DIOPTER_HOME      0
#define DIOPTER_MIN       (-17)

typedef enum {
    LL_TASK_CMD_DEFAULT = 0,
    LL_TASK_CMD_SET_VOLTAGE,
    LL_TASK_CMD_GET_VOLTAGE,
    LL_TASK_CMD_GET_STATUS,
    LL_TASK_CMD_GET_CHIP_VERSION,
    LL_TASK_CMD_GET_FAIL,
    LL_TASK_CMD_ENABLE,
    LL_TASK_CMD_DISABLE,
    LL_TASK_CMD_HOME,
    LL_TASK_CMD_GOTO_ABS, // move to absolute
    LL_TASK_CMD_GOTO_REL, // move to relative
    LL_TASK_CMD_GET_POSITION,
    LL_TASK_CMD_GET_TEMPERATURE,
    LL_TASK_CMD_GOTO_ABS_WITH_TEMP_CALIBRATION,
    LL_TASK_CMD_AS_MODE,
} ll_task_cmd_e;

typedef struct
{
    ll_task_cmd_e ll_cmd;
    INT16U        voltage;
    INT8S         diopter;
} ll_task_message_t;

INT32S max14574_write_reg(INT8U reg, INT8U data);
INT32S max14574_read_reg(INT8U reg, INT8U* out_data);
INT8S  max14574_set_voltage(INT16U voltage);
INT16S max14574_get_voltage(void);
void   max14574_get_chip_version(void);
INT8U  max14574_get_status(void);
INT8U  max14574_get_fail(void);
void   max14574_setup(void);
void   max14574_init(void);
void   max14574_task_runner(void);
#endif //__MAX14574_H__
