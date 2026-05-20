#ifndef __WT7026_H__
#define __WT7026_H__

#include "typedef.h"
/*******************************************************************
 * Constants
 *******************************************************************/
// Define SPI configuration
#define MOTOR_EN_PIN                            IO_F3
#define MOTOR_LIMIT_SWITCH_PIN                  IO_F7
#define SPI_CH                                  SPI_1

// WT7026 Register Addresses
#define WT7026_REG_CH1_2_SERIAL_CTL             0x00
#define WT7026_REG_DISABLE_CH1_2                0x02
#define WT7026_REG_DAC_CH1_2                    0x03
#define WT7026_PI_DRV_CTRL                      0x07

// WT7026 System Control Values
#define WT7026_SYS_EN                           0x10
#define WT7026_SYS_DISABLE                      0x00
#define WT7026_CH1_CH2_DAC_3V4_CONST            0x0E

#define SET_DAC_VAL(val)    ((val) << 1)

#define WT7026_CH1_CH2_DAC_RAW_4V8    0x0
#define WT7026_CH1_CH2_DAC_RAW_4V6    0x1
#define WT7026_CH1_CH2_DAC_RAW_4V4    0x2
#define WT7026_CH1_CH2_DAC_RAW_4V2    0x3
#define WT7026_CH1_CH2_DAC_RAW_4V0    0x4
#define WT7026_CH1_CH2_DAC_RAW_3V8    0x5
#define WT7026_CH1_CH2_DAC_RAW_3V6    0x6
#define WT7026_CH1_CH2_DAC_RAW_3V4    0x7
#define WT7026_CH1_CH2_DAC_RAW_3V2    0x8
#define WT7026_CH1_CH2_DAC_RAW_3V0    0x9
#define WT7026_CH1_CH2_DAC_RAW_2V8    0xA
#define WT7026_CH1_CH2_DAC_RAW_2V6    0xB
#define WT7026_CH1_CH2_DAC_RAW_2V4    0xC
#define WT7026_CH1_CH2_DAC_RAW_2V2    0xD
#define WT7026_CH1_CH2_DAC_RAW_2V0    0xE
#define WT7026_CH1_CH2_DAC_RAW_1V8    0xF
#define WT7026_REG_PS_PI_RESET_INIT             0x12

#define PHASE_COUNT_PER_STEP                    4
#define MAX_STEPS                               (1400/PHASE_COUNT_PER_STEP)
#define TIMEOUT_STEPS                           (1420/PHASE_COUNT_PER_STEP)
// Finish home: leave limit switch by fixed steps
#define PULLOFF_STEPS                           (16/PHASE_COUNT_PER_STEP)


#define POS_0D                                  450

/*******************************************************************
 * Data Types
 *******************************************************************/
typedef enum {
    MOTOR_TASK_CMD_DEFAULT = 0,
    MOTOR_TASK_CMD_HOME,
    MOTOR_TASK_CMD_MOVE,
    MOTOR_TASK_CMD_GOTO,
} MotorTask_cmd_e;

typedef struct{
    INT8U motor_cmd;
    INT32S position;
}MotorManualParam_Typedef;

typedef struct {
    INT32S cur_pos;
    INT32S step_phase;   /* Current 8‑phase index 0‑7 */
} actuator_s;

/*******************************************************************
 * Global variables
 *******************************************************************/
extern actuator_s  wt7026_act;

/*******************************************************************
 * External Functions
 *******************************************************************/
/**
  *@brief Initializes the WT7026 actuator driver.
  *@param None
  *@retval None
  */
extern void wt7026_init(void);
/**
  *@brief Uninitializes the WT7026 actuator driver.
  *@param None
  *@retval None
  */
extern void wt7026_uninit(void);
/**
  *@brief Moves the actuator to the home position by driving it until it touches the limit switch.
  *@param None
  *@retval The total number of steps moved during the homing process.
  */
extern INT32U wt7026_home(void);
/**
  *@brief Moves the actuator to a specified position
  *@param rel_pos: The relative position to move the actuator to
  *@retval Result: 0 for success, other values indicate failure.
  */
extern INT8U wt7026_move(INT32S rel_pos);
/**
  *@brief Moves the actuator to an absolute position by invoking wt7026_move()
  *@param abs_pos: The target absolute position to move the actuator to
  *@retval Result: 0 for success, other values indicate failure.
  */
extern INT8U wt7026_goto(INT32S abs_pos);

#endif //__WT7026_H__
