#ifndef __I2C_CMD_H__
#define __I2C_CMD_H__

#include "stdio.h"
#include "stdarg.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "limits.h"
#include "NuMicro.h"
#include "uart_cmd.h"
#include "motor_ctrl.h"
#include "user_config.h"
#include "main.h"
#include "version.h"
#include "flash.h"

#define ClearI2C0DataBuffer(buf) memset(buf,0,sizeof(buf));

#define ACK                 true
#define NACK               false
#define MOTORADDR_1         0x01
#define MOTORADDR_2         0x02
#define MOTORADDR_3         0x04
#define MOTORID_1              0
#define MOTORID_2              1
#define MOTORID_3              2
#define PACK_STS_AND_POS       1
#define PACK_BUSY              2
#define PACK_POS               3
#define PACK_VERSION           4
#define GET_VERSION          0x0
#define STEP_MOVE_HOME       0x1
#define STEP_MOVE_ABS        0x2
#define STEP_MOVE_REL        0x3
#define GET_STEP_STS         0x4
#define GET_STEP_BUSY        0x5
#define GET_STEP_POS         0x6
#define RESET_MOTOR_POS      0x7
#define MOTOR_CONFIG_SPEED   0x8
#define GET_MOTOR_CONFIG     0x9
#define SET_MOTOR_DIR        0xA
#define RESERVED_0XB         0xB //Not in use
#define SAVE_MOTOR_PARA      0xC
#define SET_NO_TRESPASS      0xD
#define SAVE_NO_TRESPASS     0xE

#endif /*__I2C_CMD_H__*/