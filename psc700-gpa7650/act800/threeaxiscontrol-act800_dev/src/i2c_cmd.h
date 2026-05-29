/**
 * @file  i2c_cmd.h
 * @ingroup i2c_command_parser
 * @brief Declarations I2C commands.
 */
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
#define PACK_STS_AND_POS       1                /**< I2C command to package status and position */
#define PACK_BUSY              2                /**< I2C command to package status*/
#define PACK_POS               3                /**< I2C command to package position */
#define PACK_VERSION           4                /**< I2C command to package version */
#define GET_VERSION          0x0                /**< I2C command to get FW version */
#define STEP_MOVE_HOME       0x1                /**< I2C command to move motor to home position */
#define STEP_MOVE_ABS        0x2                /**< I2C command to move motor to an absolute position */
#define STEP_MOVE_REL        0x3                /**< I2C command to move motor to a relative position */
#define SET_TORQUE           0x4                /**< I2C command to set dribing torque */
#define GET_STEP_BUSY        0x5                /**< I2C command to get motor status */
#define GET_STEP_POS         0x6                /**< I2C command to get motor position */
#define RESET_MOTOR_POS      0x7                /**< I2C command to reset motor position */
#define MOTOR_CONFIG_SPEED   0x8                /**< I2C command to set motor speed */
#define GET_MOTOR_CONFIG     0x9                /**< I2C command to get motor configuration */
#define SET_MOTOR_DIR        0xA                /**< I2C command to set motor direction and limit switch */
#define SAVE_TORQUE          0xB                /**< I2C command to save driving torque */
#define SAVE_MOTOR_PARA      0xC                /**< I2C command to save user config */
#define SET_NO_TRESPASS      0xD                /**< I2C command to set no trespass border */
#define SAVE_NO_TRESPASS     0xE                /**< I2C command to save no trespass border */

#endif /*__I2C_CMD_H__*/