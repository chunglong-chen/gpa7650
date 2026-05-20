#ifndef __MAX5360_H__
#define __MAX5360_H__

#include "typedef.h"
/*******************************************************************
 * Constants
 *******************************************************************/
#define MAX5360_I2C_CLK_RATE        200
#define MAX5360L_SLAVE_ADDR         0x60

/*******************************************************************
 * External Functions
 *******************************************************************/
/**
  *@brief Sets the MAX5360 DAC output level.
  *@param level 6-bit value (0–63), where each step = VREF / 64.
  *@retval -1 for failure, other values indicate success.
  */
extern INT32S max5360_set_level(INT8U level);
/**
  *@brief Initializes the MAX5360 DAC driver.
  *@param None
  *@retval None
  */
extern void max5360_init(void);

#endif //__MAX5360_H__