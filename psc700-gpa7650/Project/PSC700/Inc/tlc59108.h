#ifndef __TLC59108_H__
#define __TLC59108_H__

#include "typedef.h"

/*******************************************************************
 * Constants
 *******************************************************************/
#define TLC59108_I2C_CLK_RATE 200
#define TLC59108_SLAVE_ADDR   (0x90)

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

#define TLC59108_REG_LEDOUT0       0x0C
#define TLC59108_REG_LEDOUT1       0x0D
// REG LEDOUT0 0x0C for Internal IR
// REG LEDOUT1 0x0D for External IR
#define MIIS_EXTERNAL_IR_LED_REG TLC59108_REG_LEDOUT1
#define MIIS_INTERNAL_IR_LED_REG TLC59108_REG_LEDOUT0

#define TLC59108_REG_ALLCALLADR    0x11
#define TLC59108_REG_IREF          0x12
#define TLC59108_REG_EFLAG         0x13

#define LDRx_CTRL_OFF              0x0                            /* LED driver off */
#define LDRx_CTRL_ON               (1 << 0)                       /* LED driver fully on */
#define LDRx_CTRL_PWM              (1 << 1)                       /* LED driver brightness can be controlled by PWMx */
#define LDRx_CTRL_PWM_GRPPWM       (LDRx_CTRL_ON | LDRx_CTRL_PWM) /* LED driver brightness can be controlled by PWMx and GRPPWM*/
/*******************************************************************
 * Data Types
 *******************************************************************/
typedef enum {
    LED_TASK_LOCATION_EXTERNAL = 0,
    LED_TASK_LOCATION_INTERNAL,
    LED_TASK_LOCATION_ALL,
    LED_TASK_LOCATION_MAX,
} led_location_e;

/*******************************************************************
 * External Functions
 *******************************************************************/
/**
  *@brief Init TLC59108
  *@param None
  *@retval None
  */
extern void irled_init(void);
/**
  *@brief  Control IR LED.
  *@param  location: 0 for external IR, 1 for internal IR.
  *@param  led_cur: LED current (0~255).
  *@retval Result: 0 for success, other values indicate failure.
  */
INT8U irled_ctrl(INT8U location, INT16U led_cur);

#endif //__TLC59108_H__
