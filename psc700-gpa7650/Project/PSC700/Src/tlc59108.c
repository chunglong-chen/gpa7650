/** @file ir_task.c
 *  @brief IR LED driver source file
 *
 *  This task is responsible for the I2C_1 interface initial and control the TLC59108.
 */
#include "tlc59108.h"
#include "drv_l1_i2c.h"
#include "gplib_print_string.h"

/*******************************************************************
 * Constants
 *******************************************************************/
#define IR_DRV_TAG               "TLC59108"

/*******************************************************************
 * Private Variables
 *******************************************************************/
static drv_l1_i2c_bus_handle_t TLC59108;

/*******************************************************************
 * Declare Functions
 *******************************************************************/
/**
  *@brief  Write a byte to a register of the TLC59108 IR LED driver.
  *@param  reg: Register address to write.
  *@param  value: Data byte to write to the specified register.
  *@retval Result: 0 for success, other values indicate failure.
  */
static INT32S irled_write_reg(INT8U, INT8U);
/**
  *@brief  Read a byte from a register of the TLC59108 IR LED driver.
  *@param  reg: Register address to read from.
  *@param  data: Pointer to store the read data.
  *@retval Result: 0 for success, other values indicate failure.
  */
static INT32S irled_read_reg(INT8U, INT8U*);
/**
  *@brief  Power on the IR LED and set its current.
  *@param  location: 0 for external IR (LED7,LED6,LED5,LED4), 1 for internal IR(LED2).
  *@param  led_cur: LED current (0~255).
  *@retval -1 for failure, other values indicate success.
  */
static INT32S set_irled_on(INT8U location, INT16U led_cur);
/**
  *@brief  Power off the IR LED.
  *@param  location: 0 for external IR, 1 for internal IR, 2 for all IR.
  *@retval -1 for failure, other values indicate success.
  */
static INT32S set_irled_off(INT8U location);

/*******************************************************************
 * Private Functions
 *******************************************************************/
/**
  *@brief  Write a byte to a register of the TLC59108 IR LED driver.
  *@param  reg: Register address to write.
  *@param  value: Data byte to write to the specified register.
  *@retval Result: 0 for success, other values indicate failure.
  */
static INT32S irled_write_reg(INT8U reg, INT8U value) {
    INT32S ret;
    ret = drv_l1_reg_1byte_data_1byte_write(&TLC59108, reg, value);
    return ret;
}

/**
  *@brief  Read a byte from a register of the TLC59108 IR LED driver.
  *@param  reg: Register address to read from.
  *@param  data: Pointer to store the read data.
  *@retval Result: 0 for success, other values indicate failure.
  */
static INT32S irled_read_reg(INT8U reg, INT8U* data) {
    INT32S ret;
    ret = drv_l1_reg_1byte_data_1byte_read(&TLC59108, reg, data);
    return ret;
}

/**
  *@brief  Power on the IR LED and set its current.
  *@param  location: 0 for external IR (LED7,LED6,LED5,LED4), 1 for internal IR(LED2).
  *@param  led_cur: LED current (0~255).
  *@retval -1 for failure, other values indicate success.
  */
static INT32S set_irled_on(INT8U location, INT16U led_cur) // location:0 or 1; led_cur:0~255
{
    INT8U  value;
    INT8U  reg_ledout;
    INT32S ret = -1;

    if (location >= LED_TASK_LOCATION_MAX)
        DBG_PRINT("[%s]IR LED out of number\r\n", IR_DRV_TAG);
    else if (led_cur > TLC59108_MAX_LED_CURR)
        DBG_PRINT("[%s]IR LED current out of range\r\n", IR_DRV_TAG);
    else {
        if (location == LED_TASK_LOCATION_EXTERNAL) {
            irled_write_reg(TLC59108_REG_PWM(7), led_cur);
            irled_write_reg(TLC59108_REG_PWM(6), led_cur);
            irled_write_reg(TLC59108_REG_PWM(5), led_cur);
            irled_write_reg(TLC59108_REG_PWM(4), led_cur);
            ret = irled_write_reg(MIIS_EXTERNAL_IR_LED_REG, ((LDRx_CTRL_PWM << 6)|(LDRx_CTRL_PWM << 4)|(LDRx_CTRL_PWM << 2)|(LDRx_CTRL_PWM << 0))); // LED7, LED6, LED5, LED4 on
        } else {
            irled_write_reg(TLC59108_REG_PWM(2), led_cur); // Set PWM2
            ret = irled_write_reg(MIIS_INTERNAL_IR_LED_REG, (LDRx_CTRL_PWM << 4)); // LED2 Individual mode
        }
    }

    return ret;
}

/**
  *@brief  Power off the IR LED.
  *@param  location: 0 for external IR, 1 for internal IR, 2 for all IR.
  *@retval -1 for failure, other values indicate success.
  */
static INT32S set_irled_off(INT8U location)
{
    INT8U  value;
    INT8U  reg_ledout;
    INT32S ret = -1;

    if (location >= LED_TASK_LOCATION_MAX)
        DBG_PRINT("[%s]IR LED out of number\r\n", IR_DRV_TAG);
    else {
        if (location == LED_TASK_LOCATION_ALL) {
            ret = irled_write_reg(MIIS_INTERNAL_IR_LED_REG, 0x00); // LED2 off
            ret = irled_write_reg(MIIS_EXTERNAL_IR_LED_REG, 0x00); // LED7,LED6,LED5,LED4 off
        } else if (location == LED_TASK_LOCATION_EXTERNAL) {
            ret = irled_write_reg(MIIS_EXTERNAL_IR_LED_REG, 0x00); // LED7,LED6,LED5,LED4 off
        } else {
            ret = irled_write_reg(MIIS_INTERNAL_IR_LED_REG, 0x00); // LED2 off
        }
    }
    return ret;
}
/*******************************************************************
 * Public Functions
 *******************************************************************/
/**
  *@brief Init TLC59108
  *@param None
  *@retval None
  */
void irled_init(void) {
    DBG_PRINT("[%s]IR LED init\r\n", IR_DRV_TAG);
    TLC59108.devNumber = I2C_1;
    TLC59108.slaveAddr = TLC59108_SLAVE_ADDR;
    TLC59108.clkRate = TLC59108_I2C_CLK_RATE;
    drv_l1_i2c_init(TLC59108.devNumber);
    irled_write_reg(TLC59108_REG_MODE1, 0x01); // Normal mode
}

/**
  *@brief  Control IR LED.
  *@param  location: 0 for external IR, 1 for internal IR.
  *@param  led_cur: LED current (0~255).
  *@retval Result: 0 for success, other values indicate failure.
  */
INT8U irled_ctrl(INT8U location, INT16U led_cur)
{
    INT32S ret;
    if (location >= LED_TASK_LOCATION_MAX) return 1;
    if (led_cur > TLC59108_MAX_LED_CURR) return 1;

    if (led_cur > 0)
        ret = set_irled_on(location, led_cur);
    else
        ret = set_irled_off(location);

    if (ret < 0)
        return 1;
    else
        return 0;

}
