/** @file max5360.c
 *  @brief Flash current driver source file
 *
 *  This file contains functions to initialize the I2C_1 interface
 *  and to set the flash current value.
 */
#include "max5360.h"
#include "drv_l1_i2c.h"

/*******************************************************************
 * Constants
 *******************************************************************/
#define FLASH_CURRENT_DRIVER_TAG                        "MAX5360"

// MAX5360 DAC code: 6-bit resolution + 2-bit sub-LSBs
// Valid range: 0x00 (0V) to 0xFC (max ~2V)
// Each step = VREF / 64 = 2V / 64 = 31.25mV
#define MAX5360_MIN_LEVEL (0x00) // 0b000000(00) = 0 × (VREF / 64)
#define MAX5360_MAX_LEVEL (0x3F) // 0b111111(00) = 63 × (VREF / 64)
/*******************************************************************
 * Private Variables
 *******************************************************************/
static drv_l1_i2c_bus_handle_t MAX5360L;

/*******************************************************************
 * Declare Functions
 *******************************************************************/
/**
  *@brief Sends a single byte to MAX5360 via native I2C API.
  *@param data Pointer to the data to write.
  *@retval -1 for failure, other values indicate success.
  */
static INT32S max5360_write_reg(INT8U* data);
/*******************************************************************
 * Private Functions
 *******************************************************************/
/**
  *@brief Sends a single byte to MAX5360 via native I2C API.
  *@param data Pointer to the data to write.
  *@retval -1 for failure, other values indicate success.
  */
static INT32S max5360_write_reg(INT8U* data) {
    INT32S ret;
    ret = drv_l1_i2c_bus_write(&MAX5360L, data, 1);
    return ret;
}

/*******************************************************************
 * Public Functions
 *******************************************************************/
/**
  *@brief Initializes the MAX5360 DAC driver.
  *@param None
  *@retval None
  */
void max5360_init(void) {
    DBG_PRINT("[%s][LOG] init\r\n", FLASH_CURRENT_DRIVER_TAG);
    MAX5360L.devNumber = I2C_1;
    MAX5360L.slaveAddr = MAX5360L_SLAVE_ADDR;
    MAX5360L.clkRate = MAX5360_I2C_CLK_RATE;
    drv_l1_i2c_init(MAX5360L.devNumber);

    max5360_set_level(MAX5360_MIN_LEVEL);
}
/**
  *@brief Sets the MAX5360 DAC output level.
  *@param level 6-bit value (0–63), where each step = VREF / 64.
  *@retval -1 for failure, other values indicate success.
  */
INT32S max5360_set_level(INT8U level) {
    if (level > MAX5360_MAX_LEVEL)
        return -1;

    INT8U value = (level <<2); // MAX5360 expects 8-bit code: 6-bit value + 2 subbits = 00
    DBG_PRINT("[%s][LOG] set %#x\r\n", FLASH_CURRENT_DRIVER_TAG, value);
    return max5360_write_reg(&value);
}