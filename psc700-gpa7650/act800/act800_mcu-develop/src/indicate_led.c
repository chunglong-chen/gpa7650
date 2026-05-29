/**
 * @file    indicate_led.c
 * @brief   Indicator LED control module using LP5012 via I2C.
 * @details This module implements control of indicator LEDs through the
 *          LP5012 LED driver over I2C interface. It provides APIs for
 *          setting LED brightness, color, and enable state.
 *
 *          The module uses a state machine to perform initialization and
 *          configuration of the LP5012 device due to hardware timing
 *          constraints. I2C transactions are synchronized using callback
 *          flags to ensure proper sequencing.
 *
 *          Key functionalities:
 *          - LP5012 register access via I2C
 *          - LED brightness control
 *          - RGB color control
 *          - LED enable/disable control
 *          - Initialization state machine
 * @ingroup indicator_led_controller
 */
#include "indicate_led.h"
#include "config/default/peripheral/port/plib_port.h"
#include "peripheral/systick/plib_systick.h"
#include "peripheral/sercom/i2c_master/plib_sercom5_i2c_master.h"

INDICATE_LED_DATA indicate_led_data;
/**
 * @brief   Write a register value to LP5012 LED driver via I2C.
 * @details Sends a register address and corresponding value to the LP5012
 *          device using I2C communication. The function blocks until the
 *          transfer is completed and checks for transfer errors.
 *
 * @param[in] reg    Register address.
 * @param[in] value  Value to be written to the register.
 *
 * @return true  If the write operation is successful.
 * @return false If an I2C error occurs.
 */
bool lp5012_write_reg(uint8_t reg, uint8_t value) {
    bool ret = true;
    //    printf("[i2c] %x, %x\r\n", reg, value);
    indicate_led_data.txBuffer[0] = reg;
    indicate_led_data.txBuffer[1] = value;
    SERCOM5_I2C_Write(LP5012_BASE, &indicate_led_data.txBuffer[0], 2);

    while (SERCOM5_I2C_IsBusy());

    /* Check if any error occurred */
    if (SERCOM5_I2C_ErrorGet() == SERCOM_I2C_ERROR_NONE) {
        //Transfer is completed successfully.
    } else {
        //Error occurred during transfer.
        ret = false;
    }

    return ret;
}

/**
 * @brief   Set brightness level of specified LED.
 * @details Adjusts the brightness of the selected LED channel by writing
 *          to the corresponding LP5012 brightness register.
 *
 * @param[in] led_id  LED index (1~3).
 * @param[in] level   Brightness level (0~255).
 *
 * @return true  If operation is successful.
 * @return false If input is invalid or write fails.
 */
bool indicate_led_level(uint8_t led_id, uint16_t level) {
    bool ret = true;
    if (level > 0xFF) {
//        printf("invalid level %d\r\n", level);
        ret = false;
        return ret;
    }
    switch (led_id) {
        case INDEX_INDICATE_LED1:
            ret = lp5012_write_reg(LP5012_REG_LED0_BRIGHTNESS, level);
            break;
        case INDEX_INDICATE_LED2:
            ret = lp5012_write_reg(LP5012_REG_LED1_BRIGHTNESS, level);
            break;
        case INDEX_INDICATE_LED3:
            ret = lp5012_write_reg(LP5012_REG_LED2_BRIGHTNESS, level);
            break;
        default:
            ret = false;
            break;
    }
    return ret;
}

/**
 * @brief   Set RGB color of specified LED.
 * @details Configures the red, green, and blue intensity of the selected
 *          LED by writing to corresponding LP5012 color registers.
 *
 * @param[in] led_id  LED index (1~3).
 * @param[in] red     Red intensity (0~255).
 * @param[in] green   Green intensity (0~255).
 * @param[in] blue    Blue intensity (0~255).
 *
 * @return true  If operation is successful.
 * @return false If input is invalid or write fails.
 */
bool indicate_led_color(uint8_t led_id, uint16_t red, uint16_t green, uint16_t blue) {
    bool ret = true;
    if (red > 0xFF || green > 0xFF || blue > 0xFF) {
        ret = false;
        return ret;
    }
    //    printf("(r,g,b) = (%d, %d, %d)\r\n", red, green, blue);
    switch (led_id) {
        case INDEX_INDICATE_LED1:
            ret = lp5012_write_reg(LP5012_REG_OUT0_COLOR, red);
            ret &= lp5012_write_reg(LP5012_REG_OUT1_COLOR, green);
            ret &= lp5012_write_reg(LP5012_REG_OUT2_COLOR, blue);
            break;
        case INDEX_INDICATE_LED2:
            ret = lp5012_write_reg(LP5012_REG_OUT3_COLOR, red);
            ret &= lp5012_write_reg(LP5012_REG_OUT4_COLOR, green);
            ret &= lp5012_write_reg(LP5012_REG_OUT5_COLOR, blue);
            break;
        case INDEX_INDICATE_LED3:
            ret = lp5012_write_reg(LP5012_REG_OUT6_COLOR, red);
            ret &= lp5012_write_reg(LP5012_REG_OUT7_COLOR, green);
            ret &= lp5012_write_reg(LP5012_REG_OUT8_COLOR, blue);
            break;
        default:
            ret = false;
            break;
    }
    return ret;
}

/**
 * @brief   Enable or disable specified LED.
 * @details Enables or disables the LED by setting its brightness level.
 *          When enabled, a default brightness is applied; when disabled,
 *          brightness is set to zero.
 *
 * @param[in] led_id  LED index (1~3).
 * @param[in] enable  true to enable, false to disable.
 *
 * @return true  If operation is successful.
 * @return false If LED ID is invalid.
 */
bool indicate_led_enable(uint8_t led_id, bool enable) {
    bool ret = true;
    switch (led_id) {
        case INDEX_INDICATE_LED1:
        case INDEX_INDICATE_LED2:
        case INDEX_INDICATE_LED3:
            ret = indicate_led_level(led_id, enable ? LED_DEFAULT_BRIGHTNESS : 0);
            break;
        default:
            ret = false;
            break;
    }
    return ret;
}

/**
 * @brief   I2C transfer callback handler.
 * @details This callback is invoked upon completion of an I2C transfer.
 *          It updates the transfer status flag based on the error state.
 *
 * @param[in] context  User-defined context (unused).
 */
void SERCOM_I2C_Callback(uintptr_t context) {

    if (SERCOM5_I2C_ErrorGet() == SERCOM_I2C_ERROR_NONE) {
        indicate_led_data.isTransferDone = true;
    } else {
        indicate_led_data.isTransferDone = false;
    }
}

/**
 * @brief   Initialize indicator LED module.
 * @details Initializes internal state machine, registers I2C callback,
 *          and enables the LED driver hardware by setting the enable pin.
 */
void INDICATE_LED_Initialize(void) {

    indicate_led_data.state = INDICATE_LED_STATE_INIT;
    indicate_led_data.isTransferDone = false;
    SERCOM5_I2C_CallbackRegister(SERCOM_I2C_Callback, (uintptr_t) NULL);
    LED_EN_Set(); // pull LED_EN pin high
}

uint8_t retry_cnt = 0;

/**
 * @brief   Indicator LED task state machine.
 * @details Implements a state machine to initialize and control the
 *          LP5012 LED driver. Due to hardware timing constraints,
 *          I2C initialization is performed within this task instead of
 *          the initialization function.
 *
 *          State flow:
 *          - INIT: Enable LED driver chip
 *          - WAIT_CHIP_ENABLE: Wait for chip enable completion
 *          - WAIT_LEDx_BRIGHTNESS: Set brightness for each LED
 *          - WAIT_LEDx_COLOR: Set color for each LED
 *          - IDLE: Initialization complete
 *          - ERROR: Initialization failure
 *

 */
/* For some unknown reason, can't send I2C data in the INDICATE_LED_Initialize() function.
 * Therefore, use a state machine within a task to initialize the indicator LED. */
void INDICATE_LED_Tasks(void) {
    switch (indicate_led_data.state) {
        case INDICATE_LED_STATE_INIT:
            // Enable chip
            lp5012_write_reg(LP5012_REG_DEV_CONFIG0, LP5012_REG_DEV_CONFIG0_CHIP_ENABLE);
            indicate_led_data.state = INDICATE_LED_STATE_WAIT_CHIP_ENABLE;
            break;
        case INDICATE_LED_STATE_WAIT_CHIP_ENABLE:
            if (indicate_led_data.isTransferDone) {
                // Set LED1 (D5) brightness
                indicate_led_data.isTransferDone = false;
                indicate_led_level(INDEX_INDICATE_LED1, LED_DEFAULT_BRIGHTNESS);
                indicate_led_data.state = INDICATE_LED_STATE_WAIT_LED1_BRIGHTNESS;
            } else
                retry_cnt++;

            if (retry_cnt > 10) {
//                printf("LP5012 init failed\r\n");
                indicate_led_data.state = INDICATE_LED_STATE_ERROR;
            }
            break;
        case INDICATE_LED_STATE_WAIT_LED1_BRIGHTNESS:
            if (indicate_led_data.isTransferDone) {
                // Set LED2 (D6) brightness
                indicate_led_data.isTransferDone = false;
                indicate_led_level(INDEX_INDICATE_LED2, LED_DEFAULT_BRIGHTNESS);
                indicate_led_data.state = INDICATE_LED_STATE_WAIT_LED2_BRIGHTNESS;
            }
            break;
        case INDICATE_LED_STATE_WAIT_LED2_BRIGHTNESS:
            if (indicate_led_data.isTransferDone) {
                // Set LED3 (D7) brightness
                indicate_led_data.isTransferDone = false;
                indicate_led_level(INDEX_INDICATE_LED3, LED_DEFAULT_BRIGHTNESS);
                indicate_led_data.state = INDICATE_LED_STATE_WAIT_LED3_BRIGHTNESS;
            }
            break;
        case INDICATE_LED_STATE_WAIT_LED3_BRIGHTNESS:
            if (indicate_led_data.isTransferDone) {
                // Set LED1 (D5) color white
                indicate_led_data.isTransferDone = false;
                indicate_led_color(INDEX_INDICATE_LED1, LED_DEFAULT_BRIGHTNESS, LED_DEFAULT_BRIGHTNESS, LED_DEFAULT_BRIGHTNESS);
                indicate_led_data.state = INDICATE_LED_STATE_WAIT_LED1_COLOR;
            }
            break;
        case INDICATE_LED_STATE_WAIT_LED1_COLOR:
            if (indicate_led_data.isTransferDone) {
                // Set LED2 (D6) color blue
                indicate_led_data.isTransferDone = false;
                indicate_led_color(INDEX_INDICATE_LED2, 0, 0, LED_DEFAULT_BRIGHTNESS);
                indicate_led_data.state = INDICATE_LED_STATE_WAIT_LED2_COLOR;
            }
            break;
        case INDICATE_LED_STATE_WAIT_LED2_COLOR:
            if (indicate_led_data.isTransferDone) {
                // Set LED3 (D7) color green
                indicate_led_data.isTransferDone = false;
                indicate_led_color(INDEX_INDICATE_LED3, 0, LED_DEFAULT_BRIGHTNESS, 0);
                indicate_led_data.state = INDICATE_LED_STATE_WAIT_LED3_COLOR;
            }
            break;
        case INDICATE_LED_STATE_WAIT_LED3_COLOR:
            if (indicate_led_data.isTransferDone) {
                // Indicate LED initialization finished.
                indicate_led_data.isTransferDone = false;
                indicate_led_data.state = INDICATE_LED_STATE_IDLE;
            }
            break;
        case INDICATE_LED_STATE_ERROR:
        case INDICATE_LED_STATE_IDLE:
            break;

    }
}