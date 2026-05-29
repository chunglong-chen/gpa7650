/**
 * @file    indicate_led.h
 * @brief   Indicator LED control interface definitions.
 * @details This file declares the data structures, macros, and APIs
 *          for controlling indicator LEDs via the LP5012 LED driver
 *          using I2C communication.
 *
 *          It provides interfaces for LED brightness control, RGB color
 *          configuration, enable/disable functionality, and task-based
 *          initialization.
 * @ingroup indicator_led_controller
 */
#ifndef LED_H
#define	LED_H

#include "driver/i2c/drv_i2c.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define LED_DEFAULT_BRIGHTNESS 0x80

    //#define LP5012_BASE        0x28 // 8bit: 0b00101000
#define LP5012_BASE        0x0014 // 7bit: 0b0010100

#define LP5012_REG_DEV_CONFIG0   0x0
#define LP5012_REG_DEV_CONFIG0_CHIP_ENABLE 1<<6
#define LP5012_REG_DEV_CONFIG1   0x1
#define LP5012_REG_LED_CONFIG0      0x2
#define LP5012_REG_BANK_BRIGHTNESS    0x3
#define LP5012_REG_BANK_A_COLOR    0x4
#define LP5012_REG_BANK_B_COLOR    0x5
#define LP5012_REG_BANK_C_COLOR    0x6
#define LP5012_REG_LED0_BRIGHTNESS    0x7
#define LP5012_REG_LED1_BRIGHTNESS    0x8
#define LP5012_REG_LED2_BRIGHTNESS    0x9
#define LP5012_REG_LED3_BRIGHTNESS    0xA
#define LP5012_REG_OUT0_COLOR    0xB
#define LP5012_REG_OUT1_COLOR    0xC
#define LP5012_REG_OUT2_COLOR    0xD
#define LP5012_REG_OUT3_COLOR    0xE
#define LP5012_REG_OUT4_COLOR    0xF
#define LP5012_REG_OUT5_COLOR    0x10
#define LP5012_REG_OUT6_COLOR    0x11
#define LP5012_REG_OUT7_COLOR    0x12
#define LP5012_REG_OUT8_COLOR    0x13
#define LP5012_REG_OUT9_COLOR    0x14
#define LP5012_REG_OUT10_COLOR    0x15
#define LP5012_REG_OUT11_COLOR    0x16
#define LP5012_REG_RESET    0x17


#define INDEX_INDICATE_LED1 1 // D5
#define INDEX_INDICATE_LED2 2 // D6
#define INDEX_INDICATE_LED3 3 // D7

    typedef enum {
        INDICATE_LED_STATE_INIT,
        INDICATE_LED_STATE_WAIT_CHIP_ENABLE,
        INDICATE_LED_STATE_WAIT_LED1_BRIGHTNESS,
        INDICATE_LED_STATE_WAIT_LED2_BRIGHTNESS,
        INDICATE_LED_STATE_WAIT_LED3_BRIGHTNESS,
        INDICATE_LED_STATE_WAIT_LED1_COLOR,
        INDICATE_LED_STATE_WAIT_LED2_COLOR,
        INDICATE_LED_STATE_WAIT_LED3_COLOR,
        INDICATE_LED_STATE_ERROR,
        INDICATE_LED_STATE_IDLE,
    } INDICATE_LED_STATES;

    typedef struct {
        INDICATE_LED_STATES state;

        uint8_t txBuffer[2];

        volatile bool isTransferDone;

    } INDICATE_LED_DATA;

    void INDICATE_LED_Initialize(void);

    void INDICATE_LED_Tasks(void);

    bool indicate_led_enable(uint8_t led_id, bool enable);

    bool indicate_led_level(uint8_t led_id, uint16_t level);

    bool indicate_led_color(uint8_t led_id, uint16_t red, uint16_t green, uint16_t blue);
#ifdef	__cplusplus
}
#endif

#endif	/* LED_H */

