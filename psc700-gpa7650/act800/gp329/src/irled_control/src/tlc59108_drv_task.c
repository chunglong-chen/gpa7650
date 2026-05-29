/**
 * @file    tlc59108_drv_task.c
 * @brief   IR LED (TLC59108) control task and driver implementation.
 * @ingroup pupil_led_controller
 * @details
 * This module controls IR LEDs via the TLC59108 I2C driver.
 * It supports LED on/off control, brightness (current) adjustment,
 * and status query through a FreeRTOS task mechanism.
 *
 * Command flow:
 *   Console ¡÷ cmd parser ¡÷ queue_cmd_to_irled_task ¡÷ this module
 *
 * All responses (ACK / NAK / value) are returned via UART using
 * cmd_tx_task interface.
 */
#include "tlc59108_drv_task.h"
#include "cmd_tx_task.h"
#include "drv_l1_i2c.h"
#include "gplib_print_string.h"

static drv_l1_i2c_bus_handle_t TLC59108;

xQueueHandle queue_cmd_to_irled_task = NULL;

INT32S irled_write_reg(INT8U reg, INT8U value) {
    INT32S ret;
    ret = drv_l1_reg_1byte_data_1byte_write(&TLC59108, reg, value); // initial value
    return ret;
}

INT32S irled_read_reg(INT8U reg, INT8U* data) {
    INT32S ret;
    ret = drv_l1_reg_1byte_data_1byte_read(&TLC59108, reg, data);
    return ret;
}

/**
 * @brief Read IR LED brightness value.
 *
 * @details
 * This function reads the PWM register of the specified LED
 * and returns the current brightness value.
 *
 * @param led_no LED index (0~7).
 * @return LED current value (0~255), or negative value on error.
 */
INT32S irled_get_cur(INT8U led_no) // led_no:0~7
{
    INT8U  value;
    INT32S ret = -1;
    if (led_no >= TLC59108_MAX_LED_NUM)
        DBG_PRINT("IR LED out of number\r\n");
    else {
        irled_read_reg(TLC59108_REG_PWM(led_no), &value);
        ret = value;
    }
    return ret;
}

/**
 * @brief Enable IR LED and set brightness.
 *
 * @details
 * This function sets the PWM value of the specified LED and
 * configures the LEDOUT register to enable PWM output mode.
 *
 * The brightness is controlled by writing to the PWM register,
 * and the LED output behavior is configured through LEDOUT register bits.
 *
 * @param led_no LED index (0~7).
 * @param led_cur LED current value (0~255).
 */
INT32S set_irled_on(INT8U led_no, INT16U led_cur) // led_no:0~7; led_cur:0~255
{
    INT8U  value;
    INT8U  reg_ledout;
    INT32S ret = -1;

    if (led_no >= TLC59108_MAX_LED_NUM)
        DBG_PRINT("IR LED out of number\r\n");
    else if (led_cur > TLC59108_MAX_LED_CURR)
        DBG_PRINT("IR LED current out of range\r\n");
    else {
        irled_write_reg(TLC59108_REG_PWM(led_no), led_cur); // Normal mode
        // Get REG LEDOUTx
        reg_ledout = TLC59108_REG_LEDOUT(led_no);
        irled_read_reg(reg_ledout, &value);
        switch (led_no % 4) {
        case 0:
            // set bit[1:0] to 0b10 LED0 or LED4
            value &= ~(0b11 << 0);
            value |= (LDRx_CTRL_PWM << 0);
            break;
        case 1:
            // set bit[3:2] to 0b10 LED1 or LED5
            value &= ~(0b11 << 2);
            value |= (LDRx_CTRL_PWM << 2);
            break;
        case 2:
            // set bit[5:4] to 0b10 LED2 or LED6
            value &= ~(0b11 << 4);
            value |= (LDRx_CTRL_PWM << 4);
            break;
        case 3:
            // set bit[7:6] to 0b10 LED3 or LED7
            value &= ~(0b11 << 6);
            value |= (LDRx_CTRL_PWM << 6);
            break;
        }
        // DBG_PRINT("LED on val %#x\r\n", value);
        ret = irled_write_reg(reg_ledout, value);
    }

    return ret;
}

INT32S set_irled_off(INT8U led_no) // led_no: 0~7
{
    INT8U  value;
    INT8U  reg_ledout;
    INT32S ret = -1;

    if (led_no >= TLC59108_MAX_LED_NUM)
        DBG_PRINT("IR LED out of number\r\n");
    else {
        reg_ledout = TLC59108_REG_LEDOUT(led_no);
        // DBG_PRINT("LED out REG %#x\r\n", reg_ledout);
        irled_read_reg(reg_ledout, &value);
        switch (led_no % 4) {
        case 0:
            // set bit[1:0] to 0b00 LED0 or LED4
            value &= 0xfc;
            break;
        case 1:
            // set bit[3:2] to 0b00 LED1 or LED5
            value &= 0xf3;
            break;
        case 2:
            // set bit[5:4] to 0b00 LED2 or LED6
            value &= 0xcf;
            break;
        case 3:
            // set bit[7:6] to 0b00 LED3 or LED7
            value &= 0x3f;
            break;
        }
        // DBG_PRINT("LED off val %#x\r\n", value);
        ret = irled_write_reg(reg_ledout, value);
    }
    return ret;
}
/**
 * @brief Initialize IR LED driver (TLC59108).
 *
 * @details
 * This function initializes the I2C interface and configures
 * the TLC59108 device into normal operation mode.
 *
 * It sets device parameters including bus number, slave address,
 * and clock rate, then writes MODE1 register to enable normal mode.
 */
void irled_init(void) {
    DBG_PRINT("IR LED init\r\n");
    TLC59108.devNumber = I2C_1;
    TLC59108.slaveAddr = TLC59108_SLAVE_ADDR;
    TLC59108.clkRate = TLC59108_I2C_CLK_RATE;
    drv_l1_i2c_init(TLC59108.devNumber);
    irled_write_reg(TLC59108_REG_MODE1, 0x01); // Normal mode

}
/**
 * @brief IR LED control task.
 *
 * @details
 * This task receives LED control messages from a queue and
 * dispatches them to corresponding driver functions.
 *
 * It handles LED on/off control, brightness setting,
 * and current reading operations based on received commands.
 *
 * Some commands generate ACK/NAK responses via UART.
 */
void irled_task_runner(void) {
    led_task_message_t msg;
    queue_cmd_to_irled_task = xQueueCreate(10, sizeof(led_task_message_t));
    INT32 ret;
    // initial tlc59108
    irled_init();


    while (1) {
        /* block forever until a new command is received */
        if (queue_cmd_to_irled_task && (xQueueReceive(queue_cmd_to_irled_task, &msg, portMAX_DELAY) == pdTRUE)) {
            char char_seq[16];
            snprintf(char_seq, sizeof(char_seq), "%d", msg.index);
            switch (msg.ir_cmd) {
            case LED_TASK_CMD_SET_IRLED_ON:
            case LED_TASK_CMD_SET_IR_CURRENT:
                set_irled_on(msg.led_no, msg.led_cur);
                break;
            case LED_TASK_CMD_SET_IRLED_OFF:
                set_irled_off(msg.led_no);
                break;
            case LED_TASK_CMD_GET_IR_CURRENT:
                irled_get_cur(msg.led_no);
                break;
            case LED_TASK_CMD_MIIS_SET_IR_CURRENT:
                if (msg.led_cur > 0)
                    ret = set_irled_on(CONVERT_TO_TLC59108_NUM(msg.led_no), msg.led_cur);
                else
                    ret = set_irled_off(CONVERT_TO_TLC59108_NUM(msg.led_no));

                if (ret > 0) {
                    DBG_PRINT("Set LED%d val %d\r\n", msg.led_no, msg.led_cur);
                    char result[16];                               // enough for "255" + NUL
                    snprintf(result, sizeof(result), "%d", msg.led_cur);
                    ack_with_info(char_seq, result);                    // -> [seq] ack "value"
                }
                else{
                    nak_with_info(char_seq);
                }
                break;
            case LED_TASK_CMD_MIIS_GET_IR_CURRENT:
                ret = irled_get_cur(CONVERT_TO_TLC59108_NUM(msg.led_no));
                if (ret >= 0) {
                    DBG_PRINT("Get LED%d val %d\r\n", msg.led_no, ret);
                    char result[16];                               // enough for "255" + NUL
                    snprintf(result, sizeof(result), "%d", ret);
                    ack_with_info(char_seq, result);                    // -> [seq] ack "value"
                }
                else{
                    nak_with_info(char_seq);
                }
                break;
            default:
                break;
            }
        }
    }
}
