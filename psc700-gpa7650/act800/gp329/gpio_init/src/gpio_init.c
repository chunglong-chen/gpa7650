#include "gpio_init.h"

gpio_config_t m_act200_gpio_table[] =
    {
        {OCT_CAP, GPIO_OUTPUT, GPIO_LOW},
        {LL_EN, GPIO_OUTPUT, GPIO_LOW},
        {ASPS_CTRL1, GPIO_OUTPUT, GPIO_LOW},
        {ASPS_CTRL2, GPIO_OUTPUT, GPIO_LOW},
        {UDIS_M_CHK, GPIO_INPUT, GPIO_LOW},
        {UDIS_H_CHK, GPIO_INPUT, GPIO_LOW},
        {PS_CHK, GPIO_INPUT, GPIO_LOW},
        {AS_CHK, GPIO_INPUT, GPIO_LOW},
        {MOT_STEP, GPIO_OUTPUT, GPIO_LOW},
        {MOT_NSLEEP, GPIO_OUTPUT, GPIO_LOW},
        {MOT_L1, GPIO_OUTPUT, GPIO_LOW},
        {MOT_L0, GPIO_OUTPUT, GPIO_LOW},
        {MOT_DIR, GPIO_OUTPUT, GPIO_LOW},
        {FL_LE, GPIO_OUTPUT, GPIO_LOW},
        {FL_A2, GPIO_OUTPUT, GPIO_LOW},
        {FL_A1, GPIO_OUTPUT, GPIO_LOW},
        {FL_A0, GPIO_OUTPUT, GPIO_LOW},
        {MOT_PWR_EN, GPIO_OUTPUT, GPIO_HIGH}, // work around for I2C voltage stable
        {AUDIO_EN, GPIO_OUTPUT, GPIO_LOW},
        {TFT_DIR, GPIO_OUTPUT, GPIO_LOW},
        {TFT_RST, GPIO_OUTPUT, GPIO_LOW},
        {M_TFT_EN, GPIO_OUTPUT, GPIO_LOW},
        {UDIS_RST, GPIO_OUTPUT, GPIO_LOW},
        {GP329_LED, GPIO_OUTPUT, GPIO_LOW},
        {SPI0_CS, GPIO_OUTPUT, GPIO_HIGH},
        {UDIS_EN, GPIO_OUTPUT, GPIO_LOW},
        {SOLENOID_CTRL1, GPIO_OUTPUT, GPIO_LOW},
        {SOLENOID_CTRL2, GPIO_OUTPUT, GPIO_LOW},
        {USB_HUB_RST, GPIO_OUTPUT, GPIO_LOW},
        {M261_RST, GPIO_OUTPUT, GPIO_LOW}};

BOOLEAN gpio_set_value(INT32U in_port, BOOLEAN in_data) {
    return gpio_write_io(in_port, in_data);
}

// let the user know that the device is ready by turning on the LED
void gpio_signal_leds_device_ready(void) {
    gpio_set_value(GP329_LED, GPIO_HIGH);
}

/**
 * @brief   GPIO initial function
 * @param   none
 * @return  none
 */
void gpio_ctrl_init(void) {
    INT16U table_size = (sizeof(m_act200_gpio_table) / sizeof(gpio_config_t));
    INT16U i;
    R_GPIO_CTRL &= ~(1 << 0); /* Disable JTAG pin, IOC[15:12] used as GPIOs */
    for (i = 0; i < table_size; i++) {
        if (m_act200_gpio_table[i].dir == GPIO_OUTPUT) {
            gpio_init_io(m_act200_gpio_table[i].gpio, m_act200_gpio_table[i].dir);
            gpio_write_io(m_act200_gpio_table[i].gpio, m_act200_gpio_table[i].value);
        } else {
            gpio_init_io(m_act200_gpio_table[i].gpio, m_act200_gpio_table[i].dir);
        }
    }
}
// Reset USB hub for pupil cam
void gpio_reset_usb_hub(void) {
    gpio_set_value(USB_HUB_RST, GPIO_LOW);
    osDelay(5);
    gpio_set_value(USB_HUB_RST, GPIO_HIGH);
}
