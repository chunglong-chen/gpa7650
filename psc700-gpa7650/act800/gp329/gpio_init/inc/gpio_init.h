#ifndef __GPIO_INIT_H__
#define __GPIO_INIT_H__

#include "drv_l1_gpio.h"

#define GPIO_TEST       0
#define GPIO_INPUT_TEST 0

#define GPIO_LOW        0
#define GPIO_HIGH       1

/* ACT200 GPIO Pin define */
#define OCT_CAP        IO_F0
#define LL_EN          IO_C12 // Liquid Lens EN
#define ASPS_CTRL1     IO_C13 // Motor Forward
#define ASPS_CTRL2     IO_C14 // Motor Reverse

#define UDIS_M_CHK     IO_E1
#define UDIS_H_CHK     IO_E2
#define PS_CHK         IO_E3 // Check PS position
#define AS_CHK         IO_E4 // Check AS position
#define MOT_STEP       IO_E5
#define MOT_NSLEEP     IO_E6 // Motor Disable
#define MOT_L1         IO_E7
#define MOT_L0         IO_C3
#define MOT_DIR        IO_C2  // Motor Direction
#define FL_LE          IO_C11 // Fixation LED
#define FL_A2          IO_C10 // Fixation LED
#define FL_A1          IO_C9  // Fixation LED
#define FL_A0          IO_C8  // Fixation LED
#define MOT_PWR_EN     IO_C7  // Step Motor Power
#define AUDIO_EN       IO_C6
#define TFT_DIR        IO_F4 // Voltage translator Direction
#define TFT_RST        IO_F5 // Sony LCM RST
#define M_TFT_EN       IO_F6 // Voltage translator Enable
#define UDIS_RST       IO_F7
#define GP329_LED      IO_F8 // GP329 system ready indicate LED
#define UDIS_EN        IO_F9 // Sony LCM EN
#define SPI0_CS        IO_D6 // SPI0 chip select pin
#define SOLENOID_CTRL1 IO_F2 // Solenoid IN1
#define SOLENOID_CTRL2 IO_F3 // Solenoid IN2
#define USB_HUB_RST    IO_F1 // USB Hub reset pin
#define M261_RST       IO_F0 // M261_RST reset pin

typedef struct {
    INT32U  gpio;
    BOOLEAN dir;
    BOOLEAN value;
} gpio_config_t;

extern BOOLEAN gpio_set_value(INT32U port, BOOLEAN data);
void           gpio_signal_leds_device_ready(void);
void           gpio_ctrl_init(void);
void           gpio_reset_usb_hub(void);

#endif
