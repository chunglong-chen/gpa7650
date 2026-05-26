#define LED 	PB10
#define LED_ON	0
#define LED_OFF	1
#define SW3_PIN_MASK (BIT8 | BIT9 | BIT10 | BIT11)  // Address = PA8~PA11
#define I2C_UART_SW   PA8   // HIGH = I2C, LOW = UART 