#include "blink_led_task.h"
#include "gpio_init.h"
#include "gplib_print_string.h"

void blink_task_runner(void) {

    portTickType xLastWakeTimevBlink;
    DBG_PRINT("blink task start\r\n");

    xLastWakeTimevBlink = xTaskGetTickCount();

    while (1) {
        gpio_write_io(GP329_LED, !gpio_read_io(GP329_LED));
        vTaskDelayUntil(&xLastWakeTimevBlink, pdMS_TO_TICKS(BLINK_TASK_CYCLE));
    }
}