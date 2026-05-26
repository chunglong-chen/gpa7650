#include "power_task_entry.h"
#include "FreeRTOS.h"
#include "drv_l1_sfr.h"
#include "gplib_print_string.h"

#define PWR_ON_0                  (1 << 0)
#define PWR_ON_1                  (1 << 1)
#define PWR_ON_2                  (1 << 2)
#define PWR_ON_3                  (1 << 3)
#define PM_PROCESS_RATE_MS        100 /* Process the power manager every PM_PROCESS_RATE_MS milliseconds */
#define NUM_PM_TICKS_FOR_SHUTDOWN 30  /* Hold the power button this many ticks to turn off ODD */

// Turn off device
void power_manager_power_off(void) {
    DBG_PRINT("Power off\r\n");
    R_SYSTEM_POWER_CTRL0 &= ~0x1; // power down
}

void power_task_runner(void) {

    portTickType  xLastWakeTimevPM;
    unsigned char Power_Key = 0;
    DBG_PRINT("Power manager start\r\n");

    xLastWakeTimevPM = xTaskGetTickCount();

    while (1) {
        if (R_SYSTEM_POWER_CTRL1 & PWR_ON_2) {

            Power_Key++;

            // If the button is held for 3 seconds, turn off GP329.
            if (Power_Key == NUM_PM_TICKS_FOR_SHUTDOWN) {
                power_manager_power_off();
            }
        }
        vTaskDelayUntil(&xLastWakeTimevPM, pdMS_TO_TICKS(PM_PROCESS_RATE_MS));
    }
}
