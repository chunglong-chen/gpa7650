#include "Task_Manager.h"
#include "gpio_init.h"
#include "gplib.h"
#include "gplib_print_string.h"
#include "sony_ecx336.h"
#include "task_control.h"
#include "drv_l2_spi_flash.h"

void Task_Manager(void) {

    create_task_power();

    create_task_cmd_receive();

    create_task_cmd_transmit();

    create_task_ir_led();

    create_task_miis_3axis();

    create_task_liquid_lens();

    create_task_sony_oled();

    create_task_asps_switch();
    // while (1) {
        if (_devicemount(FS_NOR)) {
            DBG_PRINT("Mount Disk Fail[%d]\r\n", FS_NOR);
        } else {
            DBG_PRINT("Mount Disk success[%d]\r\n", FS_NOR);
            // break;
        }
    // }

    create_task_new_liquid_lens_module();

    // Always keep the following 2 functions at the end
    gpio_signal_leds_device_ready();

    create_task_blink_led();
}