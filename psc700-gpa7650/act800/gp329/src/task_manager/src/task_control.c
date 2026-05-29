#include "task_control.h"
#include "a_pe_25hx.h"
#include "asps_switch_task.h"
#include "blink_led_task.h"
#include "cmd_rx_task.h"
#include "cmd_tx_task.h"
#include "cmsis_os.h"
#include "gplib_print_string.h"
#include "max14574.h"
#include "miis_3axis_task.h"
#include "power_task_entry.h"
#include "sony_ecx336.h"
#include "tlc59108_drv_task.h"

osThreadId rx_id = NULL;
osThreadId tx_id = NULL;
osThreadId ir_led_id = NULL;
osThreadId miis_3axis_id = NULL;
osThreadId ll_id = NULL;
osThreadId sony_id = NULL;
osThreadId power_id = NULL;
osThreadId asps_switch_id = NULL;
osThreadId blink_id = NULL;
osThreadId ud_blink_id = NULL;
osThreadId a_pe_25hx_id = NULL;

void create_task_cmd_receive(void) {
    osThreadDef_t rx_task = {"cmd_task_receive", (void*)cmd_rx_task, osPriorityNormal, 1, CMD_STACKSIZE};
    // Cmd Receive Task
    rx_id = osThreadCreate(&rx_task, (void*)NULL);
    if (rx_id == 0) {
        DBG_PRINT("cmd_task_receive error\r\n");
        while (1)
            ;
    } else
        osDelay(5);

    DBG_PRINT("cmd task rx started\r\n");
}

void create_task_cmd_transmit(void) {
    osThreadDef_t tx_task = {"cmd_task_transmit", (void*)cmd_tx_task, osPriorityNormal, 1, CMD_STACKSIZE};
    // Cmd Transmit Task
    tx_id = osThreadCreate(&tx_task, (void*)NULL);
    if (tx_id == 0) {
        DBG_PRINT("cmd_task_receive error\r\n");
        while (1)
            ;
    } else
        osDelay(5);

    DBG_PRINT("cmd task tx started\r\n");
}


void create_task_ir_led(void) {

    osThreadDef_t ir_led_task = {"irled_task_runner", (void*)irled_task_runner, osPriorityNormal, 1, IRLED_STACKSIZE};
    // IR LED Task
    ir_led_id = osThreadCreate(&ir_led_task, (void*)NULL);
    if (ir_led_id == 0) {
        DBG_PRINT("ir_led_task_receive error\r\n");
        while (1)
            ;
    } else
        osDelay(5);

    DBG_PRINT("IR led task started\r\n");
}


void create_task_miis_3axis(void) {

    osThreadDef_t miis_3axis_task = {"miis_3axis_task_runner", (void*)miis_3axis_task_runner, osPriorityNormal, 1, MIIS_3AXIS_STACKSIZE};
    // 3-Axis Task
    miis_3axis_id = osThreadCreate(&miis_3axis_task, (void*)NULL);
    if (miis_3axis_id == 0) {
        DBG_PRINT("miis_3axis_task error\r\n");
        while (1)
            ;
    } else
        osDelay(5);

    DBG_PRINT("miis 3-axis task started\r\n");
}


void create_task_liquid_lens(void) {

    osThreadDef_t liquid_lens_task = {"liquid_lens_task", (void*)max14574_task_runner, osPriorityNormal, 1, LIQUID_LENS_STACKSIZE};
    // Liquid Lens Task
    ll_id = osThreadCreate(&liquid_lens_task, (void*)NULL);
    if (ll_id == 0) {
        DBG_PRINT("liquid_lens_task error\r\n");
        while (1)
            ;
    } else
        osDelay(5);

    DBG_PRINT("liquid lens task started\r\n");
}


void create_task_sony_oled(void) {

    osThreadDef_t sony_oled_task = {"sony_oled_task", (void*)sony_task_runner, osPriorityNormal, 1, SONY_OLED_STACKSIZE};
    // Sony OLED Task
    sony_id = osThreadCreate(&sony_oled_task, (void*)NULL);
    if (sony_id == 0) {
        DBG_PRINT("sony_oled_task error\r\n");
        while (1)
            ;
    } else
        osDelay(5);

    DBG_PRINT("sony oled task started\r\n");
}


void create_task_power(void) {

    osThreadDef_t power_task = {"power_task", (void*)power_task_runner, osPriorityNormal, 1, POWER_STACKSIZE};
    // POWER Task
    power_id = osThreadCreate(&power_task, (void*)NULL);
    if (power_id == 0) {
        DBG_PRINT("power_task error\r\n");
        while (1)
            ;
    } else
        osDelay(5);

    DBG_PRINT("power task started\r\n");
}


void create_task_asps_switch(void) {

    osThreadDef_t asps_task = {"ap_switch_task", (void*)asps_switch_task, osPriorityNormal, 1, ASPS_STACKSIZE};
    // ASPS Task
    asps_switch_id = osThreadCreate(&asps_task, (void*)NULL);
    if (asps_switch_id == 0) {
        DBG_PRINT("asps_switch_task error\r\n");
        while (1)
            ;
    } else
        osDelay(5);

    DBG_PRINT("asps switch task started\r\n");
}

void create_task_blink_led(void) {

    osThreadDef_t blink_task = {"blink_task", (void*)blink_task_runner, osPriorityNormal, 1, BLINK_STACKSIZE};
    // Blink Green Led Task
    blink_id = osThreadCreate(&blink_task, (void*)NULL);
    if (blink_id == 0) {
        DBG_PRINT("blink_task error\r\n");
        while (1)
            ;
    } else
        osDelay(5);

    DBG_PRINT("blink task started\r\n");
}

void create_task_new_liquid_lens_module(void) {

    osThreadDef_t a_pe_25hx_task = {"a_pe_25hx_task", (void*)a_pe_25hx_task_runner, osPriorityNormal, 1, LIQUID_LENS_STACKSIZE};
    // New Liquid Lens Module Task
    a_pe_25hx_id = osThreadCreate(&a_pe_25hx_task, (void*)NULL);
    if (a_pe_25hx_id == 0) {
        DBG_PRINT("a_pe_25hx_task error\r\n");
        while (1)
            ;
    } else
        osDelay(5);

    DBG_PRINT("A_PE_25HX task started\r\n");
}

