#ifndef __TASK_CONTROL_H__
#define __TASK_CONTROL_H__

#define CMD_STACKSIZE         16384
#define IRLED_STACKSIZE       4096
#define MIIS_3AXIS_STACKSIZE  4096
#define LIQUID_LENS_STACKSIZE 4096
#define SONY_OLED_STACKSIZE   4096
#define POWER_STACKSIZE       4096
#define ASPS_STACKSIZE        4096
#define BLINK_STACKSIZE       4096
#define UD_BLINK_STACKSIZE    4096

void create_task_cmd_receive(void);
void create_task_cmd_transmit(void);
void create_task_ir_led(void);
void create_task_miis_3axis(void);
void create_task_liquid_lens(void);
void create_task_sony_oled(void);
void create_task_power(void);
void create_task_asps_switch(void);
void create_task_blink_led(void);
void create_task_new_liquid_lens_module(void);

#endif // __TASK_CONTROL_H__
