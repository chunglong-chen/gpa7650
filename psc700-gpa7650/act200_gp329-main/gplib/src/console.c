#include "console.h"
#include "a_pe_25hx.h"
#include "asps_switch_task.h"
#include "build_info.h"
#include "cmd_tx_task.h"
#include "gp_stdlib.h"
#include "gpio_init.h"
#include "gplib_mm_gplus.h"
#include "gplib_print_string.h"
#include "max14574.h"
#include "miis_3axis_task.h"
#include "sony_ecx336.h"
#include "task_control.h"
#include "tlc59108_drv_task.h"
#include "version.h"

#define CMD_HISTORIES 4
#define CMD_BUF_SIZE  128
#define CMD_ARG_SIZE  20

#if _OPERATING_SYSTEM != _OS_NONE
#if _OPERATING_SYSTEM == _OS_UCOS2
#elif _OPERATING_SYSTEM == _OS_FREERTOS
extern xTaskHandle cmdTaskHandle;
#else
#endif
#endif

// Macro to check the char is digit or not
#define in_range(c, lo, up) ((INT8U)c >= lo && (INT8U)c <= up)
#define isdigit(c)          in_range(c, '0', '9')

static cmd_t*        pcmds = NULL;
static unsigned char cmdBuffer[CMD_BUF_SIZE];
static unsigned int  pos = 0;
static unsigned int  logIn = 0;

static unsigned int histBuf[CMD_HISTORIES][CMD_BUF_SIZE];
static unsigned int histPos[CMD_HISTORIES];
static unsigned int histIns;
static unsigned int histOutput;
static unsigned int histInsWrap;
static unsigned int histOutputWrap;

static void default_cmd_handler(int argc, char* argv[]);
static void console_get_version(int argc, char* argv[]);
static void console_get_info(int argc, char* argv[]);
static void console_pupil_ir(int argc, char* argv[]);
static void console_oct_capture(int argc, char* argv[]);
static void console_as_motor_ctrl(int argc, char* argv[]);
static void console_ps_motor_ctrl(int argc, char* argv[]);
static void console_led_ctrl(int argc, char* argv[]);
static void console_uart2_ctrl(int argc, char* argv[]);
static void console_uart2_ack(int argc, char* argv[]);
static void console_uart2_nak(int argc, char* argv[]);
static void console_ir_ctrl(int argc, char* argv[]);
static void console_gpio_read_test(int argc, char* argv[]);
static void console_motor_ctrl(int argc, char* argv[]);
static void console_ll_ctrl(int argc, char* argv[]);
static void console_miis_ll_ctrl(int argc, char* argv[]);
static void console_sony_ctrl(int argc, char* argv[]);
static void console_as_ps_mode_ctrl(int argc, char* argv[]);
static void console_solenoid_ctrl(int argc, char* argv[]);
static void console_new_ll_ctrl(int argc, char* argv[]);
static void console_i2c_debug_ctrl(int argc, char* argv[]);

extern xQueueHandle queue_cmd_to_microchip;
extern xQueueHandle queue_cmd_to_irled_task;
extern xQueueHandle queue_cmd_to_3axis_task;
extern xQueueHandle queue_cmd_to_ll_task;
extern xQueueHandle queue_cmd_to_new_ll_task;
extern xQueueHandle queue_cmd_to_sony_task;
extern xQueueHandle queue_cmd_to_asps_switch_task;

extern void set_i2c_debug_print_flag(BOOLEAN enalbe);

static cmd_t default_cmd_list[] =
    {
        {"all", default_cmd_handler, NULL},
        {"test", console_get_version, NULL},
        {"info", console_get_info, NULL},
        {"pir", console_pupil_ir, NULL},
        {"octcap", console_oct_capture, NULL},
        {"as", console_as_motor_ctrl, NULL},
        {"ps", console_ps_motor_ctrl, NULL},
        {"led", console_led_ctrl, NULL},
        {"cmd", console_uart2_ctrl, NULL},
        {"ack", console_uart2_ack, NULL},
        {"nak", console_uart2_nak, NULL},
        {"irctrl", console_ir_ctrl, NULL},
        {"gpio?", console_gpio_read_test, NULL},
        {"3as", console_motor_ctrl, NULL},
        {"ll", console_ll_ctrl, NULL},
        {"motor", console_miis_ll_ctrl, NULL},
        {"ud", console_sony_ctrl, NULL},
        {"apsw", console_as_ps_mode_ctrl, NULL},
        {"sonoid", console_solenoid_ctrl, NULL},
        {"nl", console_new_ll_ctrl, NULL},
        {"dbg", console_i2c_debug_ctrl, NULL},
        {NULL, NULL, NULL}};

static void default_cmd_help(void) {
    cmd_t* curr;

    curr = pcmds->pnext;
    if (curr != NULL)
        DBG_PRINT("\r\nUsage:\r\n");
    while (curr) {
        DBG_PRINT("    %s help\r\n", curr->cmd);
        curr = curr->pnext;
    }
}

static void default_cmd_handler(int argc, char* argv[]) {
    if (STRCMP(argv[1], "help") == 0) {
        default_cmd_help();
    } else {
        default_cmd_help();
    }
}

static void console_get_version(int argc, char* argv[]) {
    DBG_PRINT("MiiS ver:%s, FW ver:%s, git:%s branch:%s\r\n", MIIS_VERSION, FIRMWARE_VERSION, GIT_HASH, GIT_BRANCH);
    uart2_printf("MiiS ver:%s, FW ver:%s, git:%s branch:%s\r\n", MIIS_VERSION, FIRMWARE_VERSION, GIT_HASH, GIT_BRANCH);
}

static void console_get_info(int argc, char* argv[]) {
    if (argc > 1) {
        if (STRCMP(argv[1], "Ver") == 0) {
            DBG_PRINT("%s\r\n", FIRMWARE_VERSION);
            uart2_printf("%s\r\n", FIRMWARE_VERSION);
            return;
        } else if (STRCMP(argv[1], "Date") == 0) {
            char year[5], month[3], day[3];
            strncpy(year, DATETIME, 4);
            year[4] = '\0';

            strncpy(month, DATETIME + 4, 2);
            month[2] = '\0';

            strncpy(day, DATETIME + 6, 2);
            day[2] = '\0';
            DBG_PRINT("%s %s %s\r\n", year, month, day);
            uart2_printf("%s %s %s\r\n", year, month, day);
            return;
        } else if (STRCMP(argv[1], "Time") == 0) {
            char hour[3], minute[3], second[3];
            strncpy(hour, DATETIME + 8, 2);
            hour[2] = '\0';
            strncpy(minute, DATETIME + 10, 2);
            minute[2] = '\0';
            strncpy(second, DATETIME + 12, 2);
            second[2] = '\0';
            DBG_PRINT("%s:%s:%s\r\n", hour, minute, second);
            uart2_printf("%s:%s:%s\r\n", hour, minute, second);
            return;
        } else if (STRCMP(argv[1], "Git") == 0) {
            DBG_PRINT("MiiS ver:%s, FW ver:%s, git:%s branch:%s\r\n", MIIS_VERSION, FIRMWARE_VERSION, GIT_HASH, GIT_BRANCH);
            uart2_printf("MiiS ver:%s, FW ver:%s, git:%s branch:%s\r\n", MIIS_VERSION, FIRMWARE_VERSION, GIT_HASH, GIT_BRANCH);
            return;
        }
    }
}

static void console_pupil_ir(int argc, char* argv[]) {

    led_task_message_t msg = {
        .ir_cmd = LED_TASK_CMD_DEFAULT,
        .led_no = TLC59108_MAX_LED_NUM,
        .led_cur = TLC59108_MAX_LED_CURR};
    if (argc >= 3) {
        if ((strncmp(argv[1], "C", 1) == 0)) {
            CHAR* p_index = argv[1];
            p_index++; // Get number

            if (atoi(p_index) > 0 && atoi(p_index) <= MIIS_MAX_LED_NUM) // Should be C1, C2, C3 or C4
                msg.led_no = atoi(p_index);
            else
                goto __cmderr;
            if (STRCMP(argv[2], "Get") == 0) {
                msg.ir_cmd = LED_TASK_CMD_MIIS_GET_IR_CURRENT;
            } else if (STRCMP(argv[2], "Set") == 0) {
                msg.ir_cmd = LED_TASK_CMD_MIIS_SET_IR_CURRENT;
                if (argc > 3) {
                    msg.led_cur = atoi(argv[3]);
                } else
                    goto __cmderr;
            }
            BaseType_t xStatus;
            xStatus = xQueueSendToBack(queue_cmd_to_irled_task, &msg, pdMS_TO_TICKS(10));
            if (xStatus != pdPASS) {
                DBG_PRINT("ERROR queuing LED task\r\n");
            }
            return;
        }
    }
__cmderr:
    DBG_PRINT("Usage: pir C1~C4 Get | Set Curr\r\n");
    return;
}

// OCT capture pin (IOF0) control
static void console_oct_capture(int argc, char* argv[]) {
    if (argc == 2) {
        if (STRCMP(argv[1], "on") == 0) {
            DBG_PRINT("oct capture on\r\n");
            gpio_set_value(OCT_CAP, GPIO_HIGH);
        } else if (STRCMP(argv[1], "off") == 0) {
            DBG_PRINT("oct capture off\r\n");
            gpio_set_value(OCT_CAP, GPIO_LOW);
        }
    } else
        DBG_PRINT("Usage: octcap on|off\r\n");
}

static void console_as_ps_mode_ctrl(int argc, char* argv[]) {
    INT8U msg = 0;
    if (argc == 2) {
        if (STRCMP(argv[1], "as") == 0) {
            msg = ASPS_TASK_CMD_SET_AS_MODE;
        } else if (STRCMP(argv[1], "ps") == 0) {
            msg = ASPS_TASK_CMD_SET_PS_MODE;
        } else if (STRCMP(argv[1], "Info") == 0) {
            msg = ASPS_TASK_CMD_GET_MODE;
        } else
            goto __cmderr;
        BaseType_t xStatus;
        xStatus = xQueueSendToBack(queue_cmd_to_asps_switch_task, &msg, pdMS_TO_TICKS(10));
        if (xStatus != pdPASS) {
            DBG_PRINT("ERROR queuing ASPS task\r\n");
        }
        return;
    }
__cmderr:
    DBG_PRINT("Usage: apsw as|ps\r\n");
}

// move to AS side (Rear)
static void console_as_motor_ctrl(int argc, char* argv[]) {
    if (argc == 2) {
        if (STRCMP(argv[1], "+") == 0) {
            gpio_set_value(ASPS_CTRL1, GPIO_LOW);
            gpio_set_value(ASPS_CTRL2, GPIO_HIGH);
            osDelay(20);
            gpio_set_value(ASPS_CTRL1, GPIO_HIGH);
            gpio_set_value(ASPS_CTRL2, GPIO_HIGH);
        }
    } else
        DBG_PRINT("Usage: as +\r\n");
}

// move to PS side (Front)
static void console_ps_motor_ctrl(int argc, char* argv[]) {
    if (argc == 2) {
        if (STRCMP(argv[1], "+") == 0) {
            gpio_set_value(ASPS_CTRL1, GPIO_HIGH);
            gpio_set_value(ASPS_CTRL2, GPIO_LOW);
            osDelay(20);
            gpio_set_value(ASPS_CTRL1, GPIO_HIGH);
            gpio_set_value(ASPS_CTRL2, GPIO_HIGH);
        }
    } else
        DBG_PRINT("Usage: ps +\r\n");
}

// LED pin (IOF8) control
static void console_led_ctrl(int argc, char* argv[]) {
    if (argc == 2) {
        if (STRCMP(argv[1], "on") == 0) {
            DBG_PRINT("led on\r\n");
            gpio_set_value(GP329_LED, GPIO_HIGH);
        } else if (STRCMP(argv[1], "off") == 0) {
            DBG_PRINT("led off\r\n");
            gpio_set_value(GP329_LED, GPIO_LOW);
        }
    } else
        DBG_PRINT("Usage: led on|off\r\n");
}

/* To send command to UART2 */
static void console_uart2_ctrl(int argc, char* argv[]) {
    INT32U i;
    DBG_PRINT(" [console] uart argc %d\r\n", argc);
    if (argc >= 2) {
        if (queue_cmd_to_microchip) {
            for (i = 1; i < argc; i++) {
                xQueueSend(queue_cmd_to_microchip, argv[i], 0);
            }
            xQueueSend(queue_cmd_to_microchip, "\n", 0);
        } else {
            DBG_PRINT(" [console] cmdQ is null\r\n");
        }
        return;
    }
    DBG_PRINT("Usage: cmd \r\n");
}

/* Receive ack from PIC32 */
static void console_uart2_ack(int argc, char* argv[]) {
    DBG_PRINT(" [console] ack\r\n");
}

/* Receive nak from PIC32 */
static void console_uart2_nak(int argc, char* argv[]) {
    DBG_PRINT(" [console] nak\r\n");
}

static void console_ir_ctrl(int argc, char* argv[]) {

    led_task_message_t msg = {
        .ir_cmd = LED_TASK_CMD_DEFAULT,
        .led_no = TLC59108_MAX_LED_NUM,
        .led_cur = TLC59108_MAX_LED_CURR};
    if (argc >= 3) {
        if (STRCMP(argv[1], "on") == 0) {
            msg.ir_cmd = LED_TASK_CMD_SET_IRLED_ON;
        } else if (STRCMP(argv[1], "off") == 0) {
            msg.ir_cmd = LED_TASK_CMD_SET_IRLED_OFF;
        } else
            goto __cmderr;

        // Check LEDx number is digit
        CHAR* p_index = argv[2];
        if (isdigit(*p_index))
            p_index++;
        else
            goto __cmdfmterr;
        msg.led_no = atoi(argv[2]);

        if (argc > 3) {
            msg.led_cur = atoi(argv[3]);
            CHAR* p_index = argv[3];
            if (isdigit(*p_index))
                p_index++;
            else
                goto __cmdfmterr;
        }

        BaseType_t xStatus;
        xStatus = xQueueSendToBack(queue_cmd_to_irled_task, &msg, pdMS_TO_TICKS(10));
        if (xStatus != pdPASS) {
            DBG_PRINT("ERROR queuing LED task\r\n");
        }
        return;
    }

__cmderr:
    DBG_PRINT("Usage: irctrl on|off LEDx\r\n");
    return;

__cmdfmterr:
    DBG_PRINT("Must be an integer.\r\n");
}

static void console_gpio_read_test(int argc, char* argv[]) {
    BOOLEAN io_e1 = gpio_read_io(UDIS_M_CHK);
    BOOLEAN io_e2 = gpio_read_io(UDIS_H_CHK);
    BOOLEAN io_e3 = gpio_read_io(PS_CHK);
    BOOLEAN io_e4 = gpio_read_io(AS_CHK);

    DBG_PRINT("IOE1 = %u\r\n", io_e1);
    DBG_PRINT("IOE2 = %u\r\n", io_e2);
    DBG_PRINT("IOE3 = %u\r\n", io_e3);
    DBG_PRINT("IOE4 = %u\r\n", io_e4);
}

static void console_motor_ctrl(int argc, char* argv[]) {
    miis_3axis_message_t msg = {
        .cmd = MIIS_3X_FW_VERSION,
        .data = {
            .axes = MIIS_3X_X_AXIS | MIIS_3X_Y_AXIS | MIIS_3X_Z_AXIS, // 0b0111 means all motor
            .x = 0,
            .y = 0,
            .z = 0}};
    if (argc >= 2) {
        BaseType_t xStatus;
        if (STRCMP(argv[1], "Ver") == 0) {
            msg.cmd = MIIS_3X_FW_VERSION;
        } else if (STRCMP(argv[1], "home") == 0) { // 3as Home is a command for moving x,y,z to home
            msg.cmd = MIIS_3X_HOME;
            if (argc > 2) {
                msg.data.z = atoi(argv[2]);
                goto __cmdcomplete;
            }
        } else if (STRCMP(argv[1], "abs") == 0) {
            msg.cmd = MIIS_3X_MOVE_ABS;
        } else if (STRCMP(argv[1], "rel") == 0) {
            msg.cmd = MIIS_3X_MOVE_REL;
        } else if (STRCMP(argv[1], "pos") == 0) {
            msg.cmd = MIIS_3X_POS;
        } else if (STRCMP(argv[1], "setzero") == 0) {
            msg.cmd = MIIS_3X_SET_ZERO;
        } else if (STRCMP(argv[1], "on") == 0) {
            gpio_set_value(MOT_PWR_EN, GPIO_HIGH);
            return;
        } else if (STRCMP(argv[1], "off") == 0) {
            gpio_set_value(MOT_PWR_EN, GPIO_LOW);
            return;
        } else if (STRCMP(argv[1], "center") == 0) { // 3as center is a command for moving to home first then moving x&y to center
            msg.cmd = MIIS_3X_MOVE_CENTER;
        } else if (STRCMP(argv[1], "status") == 0) {
            msg.cmd = MIIS_3X_STATUS;
        } else if ((strncmp(argv[1], "+", 1) == 0) || (strncmp(argv[1], "-", 1) == 0)) {
            // cmd: 3as val_x, val_y, val_z
            msg.cmd = MIIS_3X_MOVE_REL;
            if (argc > 3) {
                msg.data.x = atoi(argv[1]);
                msg.data.y = atoi(argv[2]);
                msg.data.z = atoi(argv[3]);
                goto __cmdcomplete;
            } else
                goto __cmderr;
        } else if (STRCMP(argv[1], "rst") == 0) {
            miis_3axis_reset_i2c_bus();
            return;
        } else if (STRCMP(argv[1], "reginfo") == 0) {
            miis_3axis_show_i2c_reg_info();
            return;
        } else
            goto __cmderr;

        if (argc > 2) {
            msg.data.axes = atoi(argv[2]);
        }
        if (argc > 3) {
            msg.data.x = atoi(argv[3]);
        }
        if (argc > 4) {
            msg.data.y = atoi(argv[4]);
        }
        if (argc > 5) {
            msg.data.z = atoi(argv[5]);
        }
    __cmdcomplete:
        xStatus = xQueueSendToBack(queue_cmd_to_3axis_task, &msg, pdMS_TO_TICKS(10));
        if (xStatus != pdPASS) {
            DBG_PRINT("ERROR queuing 3X task\r\n");
        }
        return;
    }
__cmderr:
    DBG_PRINT("Usage: 3as Ver|home|rel|abs\r\n");
}

static void console_ll_ctrl(int argc, char* argv[]) {
    ll_task_message_t msg = {
        .ll_cmd = LL_TASK_CMD_DEFAULT,
        .voltage = MAX14574_MAX_VOLT};
    if (argc >= 2) {
        if (STRCMP(argv[1], "ver") == 0) {
            msg.ll_cmd = LL_TASK_CMD_GET_CHIP_VERSION;
        } else if (STRCMP(argv[1], "setv") == 0) {
            msg.ll_cmd = LL_TASK_CMD_SET_VOLTAGE;
        } else if (STRCMP(argv[1], "getv") == 0) {
            msg.ll_cmd = LL_TASK_CMD_GET_VOLTAGE;
        } else if (STRCMP(argv[1], "status") == 0) {
            msg.ll_cmd = LL_TASK_CMD_GET_STATUS;
        } else if (STRCMP(argv[1], "fail") == 0) {
            msg.ll_cmd = LL_TASK_CMD_GET_FAIL;
        } else if (STRCMP(argv[1], "on") == 0) {
            msg.ll_cmd = LL_TASK_CMD_ENABLE;
        } else if (STRCMP(argv[1], "off") == 0) {
            msg.ll_cmd = LL_TASK_CMD_DISABLE;
        } else
            goto __cmderr;

        if (argc > 2) {
            msg.voltage = atoi(argv[2]);
        }
        BaseType_t xStatus;
        xStatus = xQueueSendToBack(queue_cmd_to_ll_task, &msg, pdMS_TO_TICKS(10));
        if (xStatus != pdPASS) {
            DBG_PRINT("ERROR queuing ll task\r\n");
        }
        return;
    }

__cmderr:
    DBG_PRINT("Usage: ll ver|setv|getv|status|fail LLV#\r\n");
}

static void console_miis_ll_ctrl(int argc, char* argv[]) {
    ll_task_message_t msg = {
        .ll_cmd = LL_TASK_CMD_DEFAULT,
        .diopter = DIOPTER_HOME};
    if (argc >= 2) {
        if (STRCMP(argv[1], "On") == 0) {
            msg.ll_cmd = LL_TASK_CMD_ENABLE;
        } else if (STRCMP(argv[1], "Off") == 0) {
            msg.ll_cmd = LL_TASK_CMD_DISABLE;
        } else if (STRCMP(argv[1], "Home") == 0) {
            msg.ll_cmd = LL_TASK_CMD_HOME;
        } else if (STRCMP(argv[1], "Goto") == 0) {
            msg.ll_cmd = LL_TASK_CMD_GOTO_ABS;
        } else if (STRCMP(argv[1], "Pos") == 0) {
            msg.ll_cmd = LL_TASK_CMD_GET_POSITION;
        } else if ((strncmp(argv[1], "+", 1) == 0) || (strncmp(argv[1], "-", 1) == 0)) {
            msg.ll_cmd = LL_TASK_CMD_GOTO_REL;
            msg.diopter = atoi(argv[1]);
        } else if (STRCMP(argv[1], "Temp") == 0) {
            msg.ll_cmd = LL_TASK_CMD_GET_TEMPERATURE;
        } else if (STRCMP(argv[1], "calD") == 0) {
            msg.ll_cmd = LL_TASK_CMD_GOTO_ABS_WITH_TEMP_CALIBRATION;
        } else
            goto __cmderr;

        if (argc > 2) {
            msg.diopter = atoi(argv[2]);
        }
        BaseType_t xStatus;
        xStatus = xQueueSendToBack(queue_cmd_to_ll_task, &msg, pdMS_TO_TICKS(10));
        if (xStatus != pdPASS) {
            DBG_PRINT("ERROR queuing ll task\r\n");
        }
        return;
    }

__cmderr:
    DBG_PRINT("Usage: motor On|Off|Home|Goto|+d|-d|Pos\r\n");
}

#if 1
static void console_sony_ctrl(int argc, char* argv[]) {
    sony_task_message_t msg = {
        .sony_cmd = SONY_TASK_CMD_DEFAULT};
    if (argc >= 2) {
        if (STRCMP(argv[1], "On") == 0) {
            msg.sony_cmd = SONY_TASK_CMD_ENABLE;
        } else if (STRCMP(argv[1], "Off") == 0) {
            msg.sony_cmd = SONY_TASK_CMD_DISABLE;
        } else if (STRCMP(argv[1], "S") == 0) {
            msg.sony_cmd = SONY_TASK_CMD_SHOW;
            if (argc == 8) {
                msg.x = atoi(argv[2]);
                msg.y = atoi(argv[3]);
                msg.w_size = atoi(argv[4]);
                msg.h_size = atoi(argv[5]);
                msg.pattern = atoi(argv[6]);
                msg.color = atoi(argv[7]);
            } else
                goto __cmderr;
        } else if (STRCMP(argv[1], "M") == 0) {
            msg.sony_cmd = SONY_TASK_CMD_FIXATION_MODE;
        } else if (STRCMP(argv[1], "Disc") == 0) {
            msg.sony_cmd = SONY_TASK_CMD_DISC;
        } else if (STRCMP(argv[1], "Full") == 0) {
            msg.sony_cmd = SONY_TASK_CMD_FULL;
            msg.pattern = SONY_PATTERN_FULL;
            if (argc == 3) {
                msg.color = atoi(argv[2]);
            } else
                goto __cmderr;
        } else if (STRCMP(argv[1], "Size") == 0) {
            msg.sony_cmd = SONY_TASK_CMD_CHANGE_SIZE;
            if (argc == 4) {
                msg.w_size = atoi(argv[2]);
                msg.h_size = atoi(argv[3]);
            } else
                goto __cmderr;
        } else if (STRCMP(argv[1], "Pos") == 0) {
            msg.sony_cmd = SONY_TASK_CMD_CHANGE_POSITION;
            if (argc == 4) {
                msg.x = atoi(argv[2]);
                msg.y = atoi(argv[3]);
            } else
                goto __cmderr;
        } else if (STRCMP(argv[1], "Info") == 0) {
            msg.sony_cmd = SONY_TASK_CMD_GET_INFO;
        } else if (STRCMP(argv[1], "Spd") == 0) {
            msg.sony_cmd = SONY_TASK_CMD_CHANGE_SPEED;
            if (argc == 3) {
                msg.speed = atof(argv[2]);
            } else
                goto __cmderr;
        } else if (STRCMP(argv[1], "Bri") == 0) {
            msg.sony_cmd = SONY_TASK_CMD_CHANGE_BRIGHTNESS;
            if (argc == 3) {
                msg.brightness = atoi(argv[2]);
            } else
                goto __cmderr;
        } else
            goto __cmderr;

        BaseType_t xStatus;
        xStatus = xQueueSendToBack(queue_cmd_to_sony_task, &msg, pdMS_TO_TICKS(10));
        if (xStatus != pdPASS) {
            DBG_PRINT("ERROR queuing sony task\r\n");
        }
        return;
    }

__cmderr:
    DBG_PRINT("Usage: sony On|Off|S|Pos|Size|Info\r\n");
}
#endif

static void console_solenoid_ctrl(int argc, char* argv[]) {
    if (argc == 2) {
        if (STRCMP(argv[1], "+") == 0) {
            // Reverse
            gpio_set_value(SOLENOID_CTRL1, GPIO_LOW);
            gpio_set_value(SOLENOID_CTRL2, GPIO_HIGH);
            osDelay(150);
            // Sleep mode
            gpio_set_value(SOLENOID_CTRL1, GPIO_LOW);
            gpio_set_value(SOLENOID_CTRL2, GPIO_LOW);
            ack();
        } else if (STRCMP(argv[1], "-") == 0) {
            // Forward
            gpio_set_value(SOLENOID_CTRL1, GPIO_HIGH);
            gpio_set_value(SOLENOID_CTRL2, GPIO_LOW);
            osDelay(150);
            // Sleep mode
            gpio_set_value(SOLENOID_CTRL1, GPIO_LOW);
            gpio_set_value(SOLENOID_CTRL2, GPIO_LOW);
            ack();
        } else if (STRCMP(argv[1], "off") == 0) {
            // Sleep mode
            gpio_set_value(SOLENOID_CTRL1, GPIO_LOW);
            gpio_set_value(SOLENOID_CTRL2, GPIO_LOW);
        }
    } else
        DBG_PRINT("Usage: sonoid +|-|off\r\n");
}

static void console_new_ll_ctrl(int argc, char* argv[]) {
    a_pe_25hx_task_message_t msg = {
        .ll_cmd = A_PE_25HX_TASK_CMD_DEFAULT,
        .voltage = MIN_VRMS,
        .diopter = DIOPTER_HOME,
        .enable = 0};
    if (argc >= 2) {
        if (STRCMP(argv[1], "opmode") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_OP_MODE_CTRL;
            if (argc > 2) {
                msg.enable = atoi(argv[2]);
            }
        } else if (STRCMP(argv[1], "opmode?") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_GET_OP_MODE_STATUS;
        } else if (STRCMP(argv[1], "vtemp") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_V_TEMP_CTRL;
            if (argc > 2) {
                msg.enable = atoi(argv[2]);
            }
        } else if (STRCMP(argv[1], "vtemp?") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_GET_V_TEMP_STATUS;
        } else if (STRCMP(argv[1], "vspeed") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_V_SPEED_CTRL;
            if (argc > 2) {
                msg.enable = atoi(argv[2]);
            }
        } else if (STRCMP(argv[1], "vspeed?") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_GET_V_SPEED_STATUS;
        } else if (STRCMP(argv[1], "ver") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_GET_VERSION;
        } else if (STRCMP(argv[1], "setv") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_SET_VOLTAGE;
            if (argc > 2) {
                double voltage = atof(argv[2]);
                msg.voltage = voltage;
            }
        } else if (STRCMP(argv[1], "getv") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_GET_VOLTAGE;
        } else if (STRCMP(argv[1], "getop") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_GET_OPTICAL_POWER;
        } else if (STRCMP(argv[1], "pos") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_GET_POSITION;
        } else if (STRCMP(argv[1], "home") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_HOME;
        } else if (STRCMP(argv[1], "goto") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_GOTO_ABS;
            // Use strtoul to correctly handle unsigned long values
            // when converting hexadecimal input, ensuring no negative values are processed.
            msg.diopter_hex = strtoul(argv[2], NULL, 16);
        } else if (STRCMP(argv[1], "as") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_AS_MODE;
        } else if ((strncmp(argv[1], "+", 1) == 0) || (strncmp(argv[1], "-", 1) == 0)) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_GOTO_REL;
            msg.diopter = atoi(argv[1]);
        } else if (STRCMP(argv[1], "temp?") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_GET_TEMPERATURE;
        } else if (STRCMP(argv[1], "status?") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_GET_STATUS;
        } else if (STRCMP(argv[1], "fail?") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_GET_FAIL;
        } else
            goto __cmderr;

        BaseType_t xStatus;
        xStatus = xQueueSendToBack(queue_cmd_to_new_ll_task, &msg, pdMS_TO_TICKS(10));
        if (xStatus != pdPASS) {
            DBG_PRINT("ERROR queuing ll task\r\n");
        }
        return;
    }

__cmderr:
    DBG_PRINT("Usage: nl ver|home|goto|+d|-d\r\n");
}

static void console_i2c_debug_ctrl(int argc, char* argv[]) {
    if (argc == 2) {
        if (STRCMP(argv[1], "on") == 0) {
            set_i2c_debug_print_flag(TRUE);
        } else if (STRCMP(argv[1], "off") == 0) {
            set_i2c_debug_print_flag(FALSE);
        }
    } else
        DBG_PRINT("Usage: dbg on|off\r\n");
}
static void default_cmd_register(void) {
    cmd_t* pcmd = &default_cmd_list[0];

    while (pcmd->cmd != NULL) {
        cmdRegister(pcmd);
        pcmd += 1;
    }
}

static int console_fgetchar(void) {
    INT8U input_ch = 0;

#if _DRV_L1_UART2 == 1
// while (drv_l1_uart2_data_get(&input_ch, 0) != STATUS_OK)
#endif
#if _DRV_L1_UART1 == 1
    while (drv_l1_uart1_data_get(&input_ch, 0) != STATUS_OK)
#endif
#if _DRV_L1_UART0 == 1
        while (drv_l1_uart0_data_get(&input_ch, 0) != STATUS_OK)
#endif
        {
#if _OPERATING_SYSTEM != _OS_NONE
#if _OPERATING_SYSTEM == _OS_UCOS2
            OSTimeDly(1);
#elif _OPERATING_SYSTEM == _OS_FREERTOS
            if (cmdTaskHandle != NULL) {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                // Timeout_Update(); // update timeout after console get input
            } else
                osDelay(1);
#else
#error "which OS api for OSTimeDly ?"
#endif
#endif
        }
    return (int)(input_ch);
}

static int isaspace(char c) {
    return (c == ' ' || c == '\t' || c == '\n' || c == '\12');
}

static char* gpstrdup(const char* str) {
    size_t siz;
    char*  copy;

    siz = STRLEN(str) + 1;
    if ((copy = (char*)MEMALLOC(siz)) == NULL)
        return NULL;
    MEMCPY(copy, str, siz);

    return copy;
}

static int setargs(char* args, char** argv) {
    int count = 0;

    while (isaspace(*args))
        ++args;
    while (*args) {
        if (argv)
            argv[count] = args;
        while (*args && !isaspace(*args))
            ++args;
        if (argv && *args)
            *args++ = '\0';
        while (isaspace(*args))
            ++args;
        count++;
    }
    return count;
}

static char** parsedargs(char* args, int* argc) {
    char** argv = NULL;
    int    argn = 0;

    if (args && *args) {
        args = (char*)gpstrdup(args);
        if (args) {
            argn = setargs(args, NULL);
            if (argn) {
                argv = (char**)MEMALLOC((argn + 1) * sizeof(char*));
                if (argv) {
                    *argv++ = args;
                    argn = setargs(args, argv);
                }
            }
        }
    }
    if (args && !argv)
        FREE(args);

    *argc = argn;
    return argv;
}

static void freeparsedargs(char** argv) {
    if (argv) {
        FREE(argv[-1]);
        FREE(argv - 1);
    }
}

int parse_to_argv(char* input_str, char*** out_av) {
    char** av;
    int    ac;
    char*  as = NULL;

    as = input_str;

    av = parsedargs(as, &ac);

#if 0
    int i;
    printf("== %d\r\n",ac);
    for (i = 0; i < ac; i++)
        DBG_PRINT("[%s]\r\n",av[i]);
#endif

    *out_av = av;

    return ac;
}
#if RTK_CONSOLE == 0
static void cmdProcess(
    unsigned char* cmd,
    unsigned int   repeating) {
    cmd_t*       bc = pcmds;
    unsigned int idx = 0;
    unsigned int copy = 0;
    int          argc = 0;
    char**       argv = NULL;
    int          found = 0;

    /*
     * Strip the white space from the command.
     */
    while (cmd[idx] != '\0') {
        if ((cmd[idx] != ' ') &&
            (cmd[idx] != '\t') &&
            (cmd[idx] != '\r') &&
            (cmd[idx] != '\n')) {
            break;
        }
        idx++;
    }

    if (idx > 0) {
        /* Reached a non-white space character, compact the string */
        while (cmd[idx] != '\0') {
            cmd[copy++] = cmd[idx++];
        }
        cmd[copy] = '\0';
    }

    /*
     * Index points to the end of the string, move backwards.
     */
    idx = STRLEN(cmd);

    while (idx > 0) {
        idx--;
        if ((cmd[idx] == ' ') ||
            (cmd[idx] == '\t') ||
            (cmd[idx] == '\r') ||
            (cmd[idx] == '\n')) {
            cmd[idx] = '\0';
        } else {
            break;
        }
    }

    /*
     * Find the command.
     */
    idx = 0;

    while (cmd[idx] != '\0') {
        if ((cmd[idx] == ' ') ||
            (cmd[idx] == '\t') ||
            (cmd[idx] == '\r') ||
            (cmd[idx] == '\n')) {
            break;
        }
        idx++;
    }

    argc = parse_to_argv((char*)cmd, &argv);

    cmd[idx] = '\0';

    if (argc > 0) {
        while (bc) {
            // DBG_PRINT("check %s, %s\r\n", bc->cmd, (argc == 0) ? "" : argv[0]);

            if (STRCMP(bc->cmd, argv[0]) == 0) {
                (bc->phandler)(argc, argv);
                found = 1;
                break;
            }
            bc = bc->pnext;
        }
    }

    if (!found) {
        DBG_PRINT("command `%s' not found, try `all help'\r\n", (argc == 0) ? "" : argv[0]);
        uart2_printf("receive\r\n");
    }

    if (argv != NULL) {
        freeparsedargs(argv);
        argv = NULL;
    }
}
#endif

static unsigned int cmdIdxIncrease(
    unsigned int* pcmdIdx) {
    unsigned int localIdx;
    unsigned int ret = 0;

    localIdx = *pcmdIdx;
    localIdx++;
    if (localIdx == CMD_HISTORIES) {
        localIdx = 0;
        ret = 1;
    }
    *pcmdIdx = localIdx;

    return ret;
}

static unsigned int cmdFlushCopy(
    unsigned int   cursorPos,
    unsigned char* pcmdBuf,
    unsigned char* pcmdSrc,
    unsigned int   cmdLen) {
    if (cursorPos > 0) {
        for (; cursorPos > 0; cursorPos--) {
            DBG_PRINT("\b \b");
            cmdBuffer[cursorPos] = '\0';
        }
    }
    MEMCPY(pcmdBuf, pcmdSrc, cmdLen);

    return 0;
}

/**
 * @brief   This function is a duplicated of cmdMonitor. To handle the received characters from UART2 (Microchip PIC32).
 * @param   c : character from UART2 (Microchip PIC32)
 * @return  none
 */
void cmdMonitor_UART2(INT8U c) {
    unsigned int        repeating;
    unsigned int        histDownArw;
    static unsigned int upArrowCnt;

    switch (c) {
    case '\b':
    case '\x7f':
        if (pos > 0) {
            DBG_PRINT("\b \b");
            pos--;
        }
        cmdBuffer[pos] = '\0';
        break;

    case '\r': /* Process the command. */
        DBG_PRINT("\r\n");
        if (pos) {
            /*
             * Do not place the same last command into the history if the same.
             */
            if (STRCMP((unsigned char*)histBuf[histIns], cmdBuffer)) {
                if (cmdIdxIncrease(&histIns) == 1) {
                    histInsWrap = 1;
                }
                MEMCPY(histBuf[histIns], cmdBuffer, CMD_BUF_SIZE);
                histPos[histIns] = pos;
            }
            histOutput = histIns;
            histOutputWrap = 0;
            upArrowCnt = 0;
            repeating = FALSE;
        }
        if (pos) {
#if RTK_CONSOLE == 1
            /*extern char log_buf[128];
            extern xSemaphoreHandle log_rx_interrupt_sema;
            MEMCPY(log_buf,cmdBuffer, CMD_BUF_SIZE);
            xSemaphoreGive(log_rx_interrupt_sema);*/
            log_handler((char*)cmdBuffer);
#else
            cmdProcess(cmdBuffer, repeating);
#endif
            pos = 0;
            MEMSET(cmdBuffer, 0, CMD_BUF_SIZE);
            DBG_PRINT("\r\n");
        }
        // DBG_PRINT("cmd>");
        break;

    case '[': /* Non ASCII characters, arrow. */
        c = GETCH();
        switch (c) {
        case 'A': /* Key: up arrow */
            if (histOutputWrap == 1) {
                if (histOutput == histIns) {
                    break;
                }
            }
            if (histInsWrap == 0) {
                if (histOutput == 0) {
                    break;
                }
            }
            upArrowCnt++;
            cmdFlushCopy(
                pos,
                cmdBuffer,
                (unsigned char*)histBuf[histOutput],
                histPos[histOutput]);
            pos = histPos[histOutput];
            cmdBuffer[pos + 1] = '\0';
            DBG_PRINT((CHAR*)cmdBuffer);
            if (histInsWrap == 1) {
                if (histOutput == 0) {
                    histOutput = CMD_HISTORIES - 1;
                    histOutputWrap = 1;
                } else {
                    histOutput--;
                }
            } else {
                if (histOutput != 0) {
                    /* Note that when wrap around does not occur, the least
                     * of index is 1 because it is the first one to be
                     * written.
                     */
                    histOutput--;
                }
                /* Nothing to do with histOutput == 0,
                 * because there is no more commands.
                 */
            }
            break;

        case 'B': /* Key: down arrow */
            if (upArrowCnt <= 1) {
                break;
            }
            upArrowCnt--;
            cmdIdxIncrease(&histOutput);
            histDownArw = histOutput;
            cmdIdxIncrease(&histDownArw);
            cmdFlushCopy(
                pos,
                cmdBuffer,
                (unsigned char*)histBuf[histDownArw],
                histPos[histDownArw]);
            pos = histPos[histDownArw];
            cmdBuffer[pos + 1] = '\0';
            DBG_PRINT((CHAR*)cmdBuffer);
            break;

        case 'C': /* Key: right arrow */
            break;
        case 'D': /* Key: left arrow */
            break;
        default:
            if ((pos < (CMD_BUF_SIZE - 1)) && (c >= ' ') && (c <= 'z')) {
                cmdBuffer[pos++] = c;
                cmdBuffer[pos] = '\0';
                DBG_PRINT((CHAR*)cmdBuffer + pos - 1);
            }
            if (c == '\x7e') {
                cmdBuffer[pos++] = c;
                cmdBuffer[pos] = '\0';
                DBG_PRINT((CHAR*)cmdBuffer + pos - 1);
            }
            break;
        }
        break;

    default:
        if ((pos < (CMD_BUF_SIZE - 1)) && (c >= ' ') && (c <= 'z')) {
            cmdBuffer[pos++] = c;
            cmdBuffer[pos] = '\0';
            DBG_PRINT((CHAR*)cmdBuffer + pos - 1);
        }
        if (c == '\x7e') {
            cmdBuffer[pos++] = c;
            cmdBuffer[pos] = '\0';
            DBG_PRINT((CHAR*)cmdBuffer + pos - 1);
        }
        break;
    }
}

void cmdMonitor(void) {
    unsigned char       c;
    unsigned int        repeating;
    unsigned int        histDownArw;
    static unsigned int upArrowCnt;

    if (!logIn) {
        // if ( c == '\r' )
        {
            logIn = TRUE;
            DBG_PRINT("\r\n\r\ncmd>");
        }
    } else {
        c = (unsigned char)GETCH();

        switch (c) {
        case '\b':
        case '\x7f':
            if (pos > 0) {
                DBG_PRINT("\b \b");
                pos--;
            }
            cmdBuffer[pos] = '\0';
            break;

        case '\r': /* Process the command. */
            DBG_PRINT("\r\n");
            if (pos) {
                /*
                 * Do not place the same last command into the history if the same.
                 */
                if (STRCMP((unsigned char*)histBuf[histIns], cmdBuffer)) {
                    if (cmdIdxIncrease(&histIns) == 1) {
                        histInsWrap = 1;
                    }
                    MEMCPY(histBuf[histIns], cmdBuffer, CMD_BUF_SIZE);
                    histPos[histIns] = pos;
                }
                histOutput = histIns;
                histOutputWrap = 0;
                upArrowCnt = 0;
                repeating = FALSE;
            }
            if (pos) {
#if RTK_CONSOLE == 1
                /*extern char log_buf[128];
                extern xSemaphoreHandle log_rx_interrupt_sema;
                MEMCPY(log_buf,cmdBuffer, CMD_BUF_SIZE);
                xSemaphoreGive(log_rx_interrupt_sema);*/
                log_handler((char*)cmdBuffer);
#else
                cmdProcess(cmdBuffer, repeating);
#endif
                pos = 0;
                MEMSET(cmdBuffer, 0, CMD_BUF_SIZE);
                DBG_PRINT("\r\n");
            }
            // DBG_PRINT("cmd>");
            break;

        case '[': /* Non ASCII characters, arrow. */
            c = GETCH();
            switch (c) {
            case 'A': /* Key: up arrow */
                if (histOutputWrap == 1) {
                    if (histOutput == histIns) {
                        break;
                    }
                }
                if (histInsWrap == 0) {
                    if (histOutput == 0) {
                        break;
                    }
                }
                upArrowCnt++;
                cmdFlushCopy(
                    pos,
                    cmdBuffer,
                    (unsigned char*)histBuf[histOutput],
                    histPos[histOutput]);
                pos = histPos[histOutput];
                cmdBuffer[pos + 1] = '\0';
                DBG_PRINT((CHAR*)cmdBuffer);
                if (histInsWrap == 1) {
                    if (histOutput == 0) {
                        histOutput = CMD_HISTORIES - 1;
                        histOutputWrap = 1;
                    } else {
                        histOutput--;
                    }
                } else {
                    if (histOutput != 0) {
                        /* Note that when wrap around does not occur, the least
                         * of index is 1 because it is the first one to be
                         * written.
                         */
                        histOutput--;
                    }
                    /* Nothing to do with histOutput == 0,
                     * because there is no more commands.
                     */
                }
                break;

            case 'B': /* Key: down arrow */
                if (upArrowCnt <= 1) {
                    break;
                }
                upArrowCnt--;
                cmdIdxIncrease(&histOutput);
                histDownArw = histOutput;
                cmdIdxIncrease(&histDownArw);
                cmdFlushCopy(
                    pos,
                    cmdBuffer,
                    (unsigned char*)histBuf[histDownArw],
                    histPos[histDownArw]);
                pos = histPos[histDownArw];
                cmdBuffer[pos + 1] = '\0';
                DBG_PRINT((CHAR*)cmdBuffer);
                break;

            case 'C': /* Key: right arrow */
                break;
            case 'D': /* Key: left arrow */
                break;
            default:
                if ((pos < (CMD_BUF_SIZE - 1)) && (c >= ' ') && (c <= 'z')) {
                    cmdBuffer[pos++] = c;
                    cmdBuffer[pos] = '\0';
                    DBG_PRINT((CHAR*)cmdBuffer + pos - 1);
                }
                if (c == '\x7e') {
                    cmdBuffer[pos++] = c;
                    cmdBuffer[pos] = '\0';
                    DBG_PRINT((CHAR*)cmdBuffer + pos - 1);
                }
                break;
            }
            break;

        default:
            if ((pos < (CMD_BUF_SIZE - 1)) && (c >= ' ') && (c <= 'z')) {
                cmdBuffer[pos++] = c;
                cmdBuffer[pos] = '\0';
                DBG_PRINT((CHAR*)cmdBuffer + pos - 1);
            }
            if (c == '\x7e') {
                cmdBuffer[pos++] = c;
                cmdBuffer[pos] = '\0';
                DBG_PRINT((CHAR*)cmdBuffer + pos - 1);
            }
            break;
        }
    } /* else of if !logged_in */
}

void cmdUartRxIsr(INT8U dev_idx, INT8U ch) {
#if _OPERATING_SYSTEM != _OS_NONE
#if _OPERATING_SYSTEM == _OS_UCOS2
#elif _OPERATING_SYSTEM == _OS_FREERTOS
    BaseType_t xHigherPriorityTaskWoken = 0;

    vTaskNotifyGiveFromISR(cmdTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#else
#error "which OS api for vTaskNotifyGiveFromISR ?"
#endif
#endif
}

void Cmd_Task(void* para) {
#if _OPERATING_SYSTEM != _OS_NONE
#if _OPERATING_SYSTEM == _OS_UCOS2
#elif _OPERATING_SYSTEM == _OS_FREERTOS
    if (cmdTaskHandle != NULL) {
#if _DRV_L1_UART2 == 1
// drv_l1_uart2_rx_isr_set(cmdUartRxIsr);
#endif
#if _DRV_L1_UART1 == 1
        drv_l1_uart1_rx_isr_set(cmdUartRxIsr);
#endif
#if _DRV_L1_UART0 == 1
        drv_l1_uart0_rx_isr_set(cmdUartRxIsr);
#endif
    }
#else
#endif
#endif

    default_cmd_register();

    while (1) {
        cmdMonitor();
    }
}

void cmdRegister(cmd_t* bc) {
    cmd_t* prev;
    cmd_t* curr;

    bc->pnext = NULL;
    if (pcmds == NULL) {
        pcmds = bc;
    } else {
        prev = NULL;
        curr = pcmds;
        while (curr) {
            /* The list is sorted by alphabetic order. */
            if (STRCMP(bc->cmd, curr->cmd) <= 0) {
                bc->pnext = curr;
                if (prev) {
                    prev->pnext = bc;
                } else {
                    pcmds = bc;
                }
                return;
            }
            prev = curr;
            curr = curr->pnext;
        }

        /* Last on the list. */

        prev->pnext = bc;
    } /* else boot_commands */
}
