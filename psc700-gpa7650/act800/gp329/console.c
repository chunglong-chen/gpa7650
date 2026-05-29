/**
 * @file    console.c
 * @brief   Console command parser and command dispatch module.
 * @ingroup command_parser
 * @details
 * This module manages console command input,command parsing, and dispatching to corresponding command handlers.
 *
 * It also provides default help handling and integrates with
 * multiple subsystem command queues for device control.
 */
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
#include <errno.h>
#include <math.h>     // isfinite
#include <ctype.h>

#define CMD_HISTORIES 4
#define CMD_BUF_SIZE  128

#if _OPERATING_SYSTEM != _OS_NONE
	#if _OPERATING_SYSTEM == _OS_UCOS2
	#elif _OPERATING_SYSTEM == _OS_FREERTOS
	extern xTaskHandle cmdTaskHandle;
	#else
	#endif
#endif


static cmd_t  *pcmds = NULL;
static unsigned char cmdBuffer[CMD_BUF_SIZE];
static unsigned int pos = 0;
static unsigned int logIn = 0;

static unsigned int histBuf[CMD_HISTORIES][CMD_BUF_SIZE];
static unsigned int histPos[CMD_HISTORIES];
static unsigned int histIns;
static unsigned int histOutput;
static unsigned int histInsWrap;
static unsigned int histOutputWrap;



static void default_cmd_handler(int argc, char *argv[]);
static void console_get_version(int argc, char* argv[]);
static void console_probebd(int argc, char *argv[]);
static void console_pupil_ir(int argc, char* argv[]);

static void console_motor_ctrl(int argc, char* argv[]);
static void console_sony_ctrl(int argc, char* argv[]);

static void console_ap_mode_ctrl(int argc, char *argv[]);
static void console_solenoid_ctrl(int argc, char* argv[]);

static void console_new_ll_ctrl(int argc, char* argv[]);

static void console_gpio_read_test(int argc, char* argv[]);
static void console_reset_usb_hub(int argc, char* argv[]);

int parse_cmd_seq(const char* line);
static inline int parse_long_in_range(const char* s, long min, long max, long* out);
static inline int parse_double_in_range(const char* s, double min, double max, double* out);
static inline int parse_decimal_str(const char* s, int allow_sign);
static INT32S miis_3axis_sign_to_val(const char *s, INT8S *val);
extern xQueueHandle queue_cmd_to_microchip;
extern xQueueHandle queue_cmd_to_irled_task;
extern xQueueHandle queue_cmd_to_3axis_task;
extern xQueueHandle queue_cmd_to_ll_task;
extern xQueueHandle queue_cmd_to_new_ll_task;
extern xQueueHandle queue_cmd_to_sony_task;
extern xQueueHandle queue_cmd_to_asps_switch_task;
#ifdef WIFI_DEMO
void* log_handler(char *cmd);
#endif
/**
 * @brief   Console command lookup table.
 * @details Maps console command strings to their corresponding handler functions.
 *          This table is used by the console parser to dispatch incoming
 *          commands to the appropriate handler.
 *
 *          Each entry consists of:
 *          - cmd      : Command string entered from console
 *          - handler  : Corresponding command handler function
 *          - pnext    : Pointer for linked-list chaining (reserved/optional)
 *
 *          The table is terminated by a NULL entry (NULL, NULL, NULL).
 *
 * @note    This table is part of the console command parsing mechanism.
 */
static cmd_t default_cmd_list[] =
{
	{"all",   default_cmd_handler,  NULL },/**< Default command handler */
	{"ap", console_ap_mode_ctrl, NULL},/**< AP mode control */
	{"gpio?", console_gpio_read_test, NULL},
	{"ll", console_new_ll_ctrl, NULL},/**< Liquid lens control */
	{"probebd", console_probebd, NULL},/**< Probe board control */
    {"pir", console_pupil_ir, NULL},/**< Pupil IR control */
    {"rstusb", console_reset_usb_hub, NULL},
    {"sonoid", console_solenoid_ctrl, NULL},/**< Solenoid control */
    {"ud", console_sony_ctrl, NULL}, /**< Sony device control */
    {"version", console_get_version, NULL},
    {"3as", console_motor_ctrl, NULL},/**< 3-axis motor control */

	{NULL,    NULL,   NULL}
};
/**
 * @brief   Display usage help for all registered console commands.
 * @details
 * This function iterates through the registered command list
 * and prints the usage format for each command.
 *
 * The output format is:
 *     <command> help
 *
 * Only commands linked through the command list (pcmds) are displayed.
 */
static void default_cmd_help(void)
{
	cmd_t *curr;

	curr = pcmds->pnext;
	if (curr != NULL)
		DBG_PRINT("\r\nUsage:\r\n");
	while ( curr )
	{
		DBG_PRINT("    %s help\r\n", curr->cmd);
		curr = curr->pnext;
	}
}
/**
 * @brief   Default console command handler.
 * @param   argc Argument count.
 * @param   argv Argument vector.
 * @details
 * This function handles the default command behavior.
 *
 * If the second argument is "help", it displays the help message.
 * For any other input, it also falls back to displaying help.
 *
 * This ensures that unknown or incomplete commands always
 * provide usage guidance to the user.
 */
static void default_cmd_handler(int argc, char *argv[])
{
	if (STRCMP(argv[1],"help") == 0)
    {
    	default_cmd_help();
    }
    else
    {
       	default_cmd_help();
    }
}

/**
 * @brief Handle "ap" (AP/PS mode control) console command.
 *
 * Grammar:
 *   #[seq] ap set <as|ps>
 *   #[seq] ap get <mode|pi|status|all>
 *   #[seq] ap move <as|ps> <delay_ms>
 *
 *
 * Behavior:
 *   - "set" sends a mode switch request to the ASPS switch task queue.
 *   - "get" sends a status query request to the ASPS switch task queue.
 *   - "move" directly controls GPIO pins to switch hardware mode,
 *
 * ACK/NAK policy:
 *   - For "set" and "get", success response is expected to be produced
 *     by the ASPS switch task after processing the queued message.
 *   - For "move", ACK is produced immediately in this function on success.
 *
 * @param argc Number of arguments.
 * @param argv Argument list:
 *             argv[0]="ap", argv[1]=subcommand, argv[2...]=parameters.
 * @return void (N/A)
 */
static void console_ap_mode_ctrl(int argc, char* argv[]) {
    char line[256] = {0};
    int i;
    for (i = 0; i < argc; i++) {
        strncat(line, argv[i], sizeof(line) - strlen(line) - 1);
        if (i < argc - 1) strncat(line, " ", sizeof(line) - strlen(line) - 1);
    }

    int seq_id = parse_cmd_seq(line);
    if (seq_id == 0) {
        fmt_err_with_info(argv[0]);
        return;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", seq_id);
    asps_task_message_t msg =
    {
        .mode = 0,
        .index = 0
    };
    if(STRCMP(argv[1], "set") == 0) {
        if (argc !=4) {fmt_err_with_info(buf); return;}

        if (STRCMP(argv[2], "as") == 0) {
            msg.mode = ASPS_TASK_CMD_SET_AS_MODE;
        } else if (STRCMP(argv[2], "ps") == 0) {
            msg.mode = ASPS_TASK_CMD_SET_PS_MODE;
        } else {
            fmt_err_with_info(buf);
            return;
        }
        msg.index=seq_id;
    }else if(STRCMP(argv[1], "get") == 0) {
        if (argc !=4) {fmt_err_with_info(buf); return;}

        if (STRCMP(argv[2], "mode") == 0) {
            msg.mode = ASPS_TASK_CMD_GET_MODE;
        } else if (STRCMP(argv[2], "pi") == 0) {
            msg.mode = ASPS_TASK_CMD_GET_PI_GPIO;
        } else if (STRCMP(argv[2], "status") == 0) {
            msg.mode = ASPS_TASK_CMD_GET_GPIO_STATUS;
        }else if (STRCMP(argv[2], "all") == 0) {
            msg.mode = ASPS_TASK_CMD_GET_ALL;
        } else {
            fmt_err_with_info(buf);
            return;
        }
        msg.index=seq_id;
    }else if(STRCMP(argv[1], "move") == 0) {
        if (argc !=5) {fmt_err_with_info(buf); return;}
        char *endptr = NULL;
        long delay_ms = strtol(argv[3], &endptr, 10);
        if (STRCMP(argv[2], "as") == 0) {
            if (*endptr != '\0') {fmt_err_with_info(buf); return;}
            gpio_set_value(ASPS_CTRL1, GPIO_LOW);
            gpio_set_value(ASPS_CTRL2, GPIO_HIGH);
            //gpio_set_value(USB_HUB_RST, GPIO_HIGH);
            osDelay(delay_ms);
            //gpio_set_value(USB_HUB_RST, GPIO_LOW);
            gpio_set_value(ASPS_CTRL1, GPIO_HIGH);
            gpio_set_value(ASPS_CTRL2, GPIO_HIGH);
            ack_with_info(buf,NULL);

        } else if (STRCMP(argv[2], "ps") == 0) {
            if (*endptr != '\0') {fmt_err_with_info(buf); return;}
            gpio_set_value(ASPS_CTRL1, GPIO_HIGH);
            gpio_set_value(ASPS_CTRL2, GPIO_LOW);
            //gpio_set_value(USB_HUB_RST, GPIO_HIGH);
            osDelay(delay_ms);
           //gpio_set_value(USB_HUB_RST, GPIO_LOW);
            gpio_set_value(ASPS_CTRL1, GPIO_HIGH);
            gpio_set_value(ASPS_CTRL2, GPIO_HIGH);
            ack_with_info(buf,NULL);
        } else {
            fmt_err_with_info(buf);
            return;
        }
    }else {
        fmt_err_with_info(buf);
        return;
    }
    BaseType_t xStatus = xQueueSendToBack(queue_cmd_to_asps_switch_task, &msg, pdMS_TO_TICKS(10));
    if (xStatus != pdPASS) {
        DBG_PRINT("ERROR queuing ASPS task\r\n");
        nak_with_info(buf);
    }
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



/**
 * @brief Handle "ll" (liquid lens) console command.
 *
 * Grammar (per spec images):
 *   #[seq] ll openable <1|0>
 *   #[seq] ll setop    <float:-5.0~10.0>   // requires OP mode enabled
 *   #[seq] ll setv     <float:24.0~70.0>   // requires OP mode disabled
 *   #[seq] ll info     <getmo|getop|getvo>
 *
 * Rules:
 *   - Missing/invalid #[seq]  -> [ll] nak           (via fmt_err_with_info(argv[0]))
 *   - Any parse error          -> [seq] fmt_err_with_info
 *   - Queue failure            -> [seq] nak_with_info
 *   - ACK should be produced by the task upon success.
 *
 * @param argc Number of arguments.
 * @param argv Argument list: argv[0]="ll", argv[1]=subcommand, argv[2]=params.
 * @return void (N/A)
 */
static void console_new_ll_ctrl(int argc, char* argv[]) {
    /* 1) Parse #[seq] */
    char line[256] = {0};
    int i = 0;
    for (i = 0; i < argc; i++) {
        strncat(line, argv[i], sizeof(line) - strlen(line) - 1);
        if (i < argc - 1) strncat(line, " ", sizeof(line) - strlen(line) - 1);
    }

    int seq_id = parse_cmd_seq(line);
    if (seq_id == 0) {                 /* No #[int] or invalid format */
        fmt_err_with_info(argv[0]);    /* [ll] nak (format error) */
        return;
    }

    char buf[16];
    snprintf(buf, sizeof(buf), "%d", seq_id);

    if (argc <= 1) { fmt_err_with_info(buf); return; }

    /* 2) Prepare message with defaults, carry seq via .index */
    a_pe_25hx_task_message_t msg = {
        .ll_cmd  = A_PE_25HX_TASK_CMD_DEFAULT,
        .voltage = MIN_VRMS,
        .diopter = DIOPTER_HOME,
        .enable  = 0,
        .index   = seq_id
    };

    /* ---- openable <1|0> ---- */
    if (STRCMP(argv[1], "openable") == 0) {
        if (argc != 4) { fmt_err_with_info(buf); return; }

        long en;
        if (!parse_long_in_range(argv[2], 0, 1, &en)) { fmt_err_with_info(buf); return; }

        msg.ll_cmd = A_PE_25HX_TASK_CMD_OP_MODE_CTRL;
        msg.enable = (INT8U)en;

        BaseType_t xStatus = xQueueSendToBack(queue_cmd_to_new_ll_task, &msg, pdMS_TO_TICKS(10));
        if (xStatus != pdPASS) { DBG_PRINT("ERROR queuing ll task\r\n"); nak_with_info(buf); }
        return;
    }

    /* ---- setop <float -5.0~10.0> ---- */
    if (STRCMP(argv[1], "setop") == 0) {
        if (argc != 4) { fmt_err_with_info(buf); return; }

        /* Parse 32-bit hex (with or without 0x prefix). */
        char *end = NULL;
        errno = 0;
        unsigned long bits_ul = strtoul(argv[2], &end, 16);   /* base 16; accepts 0x prefix as well */
        if (errno == ERANGE || end == argv[2] || *end != '\0') {
            /* ERANGE: overflow; end==argv[2]: no digits; *end!=0: trailing junk */
            fmt_err_with_info(buf);
            return;
        }
        /* Reinterpret bits as float for range validation. */
        uint32_t bits = (uint32_t)bits_ul;
        float f;
        memcpy(&f, &bits, sizeof(f));          /* avoid strict-aliasing UB */

        /* Validate numeric value: finite and within [-5.0, 10.0]. */
        if (!isfinite(f) || f < -5.0f || f > 10.0f) {
            fmt_err_with_info(buf);
            return;
        }

        /* Send the original 32-bit pattern to the task. */
        msg.ll_cmd      = A_PE_25HX_TASK_CMD_GOTO_ABS;
        msg.diopter_hex = bits;

        BaseType_t xStatus = xQueueSendToBack(queue_cmd_to_new_ll_task, &msg, pdMS_TO_TICKS(10));
        if (xStatus != pdPASS) {
            DBG_PRINT("ERROR queuing ll task\r\n");
            nak_with_info(buf);
        }
        return;
    }

    /* ---- setv <float 24.0~70.0> ---- */
    if (STRCMP(argv[1], "setv") == 0) {
        if (argc != 4) { fmt_err_with_info(buf); return; }

        double v;
        if (!parse_double_in_range(argv[2], 24.0, 70.0, &v)) { fmt_err_with_info(buf); return; }

        msg.ll_cmd  = A_PE_25HX_TASK_CMD_SET_VOLTAGE;
        msg.voltage = v;

        BaseType_t xStatus = xQueueSendToBack(queue_cmd_to_new_ll_task, &msg, pdMS_TO_TICKS(10));
        if (xStatus != pdPASS) { DBG_PRINT("ERROR queuing ll task\r\n"); nak_with_info(buf); }
        return;
    }

    /* ---- info <getmo|getop|getvo|version> ---- */
    if (STRCMP(argv[1], "info") == 0) {
        if (argc != 4) { fmt_err_with_info(buf); return; }

        if (STRCMP(argv[2], "getmo") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_GET_OP_MODE_STATUS;
        } else if (STRCMP(argv[2], "getop") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_GET_OPTICAL_POWER;
        } else if (STRCMP(argv[2], "getvo") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_GET_VOLTAGE;
        } else if (STRCMP(argv[2], "version") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_GET_VERSION;
        } else if (STRCMP(argv[2], "gettemp") == 0) {
            msg.ll_cmd = A_PE_25HX_TASK_CMD_GET_TEMPERATURE;
        } else {
            fmt_err_with_info(buf);
            return;
        }

        BaseType_t xStatus = xQueueSendToBack(queue_cmd_to_new_ll_task, &msg, pdMS_TO_TICKS(10));
        if (xStatus != pdPASS) { DBG_PRINT("ERROR queuing ll task\r\n"); nak_with_info(buf); }
        return;
    }

    /* Unknown subcommand */
    fmt_err_with_info(buf);
}

/**
 * @brief Handle "probebd" console command.
 *
 * Assemble argv into a line, parse sequence ID, and process subcommands:
 *   - info version/date/time/git
 * Responds with firmware/build info, or NAK on error.
 *
 * Supported subcommands under "info":
 *   - version : prints firmware version from `FIRMWARE_VERSION`
 *   - date    : prints YYYY-MM-DD from `DATETIME` (expects length >= 8)
 *   - time    : prints HH:MM:SS from `DATETIME` (expects length >= 14)
 *   - git     : prints `MIIS_VERSION`, `FIRMWARE_VERSION`, `GIT_HASH`, `GIT_BRANCH`
 *
 * @param argc Number of arguments.
 * @param argv Argument list: argv[0]="probebd", argv[1]="info", argv[2]=subcommand.
 * @return void (N/A)
 */
static void console_probebd(int argc, char* argv[]) {
    char line[256] = {0};
    int i = 0;
    for (i = 0; i < argc; i++) {
        strncat(line, argv[i], sizeof(line) - strlen(line) - 1);
        if (i < argc - 1) {
            strncat(line, " ", sizeof(line) - strlen(line) - 1);
        }
    }
    int seq_id = parse_cmd_seq(line);
    if (seq_id == 0) {
        fmt_err_with_info(argv[0]);
        return;
    }
    //
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", seq_id);
    if (argc == 4 && STRCMP(argv[1], "info") == 0) {
        if (STRCMP(argv[2], "version") == 0) {
            DBG_PRINT("[%d] %s\r\n", seq_id, FIRMWARE_VERSION);
            uart2_printf("[%d] ack %s\r\n", seq_id, FIRMWARE_VERSION);
            return;

        } else if (STRCMP(argv[2], "date") == 0) {
            if (!DATETIME || strlen(DATETIME) < 8) { fmt_err_with_info(buf); return; }
            char year[5], month[3], day[3];
            strncpy(year,  DATETIME,     4); year[4]  = '\0';
            strncpy(month, DATETIME + 4, 2); month[2] = '\0';
            strncpy(day,   DATETIME + 6, 2); day[2]   = '\0';

            DBG_PRINT("[%d] %s-%s-%s\r\n", seq_id, year, month, day);
            uart2_printf("[%d] ack %s-%s-%s\r\n", seq_id, year, month, day);
            return;

        } else if (STRCMP(argv[2], "time") == 0) {
            if (!DATETIME || strlen(DATETIME) < 14) { fmt_err_with_info(buf); return; }
            char hour[3], minute[3], second[3];
            strncpy(hour,   DATETIME + 8,  2); hour[2]   = '\0';
            strncpy(minute, DATETIME + 10, 2); minute[2] = '\0';
            strncpy(second, DATETIME + 12, 2); second[2] = '\0';

            DBG_PRINT("[%d] %s:%s:%s\r\n", seq_id, hour, minute, second);
            uart2_printf("[%d] ack %s:%s:%s\r\n", seq_id, hour, minute, second);
            return;

        } else if (STRCMP(argv[2], "git") == 0) {
            DBG_PRINT("[%d] MiiS ver:%s, FW ver:%s, git:%s branch:%s\r\n",
                      seq_id, MIIS_VERSION, FIRMWARE_VERSION, GIT_HASH, GIT_BRANCH);
            uart2_printf("[%d] ack MiiS ver:%s, FW ver:%s, git:%s branch:%s\r\n",
                         seq_id, MIIS_VERSION, FIRMWARE_VERSION, GIT_HASH, GIT_BRANCH);
            return;
        }

        fmt_err_with_info(buf); return; //
    }else {
        fmt_err_with_info(buf); //
    }
}

/**
 * @brief Handle "pupil_ir" console command.
 *
 * Assemble argv into a line, parse sequence ID, and process subcommands:
 *   - set <id> <0-255>
 *   - get <id>
 * Responds with ACK/NAK and queues a message to the IR LED task.
 *
 * Supported subcommands:
 *   - set : configure LED current
 *           args: <id: 1-4> <value: 0-255>
 *   - get : request LED current
 *           args: <id: 1-4>
 *
 * @param argc Number of arguments.
 * @param argv Argument list: argv[0]="pir", argv[1]=subcommand, argv[2] parameters.
 * @return void (N/A)
 */
static void console_pupil_ir(int argc, char* argv[]) {
    // 1) #[seq]
    char line[256] = {0};
    int i = 0;
    char *end;
    for (i = 0; i < argc; i++) {
        strncat(line, argv[i], sizeof(line) - strlen(line) - 1);
        if (i < argc - 1) {
            strncat(line, " ", sizeof(line) - strlen(line) - 1);
        }
    }

    int seq_id = parse_cmd_seq(line);
    if (seq_id == 0) {
        fmt_err_with_info(argv[0]);        // [pir] nak
        return;
    }

    //[seq] nak
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", seq_id);

    // 2) parse£ºpir set <id 1-4> <0-255>  /  pir get <id>
    if (argc <= 1) { fmt_err_with_info(buf); return; }

    led_task_message_t msg = {
        .ir_cmd = LED_TASK_CMD_DEFAULT,
        .led_no = TLC59108_MAX_LED_NUM,
        .led_cur = TLC59108_MAX_LED_CURR,
        .index = seq_id
    };


    if (STRCMP(argv[1], "set") == 0) {// ---- set ----
        if (argc != 5) { fmt_err_with_info(buf); return; }  //  id + value
        // parse id
        end = NULL;
        long id_val = strtol(argv[2], &end, 10);
        if (*end != '\0' || id_val < 1 || id_val > MIIS_MAX_LED_NUM) {
            fmt_err_with_info(buf);
            return;
        }
        int id = (int)id_val;
        end = NULL;
        // parse current 0~255
        long val_temp = strtol(argv[3], &end, 10);
        if (*end != '\0' || val_temp < 0 || val_temp > 255) {
            fmt_err_with_info(buf);
            return;
        }
        int val = (int)val_temp;

        msg.ir_cmd = LED_TASK_CMD_MIIS_SET_IR_CURRENT;
        msg.led_no = id;
        msg.led_cur = val;
        msg.index = seq_id;

        BaseType_t xStatus = xQueueSendToBack(queue_cmd_to_irled_task, &msg, pdMS_TO_TICKS(10));
        if (xStatus != pdPASS) {
            DBG_PRINT("ERROR queuing LED task\r\n");
            nak_with_info(buf);
        }
        return;
    }else if (STRCMP(argv[1], "get") == 0) {// ---- get ----
        if (argc != 4) { fmt_err_with_info(buf); return; }  // need id
        end = NULL;
        long id_val = strtol(argv[2], &end, 10);
        if (*end != '\0' || id_val < 1 || id_val > MIIS_MAX_LED_NUM) {
            fmt_err_with_info(buf);
            return;
        }

        int id = (int)id_val;

        msg.ir_cmd = LED_TASK_CMD_MIIS_GET_IR_CURRENT;
        msg.led_no = id;
        BaseType_t xStatus = xQueueSendToBack(queue_cmd_to_irled_task, &msg, pdMS_TO_TICKS(10));
        if (xStatus != pdPASS) {
            DBG_PRINT("ERROR queuing LED task\r\n");
            nak_with_info(buf);
        }
        return;
    }else {
        fmt_err_with_info(buf);
    }
}

static void console_reset_usb_hub(int argc, char* argv[]) {
   gpio_reset_usb_hub();
   ack();
}
/**
 * @brief Handle "sonoid" console command.
 *
 * Grammar:
 *   #[seq] sonoid set +
 *   #[seq] sonoid set -
 *
 * Rules:
 *   - Missing/invalid #[seq] => [sonoid] nak
 *   - Any parse error => [seq] nak
 *   - On success => [seq] ack
 *   - "+" keeps previous mapping (Reverse), "-" keeps previous mapping (Forward)
 * @param argc Number of arguments.
 * @param argv Argument list: argv[0]="sonoid", argv[1]="set", argv[2] is "+" or "-".
 * @return void (N/A)
 */
static void console_solenoid_ctrl(int argc, char* argv[]) {
    // 1) Rebuild the full line and parse #[seq]
    char line[256] = {0};
    int i;
    for (i = 0; i < argc; i++) {
        strncat(line, argv[i], sizeof(line) - strlen(line) - 1);
        if (i < argc - 1) strncat(line, " ", sizeof(line) - strlen(line) - 1);
    }

    int seq_id = parse_cmd_seq(line);
    if (seq_id == 0) {                 // No #[int] or invalid format
        fmt_err_with_info(argv[0]);        // [sonoid] nak
        return;
    }

    char buf[16];
    snprintf(buf, sizeof(buf), "%d", seq_id);
    if(STRCMP(argv[1], "set") == 0) {
        // 2) Expect: sonoid set <+|->
        if (argc !=4) {fmt_err_with_info(buf);return;}


        if (STRCMP(argv[2], "+") == 0) {
            // Reverse
            gpio_set_value(SOLENOID_CTRL1, GPIO_LOW);
            gpio_set_value(SOLENOID_CTRL2, GPIO_HIGH);
            osDelay(150);
            // Enter sleep mode
            gpio_set_value(SOLENOID_CTRL1, GPIO_LOW);
            gpio_set_value(SOLENOID_CTRL2, GPIO_LOW);
            ack_with_info(buf,NULL);
            return;
        }else if (STRCMP(argv[2], "-") == 0) {
            // Forward
            gpio_set_value(SOLENOID_CTRL1, GPIO_HIGH);
            gpio_set_value(SOLENOID_CTRL2, GPIO_LOW);
            osDelay(150);
            // Enter sleep mode
            gpio_set_value(SOLENOID_CTRL1, GPIO_LOW);
            gpio_set_value(SOLENOID_CTRL2, GPIO_LOW);
            ack_with_info(buf,NULL);
            return;
        }else{
            fmt_err_with_info(buf);
            return;
        }
    }else {
    // Invalid parameter
        fmt_err_with_info(buf);
        return;
    }
}

/**
 * @brief Handle "sony_ctrl" console command.
 *
 * Assemble argv into a line, parse sequence ID, and process subcommands:
 *   - enable <0|1> : enable or disable Sony module
 *   - show <pattern> <color> <x> <y> <w> <h> : display test pattern
 *   - spd <speed> : change display/scan speed
 *
 * Responds with ACK/NAK and queues a message to the Sony task.
 *
 * @param argc Number of arguments.
 * @param argv Argument list: argv[0]="sony", argv[1]=subcommand, argv[2..] parameters.
 */
static void console_sony_ctrl(int argc, char* argv[]) {
    sony_task_message_t msg = {
        .sony_cmd = SONY_TASK_CMD_DEFAULT};
    char line[256] = {0};
    int i;
    for (i = 0; i < argc; i++) {
        strncat(line, argv[i], sizeof(line) - strlen(line) - 1);
        if (i < argc - 1) strncat(line, " ", sizeof(line) - strlen(line) - 1);
    }

    int seq_id = parse_cmd_seq(line);
    if (seq_id == 0) {                 // No #[int] or invalid format
        fmt_err_with_info(argv[0]);        // [ud] nak
        return;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", seq_id);

    if(STRCMP(argv[1], "enable") == 0) {
        if (argc !=4) {fmt_err_with_info(buf);return;}

        if(STRCMP(argv[2], "1") == 0) {
            msg.sony_cmd = SONY_TASK_CMD_ENABLE;
        }else if (STRCMP(argv[2], "0") == 0) {
            msg.sony_cmd = SONY_TASK_CMD_DISABLE;
        } else{
            fmt_err_with_info(buf);
            return;
        }
    } else if(STRCMP(argv[1], "show") == 0){
        if (argc !=9) {fmt_err_with_info(buf);return;}
        msg.sony_cmd = SONY_TASK_CMD_SHOW;
        if (argc == 9) {
            /* pattern: sony_pattern_e (unsigned) */
            if (!parse_decimal_str(argv[2], 0)) { fmt_err_with_info(buf); return; }
            msg.pattern = (sony_pattern_e)strtol(argv[2], NULL, 10);
            /* color: INT8U (unsigned) */
            if (!parse_decimal_str(argv[3], 0)) { fmt_err_with_info(buf); return; }
            msg.color = (INT8U)strtol(argv[3], NULL, 10);
            /* x: INT32S (signed) */
            if (!parse_decimal_str(argv[4], 1)) { fmt_err_with_info(buf); return; }
            msg.x = (INT32S)strtol(argv[4], NULL, 10);
            /* y: INT32S (signed) */
            if (!parse_decimal_str(argv[5], 1)) { fmt_err_with_info(buf); return; }
            msg.y = (INT32S)strtol(argv[5], NULL, 10);
            /* w_size: INT16U (unsigned) */
            if (!parse_decimal_str(argv[6], 0)) { fmt_err_with_info(buf); return; }
            msg.w_size = (INT16U)strtol(argv[6], NULL, 10);
            /* h_size: INT16U (unsigned) */
            if (!parse_decimal_str(argv[7], 0)) { fmt_err_with_info(buf); return; }
            msg.h_size = (INT16U)strtol(argv[7], NULL, 10);
        } else{
            fmt_err_with_info(buf);
            return;
        }
    }else if (STRCMP(argv[1], "spd") == 0){
        msg.sony_cmd = SONY_TASK_CMD_CHANGE_SPEED;
        if (argc == 4) {
            msg.speed = atof(argv[2]);
        } else{
            fmt_err_with_info(buf);
            return;
        }

    }else{
        // Invalid parameter
        fmt_err_with_info(buf);
        return;
    }
    msg.index=seq_id;
    BaseType_t xStatus;
    xStatus = xQueueSendToBack(queue_cmd_to_sony_task, &msg, pdMS_TO_TICKS(10));
    if (xStatus != pdPASS) {
        DBG_PRINT("ERROR queuing sony task\r\n");
        nak_with_info(buf);
    }
    return;
}

static void console_get_version(int argc, char* argv[]) {

    DBG_PRINT("MiiS ver:%s, FW ver:%s, git:%s branch:%s\r\n", MIIS_VERSION, FIRMWARE_VERSION, GIT_HASH, GIT_BRANCH);
    uart2_printf("MiiS ver:%s, FW ver:%s, git:%s branch:%s\r\n", MIIS_VERSION, FIRMWARE_VERSION, GIT_HASH, GIT_BRANCH);
}

/**
 * @brief Handle "motor_ctrl" console command.
 *
 * Assemble argv into a line, parse sequence ID, and process subcommands:
 *   - info <version|status|pos> : query motor info
 *   - relmove <x> <y> <z> : relative move command
 *   - absmove <x> <y> <z> : absolute move command
 *   - home [z] : move motor to home position
 *   - set0 : set motor zero position
 *
 * Responds with ACK/NAK and queues a message to the 3-axis motor task.
 *
 * @param argc Number of arguments.
 * @param argv Argument list: argv[0]="3as", argv[1]=subcommand, argv[2..] parameters.
 */
static void console_motor_ctrl(int argc, char* argv[]) {
    miis_3axis_message_t msg = {
        .cmd = MIIS_3X_FW_VERSION,
        .index = 0,
        .data = {
            .axes = MIIS_3X_X_AXIS | MIIS_3X_Y_AXIS | MIIS_3X_Z_AXIS, // 0b0111 means all motor
                .x = 0,
                .y = 0,
                .z = 0
            },
            .dirlim = {
                .x_dir = 0,
                .x_lim = 0,
                .y_dir = 0,
                .y_lim = 0,
                .z_dir = 0,
                .z_lim = 0
            }
            };

    char line[256] = {0};
    int i;
    for (i = 0; i < argc; i++) {
        strncat(line, argv[i], sizeof(line) - strlen(line) - 1);
        if (i < argc - 1) strncat(line, " ", sizeof(line) - strlen(line) - 1);
    }

    int seq_id = parse_cmd_seq(line);
    if (seq_id == 0) {                 // No #[int] or invalid format
        fmt_err_with_info(argv[0]);        // [ud] nak
        return;
    }

    msg.index=seq_id;
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", seq_id);

    if (STRCMP(argv[1], "info") == 0){
        if (argc !=4) {fmt_err_with_info(buf);return;}

        if(STRCMP(argv[2], "version") == 0) {
            msg.cmd = MIIS_3X_FW_VERSION;
        }else if (STRCMP(argv[2], "status") == 0) {
            msg.cmd = MIIS_3X_STATUS;
        }else if (STRCMP(argv[2], "pos") == 0) {
            msg.cmd = MIIS_3X_POS;
        }else{
            fmt_err_with_info(buf);
            return;
        }
    }else if (STRCMP(argv[1], "relmove") == 0){
        if (argc !=6) {fmt_err_with_info(buf);return;}
        msg.cmd = MIIS_3X_MOVE_REL;
        char *end;
        end = NULL;
        long xv = strtol(argv[2], &end, 10);
        if (*end != '\0') { fmt_err_with_info(buf); return; }
        end = NULL;
        long yv = strtol(argv[3], &end, 10);
        if (*end != '\0') { fmt_err_with_info(buf); return; }
        end = NULL;
        long zv = strtol(argv[4], &end, 10);
        if (*end != '\0') { fmt_err_with_info(buf); return; }

        msg.data.x = (INT32S)xv;
        msg.data.y = (INT32S)yv;
        msg.data.z = (INT32S)zv;
        msg.data.axes = MIIS_3X_X_AXIS | MIIS_3X_Y_AXIS | MIIS_3X_Z_AXIS;


    }else if (STRCMP(argv[1], "absmove") == 0){
        if (argc !=7) {fmt_err_with_info(buf);return;}
        msg.cmd = MIIS_3X_MOVE_ABS;
        msg.data.axes = atoi(argv[2]);
        char *end;
        end = NULL;
        long xv = strtol(argv[3], &end, 10);
        if (*end != '\0') { fmt_err_with_info(buf); return; }
        end = NULL;
        long yv = strtol(argv[4], &end, 10);
        if (*end != '\0') { fmt_err_with_info(buf); return; }
        end = NULL;
        long zv = strtol(argv[5], &end, 10);
        if (*end != '\0') { fmt_err_with_info(buf); return; }

        msg.data.x = (INT32S)xv;
        msg.data.y = (INT32S)yv;
        msg.data.z = (INT32S)zv;

    }else if (STRCMP(argv[1], "home") == 0){
        msg.cmd = MIIS_3X_HOME;
        if (argc == 3 || argc == 4) {
            if (argc == 4) {
                msg.data.z = atoi(argv[2]);
            }
        }else{
            fmt_err_with_info(buf);return;
        }
    }else if (STRCMP(argv[1], "set0") == 0){
        if (argc !=3) {fmt_err_with_info(buf);return;}
        msg.cmd = MIIS_3X_SET_ZERO;

    }else if (STRCMP(argv[1], "reset") == 0){
        if (argc !=3) {fmt_err_with_info(buf);return;}
        msg.cmd = MIIS_3X_RESET;

    }else if (STRCMP(argv[1], "setdirlim") == 0) {
        if (argc != 9) { fmt_err_with_info(buf); return; }

        msg.cmd = MIIS_3X_SET_DIR_LIM;
        msg.data.axes = MIIS_3X_X_AXIS | MIIS_3X_Y_AXIS | MIIS_3X_Z_AXIS;

        if (miis_3axis_sign_to_val(argv[2], &msg.dirlim.x_dir) < 0) {
            fmt_err_with_info(buf); return;
        }
        if (miis_3axis_sign_to_val(argv[3], &msg.dirlim.x_lim) < 0) {
            fmt_err_with_info(buf); return;
        }
        if (miis_3axis_sign_to_val(argv[4], &msg.dirlim.y_dir) < 0) {
            fmt_err_with_info(buf); return;
        }
        if (miis_3axis_sign_to_val(argv[5], &msg.dirlim.y_lim) < 0) {
            fmt_err_with_info(buf); return;
        }
        if (miis_3axis_sign_to_val(argv[6], &msg.dirlim.z_dir) < 0) {
            fmt_err_with_info(buf); return;
        }
        if (miis_3axis_sign_to_val(argv[7], &msg.dirlim.z_lim) < 0) {
            fmt_err_with_info(buf); return;
        }

    }else if (STRCMP(argv[1], "lockoff") == 0){
        if (argc !=3) {fmt_err_with_info(buf);return;}
        msg.cmd = MIIS_3X_LOCKOFF;

    }else{
        fmt_err_with_info(buf);
        return;
    }
    BaseType_t xStatus;
    xStatus = xQueueSendToBack(queue_cmd_to_3axis_task, &msg, pdMS_TO_TICKS(10));
    if (xStatus != pdPASS) {
            DBG_PRINT("ERROR queuing 3X task\r\n");
    }
        return;
}



static int console_fgetchar(void)
{
	INT8U input_ch = 0;

	#if DBG_UART == 0
    while (drv_l1_uart0_data_get(&input_ch, 0) != STATUS_OK)
    #elif DBG_UART == 2
    while (drv_l1_uart2_data_get(&input_ch, 0) != STATUS_OK)
    #else
    while (drv_l1_uart1_data_get(&input_ch, 0) != STATUS_OK)
    #endif

	{
		#if _OPERATING_SYSTEM != _OS_NONE
            #if _OPERATING_SYSTEM == _OS_UCOS2
            OSTimeDly(1);
            #elif _OPERATING_SYSTEM == _OS_FREERTOS
			if (cmdTaskHandle != NULL)
				ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			else
				osDelay(1);
            #else
            #error "which OS api for OSTimeDly ?"
            #endif
		#endif
	}
	return (int )(input_ch);
}

static int isaspace(char c)
{
	return (c == ' ' || c == '\t' || c == '\n' || c == '\12');
}

static char *gpstrdup(const char *str)
{
	size_t siz;
	char *copy;

	siz = STRLEN(str) + 1;
	if ((copy = (char *)MEMALLOC(siz)) == NULL)
		return NULL;
	MEMCPY(copy, str, siz);

	return copy;
}

static int setargs(char *args, char **argv)
{
    int count = 0;

    while (isaspace(*args)) ++args;
    while (*args)
    {
        if (argv) argv[count] = args;
        while (*args && !isaspace(*args)) ++args;
        if (argv && *args) *args++ = '\0';
        while (isaspace(*args)) ++args;
        count++;
    }
    return count;
}

static char **parsedargs(char *args, int *argc)
{
    char **argv = NULL;
    int argn = 0;

    if (args && *args)
    {
    	args = (char *)gpstrdup(args);
    	if (args)
    	{
        	argn = setargs(args,NULL);
        	if (argn)
        	{
        		argv = (char **)MEMALLOC((argn+1) * sizeof(char *));
        		if (argv)
    			{
        			*argv++ = args;
        			argn = setargs(args,argv);
    			}
    		}
    	}
	}
    if (args && !argv) FREE(args);

    *argc = argn;
    return argv;
 }

#ifndef WIFI_DEMO
static void freeparsedargs(char **argv)
{
    if (argv)
    {
        FREE(argv[-1]);
        FREE(argv-1);
    }
}
#endif

int parse_to_argv(char *input_str, char ***out_av)
{
    char **av;
    int ac;
    char *as = NULL;

    as = input_str;

    av = parsedargs(as,&ac);

    #if 0
    int i;
    printf("== %d\r\n",ac);
    for (i = 0; i < ac; i++)
        DBG_PRINT("[%s]\r\n",av[i]);
    #endif

    *out_av = av;

    return ac;
}
#ifndef WIFI_DEMO
static void cmdProcess(
	unsigned char *cmd,
	unsigned int repeating
)
{
	cmd_t *bc = pcmds;
	unsigned int idx = 0;
	unsigned int copy = 0;
	int argc = 0;
        char **argv = NULL;
        int found = 0;

	/*
	 * Strip the white space from the command.
	 */
	while ( cmd[idx] != '\0' )
	{
		if ( (cmd[idx] != ' ') &&
			 (cmd[idx] != '\t') &&
			 (cmd[idx] != '\r') &&
			 (cmd[idx] != '\n') )
		{
			break;
		}
		idx++;
	}

	if ( idx > 0 )
	{
		/* Reached a non-white space character, compact the string */
		while ( cmd[idx] != '\0' )
		{
			cmd[copy++] = cmd[idx++];
		}
		cmd[copy] = '\0';
	}

	/*
	 * Index points to the end of the string, move backwards.
	 */
	idx = STRLEN(cmd);

	while ( idx > 0 )
	{
		idx--;
		if ( (cmd[idx] == ' ') ||
			 (cmd[idx] == '\t') ||
			 (cmd[idx] == '\r') ||
			 (cmd[idx] == '\n') )
		{
			cmd[idx] = '\0';
		}
		else
		{
			break;
		}
	}

	/*
	 * Find the command.
	 */
	idx = 0;

	while ( cmd[idx] != '\0' )
	{
		if ( (cmd[idx] == ' ') ||
			 (cmd[idx] == '\t') ||
			 (cmd[idx] == '\r') ||
			 (cmd[idx] == '\n') )
		{
			break;
		}
		idx++;
	}

	argc = parse_to_argv((char *)cmd, &argv);

	cmd[idx] = '\0';

	if (argc > 0)
	{
		while ( bc )
		{
			// DBG_PRINT("check %s, %s\r\n", bc->cmd, (argc == 0) ? "" : argv[0]);

			if ( STRCMP(bc->cmd, argv[0]) == 0 )
			{
				(bc->phandler)(argc, argv);
                                found = 1;
				break;
			}
			bc = bc->pnext;
		}
	}

        if (!found)
          DBG_PRINT("command `%s' not found, try `all help'\r\n", (argc == 0) ? "" : argv[0]);

	if (argv != NULL)
    {
        freeparsedargs(argv);
        argv = NULL;
    }

}
#endif

static unsigned int cmdIdxIncrease(
	unsigned int *pcmdIdx
)
{
	unsigned int localIdx;
	unsigned int ret = 0;

	localIdx = *pcmdIdx;
	localIdx++;
	if ( localIdx == CMD_HISTORIES )
	{
		localIdx = 0;
		ret = 1;
	}
	*pcmdIdx = localIdx;

	return ret;
}

static unsigned int cmdFlushCopy(
	unsigned int cursorPos,
	unsigned char *pcmdBuf,
	unsigned char *pcmdSrc,
	unsigned int cmdLen
)
{
	if ( cursorPos > 0 )
	{
		for ( ; cursorPos > 0; cursorPos-- )
		{
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
void cmdMonitor(void)
{
	unsigned char c;
#ifndef WIFI_DEMO
	unsigned int repeating;
#endif
	unsigned int histDownArw;
	static unsigned int upArrowCnt;

	if ( !logIn )
	{
		//if ( c == '\r' )
		{
			logIn = TRUE;
			DBG_PRINT("\r\n\r\ncmd>");
		}
	}
	else
	{
		c = (unsigned char )GETCH();

		switch ( c )
		{
		case '\b':
		case '\x7f':
			if ( pos > 0 )
			{
				DBG_PRINT("\b \b");
				pos--;
			}
			cmdBuffer[pos] = '\0';
			break;

		case '\r':  /* Process the command. */
			DBG_PRINT("\r\n");
			if ( pos )
			{
				/*
				 * Do not place the same last command into the history if the same.
				 */
				if ( STRCMP((unsigned char *)histBuf[histIns], cmdBuffer) )
				{
					if ( cmdIdxIncrease(&histIns) == 1 )
					{
						histInsWrap = 1;
					}
					MEMCPY(histBuf[histIns], cmdBuffer, CMD_BUF_SIZE);
					histPos[histIns] = pos;
				}
				histOutput = histIns;
				histOutputWrap = 0;
				upArrowCnt = 0;
            #ifndef WIFI_DEMO
				repeating = FALSE;
            #endif
			}
			if ( pos )
			{
            #ifdef WIFI_DEMO
                log_handler((char *)cmdBuffer);
            #else
				cmdProcess(cmdBuffer, repeating);
            #endif
				pos = 0;
				MEMSET(cmdBuffer, 0, CMD_BUF_SIZE);
				DBG_PRINT("\r\n");
			}
			DBG_PRINT("cmd>");
			break;

		case '[': /* Non ASCII characters, arrow. */
			c = GETCH();
			switch ( c )
			{
			case 'A': /* Key: up arrow */
				if ( histOutputWrap == 1 )
				{
					if ( histOutput == histIns )
					{
						break;
					}
				}
				if ( histInsWrap == 0 )
				{
					if ( histOutput == 0 )
					{
						break;
					}
				}
				upArrowCnt++;
				cmdFlushCopy(
					pos,
					cmdBuffer,
					(unsigned char *)histBuf[histOutput],
					histPos[histOutput]
				);
				pos = histPos[histOutput];
				cmdBuffer[pos + 1] = '\0';
				DBG_PRINT((CHAR *)cmdBuffer);
				if ( histInsWrap == 1 )
				{
					if ( histOutput == 0 )
					{
						histOutput = CMD_HISTORIES - 1;
						histOutputWrap = 1;
					}
					else
					{
						histOutput--;
					}
				}
				else
				{
					if ( histOutput != 0 )
					{
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
				if ( upArrowCnt <= 1 )
				{
					break;
				}
				upArrowCnt--;
				cmdIdxIncrease(&histOutput);
				histDownArw = histOutput;
				cmdIdxIncrease(&histDownArw);
				cmdFlushCopy(
					pos,
					cmdBuffer,
					(unsigned char *)histBuf[histDownArw],
					histPos[histDownArw]
				);
				pos = histPos[histDownArw];
				cmdBuffer[pos + 1] = '\0';
				DBG_PRINT((CHAR *)cmdBuffer);
				break;

			case 'C': /* Key: right arrow */
				break;
			case 'D': /* Key: left arrow */
				break;
			default:
                if ( (pos < (CMD_BUF_SIZE - 1)) && (c >= ' ') && (c <= 'z') )
                {
                    cmdBuffer[pos++] = c;
                    cmdBuffer[pos] = '\0';
                    DBG_PRINT((CHAR *)cmdBuffer + pos - 1);
                }
                if ( c == '\x7e' )
                {
                    cmdBuffer[pos++] = c;
                    cmdBuffer[pos] = '\0';
                    DBG_PRINT((CHAR *)cmdBuffer + pos - 1);
                }
				break;
			}
			break;

		default:
			if ( (pos < (CMD_BUF_SIZE - 1)) && (c >= ' ') && (c <= 'z') )
			{
				cmdBuffer[pos++] = c;
				cmdBuffer[pos] = '\0';
				DBG_PRINT((CHAR *)cmdBuffer + pos - 1);
			}
			if ( c == '\x7e' )
			{
				cmdBuffer[pos++] = c;
				cmdBuffer[pos] = '\0';
				DBG_PRINT((CHAR *)cmdBuffer + pos - 1);
			}
			break;
		}
	} /* else of if !logged_in */
}

void cmdUartRxIsr(INT8U dev_idx, INT8U ch)
{
	#if _OPERATING_SYSTEM != _OS_NONE
		#if _OPERATING_SYSTEM == _OS_UCOS2
		#elif _OPERATING_SYSTEM == _OS_FREERTOS
		BaseType_t xHigherPriorityTaskWoken = 0;

		vTaskNotifyGiveFromISR(cmdTaskHandle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		#else
        #error "which OS api for vTaskNotifyGiveFromISR ?"
		#endif
	#endif
}
static void default_cmd_register(void)
{
	cmd_t *pcmd = &default_cmd_list[0];

	while (pcmd->cmd != NULL)
	{
		cmdRegister(pcmd);
		pcmd += 1;
	}
}
void Cmd_Task(void *para)
{
	#if _OPERATING_SYSTEM != _OS_NONE
		#if _OPERATING_SYSTEM == _OS_UCOS2
		#elif _OPERATING_SYSTEM == _OS_FREERTOS
		if (cmdTaskHandle != NULL)
		{
			#if DBG_UART == 0
			drv_l1_uart0_rx_isr_set(cmdUartRxIsr);
			#elif DBG_UART == 2
			drv_l1_uart2_rx_isr_set(cmdUartRxIsr);
			#else
			drv_l1_uart1_rx_isr_set(cmdUartRxIsr);
			#endif
		}
		#else
		#endif
	#endif

	default_cmd_register();

	while ( 1 )
	{
		cmdMonitor();
	}
}

void cmdRegister(cmd_t *bc)
{
	cmd_t *prev;
	cmd_t *curr;

	bc->pnext = NULL;
	if ( pcmds == NULL )
	{
		pcmds = bc;
	}
	else
	{
		prev = NULL;
		curr = pcmds;
		while ( curr )
		{
			/* The list is sorted by alphabetic order. */
			if ( STRCMP(bc->cmd, curr->cmd) <= 0 )
			{
				bc->pnext = curr;
				if ( prev )
				{
					prev->pnext = bc;
				}
				else
				{
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


int parse_cmd_seq(const char* line)
{
    if (!line) return 0;

    const unsigned char* p = (const unsigned char*)line;

    while (*p) {
        // skip spaces and line breaks
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;

        if (*p == '#') {
            ++p; // move past '#'

            // must have at least one digit
            if (!isdigit(*p)) return 0;

            const char* start = (const char*)p;
            char* endptr = NULL;

            unsigned long long v = strtoull(start, &endptr, 10);

            // end must be delimiter or line end
            unsigned char endch = (unsigned char)(endptr ? *endptr : '\0');
            if (endch != '\0' && endch != ' ' && endch != '\t' && endch != '\r' && endch != '\n')
                return 0;

            // check overflow: int ¹ ‡ú
            if (v > 2147483647ULL) return 0;

            return (int)v;
        }

        // skip current token until delimiter
        while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
    }

    return 0; // no "#<digits>" found
}
static inline int parse_long_in_range(const char* s, long min, long max, long* out) {
    char *end = NULL;
    errno = 0;
    long v = strtol(s, &end, 10);
    if (errno == ERANGE || !s || *s == '\0' || *end != '\0' || v < min || v > max) return 0;
    *out = v;
    return 1;
}
static inline int parse_double_in_range(const char* s, double min, double max, double* out) {
    char *end = NULL;
    errno = 0;
    double v = strtod(s, &end);
    if (errno == ERANGE || !s || *s == '\0' || *end != '\0' || v < min || v > max) return 0;
    *out = v;
    return 1;
}
static inline int parse_decimal_str(const char* s, int allow_sign) {
    if (!s || !*s) return 0;
    const unsigned char* p = (const unsigned char*)s;
    if (allow_sign && (*p == '+' || *p == '-')) ++p;
    if (*p == '\0') return 0;           /* sign-only is invalid */
    while (*p) {
        if (!isdigit(*p)) return 0;
        ++p;
    }
    return 1;
}
static INT32S miis_3axis_sign_to_val(const char *s, INT8S *val)
{
    if (s == NULL || val == NULL) {
        return -1;
    }

    if (s[0] == '-' && s[1] == '\0') {
        *val = 1;
        return 0;
    }

    if (s[0] == '+' && s[1] == '\0') {
        *val = 0;
        return 0;
    }

    return -1;
}
