/**
 * @file    proc_cmd.c
 * @brief   Command parser and dispatcher module.
 * @details This module implements a command-line interface (CLI) that parses
 *          incoming commands and dispatches them to corresponding handlers
 *          using a lookup table (opTable).
 *
 *          Flow:
 *          1. Receive command string from CDC interface
 *          2. Tokenize input into parameters
 *          3. Search opTable for matching command
 *          4. Execute corresponding handler function
 * @ingroup command_parser
 */

#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "proc_cmd.h"
#include "version.h"
#include "config/default/peripheral/port/plib_port.h"
#include "config/default/peripheral/tcc/plib_tcc3.h"
#include "peripheral/systick/plib_systick.h"
#include "spi.h"
#include "mems.h"
#include "filesys.h"
#include "indicate_led.h"
#include "gp329.h"
#include "sld.h"
#include "motor_ctrl.h"
#include "build_info.h"
#include "lowpass_filter.h"

#define DBG_PRINTF 1
#define USB_HUB_RESET_PULSE_WIDTH_MS    5
#define MAX_PARAMS   255
static void op_flash(const char params[][50], uint8_t size);
static void op_iled(const char params[][50], uint8_t size);
static void op_lpf(const char params[][50], uint8_t size);
static void op_mainbd(const char params[][50], uint8_t size);
static void op_mems(const char params[][50], uint8_t size);
static void op_motor(const char params[][50], uint8_t size);
static void op_probe(const char params[][50], uint8_t size);
static void op_sld(const char params[][50], uint8_t size);
static void op_getVersion(const char params[][50], uint8_t size);
static void op_getBuildVersion(const char params[][50], uint8_t size);
static void op_get_git_version(const char params[][50], uint8_t size);
static void op_WDTtest(const char params[][50], uint8_t size);
static void op_SPITtest(const char params[][50], uint8_t size);
static void op_show_waveform_data(const char params[][50], uint8_t size);
static void op_format_disk(const char params[][50], uint8_t size);
static void op_get_filesystem_dir(const char params[][50], uint8_t size);
static void op_list_all_cmd(const char params[][50], uint8_t size);
static void op_show_debug_info(const char params[][50], uint8_t size);
static void op_reset_usb_hub(const char params[][50], uint8_t size);

void ack_with_info(const char* seq, const char* result);

/**
 * @brief   Command operation lookup table.
 * @details Maps command strings to their corresponding handler functions.
 *          This table is used by the command parser to dispatch incoming
 *          commands to the appropriate operation function.
 *
 *          Each entry consists of:
 *          - cmdstr   : Command string received from host
 *          - function : Corresponding handler function
 *
 *          The table is terminated by an empty string entry ("", 0).
 *
 * @note    This table is part of the command parsing mechanism .
 */
const operation_t opTable[] = {
    {"flash", op_flash},            /**< Flash and filesystem operations */
    {"iled", op_iled},              /**< Indicator LED control */
    {"lpf", op_lpf},                /**< Low-pass filter control */
    {"mainbd", op_mainbd},          /**< Main board information */
    {"mems", op_mems},              /**< MEMS control */
    {"motor", op_motor},            /**< Motor control */    
    {"probe", op_probe},            /**< Probe board control */
    {"sld", op_sld},                /**< SLD module control */
    {"version?", op_getVersion},
    {"vermiis?", op_getBuildVersion},
    {"git?", op_get_git_version},
    {"wdt", op_WDTtest},
    {"spi", op_SPITtest},
    {"wave?", op_show_waveform_data},
    {"formatdisk", op_format_disk},
    {"ls", op_get_filesystem_dir},
    {"help", op_list_all_cmd},      /**< List all commands */
    {"dbg", op_show_debug_info},
    {"rstusb", op_reset_usb_hub},
    {"", 0}
};

static inline void nak() {
#if DBG_PRINTF
    printf("nak\r\n");
#endif
}
void nak_with_info(const char* seq) {
    const char* fail = "nak";
    char out[64];
    int n;
    if (seq && *seq) {
        n = snprintf(out, sizeof(out), "[%s] %s\r\n", seq, fail);
        size_t len = (n >= (int)sizeof(out)) ? sizeof(out) - 1 : (size_t)n;
        printf("%.*s", (int)len, out);
    } else {
        printf("%s", fail);
      
    }
}
void nck_with_result(const char* seq, const char* result) {
    const int has_seq = (seq && *seq);
    const int has_res = (result && *result);
    char out[256];
    const char* fail = "nak";
    int n;
    if (has_seq && has_res)
        n = snprintf(out, sizeof(out), "[%s] %s %s\r\n", seq, fail, result);
    else if (has_seq)
        n = snprintf(out, sizeof(out), "[%s] %s\r\n", seq, fail);
    else if (has_res)
        n = snprintf(out, sizeof(out), "%s %s\r\n", fail, result);
    else
        n = snprintf(out, sizeof(out), "%s\r\n", fail);
    if (n < 0) return; // 
    size_t len = (n >= (int)sizeof(out)) ? sizeof(out) - 1 : (size_t)n;
    //uint32_t s = __builtin_disable_interrupts();
    printf("%.*s", (int)len, out);
    //__builtin_enable_interrupts();
}
void ack_with_info(const char* seq, const char* result) {
    const int has_seq = (seq && *seq);
    const int has_res = (result && *result);
    char out[256];
    const char* ok = "ack";
    int n;
    if (has_seq && has_res)
        n = snprintf(out, sizeof(out), "[%s] %s %s\r\n", seq, ok, result);
    else if (has_seq)
        n = snprintf(out, sizeof(out), "[%s] %s\r\n", seq, ok);
    else if (has_res)
        n = snprintf(out, sizeof(out), "%s %s\r\n", ok, result);
    else
        n = snprintf(out, sizeof(out), "%s\r\n", ok);
    if (n < 0) return; // 
    size_t len = (n >= (int)sizeof(out)) ? sizeof(out) - 1 : (size_t)n;
    //uint32_t s = __builtin_disable_interrupts();
    printf("%.*s", (int)len, out);
    //__builtin_enable_interrupts();
}

void fmt_err_with_info(const char* seq) {
    const char* fmterr = "fmt err";
    char out[64];
    int n;
    if (seq && *seq) {
        n = snprintf(out, sizeof(out), "[%s] %s\r\n", seq, fmterr);
        size_t len = (n >= (int)sizeof(out)) ? sizeof(out) - 1 : (size_t)n;
        printf("%.*s", (int)len, out);
    } else {
        printf("%s\r\n", fmterr);
    }
}
static inline void ack() {
#if DBG_PRINTF
    printf("ack\r\n");
#endif
}

static int is_valid_float(const char *str) {
    if (*str == '\0') return 0; // Empty string, return 0 (false)

    char *endptr;
    strtod(str, &endptr);

    // Check if any characters were converted
    if (str == endptr) return 0;

    // Skip all whitespace characters
    while (isspace((unsigned char) *endptr)) endptr++;

    // If there are no other characters following the floating-point number, then the string is a valid floating-point number
    return *endptr == '\0';
}
int parse_cmd_seq(const char* line)
{
    if (!line) return 0;
    const unsigned char* p = (const unsigned char*)line;
    while (*p) {
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
        if (*p == '#') {
            ++p; 
            if (!isdigit(*p)) return 0;
            const char* start = (const char*)p;
            char* endptr = NULL;
            unsigned long long v = strtoull(start, &endptr, 10);
            unsigned char endch = (unsigned char)(endptr ? *endptr : '\0');
            if (endch != '\0' && endch != ' ' && endch != '\t' && endch != '\r' && endch != '\n')
                return 0;
            if (v > 2147483647ULL) return 0;
            return (int)v;
        }
        while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
    }
    return 0; 
}

/**
 * @brief   Handle flash and filesystem related commands.
 * @details Executes operations related to external flash memory and file system,
 *          including directory listing, disk formatting, file read/write,
 *          file removal, and parameter storage.
 *
 *          Supported sub-commands:
 *          - list        : List files in flash
 *          - fmtdisk     : Format flash storage
 *          - lofile      : Load file and apply parameters
 *          - safile      : Save data to file
 *          - rmfile      : Remove file
 *          - loadsld     : Load SLD parameters from flash
 *          - savesld     : Save SLD parameters to flash
 *
 * @param[in] params  Command parameters array.
 * @param[in] size    Number of parameters.
 */
static void op_flash(const char params[][50], uint8_t size) {
    char line[256] = {0};
    int i;
    for (i = 0; i < size; i++) {
        strncat(line, params[i], sizeof(line) - strlen(line) - 1);
        if (i < size - 1) strncat(line, " ", sizeof(line) - strlen(line) - 1);
    }
    int seq_id = parse_cmd_seq(line);
    if (seq_id == 0) {
        fmt_err_with_info(params[0]);
        return;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", seq_id);
    if (strcmp(params[1], "list") == 0) {
        if (size != 3) {fmt_err_with_info(buf); return;}
        if(SST26_ReadDir()){
            ack_with_info(buf,NULL);
        }else{
            nak_with_info(buf);
        }
    }else if(strcmp(params[1], "fmtdisk") == 0){
        if (size != 3) {fmt_err_with_info(buf); return;}
        if (SST26_FormatDisk()) {
            ack_with_info(buf,NULL);
        } else {
            nak_with_info(buf);
        }
        
    }else if(strcmp(params[1], "lofile") == 0){
        if (size != 4) {fmt_err_with_info(buf); return;}
        if (SST26_ReadFile(params[2], &m_wave_data)) {
        if (lp_filter_set_freq(m_wave_data.lowpass_filter_freq))
            ack_with_info(buf,NULL);
        else
            nak_with_info(buf);
        } else {
            lp_filter_set_freq(24000); // If loadfile error, set low-pass filter to 24000.
            nak_with_info(buf);
        } 
    }else if(strcmp(params[1], "safile") == 0){
        if (size != 4) {fmt_err_with_info(buf); return;}
        if (SST26_WriteFile(params[2], &m_wave_data)) {
            ack_with_info(buf,NULL);
        } else {
            nak_with_info(buf);
        }
        
    }else if(strcmp(params[1], "rmfile") == 0){
        if (size != 4) {fmt_err_with_info(buf); return;}
        if (SST26_RemoveFile(params[2]))
        {
            ack_with_info(buf,NULL);
        }else{
            nak_with_info(buf);
        }
    }else if(strcmp(params[1], "loadsld") == 0){
        if (size != 4) {fmt_err_with_info(buf); return;}
        const char* fname = params[2];   // e.g. "sldth" or "sldflg"
        uint16_t value = 0;
        /* Read value from flash */
        if (!SST26_ReadSLD(fname, &value)) {
            nak_with_info(buf);
            return;
        }
        /* Decide behavior by file name */
        if (strcmp(fname, "sldth") == 0) {
            sld_set_pd_threshold(value); /* Apply loaded threshold to runtime protection */
        }else if (strcmp(fname, "sldflg") == 0){
            /* Apply loaded flag to runtime */
            sld_flag = value;
        }else{
            /* Do nothing here */
        }
        /* Return value to user */
        char msg[32];
        snprintf(msg, sizeof(msg), "%u", value);
        ack_with_info(buf, msg);
    }else if(strcmp(params[1], "savesld") == 0){
        if (size != 5) {fmt_err_with_info(buf); return;}
        const char* fname = params[2];
        int val = atoi(params[3]);
        /* Basic range check: ADC raw 0 ~ 4095 */
        if (val < 0 || val > 4095) {
            nak_with_info(buf);
            return;
        }
        uint16_t value = (uint16_t)val;
        if (!SST26_WriteSLD(fname, value)) {
            /* write or close failed */
            nak_with_info(buf);
            return;
        }
        /* For pure "[seq] ack" with no extra text, pass NULL */
        ack_with_info(buf, NULL);
    }else{
        fmt_err_with_info(buf);
        return;
    }
    
}

/**
 * @brief   Control indicator LED behavior.
 * @details Controls LED enable state, brightness level, and RGB color.
 *
 *          Supported sub-commands:
 *          - enable <id> <0|1>       : Enable or disable LED
 *          - setb   <id> <level>     : Set brightness (0~255)
 *          - setc   <id> <r> <g> <b> : Set RGB color
 *
 * @param[in] params  Command parameters array.
 * @param[in] size    Number of parameters.
 */
static void op_iled(const char params[][50], uint8_t size) {
    char line[256] = {0};
    int i;
    for (i = 0; i < size; i++) {
        strncat(line, params[i], sizeof(line) - strlen(line) - 1);
        if (i < size - 1) strncat(line, " ", sizeof(line) - strlen(line) - 1);
    }
    int seq_id = parse_cmd_seq(line);
    if (seq_id == 0) {
        fmt_err_with_info(params[0]);
        return;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", seq_id);
    uint8_t id;
    id = (uint8_t) atoi(params[2]);
    if (strcmp(params[1], "enable") == 0) {
        if (size != 5) {fmt_err_with_info(buf); return;}
        uint8_t enable;
        enable = (uint8_t) atoi(params[3]);
        if (enable > 1) {
            fmt_err_with_info(buf);
            return;
        }
        if (indicate_led_enable(id, enable))
            ack_with_info(buf,NULL);
        else
            nak_with_info(buf);            
    }else if(strcmp(params[1], "setb") == 0){
        if (size != 5) {fmt_err_with_info(buf); return;}
        uint16_t level;
        level = (uint16_t) atoi(params[3]);
        if (level > 255) {
            fmt_err_with_info(buf);
            return;
        }
        if (indicate_led_level(id, level))
            ack_with_info(buf,NULL);
        else
            nak_with_info(buf);          
    }else if(strcmp(params[1], "setc") == 0){
        if (size != 7) {fmt_err_with_info(buf); return;}
        uint16_t r, g, b;
        r = (uint16_t) atoi(params[3]);
        g = (uint16_t) atoi(params[4]);
        b = (uint16_t) atoi(params[5]);
        if (r > 255 || g > 255 || b > 255) {
            fmt_err_with_info(buf);
            return;
        }
    if (indicate_led_color(id, r, g, b))
        ack_with_info(buf,NULL);
    else
        nak_with_info(buf);
    }else{
        fmt_err_with_info(buf);
        return;
    }
    
}

/**
 * @brief   Control low-pass filter settings.
 * @details Sets or retrieves the low-pass filter cutoff frequency.
 *          The configured value is applied to the system and stored
 *          for waveform processing.
 *
 *          Supported sub-commands:
 *          - set <freq> : Set filter frequency
 *          - get        : Get current filter frequency
 *
 * @param[in] params  Command parameters array.
 * @param[in] size    Number of parameters.
 */
static void op_lpf(const char params[][50], uint8_t size) {
    char line[256] = {0};
    int i;
    for (i = 0; i < size; i++) {
        strncat(line, params[i], sizeof(line) - strlen(line) - 1);
        if (i < size - 1) strncat(line, " ", sizeof(line) - strlen(line) - 1);
    }
    int seq_id = parse_cmd_seq(line);
    if (seq_id == 0) {
        fmt_err_with_info(params[0]);
        return;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", seq_id);
    if (strcmp(params[1], "set") == 0) {
        if (size != 4) {fmt_err_with_info(buf); return;}
        if (!is_valid_float(params[2])) {
            fmt_err_with_info(buf);
            return;
        }
        int pwm_hz = atoi(params[2]);
        if (pwm_hz <= 0) {
            fmt_err_with_info(buf);
            return;
        }
        if (lp_filter_set_freq(pwm_hz)) {
            // Update frequency to m_wave_data
            write_waveform_data(WAVEFORM_TARGET_FILTER_FREQ, 0, pwm_hz);
            ack_with_info(buf,NULL);
        } else
            nak_with_info(buf);

        }else if(strcmp(params[1], "get") == 0){
            if (size != 3) {fmt_err_with_info(buf); return;}
                uint32_t pwm_hz = lp_filter_get_freq();
                char freq[32];
                snprintf(freq, sizeof(freq), "%lu", (unsigned long)pwm_hz);
                ack_with_info(buf,freq);
        }else{
            fmt_err_with_info(buf);
            return;
        }
    }

/**
 * @brief   Retrieve main board information.
 * @details Provides firmware version, build information, and git metadata.
 *
 *          Supported sub-commands:
 *          - info version  : Get firmware version
 *          - info vermiis  : Get build version and timestamp
 *          - info git      : Get git version information
 *
 * @param[in] params  Command parameters array.
 * @param[in] size    Number of parameters.
 */
static void op_mainbd(const char params[][50], uint8_t size) {
    char line[256] = {0};
    char out[64];
    int i,n;
    for (i = 0; i < size; i++) {
        strncat(line, params[i], sizeof(line) - strlen(line) - 1);
        if (i < size - 1) strncat(line, " ", sizeof(line) - strlen(line) - 1);
    }
    int seq_id = parse_cmd_seq(line);
    if (seq_id == 0) {
        fmt_err_with_info(params[0]);
        return;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", seq_id);
    if (strcmp(params[1], "info") == 0) {
        if (size != 4) {fmt_err_with_info(buf); return;}
        if(strcmp(params[2], "version") == 0){
            ack_with_info(buf,VERSION);
        }else if(strcmp(params[2], "vermiis") == 0){
            n = snprintf(out, sizeof(out), "%s %s",VERSIONMIIS, DATETIME);
            if (n < 0) { nak_with_info(buf); return;}
            ack_with_info(buf,out);
        }else if(strcmp(params[2], "git") == 0){
            n = snprintf(out, sizeof(out), "%s %s",GIT_HASH, GIT_BRANCH);
            if (n < 0) { nak_with_info(buf); return;}
            ack_with_info(buf,out);            
        }else{
            fmt_err_with_info(buf);
            return;
        }
    }
}

/**
 * @brief   Control MEMS device operations.
 * @details Controls MEMS behavior including enable/disable, bias setting,
 *          position movement, waveform execution, and scan operations.
 *
 *          Supported sub-commands:
 *          - enable <0|1>          : Enable or disable MEMS
 *          - setb <bias>           : Set bias value
 *          - move <x> <y>          : Move to specified position
 *          - runwv [x y]           : Run waveform (optional position)
 *          - scstop                : Stop scan
 *          - scline <params...>    : Start line scan
 *
 * @param[in] params  Command parameters array.
 * @param[in] size    Number of parameters.
 */
static void op_mems(const char params[][50], uint8_t size) {
    char line[256] = {0};
    int i;
    for (i = 0; i < size; i++) {
        strncat(line, params[i], sizeof(line) - strlen(line) - 1);
        if (i < size - 1) strncat(line, " ", sizeof(line) - strlen(line) - 1);
    }
    int seq_id = parse_cmd_seq(line);
    if (seq_id == 0) {
        fmt_err_with_info(params[0]);
        return;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", seq_id);
    if (strcmp(params[1], "enable") == 0) {
        if (size != 4) {fmt_err_with_info(buf); return;}
        if (!is_valid_float(params[2])) {
            fmt_err_with_info(buf);
            return;
        }
        uint8_t enable;
        enable = (uint8_t) atoi(params[2]);
        if (enable > 1) {
            fmt_err_with_info(buf);
            return;
        }
        mems_enable(enable);
        ack_with_info(buf,NULL);
       
    }else if(strcmp(params[1], "setb") == 0){
        if (size != 4) {fmt_err_with_info(buf); return;}
        int bias = atoi(params[2]);
        if (mems_set_bias(bias))
            ack_with_info(buf,NULL);
        else
            nak_with_info(buf);

    }else if(strcmp(params[1], "move") == 0){
        if (size != 5) {fmt_err_with_info(buf); return;}
        int volt_x, volt_y;
        if (!is_valid_float(params[2]) || !is_valid_float(params[3])) {
            fmt_err_with_info(buf);
            return;
        }
        volt_x = atoi(params[2]);
        volt_y = atoi(params[3]);
        if (mems_move(volt_x, volt_y))
            ack_with_info(buf,NULL);
        else
            nak_with_info(buf);
        
    }else if(strcmp(params[1], "runwv") == 0){
        if (size != 3 && size != 5 ) {fmt_err_with_info(buf); return;}
        if (size == 5){
            for (int i = 2; i < size-1; i++) {
            if (!is_valid_float(params[i])) {
                    fmt_err_with_info(buf);
                    return;
                }
            }
        }
        // Default coordinates (0, 0)
        int x_coord = 0;
        int y_coord = 0;
        if (size == 5) {
            x_coord = atoi(params[2]);
            y_coord = atoi(params[3]);
        }
        if (mems_run_waveform(x_coord, y_coord))
            ack_with_info(buf,NULL);
        else
            nak_with_info(buf);
        
    }else if(strcmp(params[1], "scstop") == 0){
        if (size != 3) {fmt_err_with_info(buf); return;}
        mems_stop_scan();
        ack_with_info(buf,NULL);
    }else if(strcmp(params[1], "scline") == 0){
        if (size != 8) {fmt_err_with_info(buf); return;}
            for (int i = 2; i < size-1; i++) {
            if (!is_valid_float(params[i])) {
                fmt_err_with_info(buf);
                return;
            }
        }
        double begin_x, end_x, begin_y, end_y;
        uint8_t scancnt;
        begin_x = atof(params[2]);
        end_x = atof(params[3]);
        begin_y = atof(params[4]);
        end_y = atof(params[5]);
        scancnt = (uint8_t) atoi(params[6]);
        if (mems_start_line_scan(begin_x, end_x, begin_y, end_y, scancnt))
            ack_with_info(buf,NULL);
        else
            nak_with_info(buf);

    }else{
        fmt_err_with_info(buf);
        return;
    }
}

/**
 * @brief   Control motor operations.
 * @details Controls motor movement, including relative/absolute motion,
 *          homing, reset, and status query.
 *
 *          Supported sub-commands:
 *          - status              : Get motor status and position
 *          - relmove <steps>     : Relative movement
 *          - absmove <position>  : Absolute movement
 *          - home                : Move to home position
 *          - reset               : Reset position counter
 *
 * @param[in] params  Command parameters array.
 * @param[in] size    Number of parameters.
 */
static void op_motor(const char params[][50], uint8_t size) {
    char line[256] = {0};
    int i;
    for (i = 0; i < size; i++) {
        strncat(line, params[i], sizeof(line) - strlen(line) - 1);
        if (i < size - 1) strncat(line, " ", sizeof(line) - strlen(line) - 1);
    }
    int seq_id = parse_cmd_seq(line);
    if (seq_id == 0) {
        fmt_err_with_info(params[0]);
        return;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", seq_id);
    uint8_t motorid;
    motorid = (uint8_t) atoi(params[2]);
    if (motorid >= MOTOR_COUNT || motorid < 0) {
        fmt_err_with_info(buf);
        return;
    }
    motors_data[motorid].index = seq_id;
    if (strcmp(params[1], "status") == 0) {
        if (size != 4) {fmt_err_with_info(buf); return;}
        char msg[64];   
        MOTOR_STATUS sts = motors_data[motorid].status_reg;
        int32_t posc = motors_data[motorid].pos_current;
        snprintf(msg, sizeof(msg), "%d %ld", sts.status, posc);
        ack_with_info(buf,msg);
        
    }else if(strcmp(params[1], "relmove") == 0){
        if (size != 5) {fmt_err_with_info(buf); return;}
        char *endptr = NULL;
        long temp = strtol(params[3], &endptr, 10);
        if (*endptr != '\0') {
            fmt_err_with_info(buf);return;
        }
        if (temp > INT32_MAX || temp < INT32_MIN) {
            fmt_err_with_info(buf);
            return;
        }
        int32_t steps = (int32_t)temp;
        //ack_with_info(buf,NULL);
        if (!motormove((uint32_t) motorid, (int32_t) steps))
            nak_with_info(buf);
        
    }else if(strcmp(params[1], "absmove") == 0){
        if (size != 5) {fmt_err_with_info(buf); return;}
        char *endptr = NULL;
        long temp = strtol(params[3], &endptr, 10);
        if (*endptr != '\0') {
            fmt_err_with_info(buf);return;
        }
        if (temp > 1400 || temp < INT32_MIN) {
            fmt_err_with_info(buf);
            return;
        }
        int32_t movepos = (int32_t)temp;
        //ack_with_info(buf,NULL);
        if (!motormoveabs(motorid, movepos))
            nak_with_info(buf);
        
    }else if(strcmp(params[1], "home") == 0){
        if (size != 4) {fmt_err_with_info(buf); return;}
        //ack_with_info(buf,NULL);
        if (!motorhome(motorid))
            nak_with_info(buf);
        
    }else if(strcmp(params[1], "reset") == 0){
        if (size != 4) {fmt_err_with_info(buf); return;}
        motorresetpos((uint32_t) motorid);
        ack_with_info(buf,NULL);
    }else{
        fmt_err_with_info(buf);
        return;
    }
}

/**
 * @brief   Control probe board operations.
 * @details Controls probe module behavior, including enable/disable,
 *          firmware update, and reset operations.
 *
 *          Supported sub-commands:
 *          - enable <0|1> : Enable or disable probe
 *          - update       : Perform firmware update
 *          - reset        : Reset probe module
 *
 * @param[in] params  Command parameters array.
 * @param[in] size    Number of parameters.
 */
static void op_probe(const char params[][50], uint8_t size) {
    char line[256] = {0};
    int i;
    uint8_t enable;
    for (i = 0; i < size; i++) {
        strncat(line, params[i], sizeof(line) - strlen(line) - 1);
        if (i < size - 1) strncat(line, " ", sizeof(line) - strlen(line) - 1);
    }
    int seq_id = parse_cmd_seq(line);
    if (seq_id == 0) {
        fmt_err_with_info(params[0]);
        return;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", seq_id);
    if (strcmp(params[1], "enable") == 0) {
        enable = (uint8_t) atoi(params[2]);
        if (size != 4) {fmt_err_with_info(buf); return;}
        if (enable > 1) {fmt_err_with_info(buf);return;}
        if (gp329_enable(enable))
            ack_with_info(buf,NULL);
        else
            nak_with_info(buf);
        
    }else if(strcmp(params[1], "update") == 0){
        if (size != 3) {fmt_err_with_info(buf); return;}
        gp329_fw_update();
        ack_with_info(buf,NULL);
        
    }else if(strcmp(params[1], "reset") == 0){
        if (size != 3) {fmt_err_with_info(buf); return;}
        gp329_reset();
        ack_with_info(buf,NULL);       
    }else{
        fmt_err_with_info(buf);
        return;
    }
}

/**
 * @brief   Control SLD (Super Luminescent Diode) module.
 * @details Controls SLD power, safety mechanisms, ADC monitoring,
 *          threshold configuration, and status reporting.
 *
 *          Supported sub-commands:
 *          - enable <0|1>     : Enable or disable SLD
 *          - safe <0|1>       : Enable safety check
 *          - status           : Get SLD status
 *          - adc              : Read ADC values
 *          - result           : Trigger result output
 *          - setth <value>    : Set threshold
 *          - getth            : Get threshold
 *          - reset            : Reset flag
 *          - getflag          : Get SLD flag
 *
 * @param[in] params  Command parameters array.
 * @param[in] size    Number of parameters.
 */
static void op_sld(const char params[][50], uint8_t size) {
    char line[256] = {0};
    int i;
    uint8_t enable;
    uint16_t threshold=0;
    for (i = 0; i < size; i++) {
        strncat(line, params[i], sizeof(line) - strlen(line) - 1);
        if (i < size - 1) strncat(line, " ", sizeof(line) - strlen(line) - 1);
    }
    int seq_id = parse_cmd_seq(line);
    if (seq_id == 0) {
        fmt_err_with_info(params[0]);
        return;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", seq_id);
    if (strcmp(params[1], "enable") == 0) {
        enable = (uint8_t) atoi(params[2]);
        if (size != 4) {fmt_err_with_info(buf); return;}
        if (enable > 1) {fmt_err_with_info(buf);return;}
        if(!sld_enable(enable)){
            ack_with_info(buf,NULL);
        }else{
            char msg[32];
            snprintf(msg, sizeof(msg), "%u", sld_flag);
            nck_with_result(buf,msg);
        }
    }else if(strcmp(params[1], "safe") == 0){
        enable = (uint8_t) atoi(params[2]);
        if (size != 4) {fmt_err_with_info(buf); return;}
        if (enable > 1) {fmt_err_with_info(buf);return;}
        sld_cover_check_enable(enable);
        ack_with_info(buf,NULL);
    }else if(strcmp(params[1], "status") == 0){
        if (size != 3) {fmt_err_with_info(buf); return;}
        bool enable = sld_get_enable();
        char info[16];
        snprintf(info, sizeof(info), "%d", enable ? 1 : 0);
        ack_with_info(buf, info);
    }else if (strcmp(params[1], "adc") == 0) {
        if (size != 3) { fmt_err_with_info(buf); return; }
        SLD_ADC_Result res;
        if (!sld_adc(&res)) {
            nak_with_info(buf);
            return;
        }
        /* raw only, three numbers */
        char msg[64];
        int n = snprintf(msg, sizeof(msg), "%u %u %u",
                         (unsigned)res.ch0,
                         (unsigned)res.ch2,
                         (unsigned)res.ch4);
        if (n < 0) {
            nak_with_info(buf);
            return;
        }
        ack_with_info(buf, msg);
    }else if (strcmp(params[1], "result") == 0) {
        if (size != 3) { fmt_err_with_info(buf); return; }
        
        sld_result();               
        ack_with_info(buf, "done");

    }else if (strcmp(params[1], "setth") == 0) {
        if (size != 4) { fmt_err_with_info(buf); return; }
        int val = atoi(params[2]);
        if (val < 0 || val > 4095) {nak_with_info(buf); return;}
        threshold = (uint16_t)val;
        sld_set_pd_threshold(threshold);             
        char msg[32];
        snprintf(msg, sizeof(msg), "%u", threshold);
        ack_with_info(buf, msg);
    }else if (strcmp(params[1], "getth") == 0) {
        if (size != 3) {fmt_err_with_info(buf); return;}
        uint16_t thr = sld_get_pd_threshold();
        char msg[32];
        snprintf(msg, sizeof(msg), "%u", thr);
        ack_with_info(buf, msg);
    }else if (strcmp(params[1], "reset") == 0) {
        if (size != 3) {fmt_err_with_info(buf); return;}
        uint16_t flag = 0;
        if(SST26_WriteSLD("sldflg", flag)){
            sld_flag = flag;
            char msg[32];
            snprintf(msg, sizeof(msg), "%u", sld_flag);
            ack_with_info(buf, msg);
        }else{
            nak_with_info(buf);
        }
    }else if (strcmp(params[1], "getflag") == 0) {
        if (size != 3) {fmt_err_with_info(buf); return;}
            char msg[32];
            snprintf(msg, sizeof(msg), "%u", sld_flag);
            ack_with_info(buf, msg);
    }else{
        fmt_err_with_info(buf);
        return;
    }
}

static void op_getVersion(const char params[][50], uint8_t size) {
    printf("%s\r\n", VERSION);

}

static void op_getBuildVersion(const char params[][50], uint8_t size) {
    printf("%s%s\r\n", VERSIONMIIS, DATETIME);
}

static void op_get_git_version(const char params[][50], uint8_t size) {
    printf("git:%s, branch:%s tag:%s\r\n", GIT_HASH, GIT_BRANCH,GIT_TAG);
}

static void op_WDTtest(const char params[][50], uint8_t size) {
    printf("Test WDT function, system will reboot after 4 seconds.\r\n");
    while (1);
}

static void op_SPITtest(const char params[][50], uint8_t size) {
    uint16_t data;
    data = (uint16_t) atoi(params[0]);
    printf("\r\ntxData: %x\r\n", data);
    SPI_Write(data);
}

static void op_show_waveform_data(const char params[][50], uint8_t size) {
    show_waveform_data();
}

static void op_format_disk(const char params[][50], uint8_t size) {
    if (SST26_FormatDisk()) {
        ack();
    } else {
        nak();
    }
}

static void op_get_filesystem_dir(const char params[][50], uint8_t size) {
    SST26_ReadDir();
}

// For debug use

void hexdump(char *addr, int size) {
    for (int i = 0; i < size; i++) {
        printf("%02x ", *(addr + i));
        if ((i + 1) % 16 == 0) {
            printf("\r\n");
        }
    }
}
static void op_list_all_cmd(const char params[][50], uint8_t size) {
    printf("Usage:\r\n");
    for (int i = 0; opTable[i].function != 0; i++) {
        printf("\t%s\r\n", opTable[i].cmdstr);
    }
}

static void op_show_debug_info(const char params[][50], uint8_t size) {
    show_information();
}

static void op_reset_usb_hub(const char params[][50], uint8_t size) {
    USB_HUB_RST_Clear(); // Pull low
    SYSTICK_DelayMs(USB_HUB_RESET_PULSE_WIDTH_MS);
    USB_HUB_RST_Set(); // Pull high
    ack();
}
parsed_data_t parsedData;
/**
 * @brief   Parse and execute a command string. (Ref: SD19)
 * @details This function parses the input command string into tokens
 *          separated by spaces and stores them into a structured format.
 *          The first token is treated as the command keyword, and the
 *          remaining tokens are treated as parameters.
 *
 *          The function then searches the command operation table for a
 *          matching command string and invokes the corresponding handler
 *          function. If no matching command is found, an error message is
 *          returned and a negative acknowledgment is triggered.
 *
 *          Processing steps:
 *          1. Copy input command string to local buffer
 *          2. Tokenize input using space delimiter
 *          3. Store tokens into parsedData structure
 *          4. Search operation table (opTable[])
 *          5. Execute matched command handler
 *          6. Handle unknown command case
 *
 * @param[in] cmdstr  Input command string.
 * @param[in] size    Length of the command string.
 *
 * @note    The first token (params[0]) is treated as the command keyword.
 * @note    Command handlers are defined in operation table (operation_t).
 *
 * @warning Input buffer size is limited to 256 bytes.
 * @warning Number of parameters is limited by MAX_PARAMS.
 */
void process_cmd(char cmdstr[], size_t size) {
    if (size <= 1) {
        printf("\r\n");
        return;
    }

    char buffer[256];
    strncpy(buffer, cmdstr, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char* token = strtok(buffer, " ");
    parsedData.paramCount = 0;

    //  token ("probe") params[0]
    while (token != NULL) {
        if (parsedData.paramCount < MAX_PARAMS) {
            strncpy(parsedData.params[parsedData.paramCount],
                    token,
                    sizeof(parsedData.params[parsedData.paramCount]) - 1);
            parsedData.params[parsedData.paramCount]
                [sizeof(parsedData.params[parsedData.paramCount]) - 1] = '\0';
            parsedData.paramCount++;
        }
        token = strtok(NULL, " ");
    }
    for (int i = 0; opTable[i].function != NULL; i++) {
        if (strcmp(parsedData.params[0], opTable[i].cmdstr) == 0) {
            opTable[i].function(parsedData.params, parsedData.paramCount);
            return;
        }
    }

    printf("Command \"%s\" not found!\r\n", parsedData.params[0]);
    nak();
    return;
}
/* *****************************************************************************
 End of File
 */
