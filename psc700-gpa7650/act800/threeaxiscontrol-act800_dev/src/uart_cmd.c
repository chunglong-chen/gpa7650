/**
 * @file uart_cmd.c
 * @brief Implementation of uart transceiver and uart command parser functions.
 */

#include "stdio.h"
#include "stdarg.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "limits.h"
#include "NuMicro.h"
#include "uart_cmd.h"
#include "motor_ctrl.h"
#include "user_config.h"
#include "main.h"
#include "version.h"
#include "ctype.h"
#include "delay.h"

#define UART_ECHO 1
#define PROC_DEL  1

static uint8_t uart_inbuf[512];         /**< buffer of UART input */
static uint8_t uart_outbuf[512];        /**< buffer of UART output */
static uint8_t cmd_buf[512];            /**< buffer of input command */

RingBufferU8 uartRxBuf;                 /**< UART rx ring buffer, data is in uart_inbuf */
RingBufferU8 uartTxBuf;                 /**< UART tx ring buffer, data is in uart_outbuf */

static volatile bool isTxBusy = FALSE;  /**< is TX is transmitting */

static void process_cmd(uint8_t* cmdstr, uint32_t size);
static void start_tx();
static inline void nak();
static inline void ack();

static void op_help(int argc, char** argv);

static void op_getVersion(int argc, char** argv);

static void op_wait(int argc, char** argv);

static void op_stepMoveHome(int argc, char** argv);
static void op_stepMoveHome3(int argc, char** argv);
static void op_stepMove(int argc, char** argv);
static void op_stepMove3(int argc, char** argv);

static void op_getPosition(int argc, char** argv);
static void op_setPosition(int argc, char** argv);

static void op_setSpeed(int argc, char** argv);
static void op_getSpeed(int argc, char** argv);
static void op_saveSpeed(int argc, char** argv);
    
static void op_setDirection(int argc, char** argv);
static void op_getDirection(int argc, char** argv);
static void op_saveDirection(int argc, char** argv);

static void op_setLimitSwitch(int argc, char** argv);
static void op_getLimitSwitch(int argc, char** argv);
static void op_saveLimitSwitch(int argc, char** argv);

static void op_setTorque(int argc, char** argv);
static void op_getTorque(int argc, char** argv);
static void op_saveTorque(int argc, char** argv);
static void op_eraseTorque(int argc, char** argv);

static void op_saveConfig(int argc, char** argv);
static void op_eraseConfig(int argc, char** argv);

static void op_getBorder(int argc, char** argv);
static void op_setBorder(int argc, char** argv);
static void op_saveBorder(int argc, char** argv);
static void op_eraseBorder(int argc, char** argv);

static void op_stepMoveGateHome(int argc, char** argv);
static void op_stepMoveGate(int argc, char** argv);

#define ARRAY_ITEM_COUNT(A) sizeof((A))/sizeof((A[0]))

/**
 * @brief UART command dispatch table.
 *
 * @details Each command string is mapped to a corresponding function that
 *          handles the command. These functions follow the signature:
 *          `void op_xxx(int argc, char** argv)`.
 *
 *          This table is used by the command processor to look up and execute commands.
 */
const Operation opTable[] = {
    {"help", op_help},
    
    {"version", op_getVersion},
    
    {"wait", op_wait},
    
    {"home", op_stepMoveHome},
    {"home3", op_stepMoveHome3},
    
    {"mvr", op_stepMove},
    {"mva", op_stepMove},
    
    {"mvr3", op_stepMove3},
    {"mva3", op_stepMove3},
    
    {"setpos", op_setPosition},
    {"getpos", op_getPosition},
    
    {"setspd", op_setSpeed},
    {"getspd", op_getSpeed},
    {"savespd", op_saveSpeed},
    
    {"setdir", op_setDirection},
    {"getdir", op_getDirection},
    {"savedir", op_saveDirection},
    
    {"setlim", op_setLimitSwitch},
    {"getlim", op_getLimitSwitch},
    {"savelim", op_saveLimitSwitch},
    
    {"settrq", op_setTorque},
    {"gettrq", op_getTorque},
    {"savetrq", op_saveTorque},
    {"deltrq", op_eraseTorque},
    
    {"savecfg", op_saveConfig},
    {"delcfg", op_eraseConfig},
    
    {"getbdr", op_getBorder},
    {"setbdr", op_setBorder},
    {"savebdr", op_saveBorder},
    {"delbdr", op_eraseBorder},

    {"homeg", op_stepMoveGateHome},
    {"mvgr", op_stepMoveGate},
    {"mvga", op_stepMoveGate},

};

/**
 * @brief Initializes the UART command ring buffers.
 *
 * @details This function sets up the UART RX and TX ring buffers using the pre-allocated
 *          `uart_inbuf` and `uart_outbuf` arrays. It also clears the TX busy flag and
 *          prints a startup message.
 *
 * @note This function must be called before using any UART command features.
 */
void init_uart_cmd(){
    RingBufferU8_init(&uartRxBuf, uart_inbuf, sizeof(uart_inbuf));
    RingBufferU8_init(&uartTxBuf, uart_outbuf, sizeof(uart_outbuf));  
    isTxBusy = false;
    uart_printf("\ruart ready\r\n> ");
}

/**
 * @brief Parse a numeric value into an IO_STATE.
 *
 * @details This function maps an integer value to a corresponding IO state:
 *           If the value is greater than 0, it returns `io_high`
 *           If the value is less than 0, it returns `io_hiz` (high-impedance)
 *           If the value is 0, it returns `io_low`
 *
 * @param value The input integer to parse.
 * @return IO_STATE enum indicating the IO status.
 */
static inline IO_STATE parseIoState(int32_t value){
    if (value > 0) return io_high;
    if (value < 0) return io_hiz;
    return io_low;   
}

/**
 * @brief Parse a motor ID from a string.
 *
 * @details This function converts a string (such as "0", "1", ..., or "x", "y", "z", "g")
 *          into a corresponding motor ID index (0-based). If the string does not match
 *          a valid motor ID, it returns MOTOR_COUNT.
 *
 *          Recognized mappings:
*           'x' : 0
*           'y' : 1
*           'z' : 2
*           'g' : 3
 *
 * @param str Pointer to a character string representing the motor ID.
 * @return The parsed motor ID index (0 ~ MOTOR_COUNT - 1), or MOTOR_COUNT if invalid.
 */
static uint32_t parse_motor_id(char* str){
    uint32_t id = MOTOR_COUNT;
    if(isdigit(str[0]))
        id=atoi(str);
    else{
        switch(str[0]){
            case 'x': id = 0; break;
            case 'y': id = 1; break;
            case 'z': id = 2; break;
            case 'g': id = 3; break;
            default:  id = MOTOR_COUNT;
        }
    }
    id = id >= MOTOR_COUNT ? MOTOR_COUNT : id;
    return id;
}

/**
 * @brief Parse step count from a string.
 *
 * @details This function extracts an integer step count from a string. If the first character
 *          is an alphabetic character (e.g., 'X', 'Y', 'Z', 'G'), it is skipped before parsing.
 *          This is useful for strings like "X1000", "Y-500", etc.
 *
 * @param str Pointer to the input string, e.g., "X1000", "500".
 * @return Parsed step count as an integer.
 */
static int parse_motor_step(char* str){
    int steps=0;
    if(isalpha(str[0]))
        steps=atoi(str+1);  //skip axis symbol
    else
        steps=atoi(str);
    return steps;
}

/**
 * @brief UART command `help` entrance.
 *
 * @details Prints the list of available UART commands.
 */
static void op_help(int argc, char** argv){
    uart_printf("available commands:\n\r");
    for(int opid = 0; opid<ARRAY_ITEM_COUNT(opTable); opid++){
        uart_printf("  - %s\n\r", opTable[opid].cmdstr);
    }
}

/**
 * @brief UART command `version` entrance.
 *
 * @details Prints firmware version string.
 */
static void op_getVersion(int argc, char** argv){
    uart_printf("%s\n", VERSION); 
}

/**
 * @brief UART command `wait` entrance.
 *
 * @details Waits for specified motors to complete their operations.
 */
static void op_wait(int argc, char** argv){
    uint32_t id = 0;
    char wait_for[MOTOR_COUNT]={FALSE};
    
    for(uint32_t i=1; i<argc; i++){
        id = parse_motor_id(argv[i]);
        if(id <= (MOTOR_COUNT-1)) wait_for[id] = TRUE;
    }
    
    while(1){
        char busy=FALSE;
        for(int i=0; i<MOTOR_COUNT; i++){
            if(wait_for[i])
                busy = busy || *(get_motor_ref(i)->busyPin.datAddr);
        }
        if(!busy) break;
    }
    
    ack();
}

/**
 * @brief UART command `home` entrance.
 *
 * @details Moves the motor to the home position.
 */
static void op_stepMoveHome(int argc, char** argv){
    uint32_t id = 0;
    char success = TRUE;
    for(uint32_t i=1; i<argc; i++){
        id = parse_motor_id(argv[i]);
        if(id < (MOTOR_COUNT-1))
            success = motor_home(id) && success;
        else if(id == (MOTOR_COUNT-1))
            success = motor_gate_home() && success;
    }
    success ? ack() : nak();
}

/**
 * @brief UART command `home3` entrance.
 *
 * @details Moves three motors to their home positions.
 */
static void op_stepMoveHome3(int argc, char** argv){
    motor_home3() ? ack() : nak();
}

/**
 * @brief UART command `mvr` and `mva` entrance.
 *
 * @details Moves the motor by a relative or absolute number of steps.
 */
static void op_stepMove(int argc, char** argv){
    uint32_t id = 0;
    int32_t steps = 0;
    char success = TRUE;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > (MOTOR_COUNT-1)) continue;
        
        //parse step
        if(strlen(argv[i])==1 && i < argc-1)    // movr x -500
            steps = parse_motor_step(argv[++i]);
        else                                    // movr x-500
            steps = parse_motor_step(argv[i]);
        
        //move
        if(strchr(argv[0], 'r')) {            //movr
            if(id == (MOTOR_COUNT-1))
                success = motor_move_gate_related(steps) && success;
            else
                success = motor_move_related(id, steps) && success;
        } else if(strchr(argv[0], 'a')) {      //mova
            if(id == (MOTOR_COUNT-1))
                success = motor_move_gate_abspos(steps) && success;
            else
                success = motor_move_abspos(id, steps) && success;
        }
    }
    
    success ? ack() : nak();
}

/**
 * @brief UART command `mvr3` and `mva3` entrance.
 *
 * @details Moves three motors by relative or absolute steps.
 */
static void op_stepMove3(int argc, char** argv){
    uint32_t id = 0;
    int32_t steps[3] = {0};
    char success = TRUE;
    
    for(uint32_t i=1; i<argc && i<4; i++){
        id = i-1; //0, 1, 2
        steps[id] = parse_motor_step(argv[i]);
    }
    
    if(strchr(argv[0], 'r'))            //movr3
        success = motor_move_related3(steps[0], steps[1], steps[2]) && success;
    else if(strchr(argv[0], 'a'))       //mova3
        success = motor_move_abspos3(steps[0], steps[1], steps[2]) && success;

    success ? ack() : nak();
}

/**
 * @brief UART command `getpos` entrance.
 *
 * @details Prints the current motor position.
 */
static void op_getPosition(int argc, char** argv){
    uint32_t id = 0;
    int32_t steps = 0;
    char str[128]={0};
    char success = TRUE;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > (MOTOR_COUNT-1)) continue;
        snprintf(str, 128, "%s%d ", str, get_motor_ref(id)->position);
    }
    uart_printf("%s\r\n", str);
    ack();
}

/**
 * @brief UART command `setpos` entrance.
 *
 * @details Sets the current motor position value.
 */
static void op_setPosition(int argc, char** argv){
    uint32_t id = 0;
    int32_t steps = 0;
    char success = TRUE;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > (MOTOR_COUNT-1)) continue;
        
        //parse step
        if(strlen(argv[i])==1 && i < argc-1)    // movr x -500
            steps = parse_motor_step(argv[++i]);
        else                                    // movr x-500
            steps = parse_motor_step(argv[i]);
        
        get_motor_ref(id)->position=steps;
    }
    ack();
}

/**
 * @brief Parse a speed tag from a string.
 *
 * @details This function analyzes the input string and returns the corresponding
 *          speed control tag defined in the SpeedTag enumeration.
 *
 * | Input String | Return Enum    |
 * |--------------|----------------|
 * | "s0"         | INITIAL\_SPEED |
 * | "s1"         | MAXIMUM\_SPEED |
 * | "a+"         | ACCELERATION   |
 * | "a-"         | DECELERATION   |
 * | "e"          | ACCL\_ENABLE   |
 * | "d"          | ACCL\_DISABLE  |
 *
 * @param str Pointer to the input string (e.g., "s0", "a+", "e").
 * @return Corresponding SpeedTag value. Returns NOT_SPEED_TAG if the string is unrecognized.
 */
static SpeedTag parse_speed_tag(char* str){
    switch(str[0]){
        case 's':
            switch(str[1]){
                case '0': return INITIAL_SPEED;
                case '1': return MAXIMUM_SPEED;
            }
        case 'a':
            switch(str[1]){
                case '+': return ACCELERATION;
                case '-': return DECELERATION;
            }
        case 'e':
            return ACCL_ENABLE;
        case 'd':
            return ACCL_DISABLE;
    }
    return NOT_SPEED_TAG;
}

/**
 * @brief UART command `setspd` entrance.
 *
 * @details Sets the motor speed parameters.
 */
static void op_setSpeed(int argc, char** argv){
    uint32_t id = MOTOR_COUNT;
    SpeedTag speed_tag = NOT_SPEED_TAG;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        uint32_t temp_id = parse_motor_id(argv[i]);
        if(temp_id < MOTOR_COUNT) {
            id = temp_id;
            continue;
        }
        if(id >= MOTOR_COUNT) continue;
        speed_tag = parse_speed_tag(argv[i]);
        if(speed_tag == NOT_SPEED_TAG) continue;
        
        uint32_t value=0;
        if(strlen(argv[i])>2)
            value=atoi(argv[i]+2);
        else if(argc > i+1)
            value=atoi(argv[i+1]);
        if(value==0 && speed_tag!=ACCL_ENABLE && speed_tag!=ACCL_DISABLE)
            continue;
        
        switch(speed_tag){
            case INITIAL_SPEED:
                get_motor_ref(id)->defaultPPS=value;
                break;
            case MAXIMUM_SPEED:
                get_motor_ref(id)->targetPPS=value;
                break;
            case ACCELERATION:
                get_motor_ref(id)->acceleration=value;
                break;
            case DECELERATION:
                get_motor_ref(id)->deceleration=value;
                break;
            case ACCL_ENABLE:
                get_motor_ref(id)->enableMototAccelerate=true;
                break;
            case ACCL_DISABLE:
                get_motor_ref(id)->enableMototAccelerate=false;
                break;
            case NOT_SPEED_TAG:
                break;
        }
    }
    ack();
}

/**
 * @brief UART command `getspd` entrance.
 *
 * @details Prints the current motor speed settings.
 */
static void op_getSpeed(int argc, char** argv){
    #define STRING_SIZE (512)
    uint32_t id = 0;
    char str[STRING_SIZE]={0};
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > (MOTOR_COUNT-1)) continue;
        switch(id){
            case 0: snprintf(str, STRING_SIZE, "%sx: ", str); break;
            case 1: snprintf(str, STRING_SIZE, "%sy: ", str); break;
            case 2: snprintf(str, STRING_SIZE, "%sz: ", str); break;
            case 3: snprintf(str, STRING_SIZE, "%sg: ", str); break;
        }
        snprintf(str, STRING_SIZE, "%s [%s] s0=%d, s1=%d, a+=%d, a-=%d\r\n",
           str,
          (get_motor_ref(id)->enableMototAccelerate?"accel. on":"const sp."),
           get_motor_ref(id)->defaultPPS,
           get_motor_ref(id)->targetPPS,
           get_motor_ref(id)->acceleration,
           get_motor_ref(id)->deceleration);
    }
    printf("%s", str);
    ack();
    #undef STRING_SIZE
}

/**
 * @brief UART command `savespd` entrance.
 *
 * @details Saves current motor speed settings to flash.
 */
static void op_saveSpeed(int argc, char** argv){
    uint32_t id = 0;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > (MOTOR_COUNT-1)) continue;
        copySpeedToUserConfig(id);
    }
    (saveUserConfigToFlash() ? ack() : nak());
}
    
/**
 * @brief UART command `setdir` entrance.
 *
 * @details Sets the motor rotation direction.
 */
static void op_setDirection(int argc, char** argv){
    uint32_t id = 0;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > (MOTOR_COUNT-1)) continue;
        
        //parse direction
        char flag=0;
        if(strlen(argv[i])>1){
            flag=argv[i][1];
        }else{
            flag=argv[++i][1];
        }
        
        switch(flag){
            case '+': setDirection(id, CFG_DIR_DEFAULT); break;
            case '-': setDirection(id, CFG_DIR_REVERSE); break;
        }
    }
    ack();
}

/**
 * @brief UART command `getdir` entrance.
 *
 * @details Prints the motor direction setting.
 */
static void op_getDirection(int argc, char** argv){
    uint32_t id = 0;
    char str[128]={0};
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > (MOTOR_COUNT-1)) continue;
        
        switch(getDirection(id)){
            case CFG_DIR_DEFAULT:
                snprintf(str, sizeof(str), "%s+ ", str);
            break;
            case CFG_DIR_REVERSE:
                snprintf(str, sizeof(str), "%s- ", str);
            break;
        }
    }
    uart_printf("%s\n\r", str);
    ack();
    
}

/**
 * @brief UART command `savedir` entrance.
 *
 * @details Saves motor direction setting to flash.
 */
static void op_saveDirection(int argc, char** argv){
    uint32_t id = 0;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > (MOTOR_COUNT-1)) continue;
        copyDirectionConfigToUserConfig(id);
    }
    (saveUserConfigToFlash() ? ack() : nak());
}

/**
 * @brief UART command `setlim` entrance.
 *
 * @details Sets the motor limit switch configuration.
 */
static void op_setLimitSwitch(int argc, char** argv){
    uint32_t id = 0;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > (MOTOR_COUNT-1)) continue;
        
        //parse direction
        char flag=0;
        if(strlen(argv[i])>1){
            flag=argv[i][1];
        }else{
            flag=argv[++i][1];
        }
        
        switch(flag){
            case '+': setLimitSwitch(id, CFG_LIMSW_DEFAULT); break;
            case '-': setLimitSwitch(id, CFG_LIMSW_REVERSE); break;
        }
    }
    ack();
}

/**
 * @brief UART command `getlim` entrance.
 *
 * @details Prints the limit switch configuration.
 */
static void op_getLimitSwitch(int argc, char** argv){
    uint32_t id = 0;
    char str[128]={0};
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > (MOTOR_COUNT-1)) continue;
        
        switch(getLimitSwitch(id)){
            case CFG_LIMSW_DEFAULT:
                snprintf(str, sizeof(str), "%s+ ", str);
            break;
            case CFG_LIMSW_REVERSE:
                snprintf(str, sizeof(str), "%s- ", str);
            break;
        }
    }
    uart_printf("%s\n\r", str);
    ack();
}

/**
 * @brief UART command `savelim` entrance.
 *
 * @details Saves limit switch configuration to flash.
 */
static void op_saveLimitSwitch(int argc, char** argv){
    uint32_t id = 0;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > (MOTOR_COUNT-1)) continue;
        copyPinConfigToUserConfig(id);
    }
    (saveUserConfigToFlash() ? ack() : nak());
}

/**
 * @brief UART command `settrq` entrance.
 *
 * @details Sets motor torque parameters.
 */
static void op_setTorque(int argc, char** argv){
    uint32_t id = 0;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > (MOTOR_COUNT-1)) continue;
        
        //parse number using already existing parse_motor_step()
        int torque=0;
        if(strlen(argv[i])==1 && i < argc-1)
            torque = parse_motor_step(argv[++i]);
        else
            torque = parse_motor_step(argv[i]);
        if(torque<0 || torque>8) continue;
        
        select_current(torque, &(get_motor_ref(id)->activeI0State),
                               &(get_motor_ref(id)->activeI1State));
    }
    ack();
}

/**
 * @brief UART command `gettrq` entrance.
 *
 * @details Prints current torque parameters.
 */
static void op_getTorque(int argc, char** argv){
    uint32_t id = 0;
    char str[128]={0};
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > (MOTOR_COUNT-1)) continue;
        snprintf(str, 128, "%s%d ",
                 str, parse_current_setting(get_motor_ref(id)->activeI0State,
                                            get_motor_ref(id)->activeI1State));
    }
    uart_printf("%s\n\r", str);
    ack();
}

/**
 * @brief UART command `savetrq` entrance.
 *
 * @details Saves torque settings to flash.
 */
static void op_saveTorque(int argc, char** argv){
    for(int id=0; id<MOTOR_COUNT; id++){
        torqueConfig[id].active_torque=parse_current_setting(
                                            get_motor_ref(id)->activeI0State,
                                            get_motor_ref(id)->activeI1State);
    }
    saveTorqueConfigToFlash() ? ack() : nak();
}

/**
 * @brief UART command `deltrq` entrance.
 *
 * @details Deletes saved torque settings from flash.
 */
static void op_eraseTorque(int argc, char** argv){
    eraseTorqueConfigFromFlash() ? ack() : nak();
}

/**
 * @brief UART command `savecfg` entrance.
 *
 * @details Saves all configuration to flash.
 */
static void op_saveConfig(int argc, char** argv){
    for(int i=0; i<MOTOR_COUNT; i++){
        copySpeedToUserConfig(i);
        copyDirectionConfigToUserConfig(i);
        copyPinConfigToUserConfig(i);
    }
    (saveUserConfigToFlash() ? ack() : nak());
}

/**
 * @brief UART command `delcfg` entrance.
 *
 * @details Deletes configuration stored in flash.
 */
static void op_eraseConfig(int argc, char** argv){
    eraseUserConfigFromFlash() ? ack() : nak();
}

/**
 * @brief UART command `getbdr` entrance.
 *
 * @details Prints the current no-trespass border value.
 */
static void op_getBorder(int argc, char** argv){
    char str[512]={0};
    for(int i=1; i<argc; i++){
        int id=-1;
        if(isdigit(argv[i][0])) id=atoi(argv[i]);
        if(id>=0 && id <8){
            snprintf(str, 512, "%s%d: [%s] %fx + %fy + %fz %c %f\r\n", str,
                    id, (noTrespassBorder[id].enable ? "active": "unused"),
                         noTrespassBorder[id].a,
                         noTrespassBorder[id].b,
                         noTrespassBorder[id].c,
                        (noTrespassBorder[id].greater_than?'>':'<'),
                         noTrespassBorder[id].d);
        }
    }
    uart_printf(str);
    ack();
}

/**
 * @brief UART command `setbdr` entrance.
 *
 * @details Sets the no-trespass border value.
 */
static void op_setBorder(int argc, char** argv)
{
    int id = -1;
    if(isdigit(argv[1][0]))
        id=atoi(argv[1]);
    if(id<0 || id>7){nak(); return;}
    NO_TRESPASS_BORDER border=noTrespassBorder[id];
    
    for(int i=2; i<argc; i++){
        if(strlen(argv[i]) == 1){
            switch(argv[i][0]){
                case 'e': border.enable=true;  break;
                case 'd': border.enable=false; break;
                case '>': border.greater_than=true; break;
                case '<': border.greater_than=false; break;
            }
        }
        else{
            switch(argv[i][0]){
                case 'a': border.a=atof(argv[i]+1); break;
                case 'b': border.b=atof(argv[i]+1); break;
                case 'c': border.c=atof(argv[i]+1); break;
                case 'd': border.d=atof(argv[i]+1); break;
            }
        }
    }
    setNoTrespassBorder(id, border);
    ack();
}

/**
 * @brief UART command `savebdr` entrance.
 *
 * @details Saves the no-trespass border value to flash.
 */
static void op_saveBorder(int argc, char** argv)
{
    saveNoTrespassBordersToFlash() ? ack() : nak();
}

/**
 * @brief UART command `delbdr` entrance.
 *
 * @details Deletes the saved no-trespass border configuration.
 */
static void op_eraseBorder(int argc, char** argv)
{
    eraseNoTrespassBordersFromFlash() ? ack() : nak();
}

/**
 * @brief UART command `homeg` entrance.
 *
 * @details Homes the gate motor.
 */
static void op_stepMoveGateHome(int argc, char** argv){
    motor_gate_home() ? ack() : nak();
}

/**
 * @brief UART command `mvgr` and `mvga` entrance.
 *
 * @details Moves the gate motor relatively or to absolute position.
 */
static void op_stepMoveGate(int argc, char** argv){
    int32_t steps = 0;
    char success = false;

    if (argc >=2) {
        //parse step
        steps = parse_motor_step(argv[1]);

        //move
        if(strchr(argv[0], 'r'))            //mvgr
            success = motor_move_gate_related(steps);
        else if(strchr(argv[0], 'a'))       //mvga
            success = motor_move_gate_abspos(steps);
    }
    success ? ack() : nak();
}

/**
 * @brief Handle incoming UART commands (non-blocking task).
 *
 * @details This function should be called repeatedly (e.g., inside a `while(1)` loop).
 *          It checks if a complete command line is available in the UART ring buffer.
 *          If a line is available, it reads the line into `cmd_buf` and processes it.
 *
 * @note This function is non-blocking. It performs a `__NOP()` if no data is available.
 *
 * @warning If the UART input buffer wraps around while a command is being received,
 *          this function may enter an unintended infinite loop. This should be handled properly
 *          in future revisions.
 */
void cmd_task(){
    if (uartRxBuf.line_counts > 0){
        RingBufferU8_readLine(&uartRxBuf, (uint8_t*)cmd_buf, sizeof(cmd_buf));
        process_cmd(cmd_buf, sizeof(cmd_buf));
        // todo: if the uart_inbuf used by Rx is wrapped around, this loop runs 
        //       forever. We wight need to fix this at some point.
    } else
        __NOP();
}

/**
 * @brief  Parse and execute a command from the input string.
 *
 * @details This function tokenizes the given input string into arguments, searches for a matching
 *          command in the command table `opTable`, and executes the associated function if found.
 *          If the command is not recognized, an error message is printed via UART.
 *
 * @param[in]  cmdstr  The input command string, typically read from UART. It will be modified during parsing.
 * @param[in]  size    The size of the input buffer (not directly used, but reserved for future use or safety checks).
 *
 * @note The input string is modified in place due to the use of `strtok()`.
 */
static void process_cmd(uint8_t* cmdstr, uint32_t size){
    int opid = 0, argc = 0;
    char *argv[64]={0};
    
    //segment input string into argument array
    argv[argc]=strtok((char*)cmdstr, " \n\r");
    if(argv[argc]==NULL){ // pressing enter only
        uart_printf("\n\r> ");
        return;
    }
    while((argv[++argc] = strtok(NULL, " \n\r"))!= NULL);
    
    uart_printf("\n\r");
    
    //lookup command table and call
    for(opid = 0; opid<ARRAY_ITEM_COUNT(opTable); opid++){
        if(strcmp(opTable[opid].cmdstr, argv[0]) == 0){
            opTable[opid].function(argc, argv);
            break;
        }
    }
    
    //input string not found in command table
    if (opid == ARRAY_ITEM_COUNT(opTable))
        uart_printf("unknown command: \"%s\"\r\n", argv[0]);
    
    uart_printf("> ");
}

/**
 * @brief Formatted UART print function.
 *
 * @details Sends a formatted string via UART. Similar to `printf`, this function
 *          formats the given input and pushes it to the UART transmit buffer.
 *          A carriage return `\r` is prepended automatically.
 *
 * @param[in] format The printf-style format string.
 * @param[in] ...    Variable argument list corresponding to the format string.
 */
void uart_printf(const char* format, ... ){
    va_list args;
    va_start(args, format);
    int sz = vsnprintf(NULL, 0, format, args);
    uint8_t re[sz+1];
    uint8_t CR[1]={'\r'};
    RingBufferU8_write(&uartTxBuf, CR, 1);
    vsnprintf((char *)re, sizeof(re), format, args);
    RingBufferU8_write(&uartTxBuf, re, sz);
    va_end(args);
    start_tx();
}

/**
 * @brief Starts UART transmission if not already in progress.
 *
 * @details Enables TX interrupt to begin sending data from the UART transmit buffer.
 *          This function checks `isTxBusy` to prevent re-entry.
 */
void start_tx(){
    if (!isTxBusy){
        isTxBusy = TRUE;
        //uint8_t b = RingBufferU8_readByte(&uartTxBuf);
        //UART0->DAT = (uint32_t)b;  
        UART_ENABLE_INT(UART0, UART_INTEN_TXENDIEN_Msk);
    }
}

/**
 * @brief UART0 interrupt handler.
 *
 * @details The handler is responsible for managing UART transmission and reception.
 */
void UART0_IRQHandler(void){
    
    //Rx
    if (UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk)){
        /* Read data until RX FIFO is empty */
        while(UART_IS_RX_READY(UART0))
        {
            uint8_t r = UART_READ(UART0);
            
#if PROC_DEL
            //process backspace or DEL=127
            if(r=='\b' || r=='\x7F'){
                if(RingBufferU8_deleteByte(&uartRxBuf)){
    #if UART_ECHO   //echo del char to console if a char is deleted from ring
                    UART0->DAT = (uint32_t)r;
    #endif
                }
                return;     //return anyway if it's a del char
            }
#endif
#if UART_ECHO
            UART0->DAT = (uint32_t)r;
#endif
            //the ring buffer counts only '\n' to determin lines
            r = ( r=='\r' ? '\n' : r );
            RingBufferU8_writeByte(&uartRxBuf, r);
        }
    }
    
    //Tx
    if (UART_GET_INT_FLAG(UART0, UART_INTSTS_TXENDINT_Msk)){
        if (RingBufferU8_available(&uartTxBuf)){
            uint8_t b = RingBufferU8_readByte(&uartTxBuf);
            UART0->DAT = (uint32_t)b;
        }
        else{
            UART_DISABLE_INT(UART0, UART_INTEN_TXENDIEN_Msk);
            isTxBusy = FALSE;          
        }
    }
}

/**
 * @brief Send a negative acknowledgment message via UART.
 *
 * @details Prints `"nak"`.
 */
static inline void nak(){
    uart_printf("nak\r\n");
}


/**
 * @brief Send an acknowledgment message via UART.
 *
 * @details Prints `"ack"`.
 */
static inline void ack(){
    uart_printf("ack\r\n");
}
;