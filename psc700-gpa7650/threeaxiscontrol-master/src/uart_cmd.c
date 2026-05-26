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

static uint8_t uart_inbuf[128];    //buffer of UART input
static uint8_t uart_outbuf[512];   //buffer of UART output
static uint8_t cmd_buf[128];   //buffer of input command

RingBufferU8 uartRxBuf;     //UART rx ring buffer, data is in uart_inbuf
RingBufferU8 uartTxBuf;     //UART tx ring buffer, data is in uart_outbuf

static volatile bool isTxBusy = FALSE;  //is TX is transmitting

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

#define ARRAY_ITEM_COUNT(A) sizeof((A))/sizeof((A[0]))

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
};

void init_uart_cmd(){
    RingBufferU8_init(&uartRxBuf, uart_inbuf, sizeof(uart_inbuf));
    RingBufferU8_init(&uartTxBuf, uart_outbuf, sizeof(uart_outbuf));  
    isTxBusy = false;
    uart_printf("\ruart ready\r\n> ");
}

static inline IO_STATE parseIoState(int32_t value){
    if (value > 0) return io_high;
    if (value < 0) return io_hiz;
    return io_low;   
}

static uint32_t parse_motor_id(char* str){
    uint32_t id = 3;
    if(isdigit(str[0]))
        id=atoi(str);
    else{
        switch(str[0]){
            case 'x': id = 0; break;
            case 'y': id = 1; break;
            case 'z': id = 2; break;
            default:  id = 3;
        }
    }
    id = id > 3 ? 3 : id;
    return id;
}

static int parse_motor_step(char* str){
    int steps=0;
    if(isalpha(str[0]))
        steps=atoi(str+1);  //skip axis symbol
    else
        steps=atoi(str);
    return steps;
}

static void op_help(int argc, char** argv){
    uart_printf("available commands:\n\r");
    for(int opid = 0; opid<ARRAY_ITEM_COUNT(opTable); opid++){
        uart_printf("  - %s\n\r", opTable[opid].cmdstr);
    }
}

static void op_getVersion(int argc, char** argv){
    uart_printf("%s\n", VERSION); 
}

static void op_wait(int argc, char** argv){
    uint32_t id = 0;
    char wait_for[3]={FALSE};
    
    for(uint32_t i=1; i<argc; i++){
        id = parse_motor_id(argv[i]);
        if(id <= 2) wait_for[id] = TRUE;
    }
    
    while(1){
        char busy=FALSE;
        for(int i=0; i<3; i++){
            if(wait_for[i])
                busy = busy || *(get_motor_ref(i)->busyPin.datAddr);
        }
        if(!busy) break;
    }
    
    ack();
}

static void op_stepMoveHome(int argc, char** argv){
    uint32_t id = 0;
    char success = TRUE;
    for(uint32_t i=1; i<argc; i++){
        id = parse_motor_id(argv[i]);
        if(id <= 2)
            success = motor_home(id) && success;
    }
    success ? ack() : nak();
}

static void op_stepMoveHome3(int argc, char** argv){
    motor_home3() ? ack() : nak();
}

static void op_stepMove(int argc, char** argv){
    uint32_t id = 0;
    int32_t steps = 0;
    char success = TRUE;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > 2) continue;
        
        //parse step
        if(strlen(argv[i])==1 && i < argc-1)    // movr x -500
            steps = parse_motor_step(argv[++i]);
        else                                    // movr x-500
            steps = parse_motor_step(argv[i]);
        
        //move
        if(strchr(argv[0], 'r'))            //movr
            success = motor_move_related(id, steps) && success;
        else if(strchr(argv[0], 'a'))       //mova
            success = motor_move_abspos(id, steps) && success;
    }
    
    success ? ack() : nak();
}

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

static void op_getPosition(int argc, char** argv){
    uint32_t id = 0;
    int32_t steps = 0;
    char str[128]={0};
    char success = TRUE;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > 2) continue;
        snprintf(str, 128, "%s%d ", str, get_motor_ref(id)->position);
    }
    uart_printf("%s\r\n", str);
    ack();
}

static void op_setPosition(int argc, char** argv){
    uint32_t id = 0;
    int32_t steps = 0;
    char success = TRUE;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > 2) continue;
        
        //parse step
        if(strlen(argv[i])==1 && i < argc-1)    // movr x -500
            steps = parse_motor_step(argv[++i]);
        else                                    // movr x-500
            steps = parse_motor_step(argv[i]);
        
        get_motor_ref(id)->position=steps;
    }
    ack();
}

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

static void op_setSpeed(int argc, char** argv){
    uint32_t id = 3;
    SpeedTag speed_tag = NOT_SPEED_TAG;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        if(parse_motor_id(argv[i])<3){
            id = parse_motor_id(argv[i]);
        }
        //motor id not set
        if(id > 2) continue;
        
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

static void op_getSpeed(int argc, char** argv){
    #define STRING_SIZE (512)
    uint32_t id = 0;
    char str[STRING_SIZE]={0};
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > 2) continue;
        switch(id){
            case 0: snprintf(str, STRING_SIZE, "%sx: ", str); break;
            case 1: snprintf(str, STRING_SIZE, "%sy: ", str); break;
            case 2: snprintf(str, STRING_SIZE, "%sz: ", str); break;
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

static void op_saveSpeed(int argc, char** argv){
    uint32_t id = 0;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > 2) continue;
        copySpeedToUserConfig(id);
    }
    (saveUserConfigToFlash() ? ack() : nak());
}
    
static void op_setDirection(int argc, char** argv){
    uint32_t id = 0;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > 2) continue;
        
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

static void op_getDirection(int argc, char** argv){
    uint32_t id = 0;
    char str[128]={0};
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > 2) continue;
        
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

static void op_saveDirection(int argc, char** argv){
    uint32_t id = 0;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > 2) continue;
        copyDirectionConfigToUserConfig(id);
    }
    (saveUserConfigToFlash() ? ack() : nak());
}

static void op_setLimitSwitch(int argc, char** argv){
    uint32_t id = 0;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > 2) continue;
        
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

static void op_getLimitSwitch(int argc, char** argv){
    uint32_t id = 0;
    char str[128]={0};
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > 2) continue;
        
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

static void op_saveLimitSwitch(int argc, char** argv){
    uint32_t id = 0;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > 2) continue;
        copyPinConfigToUserConfig(id);
    }
    (saveUserConfigToFlash() ? ack() : nak());
}

static void op_setTorque(int argc, char** argv){
    uint32_t id = 0;
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > 2) continue;
        
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

static void op_getTorque(int argc, char** argv){
    uint32_t id = 0;
    char str[128]={0};
    for(uint32_t i=1; i<argc; i++){
        // parse motor id
        id = parse_motor_id(argv[i]);
        if(id > 2) continue;
        snprintf(str, 128, "%s%d ",
                 str, parse_current_setting(get_motor_ref(id)->activeI0State,
                                            get_motor_ref(id)->activeI1State));
    }
    uart_printf("%s\n\r", str);
    ack();
}

static void op_saveTorque(int argc, char** argv){
    for(int id=0; id<3; id++){
        torqueConfig[id].active_torque=parse_current_setting(
                                            get_motor_ref(id)->activeI0State,
                                            get_motor_ref(id)->activeI1State);
    }
    saveTorqueConfigToFlash() ? ack() : nak();
}

static void op_eraseTorque(int argc, char** argv){
    eraseTorqueConfigFromFlash() ? ack() : nak();
}

static void op_saveConfig(int argc, char** argv){
    for(int i=0; i<3; i++){
        copySpeedToUserConfig(i);
        copyDirectionConfigToUserConfig(i);
        copyPinConfigToUserConfig(i);
    }
    (saveUserConfigToFlash() ? ack() : nak());
}

static void op_eraseConfig(int argc, char** argv){
    eraseUserConfigFromFlash() ? ack() : nak();
}

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

static void op_saveBorder(int argc, char** argv)
{
    saveNoTrespassBordersToFlash() ? ack() : nak();
}

static void op_eraseBorder(int argc, char** argv)
{
    eraseNoTrespassBordersFromFlash() ? ack() : nak();
}

void cmd_task(){
    if (uartRxBuf.line_counts > 0){
        RingBufferU8_readLine(&uartRxBuf, (uint8_t*)cmd_buf, sizeof(cmd_buf));
        process_cmd(cmd_buf, sizeof(cmd_buf));
        // todo: if the uart_inbuf used by Rx is wrapped around, this loop runs 
        //       forever. We wight need to fix this at some point.
    } else
        __NOP();
}

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

void start_tx(){
    if (!isTxBusy){
        isTxBusy = TRUE;
        //uint8_t b = RingBufferU8_readByte(&uartTxBuf);
        //UART0->DAT = (uint32_t)b;  
        UART_ENABLE_INT(UART0, UART_INTEN_TXENDIEN_Msk);
    }
}

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

static inline void nak(){
    uart_printf("nak\r\n");
}

static inline void ack(){
    uart_printf("ack\r\n");
}
;