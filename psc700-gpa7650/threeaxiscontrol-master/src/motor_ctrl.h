#ifndef _MOTOR_CTRL_    /* Guard against multiple inclusion */
#define _MOTOR_CTRL_
#include "NuMicro.h"
#include "stdbool.h"
#include "gpio_type_def.h"

#define HIGH        (1UL)                ///< Boolean true, define to use in API parameters or return value
#define LOW         (0UL)  

//#define motor_gpio_setMode(gpio, mode)  (GPIO_SetMode((gpio).port, (gpio).pin, (mode)))

#define GPIO_PIN_DATA_ADDR(port, pin)    ((uint32_t *)((GPIO_PIN_DATA_BASE+(0x40*(port))) + ((pin)<<2)))
#define portA   0
#define portB   1
#define portC   2
#define portD   3
#define portE   4
#define portF   5

//#define DEBUG_PIN  PA3

typedef enum{
    CURRENT_0_PERCENT=0,
    CURRENT_12_PERCENT,
    CURRENT_25_PERCENT,
    CURRENT_37_PERCENT,
    CURRENT_50_PERCENT,
    CURRENT_62_PERCENT,
    CURRENT_75_PERCENT,
    CURRENT_87_PERCENT,
    CURRENT_100_PERCENT
}CURRENT_SETTING;

typedef enum {
    CFG_DIR_DEFAULT,
    CFG_DIR_REVERSE
}direction_config;

typedef enum {
    CFG_LIMSW_DEFAULT,
    CFG_LIMSW_REVERSE
}limit_switch_config;

typedef enum{
    io_high = 1,
    io_low = 0,
    io_hiz = -1
}IO_STATE;

typedef enum{
    motor_move_home,
    motor_move_fwd,
    motor_move_bwd
}MOTOR_MOVE;

typedef enum{
    Motor_Idle,
    //Motor_Init,
    Motor_InitMove,
    Motor_move,
    Motor_homeBWD,
    Motor_homeDBC,
    Motor_homeFWD,
    Motor_moveDone,
    //Motor_Error,

}MOTOR_STATE;

typedef enum{
    motor_move_stop = 0,
    motor_move_accel,
    motor_move_decel,
    motor_move_const_speed,
    motor_move_noAccel
}MOTOR_ACCEL_STATE;

typedef struct{
    uint32_t        id;
    //pin defines
    GPIO_TYPEDef    sleepPin;
    GPIO_TYPEDef    dirPin;
    GPIO_TYPEDef    stepPin;
    GPIO_TYPEDef    i0Pin;
    GPIO_TYPEDef    i1Pin;
    GPIO_TYPEDef    highEdgePin; //left
    GPIO_TYPEDef    lowEdgePin;  //right
    GPIO_TYPEDef    busyPin;

    //setting
    uint32_t        fwdDirState;
    IO_STATE        stdbyI0State;
    IO_STATE        stdbyI1State;
    IO_STATE        activeI0State;
    IO_STATE        activeI1State;
    uint32_t        maxHomeStep;    //max steps for home
    //timer
    TIMER_T *       timer;


    //status
    volatile MOTOR_STATE     state;
    volatile MOTOR_MOVE      moveType;
    volatile int32_t         position;       //position current
    volatile uint32_t        movingSteps;
    int32_t         targetPos;     //position of moving to

    //accel cal
    volatile MOTOR_ACCEL_STATE   accelState;
    uint32_t        defaultPPS;     //default toggle speed in pps
    bool            enableMototAccelerate;  //should we enable accel
    uint32_t        targetPPS;      //motor max speed in pps
    uint32_t        acceleration;   //accel in PPS/sec^2
    uint32_t        deceleration;   //decel in PPS/sec^2

    uint32_t        decelStartStep; //the moving step for decel to start    
    uint32_t        minDelay;       //timer delay cnt of max speed
    int32_t         decelStepValue; //step counts of deceleration
    volatile uint32_t        stepDelay;      //#step_delay = (T1_FREQ_148 * math.sqrt(A_SQ/accel))/100
    volatile int32_t         rest;
    volatile int32_t         accelStepCnt;   //step counter for accel/decel
    
    volatile uint32_t        debounceCounter;    //edge debounce counter
}MOTOR_DATA;

void motor_init();
void motor_enable(uint32_t motorId);
void motor_disable(uint32_t motorId);
MOTOR_DATA * get_motor_ref(uint32_t motorId);
bool motor_resetPos(uint32_t motorId);

bool motor_home(uint32_t motorId);
bool motor_home3();
bool motor_move_related(uint32_t motorId, int32_t steps);
bool motor_move_abspos(uint32_t motorId, int32_t position);
bool motor_move_related3(int32_t step0, int32_t step1, int32_t step2);
bool motor_move_abspos3(int32_t pos0, int32_t pos1, int32_t pos2);
bool motor_configSpeed(uint32_t motorId, uint32_t defPPS, int32_t enableAccel, uint32_t maxPPS, uint32_t acceleration, uint32_t deceleration);
bool motor_configDriver(uint32_t motorId, uint32_t forwardDir, IO_STATE standbyi0, IO_STATE standbyi1, IO_STATE activei0, IO_STATE activei1);

void select_current(CURRENT_SETTING current, IO_STATE *i0, IO_STATE *i1);

void setLimitSwitch(uint32_t motor_id, limit_switch_config lsw_cfg);
limit_switch_config getLimitSwitch(uint32_t motor_id);

void setDirection(uint32_t motor_id, direction_config dir_cfg);
direction_config getDirection(uint32_t motor_id);

CURRENT_SETTING parse_current_setting(IO_STATE i0, IO_STATE i1);

char checkTrespass(int* s0, int* s1);

extern uint8_t trespass_log;
#endif