/**
 * @file motor_ctrl.h
 * @ingroup motor_controller
 * @brief Declarations for motor control state enums and motor configuration data.
 */
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

#define MOTOR_COUNT     4

#define MOTORID_1       0
#define MOTORID_2       1
#define MOTORID_3       2
#define MOTORID_4       3
#define MOTORADDR_1     (1<<MOTORID_1)
#define MOTORADDR_2     (1<<MOTORID_2)
#define MOTORADDR_3     (1<<MOTORID_3)
#define MOTORADDR_4     (1<<MOTORID_4)

#define PARK_AFTER_HOME 0

//#define DEBUG_PIN  PA3

/**
 * @enum CURRENT_SETTING
 * @brief Enumeration of motor current settings (0% to 100%).
 */
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

/**
 * @enum direction_config
 * @brief Direction configuration options for motor movement.
 */
typedef enum {
    CFG_DIR_DEFAULT,
    CFG_DIR_REVERSE
}direction_config;

/**
 * @enum limit_switch_config
 * @brief Limit switch configuration based on hardware wiring.
 */
typedef enum {
    CFG_LIMSW_DEFAULT,
    CFG_LIMSW_REVERSE
}limit_switch_config;

/**
 * @enum IO_STATE
 * @brief Represents the state of a digital I/O pin.
 */
typedef enum{
    io_high = 1,
    io_low = 0,
    io_hiz = -1
}IO_STATE;

/**
 * @enum MOTOR_MOVE
 * @brief Defines the type of motor movement command.
 */
typedef enum{
    motor_move_home,
    motor_move_fwd,
    motor_move_bwd
}MOTOR_MOVE;

/**
 * @enum MOTOR_STATE
 * @brief Represents the current state of the motor. @c poll_Motor()
 */
typedef enum{
    Motor_Idle,
    //Motor_Init,
    Motor_InitMove,
    Motor_move,
    MOTOR_HOME,
#if PARK_AFTER_HOME
    MOTOR_PARK,
#endif
    Motor_moveDone,
    //Motor_Error,

}MOTOR_STATE;

/**
 * @enum MOTOR_ACCEL_STATE
 * @brief Represents the acceleration phase of the motor during movement.
 */
typedef enum{
    motor_move_stop = 0,
    motor_move_accel,
    motor_move_decel,
    motor_move_const_speed,
    motor_move_noAccel
}MOTOR_ACCEL_STATE;

/**
 * @struct MOTOR_DATA
 * @brief Configuration and runtime data of a motor.
 *
 * @details This structure holds all the configuration parameters, GPIO definitions,
 *          runtime states, and acceleration profile needed to control a single motor.
 */
typedef struct{
    uint32_t        id;                             /**< motor id */
    //pin defines
    GPIO_TYPEDef    sleepPin;                       /**< motor sleep pin */
    GPIO_TYPEDef    dirPin;                         /**< motor dir pin */
    GPIO_TYPEDef    stepPin;                        /**< motor step pin */
    GPIO_TYPEDef    i0Pin;                          /**< motor i0 pin */
    GPIO_TYPEDef    i1Pin;                          /**< motor i1 pin */
    GPIO_TYPEDef    highEdgePin;                    /**< High edge (left) limit switch pin */
    GPIO_TYPEDef    lowEdgePin;                     /**< Low edge (right) limit switch pin */
    GPIO_TYPEDef    busyPin;                        /**< motor busy pin */

    //setting
    uint32_t        fwdDirState;                    /**< motor direction state */
    IO_STATE        stdbyI0State;                   /**< motor i0 pin standby state */
    IO_STATE        stdbyI1State;                   /**< motor i1 pin standby state */
    IO_STATE        activeI0State;                  /**< motor i0 pin active state */
    IO_STATE        activeI1State;                  /**< motor i1 pin active state */
    uint32_t        maxHomeStep;                    /**< max steps for home */
    //timer
    TIMER_T *       timer;


    //status
    volatile MOTOR_STATE     state;
    volatile MOTOR_MOVE      moveType;
    volatile int32_t         position;              /**< position current */
    volatile uint32_t        movingSteps;
    int32_t         targetPos;                      /**< position of moving to */

    //accel cal
    volatile MOTOR_ACCEL_STATE   accelState;
    uint32_t        defaultPPS;                     /**< default toggle speed in pps */
    bool            enableMototAccelerate;          /**< should we enable accel */
    uint32_t        targetPPS;                      /**< motor max speed in pps */
    uint32_t        acceleration;                   /**< accel in PPS/sec^2 */
    uint32_t        deceleration;                   /**< decel in PPS/sec^2 */

    uint32_t        decelStartStep;                 /**< the moving step for decel to start */
    uint32_t        minDelay;                       /**< timer delay cnt of max speed */
    int32_t         decelStepValue;                 /**< step counts of deceleration */
    volatile uint32_t        stepDelay;             /**< step_delay = (T1_FREQ_148 * math.sqrt(A_SQ/accel))/100 */
    volatile int32_t         rest;
    volatile int32_t         accelStepCnt;          /**< step counter for accel/decel */
    
    volatile uint32_t        debounceCounter;       /**< edge debounce counter */
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
bool motor_gate_home();
bool motor_move_gate_related(int32_t steps);
bool motor_move_gate_abspos(int32_t position);
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