/**
 * @file motor_ctrl.c
 * @ingroup motor_controller
 * @brief Implementation of motor control functions.
 */
#include "uart_cmd.h"
#include "motor_ctrl.h"
#include "user_config.h"
#include <math.h>

static MOTOR_DATA motors[MOTOR_COUNT];
static GPIO_TYPEDef busy;   //motor busy status
uint8_t trespass_log=0;

static void init_Motor0();
static void init_Motor1();
static void init_Motor2();
static void init_Motor_Gate();

static void init_Motor_Gpio(MOTOR_DATA * motor);
static void start_Motor(MOTOR_DATA * motor);
static void set_Motor_Accel_Timer(MOTOR_DATA * motor);
static void init_Motor_Accel(MOTOR_DATA * motor, uint32_t stepsToMove);
static void motor_gpio_setMode(GPIO_TYPEDef * gpio, uint32_t mode);
static inline void set_dac_io(GPIO_TYPEDef * gpio, IO_STATE io);

#define T1_FREQ 1000000.0f //Hz, timer base is 1us
#define DebugPrint		0

/**
 * @brief Initial all MOTOR_DATA
 * @param None
 * @return None
 */
void motor_init(){
    busy.port = PA;
    busy.pin = BIT3;
    busy.datAddr = GPIO_PIN_DATA_ADDR(portA, 3);
    motor_gpio_setMode(&busy, GPIO_MODE_OUTPUT);
    init_Motor0();
    init_Motor1();
    init_Motor2();
    init_Motor_Gate();
}

/**
 * @brief  Returns a pointer to the MOTOR_DATA structure of the specified motor.
 *
 * @details This function retrieves a reference to the motor data corresponding to the given motor ID.
 *          It allows access to the motor's status or control parameters.
 *
 * @param  motorId The ID (index) of the motor in the motors array.
 * @return MOTOR_DATA* Pointer to the MOTOR_DATA structure of the specified motor.
 */
MOTOR_DATA * get_motor_ref(uint32_t motorId){
    return &motors[motorId];
}

/**
 * @brief  Resets the position of the specified motor to zero.
 *
 * @details This function checks if the motor is busy. If it is not busy, it resets the motor's
 *          position value to zero. If the motor is busy, the function returns false.
 *
 * @param  motorId The ID (index) of the motor in the motors array.
 * @retval true  The position was successfully reset.
 * @retval false The motor is currently busy.
 */
bool motor_resetPos(uint32_t motorId){
    //busy
    if (*(motors[motorId].busyPin.datAddr)) return false;
    motors[motorId].position = 0;
    return true;
}

/**
 * @brief  Configures speed-related parameters for the specified motor.
 *
 * @details This function sets the default speed (PPS), and optionally enables acceleration
 *          and deceleration profiles. If acceleration is enabled (`enableAccel >= 0`),
 *          it updates the maximum speed, acceleration, and deceleration values.
 *
 * @param  motorId      The ID of the motor to configure.
 * @param  defPPS       Default pulses per second (speed).
 * @param  enableAccel  Acceleration enable flag. >0 to enable, <0 to skip updating speed settings.
 * @param  maxPPS       Target maximum speed in PPS (only if acceleration is enabled).
 * @param  acceleration Acceleration rate (only if acceleration is enabled).
 * @param  deceleration Deceleration rate (only if acceleration is enabled).
 * @return true if the configuration was successful; false if the motor is busy.
 */
bool motor_configSpeed(uint32_t motorId, uint32_t defPPS, int32_t enableAccel, 
                            uint32_t maxPPS, uint32_t acceleration, uint32_t deceleration){
    MOTOR_DATA * motor = &motors[motorId];
    //busy
    if (*(motor->busyPin.datAddr)) return false;

    motor->defaultPPS = defPPS;

    motor->enableMototAccelerate = enableAccel > 0;

		//don't set value when enableAccel < 0
		if (enableAccel >= 0){
			motor->targetPPS = maxPPS;    
			motor->acceleration = acceleration;
			motor->deceleration = deceleration; 		
		}
       
#if DebugPrint
		printf("motor=%d, defPPS=%d, enableAccel=%d, targetPPS=%d, accel=%d, decel=%d\n", 
				motorId, defPPS, enableAccel, maxPPS, acceleration, deceleration);
#endif
    return true;
}

/**
 * @brief  Configures motor driver states for direction and standby/active pins.
 *
 * @details This function sets the forward direction logic level, and the I/O states for
 *          standby and active control pins (I0 and I1). These settings define how the driver
 *          is controlled in different modes.
 *
 * @param  motorId     The ID of the motor to configure.
 * @param  forwardDir  Logic level for forward direction.
 * @param  standbyi0   I/O state for standby mode pin I0.
 * @param  standbyi1   I/O state for standby mode pin I1.
 * @param  activei0    I/O state for active mode pin I0.
 * @param  activei1    I/O state for active mode pin I1.
 * @retval true  The configuration was successfully .
 * @retval false The motor is currently busy.
 */
bool motor_configDriver(uint32_t motorId, uint32_t forwardDir, IO_STATE standbyi0, IO_STATE standbyi1, IO_STATE activei0, IO_STATE activei1){
    MOTOR_DATA * motor = &motors[motorId];
    //busy
    if (*(motor->busyPin.datAddr)) return false;

    motor->fwdDirState = forwardDir;
    motor->stdbyI0State = standbyi0;
    motor->stdbyI1State = standbyi1;
    motor->activeI0State = activei0;
    motor->activeI1State = activei1;

#if DebugPrint
		printf("motor=%d, forward=%d, standbyi0=%d, standbyi1=%d, activei0=%d, activei1=%d\n", 
				motorId, forwardDir, standbyi0, standbyi1, activei0, activei1);
#endif
    return true;    
}

/**
 * @brief  Initiates homing movement for the specified motor.
 *
 * @details This function sets the motor's movement type to `home`, and then starts the homing
 *          operation. Homing typically moves the motor toward a reference position (e.g., limit switch).
 *
 * @param  motorId The ID of the motor to perform homing.
 * @retval true  The homing started successfully .
 * @retval false The motor is currently busy.
 */
bool motor_home(uint32_t motorId){
    MOTOR_DATA * motor = &motors[motorId];

    //busy
    if (*(motor->busyPin.datAddr)) return false;

    motor->moveType = motor_move_home;
    start_Motor(motor);
    return true;
}

/**
 * @brief Initiates homing movement for motors 0 to 2.
 *
 * @details This function calls @c motor_home() for motor IDs 0, 1, and 2.
 *          If any of the motors is currently busy, the operation is aborted.
 *
 * @retval true  Homing started successfully for all three motors.
 * @retval false At least one motor is busy; homing not started.
 */
bool motor_home3(){
    if (*(motors[0].busyPin.datAddr) ||
        *(motors[1].busyPin.datAddr) ||
        *(motors[2].busyPin.datAddr))
        return false;

    motor_home(0);
    motor_home(1);
    motor_home(2);
    return true;
}

/**
 * @brief Move the specified motor by a relative number of steps.
 *
 * @details This function moves the motor by the specified number of steps relative to its current position.
 *
 * @param motorId The ID of the motor.
 * @param steps Number of steps to move (positive for forward, negative for backward).
 *
 * @retval true  Motor move started successfully.
 * @retval false Motor is busy and cannot start a new move.
 */
bool motor_move_related(uint32_t motorId, int32_t steps){

    MOTOR_DATA * motor = &motors[motorId];

    //busy
    if (*(motor->busyPin.datAddr)) return false;


    if (steps > 0) motor->moveType = motor_move_fwd;
    else if (steps < 0) motor->moveType = motor_move_bwd;
    else return false; //do nothing
#if DebugPrint
		printf("Move rel %d %d\n", motorId, steps);
#endif
    motor->targetPos = motor->position + steps;
    start_Motor(motor);
    return true;
}

/**
 * @brief Move the specified motor to an absolute position.
 *
 * @details This function moves the motor to the specified absolute position 
 *          (relative to its home/origin). If the motor is currently busy, 
 *          the move will not be initiated.
 *
 * @param motorId   The ID of the motor.
 * @param position  The target absolute position to move to.
 *
 * @retval true     Motor move started successfully.
 * @retval false    Motor is busy and cannot start a new move.
 */
bool motor_move_abspos(uint32_t motorId, int32_t position){

    MOTOR_DATA * motor = &motors[motorId];

    //busy
    if (*(motor->busyPin.datAddr)) return false;

    if (motor->position > position) motor->moveType = motor_move_bwd;
    else if (motor->position < position) motor->moveType = motor_move_fwd;
    else return false;
#if DebugPrint
		printf("Move abs %d %d\n", motorId, position);
#endif
    motor->targetPos = position;
    start_Motor(motor);
    return true;
}

/**
 * @brief Move motors 0 to 2 by a relative number of steps.
 *
 * @details This function calls @c motor_move_related() for motor IDs 0, 1, and 2.
 *          If any of the motors is currently busy, the operation is aborted.
 * @param step0 Number of steps for motor 0.
 * @param step1 Number of steps for motor 1.
 * @param step2 Number of steps for motor 2.
 * @retval true  Motor move started successfully.
 * @retval false At least one motor is busy.
 */
bool motor_move_related3(int32_t step0, int32_t step1, int32_t step2){
    if (*(motors[0].busyPin.datAddr) ||
        *(motors[1].busyPin.datAddr) ||
        *(motors[2].busyPin.datAddr))
        return false;
    
    int s0[3]={motors[0].position, motors[1].position, motors[2].position},
        s1[3]={step0, step1, step2};
    checkTrespass(s0, s1);
    
    motor_move_related(0, s1[0]);
    motor_move_related(1, s1[1]);
    motor_move_related(2, s1[2]);
    return true;
}

/**
 * @brief Move motors 0 to 2 to an absolute position.
 *
 * @details This function calls @c motor_move_abspos() for motor IDs 0, 1, and 2.
 *          If any of the motors is currently busy, the operation is aborted.
 * @param pos0 Position for motor 0.
 * @param pos1 Position for motor 1.
 * @param pos2 Position for motor 2.
 * @retval true  Motor move started successfully.
 * @retval false At least one motor is busy.
 */
bool motor_move_abspos3(int32_t pos0, int32_t pos1, int32_t pos2){
    if (*(motors[0].busyPin.datAddr) ||
        *(motors[1].busyPin.datAddr) ||
        *(motors[2].busyPin.datAddr))
        return false;
    
    int s0[3]={motors[0].position, motors[1].position, motors[2].position},
        s1[3]={pos0-s0[0], pos1-s0[1], pos2-s0[2]};
    checkTrespass(s0, s1);
    
    motor_move_abspos(0, s0[0]+s1[0]);
    motor_move_abspos(1, s0[1]+s1[1]);
    motor_move_abspos(2, s0[2]+s1[2]); 
    return true;  
}

/**
 * @brief Start homing procedure for the Gate motor (motor 4).
 * @param None
 * @retval true  Homing started successfully.
 * @retval false Motor is currently busy.
 */
bool motor_gate_home(){
    MOTOR_DATA * motor = &motors[MOTORID_4];

    //busy
    if (*(motor->busyPin.datAddr)) return false;

    motor->moveType = motor_move_home;
    start_Motor(motor);
    return true;
}
/**
 * @brief Move the Gate motor by a relative number of steps.
 * @param steps Number of steps to move (positive for forward, negative for backward).
 * @retval true  Motor move started successfully.
 * @retval false Motor is busy and cannot start a new move.
 */
bool motor_move_gate_related(int32_t steps){
    MOTOR_DATA * motor = &motors[MOTORID_4];

    //busy
    if (*(motor->busyPin.datAddr)) return false;

    if (steps > 0) motor->moveType = motor_move_fwd;
    else if (steps < 0) motor->moveType = motor_move_bwd;
    else return false; //do nothing
#if DebugPrint
		printf("Move g rel %d\n", steps);
#endif
    motor->targetPos = motor->position + steps;
    start_Motor(motor);
    return true;
}

/**
 * @brief Move the Gate motor to an absolute position.
 * @param position  The target absolute position to move to.
 * @retval true     Motor move started successfully.
 * @retval false    Motor is busy and cannot start a new move.
 */
bool motor_move_gate_abspos(int32_t position){
    MOTOR_DATA * motor = &motors[MOTORID_4];

    //busy
    if (*(motor->busyPin.datAddr)) return false;

    if (motor->position > position) motor->moveType = motor_move_bwd;
    else if (motor->position < position) motor->moveType = motor_move_fwd;
    else return false;
#if DebugPrint
		printf("Move g abs %d\n", position);
#endif
    motor->targetPos = position;
    start_Motor(motor);
    return true;
}

static inline void set_timer_tick(MOTOR_DATA * motor){
#define pre_scale   1000 //ns
    //motor->timer->CMP = 1000000000UL/motor->stepDelay/pre_scale/2;
    motor->timer->CMP = motor->stepDelay/2;
    //printf("%d\n", motor->stepDelay);
}

static void init_Motor_Accel(MOTOR_DATA * motor, uint32_t stepsToMove){
    int32_t  max_step_limit, accel_limit, decel_value;

    if ((!motor->enableMototAccelerate) || (stepsToMove < 5)){
        motor->accelState = motor_move_noAccel;
        motor->stepDelay = T1_FREQ/motor->defaultPPS;
        return;
    }

    //timer delay at max speed
    motor->minDelay = T1_FREQ/motor->targetPPS;
    
    //find out after how many steps does the speed reaches the max speed limit
    //max_step_limit = speed^2/(2*alpha*accel)
    max_step_limit = (motor->targetPPS * motor->targetPPS)/(2*motor->acceleration);
    
    //need to mave at least one step
    if (max_step_limit == 0) max_step_limit = 1;

    //find out after how many steps we must start deceleration
    //n1 = (n1+n2)decel / (accel + decel)
    accel_limit = ((int64_t)stepsToMove * motor->deceleration)/(motor->acceleration + motor->deceleration);
    if (accel_limit == 0) accel_limit = 1;
	
    //find the deceleration steps
    if (accel_limit < max_step_limit)
        decel_value = accel_limit - stepsToMove;
    else
        decel_value = (-1) *(((uint64_t)max_step_limit * motor->acceleration)/motor->deceleration);		

    if (decel_value == 0) decel_value = -1;
    motor->decelStartStep = stepsToMove+decel_value;

    double stepDelay;
		
    stepDelay = (T1_FREQ * 0.676 * sqrt(2.0f/motor->acceleration));
    motor->decelStepValue = decel_value;
    if (stepDelay <= motor->minDelay){
        motor->stepDelay = motor->minDelay;
        motor->accelState = motor_move_const_speed;
    }
    else{
        motor->stepDelay = stepDelay;
        motor->accelState = motor_move_accel;
    }
    motor->rest = 0;
    motor->accelStepCnt = 0;
    
#if DebugPrint
		printf("stepsToMove=%d max_step_limit=%d accel_limit=%d decel_value=%d\n", stepsToMove, max_step_limit, accel_limit, decel_value);
#endif
}

/* 
 - rearrange formula to reduce calculation steps.
 - replace sqrt() with fast inverse square root algorithm
*/

// This is THE legendary fast inverse square root algorithm form Quake III source code.
static inline float fast_inv_sqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y  = number;
    i  = * ( long * ) &y;                       // evil floating point bit level hacking
    i  = 0x5f3759df - ( i >> 1 );               // what the fuck? 
    y  = * ( float * ) &i;
    y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
  //y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

    return y;
}
// 0x5f3759df, it's MAGIC!

float truncateRatioToPlane(int *s0, int* s1, float a, float b, float c, float d){
    /*
     *            s0
     *            *        -             -
     *           / \       |             |
     *   (p0-s0)/   \sl    |(p0-s0).pn   |(sl.pn)
     *         V     \     |             |
     * --+----*-------\------------------|-------plane
     *   |    p0       \                 |
     *   V              V                -
     *   pn              * destination
     */
    float l=fast_inv_sqrt(a*a+b*b+c*c);
    float pn[3] = {a*l, b*l, c*l};
    
    //generate a point on the plane
    float p0[3] = {0, 0, 0};
    if     (c!=0)
        p0[2] = (d - a*p0[0] - b*p0[1]) / c;
    else if(b!=0)
        p0[1] = (d - a*p0[0] - c*p0[2]) / b;
    else if(a!=0)
        p0[0] = (d - b*p0[1] - c*p0[2]) / a;
    
    //calc inner products
    float p0_ps_pn=0;
    float sl_pn=0;
    for(int i=0; i<3; i++){
        p0_ps_pn+=(p0[i]-s0[i])*pn[i];
        sl_pn+=s1[i]*pn[i];
    }
    
    //truncate
    return p0_ps_pn/sl_pn;
}

//s0: (segment0) start position
//s1: (segment1) step vector
char checkTrespass(int* s0, int* s1){
    char ret = 0b00000000;
    float ratio=1.0;
    for(int i=0; i<8; i++){
        if(noTrespassBorder[i].enable)
        {
            int pos[3]={s0[0]+s1[0], s0[1]+s1[1], s0[2]+s1[2]};
            float n = noTrespassBorder[i].a*(float)pos[0]
                    + noTrespassBorder[i].b*(float)pos[1]
                    + noTrespassBorder[i].c*(float)pos[2]
                    - noTrespassBorder[i].d;
            if((n>0 &&  noTrespassBorder[i].greater_than)
            || (n<0 && !noTrespassBorder[i].greater_than))
            {
                float r=truncateRatioToPlane(s0, s1, noTrespassBorder[i].a,
                                                     noTrespassBorder[i].b,
                                                     noTrespassBorder[i].c,
                                                     noTrespassBorder[i].d);
                if(ratio > r) ratio = r;
                ret|=0b1<<i;
            }
        }
    }
    for(int i=0; i<3; i++) s1[i]*=ratio;
    return ret;
}

static inline uint32_t calc_Motor_Accel(MOTOR_DATA * motor){
    uint32_t new_delay, temp;
    float r;
	
	r = (fast_inv_sqrt((float)motor->accelStepCnt*(motor->acceleration<<1)))*T1_FREQ;
    temp = r;
	
	new_delay = (motor->accelState == motor_move_const_speed) ? motor->stepDelay : temp;

    return new_delay;
}

static void set_Motor_Accel_Timer(MOTOR_DATA * motor){
    uint32_t new_step_delay = 0;
#if DebugPrint
    //printf("%d %d %d %d %d\n", motor->movingSteps, motor->accelState, motor->accelStepCnt, motor->stepDelay, motor->rest);
#endif	
    switch (motor->accelState){
        case motor_move_accel:{
            motor->accelStepCnt++;
            new_step_delay = calc_Motor_Accel(motor);
            if (motor->accelStepCnt >= motor->decelStartStep){
                motor->accelStepCnt = abs(motor->decelStepValue);
                motor->accelState = motor_move_decel;
            }
            else if (new_step_delay <= motor->minDelay){
                new_step_delay = motor->minDelay;
                motor->rest = 0;
                motor->accelState = motor_move_const_speed;
            }
        }
        break;

        case motor_move_const_speed:{
            new_step_delay = motor->minDelay;
            if (motor->movingSteps >= motor->decelStartStep){
                motor->accelStepCnt = abs(motor->decelStepValue);
                motor->accelState = motor_move_decel;
            }
        }
        break;

        case motor_move_decel:{
			if (motor->accelStepCnt <= 0)  motor->accelState = motor_move_stop;
			new_step_delay = calc_Motor_Accel(motor);
			motor->accelStepCnt--;
        }
        break;

        case motor_move_noAccel:{
            new_step_delay = motor->stepDelay;  //no change
        }
        break;

        case motor_move_stop:
        //do nothing, make complier happy
        break;
    }
    motor->stepDelay = new_step_delay;
}

static void start_Motor(MOTOR_DATA * motor){

    uint32_t stepsToMove;

    motor->state = Motor_InitMove;
    //start timer for next poll    
    
    if (motor->moveType == motor_move_home)
        stepsToMove = motor->maxHomeStep;
    else
        stepsToMove = abs(motor->position - motor->targetPos);
    
    init_Motor_Accel(motor, stepsToMove);
    set_timer_tick(motor);
    TIMER_Start(motor->timer);
}

/**
 * @brief Poll the status and control logic of a motor.
 *
 * @details This internal static function is called periodically (e.g., by a timer interrupt)
 *          to check and update the motor's status, position, and control logic.
 *
 * @param motor Pointer to the MOTOR_DATA structure representing the motor to poll.
 */
static void poll_Motor(MOTOR_DATA * motor){
    
    set_timer_tick(motor);
    if (motor->state != Motor_Idle){     
        //start timer for next poll
        TIMER_Start(motor->timer);
    }
    
    switch (motor->state){
        case Motor_InitMove:
        {
            //set busy pin
            *(busy.datAddr) = HIGH;
            //set I0/I1 state
            set_dac_io(&motor->i0Pin, motor->activeI0State);
            set_dac_io(&motor->i1Pin, motor->activeI1State); 
            *(motor->busyPin.datAddr) = HIGH;

            //set dir state
            if (motor->moveType == motor_move_fwd){
                *(motor->dirPin.datAddr) = motor->fwdDirState;
                motor->state = Motor_move;
            }
            else if (motor->moveType == motor_move_bwd){
                *(motor->dirPin.datAddr) = !motor->fwdDirState;
                motor->state = Motor_move;                
            }
            else{   //home
#if PARK_AFTER_HOME
                if(*(motor->lowEdgePin.datAddr)){
                    *(motor->dirPin.datAddr) = !motor->fwdDirState;
                    motor->state = MOTOR_HOME;
                }else{
                    // skip moving if it's already home
                    motor->state = Motor_moveDone;
                    motor->position = 0;
                }
#else
                *(motor->dirPin.datAddr) = !motor->fwdDirState;
                motor->state = MOTOR_HOME;
#endif
            }
            motor->movingSteps = 0;
        }
        break;
        case Motor_move :{
            if (*(motor->stepPin.datAddr)){
                *(motor->stepPin.datAddr) = 0; //toggle to low
                set_Motor_Accel_Timer(motor);
            }
            else{
                if(motor->position == motor->targetPos){
                    //arrive target or trespass border
                    motor->state = Motor_moveDone;
                } else {
                    if (motor->moveType == motor_move_fwd){
                        if (*(motor->highEdgePin.datAddr)){//not at touch edge
                            motor->position++;
                            motor->movingSteps++;
                            *(motor->stepPin.datAddr) = 1;
                        }
                        else{
                            //state change to error
                            motor->state = Motor_moveDone;  
                        }
                    }
                    else{   //move backward
                        if (*(motor->lowEdgePin.datAddr)){
                            motor->position--;
                            motor->movingSteps++;
                            *(motor->stepPin.datAddr) = 1;
                        }
                        else{
                            //state change to error
                            motor->state = Motor_moveDone;                              
                        }
                    }
                }
            }
        }
        break;

        case MOTOR_HOME:
        {
            if(*(motor->stepPin.datAddr))   //step pin is high
            {
                *(motor->stepPin.datAddr) = 0;
                set_Motor_Accel_Timer(motor);
            }
            else                            //step pin is low
            {
                if (motor->movingSteps < motor->maxHomeStep) // within SW limit
                {
                    if (*(motor->lowEdgePin.datAddr)) // not reach edge sensor
                    {
                        motor->position--;
                        motor->movingSteps++;
                        *(motor->stepPin.datAddr) = 1;
                    }
                    else //reach edge sensor
                    {
#if PARK_AFTER_HOME
                        // "parking": ensure PI being blocked firmly
                        motor->state = MOTOR_PARK;
                        motor->debounceCounter=0;
#else
                        motor->state = Motor_moveDone;
                        motor->position = 0;
#endif
                    }
                }
                else // reach SW limit
                {
                    motor->state = Motor_moveDone;
                    motor->position = 0;
                }
            }
        }
        break;
#if PARK_AFTER_HOME
        // step a few steps further to ensure PI being blocked firmly
        case MOTOR_PARK:
        {
            if (*(motor->stepPin.datAddr))  //step pin is high
            {
                *(motor->stepPin.datAddr) = 0;
            }
            else                            //step pin is low
            {
                // move extra 20 steps
                if(motor->debounceCounter < 20)
                {
                    *(motor->stepPin.datAddr) = 1;
                    motor->debounceCounter++;
                }
                else
                {
                    motor->state = Motor_moveDone;
                    motor->position = 0;
                }
            }
        }
        break;
#endif
        // Set standby current
        case Motor_moveDone :{
            //set I0/I1 state
            set_dac_io(&motor->i0Pin, motor->stdbyI0State);
            set_dac_io(&motor->i1Pin, motor->stdbyI1State); 

            *(motor->busyPin.datAddr) = LOW;
#if DebugPrint
            printf("%d pos %d\n", motor->id, motor->position);
#endif
            motor->state = Motor_Idle;
            
            *(busy.datAddr) = (*(motors[0].busyPin.datAddr))
                            ||(*(motors[1].busyPin.datAddr))
                            ||(*(motors[2].busyPin.datAddr));
        }
        break;
        
        case Motor_Idle :{
            //do nothing
        }
        break;
    }
}

/**
 * @brief  Enable the specified motor.
 * @param  motor_id The ID (index) of the motor.
 * @return None
 */
void motor_enable(uint32_t motorId){
    *(motors[motorId].sleepPin.datAddr) = HIGH;
}
/**
 * @brief  Disable the specified motor.
 * @param  motor_id The ID (index) of the motor.
 * @return None
 */
void motor_disable(uint32_t motorId){
    *(motors[motorId].sleepPin.datAddr) = LOW;
}

/**
 * @brief Set the I0 and I1 states based on the desired current setting.
 *
 * @details This function maps a specified current level (0% to 100%) to corresponding
 *          I0 and I1 pin states, which control the motor driver’s current level.
 *
 * @param current Desired current setting (see @ref CURRENT_SETTING).
 * @param i0 Pointer to the variable storing I0 pin state (output).
 * @param i1 Pointer to the variable storing I1 pin state (output).
 */
void select_current(CURRENT_SETTING current, IO_STATE *i0, IO_STATE *i1){
    switch(current){
        case CURRENT_100_PERCENT:
            *i1 = io_low;
            *i0 = io_low;
            break;
        case CURRENT_87_PERCENT:
            *i1 = io_low;
            *i0 = io_hiz;
            break;
        case CURRENT_75_PERCENT:
            *i1 = io_low;
            *i0 = io_high;
            break;
        case CURRENT_62_PERCENT:
            *i1 = io_hiz;
            *i0 = io_low;
            break;
        case CURRENT_50_PERCENT:
            *i1 = io_hiz;
            *i0 = io_hiz;
            break;
        case CURRENT_37_PERCENT:
            *i1 = io_hiz;
            *i0 = io_high;
            break;
        case CURRENT_25_PERCENT:
            *i1 = io_high;
            *i0 = io_low;
            break;
        case CURRENT_12_PERCENT:
            *i1 = io_high;
            *i0 = io_hiz;
            break;
        case CURRENT_0_PERCENT:
            *i1 = io_high;
            *i0 = io_high;
            break;
    }
}

/**
 * @brief Parses the IO state and returns the corresponding current setting.
 *
 * @details This function takes two IO states (`i0` and `i1`) as inputs and determines
 *          the corresponding @c CURRENT_SETTING value. The logic is based on how the
 *          IO pins are configured to represent various current levels.
 *
 * @param i0 IO state of the I0 pin.
 * @param i1 IO state of the I1 pin.
 * @retval CURRENT_SETTING The corresponding @c CURRENT_SETTING value.
 * @retval -1  The input states do not match any known configuration.
 */
CURRENT_SETTING parse_current_setting(IO_STATE i0, IO_STATE i1){
    switch(i1){
    case io_low:
        switch(i0){
        case io_low:  return CURRENT_100_PERCENT;
        case io_hiz:  return CURRENT_87_PERCENT;
        case io_high: return CURRENT_75_PERCENT;
        default:      return -1;
        }
    case io_hiz:
        switch(i0){
        case io_low:  return CURRENT_62_PERCENT;
        case io_hiz:  return CURRENT_50_PERCENT;
        case io_high: return CURRENT_37_PERCENT;
        default:      return -1;
        }
    case io_high:
        switch(i0){
        case io_low:  return CURRENT_25_PERCENT;
        case io_hiz:  return CURRENT_12_PERCENT;
        case io_high: return CURRENT_0_PERCENT;
        default:      return -1;
        }
    default: return -1;
    }
}

/**
 * @brief  Initiate motor 0.
 * @param  None
 * @return None
 */
void init_Motor0(){
    MOTOR_DATA * motor = &motors[0];
    motor->id = 0;
    motor->maxHomeStep = 120000;
    //sleep
    motor->sleepPin.port = PA;
    motor->sleepPin.pin = BIT14;
    motor->sleepPin.datAddr = GPIO_PIN_DATA_ADDR(portA, 14);
    //dir
    motor->dirPin.port = PC;
    motor->dirPin.pin = BIT2;
    motor->dirPin.datAddr = GPIO_PIN_DATA_ADDR(portC, 2);
    //step
    motor->stepPin.port = PC;
    motor->stepPin.pin = BIT5;
    motor->stepPin.datAddr = GPIO_PIN_DATA_ADDR(portC, 5);
    //i0
    motor->i0Pin.port = PD;
    motor->i0Pin.pin = BIT3;
    motor->i0Pin.datAddr = GPIO_PIN_DATA_ADDR(portD, 3);
    //i1
    motor->i1Pin.port = PD;
    motor->i1Pin.pin = BIT2;
    motor->i1Pin.datAddr = GPIO_PIN_DATA_ADDR(portD, 2);
    //high 
    motor->highEdgePin.port = userConfig[0].highEdgePin.port;
    motor->highEdgePin.pin = userConfig[0].highEdgePin.pin;
    motor->highEdgePin.datAddr = userConfig[0].highEdgePin.datAddr;
    //low 
    motor->lowEdgePin.port = userConfig[0].lowEdgePin.port;
    motor->lowEdgePin.pin = userConfig[0].lowEdgePin.pin;
    motor->lowEdgePin.datAddr = userConfig[0].lowEdgePin.datAddr;
    //busy
    motor->busyPin.port = PA;
    motor->busyPin.pin = BIT0;
    motor->busyPin.datAddr = GPIO_PIN_DATA_ADDR(portA, 0);

    motor->fwdDirState = userConfig[0].fwdDirState;

    //select_current(torqueConfig[0].standby_torque, &(motor->stdbyI0State), &(motor->stdbyI1State));
    select_current(CURRENT_12_PERCENT, &(motor->stdbyI0State), &(motor->stdbyI1State));
    select_current(torqueConfig[0].active_torque, &(motor->activeI0State), &(motor->activeI1State));

    motor->defaultPPS = userConfig[0].defaultPPS;
    motor->targetPPS = userConfig[0].targetPPS;
    motor->acceleration = userConfig[0].acceleration;
    motor->deceleration = userConfig[0].deceleration;
    motor->enableMototAccelerate = userConfig[0].enableMototAccelerate;
    motor->timer = TIMER0;

    init_Motor_Gpio(motor);
}

/**
 * @brief  Initiate motor 1.
 * @param  None
 * @return None
 */
void init_Motor1(){
    MOTOR_DATA * motor = &motors[1];
    motor->id = 1;
    motor->maxHomeStep = 120000;
    //sleep
    motor->sleepPin.port = PA;
    motor->sleepPin.pin = BIT15;
    motor->sleepPin.datAddr = GPIO_PIN_DATA_ADDR(portA, 15);
    //dir
    motor->dirPin.port = PC;
    motor->dirPin.pin = BIT1;
    motor->dirPin.datAddr = GPIO_PIN_DATA_ADDR(portC, 1);
    //step
    motor->stepPin.port = PC;
    motor->stepPin.pin = BIT4;
    motor->stepPin.datAddr = GPIO_PIN_DATA_ADDR(portC, 4);
    //i0
    motor->i0Pin.port = PD;
    motor->i0Pin.pin = BIT1;
    motor->i0Pin.datAddr = GPIO_PIN_DATA_ADDR(portD, 1);
    //i1
    motor->i1Pin.port = PD;
    motor->i1Pin.pin = BIT0;
    motor->i1Pin.datAddr = GPIO_PIN_DATA_ADDR(portD, 0);
    //high 
    motor->highEdgePin.port = userConfig[1].highEdgePin.port;
    motor->highEdgePin.pin = userConfig[1].highEdgePin.pin;
    motor->highEdgePin.datAddr = userConfig[1].highEdgePin.datAddr;
    //low 
    motor->lowEdgePin.port = userConfig[1].lowEdgePin.port;
    motor->lowEdgePin.pin = userConfig[1].lowEdgePin.pin;
    motor->lowEdgePin.datAddr = userConfig[1].lowEdgePin.datAddr;
    //busy
    motor->busyPin.port = PA;
    motor->busyPin.pin = BIT1;
    motor->busyPin.datAddr = GPIO_PIN_DATA_ADDR(portA, 1);

    motor->fwdDirState = userConfig[1].fwdDirState;

    //select_current(torqueConfig[1].standby_torque, &(motor->stdbyI0State), &(motor->stdbyI1State));
    select_current(CURRENT_12_PERCENT, &(motor->stdbyI0State), &(motor->stdbyI1State));
    select_current(torqueConfig[1].active_torque, &(motor->activeI0State), &(motor->activeI1State));

    motor->defaultPPS = userConfig[1].defaultPPS;
    motor->targetPPS = userConfig[1].targetPPS;
    motor->acceleration = userConfig[1].acceleration;
    motor->deceleration = userConfig[1].deceleration;
    motor->enableMototAccelerate = userConfig[1].enableMototAccelerate;
    motor->timer = TIMER1;
    init_Motor_Gpio(motor);    
}

/**
 * @brief  Initiate motor 2.
 * @param  None
 * @return None
 */
void init_Motor2(){
    MOTOR_DATA * motor = &motors[2];
    motor->id = 2;
    motor->maxHomeStep = 120000;
    //sleep
    motor->sleepPin.port = PB;
    motor->sleepPin.pin = BIT15;
    motor->sleepPin.datAddr = GPIO_PIN_DATA_ADDR(portB, 15);
    //dir
    motor->dirPin.port = PC;
    motor->dirPin.pin = BIT0;
    motor->dirPin.datAddr = GPIO_PIN_DATA_ADDR(portC, 0);
    //step
    motor->stepPin.port = PC;
    motor->stepPin.pin = BIT3;
    motor->stepPin.datAddr = GPIO_PIN_DATA_ADDR(portC, 3);
    //i0
    motor->i0Pin.port = PA;
    motor->i0Pin.pin = BIT12;
    motor->i0Pin.datAddr = GPIO_PIN_DATA_ADDR(portA, 12);
    //i1
    motor->i1Pin.port = PA;
    motor->i1Pin.pin = BIT13;
    motor->i1Pin.datAddr = GPIO_PIN_DATA_ADDR(portA, 13);
    //high 
    motor->highEdgePin.port = userConfig[2].highEdgePin.port;
    motor->highEdgePin.pin = userConfig[2].highEdgePin.pin;
    motor->highEdgePin.datAddr = userConfig[2].highEdgePin.datAddr;
    //low 
    motor->lowEdgePin.port = userConfig[2].lowEdgePin.port;
    motor->lowEdgePin.pin = userConfig[2].lowEdgePin.pin;
    motor->lowEdgePin.datAddr = userConfig[2].lowEdgePin.datAddr;
    //busy
    motor->busyPin.port = PA;
    motor->busyPin.pin = BIT2;
    motor->busyPin.datAddr = GPIO_PIN_DATA_ADDR(portA, 2);

    motor->fwdDirState = userConfig[2].fwdDirState;

    //select_current(torqueConfig[2].standby_torque, &(motor->stdbyI0State), &(motor->stdbyI1State));
    select_current(CURRENT_12_PERCENT, &(motor->stdbyI0State), &(motor->stdbyI1State));
    select_current(torqueConfig[2].active_torque, &(motor->activeI0State), &(motor->activeI1State));

    motor->defaultPPS = userConfig[2].defaultPPS;
    motor->targetPPS = userConfig[2].targetPPS;
    motor->acceleration = userConfig[2].acceleration;
    motor->deceleration = userConfig[2].deceleration;
    motor->enableMototAccelerate = userConfig[2].enableMototAccelerate;
    motor->timer = TIMER2;
    init_Motor_Gpio(motor);
}

/**
 * @brief  Initiate Gate motor.
 * @param  None
 * @return None
 */
void init_Motor_Gate(){
    MOTOR_DATA * motor = &motors[MOTORID_4];
    motor->id = MOTORID_4;
    motor->maxHomeStep = 165000;//Adjust the gate return-to-home step count for PSC 700.

    //sleep
    motor->sleepPin.port = PA;
    motor->sleepPin.pin = BIT7;
    motor->sleepPin.datAddr = GPIO_PIN_DATA_ADDR(portA, 7);
    //dir
    motor->dirPin.port = PC;
    motor->dirPin.pin = BIT7;
    motor->dirPin.datAddr = GPIO_PIN_DATA_ADDR(portC, 7);
    //step
    motor->stepPin.port = PC;
    motor->stepPin.pin = BIT6;
    motor->stepPin.datAddr = GPIO_PIN_DATA_ADDR(portC, 6);
    //i0
    motor->i0Pin.port = PB;
    motor->i0Pin.pin = BIT9;
    motor->i0Pin.datAddr = GPIO_PIN_DATA_ADDR(portB, 9);
    //i1
    motor->i1Pin.port = PB;
    motor->i1Pin.pin = BIT8;
    motor->i1Pin.datAddr = GPIO_PIN_DATA_ADDR(portB, 8);

    //high
    motor->highEdgePin.port = userConfig[MOTORID_4].highEdgePin.port;
    motor->highEdgePin.pin = userConfig[MOTORID_4].highEdgePin.pin;
    motor->highEdgePin.datAddr = userConfig[MOTORID_4].highEdgePin.datAddr;
    //low
    motor->lowEdgePin.port = userConfig[MOTORID_4].lowEdgePin.port;
    motor->lowEdgePin.pin = userConfig[MOTORID_4].lowEdgePin.pin;
    motor->lowEdgePin.datAddr = userConfig[MOTORID_4].lowEdgePin.datAddr;

    //busy
    motor->busyPin.port = PA;
    motor->busyPin.pin = BIT3;
    motor->busyPin.datAddr = GPIO_PIN_DATA_ADDR(portA, 3);

    motor->fwdDirState = userConfig[MOTORID_4].fwdDirState;

    //select_current(torqueConfig[2].standby_torque, &(motor->stdbyI0State), &(motor->stdbyI1State));
    select_current(CURRENT_12_PERCENT, &(motor->stdbyI0State), &(motor->stdbyI1State));
    select_current(torqueConfig[MOTORID_4].active_torque, &(motor->activeI0State), &(motor->activeI1State));

    motor->defaultPPS = userConfig[MOTORID_4].defaultPPS;
    motor->targetPPS = userConfig[MOTORID_4].targetPPS;
    motor->acceleration = userConfig[MOTORID_4].acceleration;
    motor->deceleration = userConfig[MOTORID_4].deceleration;
    motor->enableMototAccelerate = userConfig[MOTORID_4].enableMototAccelerate;
    motor->timer = TIMER3;
    init_Motor_Gpio(motor);
}

static void motor_gpio_setMode(GPIO_TYPEDef * gpio, uint32_t mode){
    *(gpio->datAddr) = 0;
    GPIO_SetMode(gpio->port, gpio->pin, mode);
}

static inline void set_dac_io(GPIO_TYPEDef * gpio, IO_STATE io){

    switch (io){
        case io_hiz:{
            *(gpio->datAddr) = 0;
            GPIO_SetMode(gpio->port, gpio->pin, GPIO_MODE_INPUT);             
        }
        break;
        case io_high:{
            *(gpio->datAddr) = HIGH;
            GPIO_SetMode(gpio->port, gpio->pin, GPIO_MODE_OUTPUT);        
        }
        break;
        case io_low:{
            *(gpio->datAddr) = LOW;
            GPIO_SetMode(gpio->port, gpio->pin, GPIO_MODE_OUTPUT);  
        }
        break;
    }
}

/**
 * @brief  Initiate GPIO mode for the specified motor.
 * @param  motor Pointer to the MOTOR_DATA structure.
 * @return None
 */
void init_Motor_Gpio(MOTOR_DATA * motor){
    //GPIO_SetMode(motor->dirPin.port, motor->dirPin.pin, GPIO_MODE_OUTPUT);

    motor_gpio_setMode(&motor->sleepPin, GPIO_MODE_OUTPUT);
    motor_gpio_setMode(&motor->dirPin, GPIO_MODE_OUTPUT);
    motor_gpio_setMode(&motor->stepPin, GPIO_MODE_OUTPUT);  
    motor_gpio_setMode(&motor->i0Pin, GPIO_MODE_OUTPUT);
    motor_gpio_setMode(&motor->i1Pin, GPIO_MODE_OUTPUT);
    motor_gpio_setMode(&motor->busyPin, GPIO_MODE_OUTPUT);
    motor_gpio_setMode(&motor->highEdgePin, GPIO_MODE_INPUT);  
    motor_gpio_setMode(&motor->lowEdgePin, GPIO_MODE_INPUT);
    
    
    /* Skip limit switch pull-up configuration for motor ID 4 */
    /*if (motor->id != MOTORID_4) {
        if ((motor->highEdgePin.pin > motor->lowEdgePin.pin)) {
            GPIO_SetPullCtl(motor->lowEdgePin.port,  motor->lowEdgePin.pin,  GPIO_PUSEL_PULL_UP);
        } else {
            GPIO_SetPullCtl(motor->highEdgePin.port, motor->highEdgePin.pin, GPIO_PUSEL_PULL_UP);
        }
    }*/
    //GPIO_SetMode(PC, BIT5, GPIO_MODE_OUTPUT);  

    set_dac_io(&motor->i0Pin, motor->stdbyI0State);
    set_dac_io(&motor->i1Pin, motor->stdbyI1State);    
    //*(motor->i0Pin.datAddr) = motor->stdbyI0State;
    //*(motor->i1Pin.datAddr) = motor->stdbyI1State;       
}

/**
 * @brief  Set the direction configuration for the specified motor.
 * @param  motor_id The ID (index) of the motor.
 * @param  direction_config @c CFG_DIR_DEFAULT or @c CFG_DIR_REVERSE
 * @return None
 */
void setDirection(uint32_t motor_id, direction_config dir_cfg)
{
    switch(dir_cfg){
        case CFG_DIR_DEFAULT:
            motors[motor_id].fwdDirState = TRUE;
            break;
        case CFG_DIR_REVERSE:
            motors[motor_id].fwdDirState = FALSE;
            break;
    }
}

/**
 * @brief  Get the direction configuration for the specified motor.
 * @param  motor_id The ID (index) of the motor.
 * @return @c CFG_DIR_DEFAULT
 *         @c CFG_DIR_REVERSE
 */
direction_config getDirection(uint32_t motor_id)
{
    if(motors[motor_id].fwdDirState)
        return CFG_DIR_DEFAULT;
    else
        return CFG_DIR_REVERSE;
}

/**
 * @brief  Set the limit switch configuration for the specified motor.
 * @param  motor_id The ID (index) of the motor.
 * @param  limit_switch_config @c CFG_LIMSW_DEFAULT or @c CFG_LIMSW_REVERSE
 * @return None
 */
void setLimitSwitch(uint32_t motor_id, limit_switch_config lsw_cfg)
{
    switch(motor_id){
        case 0:
            switch(lsw_cfg){
                case CFG_LIMSW_REVERSE:
                    motors[0].highEdgePin.port = PB;
                    motors[0].highEdgePin.pin  = BIT4;
                    motors[0].highEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 4);
                    
                    motors[0].lowEdgePin.port = PB;
                    motors[0].lowEdgePin.pin  = BIT5;
                    motors[0].lowEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 5);
                    break;
                case CFG_LIMSW_DEFAULT:
                    motors[0].highEdgePin.port = PB;
                    motors[0].highEdgePin.pin  = BIT5;
                    motors[0].highEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 5);
                    
                    motors[0].lowEdgePin.port = PB;
                    motors[0].lowEdgePin.pin  = BIT4;
                    motors[0].lowEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 4);
                    break;
            }
            break;
        case 1:
            switch(lsw_cfg){
                case CFG_LIMSW_REVERSE:
                    motors[1].highEdgePin.port = PB;
                    motors[1].highEdgePin.pin  = BIT2;
                    motors[1].highEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 2);
                    
                    motors[1].lowEdgePin.port = PB;
                    motors[1].lowEdgePin.pin  = BIT3;
                    motors[1].lowEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 3);
                    break;
                case CFG_LIMSW_DEFAULT:
                    motors[1].highEdgePin.port = PB;
                    motors[1].highEdgePin.pin  = BIT3;
                    motors[1].highEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 3);
                    
                    motors[1].lowEdgePin.port = PB;
                    motors[1].lowEdgePin.pin  = BIT2;
                    motors[1].lowEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 2);
                    break;
            }
            break;
        case 2:
            switch(lsw_cfg){
                case CFG_LIMSW_REVERSE:
                    motors[2].highEdgePin.port = PB;
                    motors[2].highEdgePin.pin  = BIT0;
                    motors[2].highEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 0);
                    
                    motors[2].lowEdgePin.port = PB;
                    motors[2].lowEdgePin.pin  = BIT1;
                    motors[2].lowEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 1);
                    break;
                case CFG_LIMSW_DEFAULT:
                    motors[2].highEdgePin.port = PB;
                    motors[2].highEdgePin.pin  = BIT1;
                    motors[2].highEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 1);
                    
                    motors[2].lowEdgePin.port = PB;
                    motors[2].lowEdgePin.pin  = BIT0;
                    motors[2].lowEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 0);
                    break;
            }
            break;
        case MOTORID_4:
            switch(lsw_cfg){
                case CFG_LIMSW_REVERSE:
                    motors[MOTORID_4].highEdgePin.port = PB;
                    motors[MOTORID_4].highEdgePin.pin  = BIT7;
                    motors[MOTORID_4].highEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 7);
                    
                    motors[MOTORID_4].lowEdgePin.port = PB;
                    motors[MOTORID_4].lowEdgePin.pin  = BIT6;
                    motors[MOTORID_4].lowEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 6);
                    break;
                case CFG_LIMSW_DEFAULT:
                    motors[MOTORID_4].highEdgePin.port = PB;
                    motors[MOTORID_4].highEdgePin.pin  = BIT6;
                    motors[MOTORID_4].highEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 6);
                    
                    motors[MOTORID_4].lowEdgePin.port = PB;
                    motors[MOTORID_4].lowEdgePin.pin  = BIT7;
                    motors[MOTORID_4].lowEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 7);
                    break;
            }
            break;
    }
}

/**
 * @brief  Get the limit switch configuration for the specified motor.
 *
 * @details This function determines the limit switch direction (default or reverse)
 *          based on the relative values of the motor's high-edge and low-edge pin numbers.
 *
 * @param  motor_id The ID (index) of the motor.
 * @return @c CFG_LIMSW_DEFAULT if high-edge pin > low-edge pin;
 *         @c CFG_LIMSW_REVERSE otherwise.
 */
limit_switch_config getLimitSwitch(uint32_t motor_id)
{
     if(motors[motor_id].highEdgePin.pin > motors[motor_id].lowEdgePin.pin)
        return CFG_LIMSW_DEFAULT;
     else
        return CFG_LIMSW_REVERSE;
}

/**
 * @brief Timer0 interrupt handler.
 *
 * @details This function is triggered when Timer0 reaches its timeout. It clears the interrupt flag
 *          and polls the status of motor 0 by calling @c poll_Motor().
 */
void TMR0_IRQHandler(void)
{
    if (TIMER_GetIntFlag(TIMER0))
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        poll_Motor(&motors[0]);
    }
}

/**
 * @brief Timer1 interrupt handler.
 *
 * @details This function is triggered when Timer1 reaches its timeout. It clears the interrupt flag
 *          and polls the status of motor 1 by calling @c poll_Motor().
 */
void TMR1_IRQHandler(void)
{
    if (TIMER_GetIntFlag(TIMER1))
    {
        /* Clear Timer1 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER1);

        poll_Motor(&motors[1]);
    }
}

/**
 * @brief Timer2 interrupt handler.
 *
 * @details This function is triggered when Timer2 reaches its timeout. It clears the interrupt flag
 *          and polls the status of motor 2 by calling @c poll_Motor().
 */
void TMR2_IRQHandler(void)
{
    if (TIMER_GetIntFlag(TIMER2))
    {
        /* Clear Timer2 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER2);

        poll_Motor(&motors[2]);
    }
}

/**
 * @brief Timer3 interrupt handler.
 *
 * @details This function is triggered when Timer3 reaches its timeout. It clears the interrupt flag
 *          and polls the status of motor 3 by calling @c poll_Motor().
 */
void TMR3_IRQHandler(void)
{
    if (TIMER_GetIntFlag(TIMER3))
    {
        /* Clear Timer3 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER3);

        poll_Motor(&motors[3]);
    }
}