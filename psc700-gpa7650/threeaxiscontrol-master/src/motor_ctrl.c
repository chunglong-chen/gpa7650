#include "uart_cmd.h"
#include "motor_ctrl.h"
#include "user_config.h"
#include <math.h>

static MOTOR_DATA motors[3];
static GPIO_TYPEDef busy;   //motor busy status
uint8_t trespass_log=0;

static void init_Motor0();
static void init_Motor1();
static void init_Motor2();

static void init_Motor_Gpio(MOTOR_DATA * motor);
static void start_Motor(MOTOR_DATA * motor);
static void set_Motor_Accel_Timer(MOTOR_DATA * motor);
static void init_Motor_Accel(MOTOR_DATA * motor, uint32_t stepsToMove);
static void motor_gpio_setMode(GPIO_TYPEDef * gpio, uint32_t mode);
static inline void set_dac_io(GPIO_TYPEDef * gpio, IO_STATE io);

#define T1_FREQ 1000000.0f //Hz, timer base is 1us
#define DebugPrint		0

void motor_init(){
    busy.port = PA;
    busy.pin = BIT3;
    busy.datAddr = GPIO_PIN_DATA_ADDR(portA, 3);
    motor_gpio_setMode(&busy, GPIO_MODE_OUTPUT);
    init_Motor0();
    init_Motor1();
    init_Motor2();
}

MOTOR_DATA * get_motor_ref(uint32_t motorId){
    return &motors[motorId];
}

bool motor_resetPos(uint32_t motorId){
    //busy
    if (*(motors[motorId].busyPin.datAddr)) return false;
    motors[motorId].position = 0;
    return true;
}

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

bool motor_home(uint32_t motorId){
    MOTOR_DATA * motor = &motors[motorId];

    //busy
    if (*(motor->busyPin.datAddr)) return false;

    motor->moveType = motor_move_home;
    start_Motor(motor);
    return true;
}

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

static void poll_Motor(MOTOR_DATA * motor){
    
    set_timer_tick(motor);
    if (motor->state != Motor_Idle){     
        //start timer for next poll
        TIMER_Start(motor->timer);
    }
    
    switch (motor->state){
        case Motor_InitMove :{         
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
                *(motor->dirPin.datAddr) = !motor->fwdDirState;
                motor->state = Motor_homeBWD;   
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

        case Motor_homeBWD :{
            if (*(motor->stepPin.datAddr)){
                *(motor->stepPin.datAddr) = 0; //toggle to low
                set_Motor_Accel_Timer(motor);
            }
            else{
                if (motor->movingSteps < motor->maxHomeStep){    //software limiting
                    if (*(motor->lowEdgePin.datAddr)){//not at touch edge
                        motor->position--;
                        motor->movingSteps++;
                        *(motor->stepPin.datAddr) = 1;                                         
                    }
                    else{   //touch edge sensor
                        motor->state = Motor_homeDBC;
                        *(motor->dirPin.datAddr) = motor->fwdDirState;

                        //set to constant pps
                        motor->stepDelay = T1_FREQ/motor->defaultPPS;
                        motor->debounceCounter=0;
                    }
                }
                else{
                    motor->state = Motor_moveDone;
                    motor->position = 0;                    
                }
            }
        }
        break;
        
        // limsw debounce / inertia buffering interval
        case Motor_homeDBC:{
            if(motor->debounceCounter >= T1_FREQ*0.05) //50 ms debounce
            {
                motor->state = Motor_homeFWD;
                init_Motor_Accel(motor, motor->maxHomeStep);
            }
            else
            {
                //accumulate counter only when counter < threshold
                motor->debounceCounter+=motor->stepDelay;
            }
        }
        break;

        // move forward unitl limsw is released
        case Motor_homeFWD :{
            if (*(motor->stepPin.datAddr)){
                *(motor->stepPin.datAddr) = 0; //toggle to low
                set_Motor_Accel_Timer(motor);
            }else{
                if(motor->movingSteps < motor->maxHomeStep //software limiting
                && !*(motor->lowEdgePin.datAddr))          //still at edge
                {
                    motor->position++;
                    motor->movingSteps++;
                    *(motor->stepPin.datAddr) = 1;
                }else{
                    motor->state = Motor_moveDone;
                    motor->position = 0;
                }
            }
        }
        break;
    

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

void motor_enable(uint32_t motorId){
    *(motors[motorId].sleepPin.datAddr) = HIGH;
}
void motor_disable(uint32_t motorId){
    *(motors[motorId].sleepPin.datAddr) = LOW;
}

/* Choose a current and pass pointers of the io state variables to this function. *
 * The io state will be set accordingly.                                          */
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

/* Choose a current and pass pointers of the io state variables to this function. *
 * The io state will be set accordingly.                                          */
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
    //GPIO_SetMode(PC, BIT5, GPIO_MODE_OUTPUT);  

    set_dac_io(&motor->i0Pin, motor->stdbyI0State);
    set_dac_io(&motor->i1Pin, motor->stdbyI1State);    
    //*(motor->i0Pin.datAddr) = motor->stdbyI0State;
    //*(motor->i1Pin.datAddr) = motor->stdbyI1State;       
}

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

direction_config getDirection(uint32_t motor_id)
{
    if(motors[motor_id].fwdDirState)
        return CFG_DIR_DEFAULT;
    else
        return CFG_DIR_REVERSE;
}

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
    }
}

limit_switch_config getLimitSwitch(uint32_t motor_id)
{
     if(motors[motor_id].highEdgePin.pin > motors[motor_id].lowEdgePin.pin)
        return CFG_LIMSW_DEFAULT;
     else
        return CFG_LIMSW_REVERSE;
}

void TMR0_IRQHandler(void)
{
    if (TIMER_GetIntFlag(TIMER0))
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        poll_Motor(&motors[0]);
    }
}

void TMR1_IRQHandler(void)
{
    if (TIMER_GetIntFlag(TIMER1))
    {
        /* Clear Timer1 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER1);

        poll_Motor(&motors[1]);
    }
}

void TMR2_IRQHandler(void)
{
    if (TIMER_GetIntFlag(TIMER2))
    {
        /* Clear Timer2 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER2);

        poll_Motor(&motors[2]);
    }
}
