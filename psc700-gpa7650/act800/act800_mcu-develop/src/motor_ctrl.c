/**
 * @file    motor_ctrl.c
 * @brief   Motor control module using GPIO and step control.
 * @details This module implements control of stepper motors including
 *          movement, homing, direction control, and state management.
 *
 *          A task-based state machine is used to control motor behavior,
 *          including step generation, position tracking, and homing logic.
 *
 *          The module supports:
 *          - Relative and absolute movement
 *          - Homing operation using detection pin
 *          - Direction control via GPIO
 *          - Step pulse generation based on timing
 * @ingroup motor_controller
 */
#include "motor_ctrl.h"
#include "peripheral/systick/plib_systick.h"
uint32_t _12p5Us_tick;
MOTOR_DATA motors_data[MOTOR_COUNT];

void motor_data_init();
void motor_state_init(int id);
/**
 * @brief   Calculate tick difference with overflow handling.
 * @details Computes the difference between two tick values, handling
 *          counter overflow (rollover) condition.
 *
 * @param[in] tick   Current tick value.
 * @param[in] prev   Previous tick value.
 *
 * @return Tick difference.
 */
uint32_t diff_tick(uint32_t tick, uint32_t prev) {
    if (tick < prev) {
        // time rollover(overflow)
        return (0xFFFFFFFF - prev) + 1 + tick;
    } else {
        return tick - prev;
    }
}
/**
 * @brief   Initialize motor control module.
 * @details Initializes motor data structures and enables motor-related
 *          hardware by setting corresponding control pins.
 */
void MOTOR_Initialize(void) {
    motor_data_init();
    PORT_PinWrite(MOTOR_EN_PIN, true);
    SYSTICK_DelayMs(100);
    PORT_PinWrite(REFARM_EN_PIN, true);
    SYSTICK_DelayMs(100);
    PORT_PinWrite(PC_EN_PIN, true);
}
/**
 * @brief   Wake up motor driver for specified motor.
 * @details Sets control pins (I0, I1) to active state to enable motor driver.
 *
 * @param[in] id  Motor ID.
 */
void motor_local_wakeup(int id) {
    PORT_PinWrite(motors_data[id].i0_pin, motors_data[id].active_i0_state);
    PORT_PinWrite(motors_data[id].i1_pin, motors_data[id].active_i1_state);
}
/**
 * @brief   Set motor driver to standby mode.
 * @details Sets control pins (I0, I1) to standby state to disable motor driver.
 *
 * @param[in] id  Motor ID.
 */
void motor_local_standby(int id) {
    PORT_PinWrite(motors_data[id].i0_pin, motors_data[id].stdby_i0_state);
    PORT_PinWrite(motors_data[id].i1_pin, motors_data[id].stdby_i1_state);
}
/**
 * @brief   Configure motor direction and wake up driver.
 * @details Sets the direction pin according to the desired movement
 *          direction and enables the motor driver.
 *
 * @param[in] id  Motor ID.
 */
void motor_local_setDirAndWakeup(int id) {
    if (motors_data[id].move == MOTOR_MOVE_FWD) {
        //set gpio
        if (id == MOTORID_REFARM)
            PORT_PinWrite(motors_data[id].dir_pin, motors_data[id].fwd_dir_state);
        else if (id == MOTORID_PIRIS)
            PORT_PinWrite(motors_data[id].dir_pin, !motors_data[id].fwd_dir_state);
        else
            return;
    } else if ((motors_data[id].move == MOTOR_MOVE_BWD) ||
            (motors_data[id].move == MOTOR_MOVE_HOME)) {
        if (id == MOTORID_REFARM)
            PORT_PinWrite(motors_data[id].dir_pin, !motors_data[id].fwd_dir_state);
        else if (id == MOTORID_PIRIS)
            PORT_PinWrite(motors_data[id].dir_pin, motors_data[id].fwd_dir_state);
        else
            return;
    }
    motor_local_wakeup(id);
}
/**
 * @brief   Perform motor step movement.
 * @details Generates step pulses based on timing interval and updates
 *          motor position and remaining steps.
 *
 * @param[in] id  Motor ID.
 */
void motor_move(int id) {
    if (diff_tick(_12p5Us_tick, motors_data[id].last_ctrltick) >= motors_data[id].step_interval) {
        //should move
        //save move time
        motors_data[id].last_ctrltick = _12p5Us_tick;
        if (PORT_PinRead(motors_data[id].step_pin)) {
            //high -> low
            motors_data[id].step_remain--;
            if (motors_data[id].move == MOTOR_MOVE_FWD)
                motors_data[id].pos_current++;
            else if (motors_data[id].move == MOTOR_MOVE_BWD)
                motors_data[id].pos_current--;
            else if (motors_data[id].move == MOTOR_MOVE_HOME)
                motors_data[id].step_to_home++;
        } 
        PORT_PinToggle(motors_data[id].step_pin);
    }
}
/**
 * @brief   Check if homing step-out is completed.
 * @details Determines whether homing movement has finished and updates
 *          motor status accordingly.
 *
 * @param[in] id  Motor ID.
 *
 * @return true  If homing is completed.
 * @return false If still in progress.
 */
bool motor_chkhome_stepout(int id) {
    if (motors_data[id].step_remain > 0)
        return false;
    //home move over
    //finish home
    motors_data[id].state = MOTOR_STATE_IDLE;
    motors_data[id].status_reg.motor_busy = 0;
    motors_data[id].move = MOTOR_MOVE_IDLE;
    motors_data[id].pos_current = 0;
    motor_local_standby(id);
    motors_data[id].status_reg.homerr = true;
    return true;
}
/**
 * @brief   Motor control task state machine. (Ref: SD21)
 * @details Implements motor control logic including movement,
 *          homing sequence, direction setup, and state transitions.
 *
 *          States:
 *          - MOTOR_STATE_INIT
 *          - MOTOR_STATE_SETDIR
 *          - MOTOR_STATE_MOVE
 *          - MOTOR_STATE_HOMEBWD
 *          - MOTOR_STATE_HOMEFWD
 *          - MOTOR_STATE_IDLE
 *
 * @note    Handles multiple motors using a loop-based state machine.
 */
void MOTOR_Tasks() {
    int i;

    for (i = 0; i < MOTOR_COUNT; i++) {
        switch (motors_data[i].state) {
                //motor move as first item to save some state check time
            case MOTOR_STATE_MOVE:
            {
                if (motors_data[i].step_remain > 0) {
                    motor_move(i);
                } else {
                    //moving finished
                    motors_data[i].state = MOTOR_STATE_IDLE;
                    motors_data[i].status_reg.motor_busy = 0;
                    motors_data[i].move = MOTOR_MOVE_IDLE;
                    motor_local_standby(i);
                    int32_t seq = motors_data[i].index;
                    char buf[16];
                    snprintf(buf, sizeof(buf), "%d", (int)seq);   // ????
                    ack_with_info(buf, NULL);  
                    motors_data[i].step_to_home=0;
                }
            }
                break;

            case MOTOR_STATE_INIT:
            {
                motor_state_init(i);
            }
                break;

            case MOTOR_STATE_SETDIR:
            {
                if (motors_data[i].move == MOTOR_MOVE_HOME) {
                    motors_data[i].state = MOTOR_STATE_HOMEBWD;
                    motors_data[i].step_remain = Motor_Home_Max;
                } else if ((motors_data[i].move == MOTOR_MOVE_FWD) ||
                        (motors_data[i].move == MOTOR_MOVE_BWD)) {
                    motors_data[i].state = MOTOR_STATE_MOVE;
                }
                motor_local_setDirAndWakeup(i);
            }
                break;

            case MOTOR_STATE_HOMEBWD: /*home*/
            {
                if (motor_chkhome_stepout(i))
                    break;
                if (PORT_PinRead(motors_data[i].det_pin)) {
                    motor_move(i);                    
                } else {
                    motors_data[i].state = MOTOR_STATE_HOMEFWD;
                    //set fwd
                    if (i == MOTORID_REFARM) {
                        PORT_PinWrite(motors_data[i].dir_pin, motors_data[i].fwd_dir_state);
                    } else if (i == MOTORID_PIRIS) {
                        PORT_PinWrite(motors_data[i].dir_pin, !motors_data[i].fwd_dir_state);
                    }
                }
            }
                break;

            case MOTOR_STATE_HOMEFWD:
            {
                if (motor_chkhome_stepout(i))
                    break;

                if (!PORT_PinRead(motors_data[i].det_pin)) {
                    motor_move(i);
                } else {
                    //finish home
                    motors_data[i].state = MOTOR_STATE_IDLE;
                    motors_data[i].status_reg.motor_busy = 0;
                    motors_data[i].move = MOTOR_MOVE_IDLE;
                    motors_data[i].pos_current = 0;
                    motor_local_standby(i);
                    int32_t seq = motors_data[i].index;
                    char buf[16];
                    char info_buf[32];
                    snprintf(buf, sizeof(buf), "%d", (int)seq);   
                    snprintf(info_buf, sizeof(info_buf), "%lu", (unsigned long)motors_data[i].step_to_home);
                    ack_with_info(buf, info_buf);
                    motors_data[i].step_to_home=0;
                }
            }
                break;

            case MOTOR_STATE_IDLE:
                break;
        }
    }
}
/**
 * @brief   Initialize motor state.
 * @details Resets motor state and sets it to idle mode.
 *
 * @param[in] id  Motor ID.
 */
void motor_state_init(int id) {
    motors_data[id].state = MOTOR_STATE_IDLE;
    motors_data[id].status_reg.motor_busy = 0;
    motor_local_standby(id);
}
/**
 * @brief   Initialize motor configuration data.
 * @details Sets default parameters for each motor, including pins,
 *          direction polarity, standby/active states, and timing interval.
 */
void motor_data_init() {
    //// Ref arm motor
    motors_data[MOTORID_REFARM].state = MOTOR_STATE_INIT;
    motors_data[MOTORID_REFARM].pos_current = 0;
    motors_data[MOTORID_REFARM].fwd_dir_state = false;
    motors_data[MOTORID_REFARM].step_pin = REFMOT_STEP_PIN;
    motors_data[MOTORID_REFARM].dir_pin = REFMOT_DIR_PIN;
    motors_data[MOTORID_REFARM].i0_pin = REFMOT_I0_PIN;
    motors_data[MOTORID_REFARM].i1_pin = REFMOT_I1_PIN;
    motors_data[MOTORID_REFARM].det_pin = REFARM_DET_PIN;
    motors_data[MOTORID_REFARM].stdby_i0_state = false; 
    motors_data[MOTORID_REFARM].stdby_i1_state = true; 
    motors_data[MOTORID_REFARM].active_i0_state = false;
    motors_data[MOTORID_REFARM].active_i1_state = false;
    motors_data[MOTORID_REFARM].step_interval = 40; //40 = 1ms
    motors_data[MOTORID_REFARM].step_to_home = 0; //40 = 1ms
    //// P ctrl motor
    motors_data[MOTORID_PIRIS].state = MOTOR_STATE_INIT;
    motors_data[MOTORID_PIRIS].pos_current = 0;
    motors_data[MOTORID_PIRIS].fwd_dir_state = true;
    motors_data[MOTORID_PIRIS].step_pin = PC_STEP_PIN;
    motors_data[MOTORID_PIRIS].dir_pin = PC_DIR_PIN;
    motors_data[MOTORID_PIRIS].i0_pin = PC_I0_PIN;
    motors_data[MOTORID_PIRIS].i1_pin = PC_I1_PIN;
    motors_data[MOTORID_PIRIS].det_pin = P_CTRL_DET_PIN;
    motors_data[MOTORID_PIRIS].stdby_i0_state = false; 
    motors_data[MOTORID_PIRIS].stdby_i1_state = true; 
    motors_data[MOTORID_PIRIS].active_i0_state = false;
    motors_data[MOTORID_PIRIS].active_i1_state = false;
    motors_data[MOTORID_PIRIS].step_interval = 40;
    motors_data[MOTORID_PIRIS].step_to_home = 0;
}
/**
 * @brief   Start motor homing sequence.
 *
 * @param[in] id  Motor ID.
 *
 * @return true  If command accepted.
 * @return false If motor is busy.
 */
bool motorhome(uint32_t id) {
    bool ret = true;
    if (motors_data[id].status_reg.motor_busy) {
        ret = false;
        return ret;
    }
    motors_data[id].move = MOTOR_MOVE_HOME;
    motors_data[id].state = MOTOR_STATE_SETDIR;
    motors_data[id].step_to_home = 0;
    motors_data[id].status_reg.motor_busy = 1;
    return ret;
}
/**
 * @brief   Move motor by relative steps.
 *
 * @param[in] id     Motor ID.
 * @param[in] steps  Number of steps (positive or negative).
 *
 * @return true  If command accepted.
 * @return false If motor is busy.
 */
bool motormove(uint32_t id, int32_t steps) {
    bool ret = true;
    if (motors_data[id].status_reg.motor_busy) {
        ret = false;
        return ret;
    }

    if (steps < 0) {
        motors_data[id].step_remain = -steps;
        motors_data[id].move = MOTOR_MOVE_BWD;
    } else if (steps >= 0) {
        motors_data[id].step_remain = steps;
        motors_data[id].move = MOTOR_MOVE_FWD;
    }
    motors_data[id].state = MOTOR_STATE_SETDIR;
    motors_data[id].status_reg.motor_busy = 1;
    return ret;
}
/**
 * @brief   Move motor to absolute position.
 *
 * @param[in] id   Motor ID.
 * @param[in] pos  Target absolute position.
 *
 * @return true  If command accepted.
 * @return false If motor is busy.
 */
bool motormoveabs(uint32_t id, int32_t pos) {
    bool ret = true;
    if (motors_data[id].status_reg.motor_busy) {
        ret = false;
        return ret;
    }
    int32_t steps = pos - motors_data[id].pos_current;
    return motormove(id, steps);
}
/**
 * @brief   Reset motor position counter.
 *
 * @param[in] id  Motor ID.
 */
void motorresetpos(uint32_t id) {
    motors_data[id].pos_current = 0;
}