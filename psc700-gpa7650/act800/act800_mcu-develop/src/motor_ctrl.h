/**
 * @file    motor_ctrl.h
 * @brief   Motor control interface definitions.
 * @details This file declares the data types, status definitions, macros,
 *          and public APIs for motor control. It supports stepper motor
 *          movement, homing, direction control, position tracking, and
 *          task-based state machine execution.
 * @ingroup motor_controller
 */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "proc_cmd.h"
#include "config/default/peripheral/port/plib_port.h"
#include "config/default/peripheral/tcc/plib_tcc1.h"

#ifndef MOTOR_CTRL_H
#define	MOTOR_CTRL_H

#ifdef	__cplusplus
extern "C" {
#endif
#define MOTOR_COUNT         2
#define MOTORID_REFARM      0
#define MOTORID_PIRIS       1
#define Motor_Home_Max  40000
typedef void (*SS_FUNC)();
typedef enum{
    MOTOR_MOVE_IDLE,
    MOTOR_MOVE_HOME,
    MOTOR_MOVE_FWD,
    MOTOR_MOVE_BWD
}MOTOR_MOVE;
typedef enum{
    MOTOR_STATE_IDLE,
    MOTOR_STATE_INIT,
    MOTOR_STATE_SETDIR,
    MOTOR_STATE_MOVE,
    MOTOR_STATE_HOMEBWD,
    MOTOR_STATE_HOMEFWD
}MOTOR_STATE;
typedef union{
    uint8_t status;
    struct{
        unsigned motor_busy : 1;
        unsigned  : 3;
        unsigned llimit : 1;
        unsigned hlimit : 1;
        unsigned homerr : 1;
        unsigned  : 1;
    }__attribute__ ((packed));
}MOTOR_STATUS;

typedef struct{
     /*** configure ***/   
    bool fwd_dir_state;        
    bool stdby_i0_state;
    bool stdby_i1_state;
    bool active_i0_state;
    bool active_i1_state;
     
    
    /*** gpio pins ***/
    uint32_t step_pin;
    uint32_t dir_pin;
    uint32_t i0_pin;
    uint32_t i1_pin;
    uint32_t det_pin;
    
    
    MOTOR_STATE state;          
    MOTOR_MOVE move;            
    
    int32_t pos_current;        
    int32_t step_remain;        
    MOTOR_STATUS status_reg;
    uint32_t last_ctrltick;     
    uint32_t step_interval;     
    
    int32_t index;
    uint32_t step_to_home; 

}MOTOR_DATA;
extern MOTOR_DATA motors_data[MOTOR_COUNT];
extern uint32_t _12p5Us_tick;
void MOTOR_Initialize(void);
void MOTOR_Tasks();
void motor_data_init();
void motor_move(int id);
bool motormove(uint32_t id, int32_t steps);
bool motormoveabs(uint32_t id, int32_t pos);
void motorresetpos(uint32_t id);
bool motorhome(uint32_t id);
#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_CTRL_H */

