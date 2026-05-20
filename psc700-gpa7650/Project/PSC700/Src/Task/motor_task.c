/** @file motor_task.c
 *  @brief Autofocus motor task source file
 *
 *  This task is responsible for the Autofocus motor control.
 */
/*******************************************************************
 * Includes
 *******************************************************************/
#include "cmsis_os.h"

#include "motor_task.h"
#include "wt7026.h"
#include "psc700_conf.h"
/*******************************************************************
 * Constants
 *******************************************************************/
#define MOTOR_TASK_TAG                          "MOTOR_TASK"

#define MOTOR_Q_NUM                             3
#define MOTOR_AF_MAX_STEP                       (700/PHASE_COUNT_PER_STEP)
#define BACKLASH                                (100/PHASE_COUNT_PER_STEP)
#define IR_VIS_COMPENSATION                     (0)
#define AF_STEP                                 (2)
/*******************************************************************
 * Data Types
 *******************************************************************/
typedef enum{
    MOTOR_EVENT_IDLE = 0,
    MOTOR_EVENT_START,
    MOTOR_EVENT_SET_POS,
    MOTOR_EVENT_HOME,
    MOTOR_EVENT_REL_MOVE,
    MOTOR_EVENT_ABS_MOVE,
}MotorEvent_Typedef;
typedef struct{
    INT8U busy;
    INT8U ready;
    INT32U pos;
}MotorParam_Typedef;
typedef struct{
    MotorEvent_Typedef evt;
    INT32U new_pos;
}MotorMsg_Typedef;
/*******************************************************************
 * Public Variables
 *******************************************************************/
extern SemaphoreHandle_t frame_collected_sem;
/*******************************************************************
 * Private Variables
 *******************************************************************/
static osThreadId task_id = NULL;
static osMessageQId motor_msg_q_id, motor_msg_free_q_id;
static MotorParam_Typedef motor_param;
/*******************************************************************
 * Private Tasks
 *******************************************************************/
/**
  *@brief Motor Task
  *@param *param: Input parameter pointer
  *@retval None
  */
void motor_task(const void *param)
{
    osEvent evt;
    MotorMsg_Typedef *motor_msg;
    INT32U i;
    const TickType_t frame_wait_timeout = pdMS_TO_TICKS(100); // Define timeout for waiting frame synchronization (100 ms)
    //To be implemented by MIIS
    wt7026_init();
    while(1)
    {
        if(motor_msg_q_id != NULL)
        {
            evt = osMessageGet(motor_msg_q_id, osWaitForever);
            if(evt.status != osEventMessage) continue;
            if(evt.value.v == NULL) continue;

            motor_msg = evt.value.v;

            switch(motor_msg->evt)
            {
                case MOTOR_EVENT_START:
                    DBG_PRINT("[%s][LOG][MOTOR_EVENT_START]\r\n", MOTOR_TASK_TAG);
                    motor_param.busy = 1;
                    // Wait until the first frame is captured before starting motor movement (frame #0)
                    xSemaphoreTake(frame_collected_sem, frame_wait_timeout);
                    // DBG_PRINT("F");
                    for( INT8U i = 0; i < PSC700_LIVEVIEW_COL_NUM-1; i++) {
                        wt7026_move(AF_STEP);
                        // Wait for the next frame capture notification before moving again (frame #1-29)
                        xSemaphoreTake(frame_collected_sem, frame_wait_timeout);
                        // DBG_PRINT("f");
                    }
                    motor_param.busy = 0;
                    break;
                case MOTOR_EVENT_SET_POS:
                    DBG_PRINT("[%s][LOG][MOTOR_EVENT_SET_POS] New Pos[%d]\r\n", MOTOR_TASK_TAG, motor_msg->new_pos);
                    wt7026_goto(motor_msg->new_pos-BACKLASH-IR_VIS_COMPENSATION);
                    osDelay(1);
                    wt7026_goto(motor_msg->new_pos-IR_VIS_COMPENSATION);
                    motor_param.ready = 1;
                    break;
                case MOTOR_EVENT_HOME:
                    DBG_PRINT("[%s][LOG][MOTOR_EVENT_HOME]\r\n", MOTOR_TASK_TAG);
                    motor_param.busy = 1;
                    INT32U home_steps = wt7026_home();
                    console_task_tx_send("+AFH %u\r\n", home_steps);
                    motor_param.busy = 0;
                    break;
                case MOTOR_EVENT_REL_MOVE:
                    DBG_PRINT("[%s][LOG][MOTOR_EVENT_REL_MOVE] Relative Pos[%d]\r\n", MOTOR_TASK_TAG, motor_msg->new_pos);
                    motor_param.busy = 1;
                    if (wt7026_move(motor_msg->new_pos))
                        psc700_uart_cmd_resp_err();
                    else
                        psc700_uart_cmd_resp_ok();
                    motor_param.busy = 0;
                    break;
                case MOTOR_EVENT_ABS_MOVE:
                    DBG_PRINT("[%s][LOG][MOTOR_EVENT_ABS_MOVE] Absolute Pos[%d]\r\n", MOTOR_TASK_TAG, motor_msg->new_pos);
                    motor_param.busy = 1;
                    if (wt7026_goto(motor_msg->new_pos))
                        psc700_uart_cmd_resp_err();
                    else
                        psc700_uart_cmd_resp_ok();
                    motor_param.busy = 0;
                    break;
            }
            osMessagePut(motor_msg_free_q_id, &evt.value.v, 1);
        }
        else
        {
            osDelay(1);
        }
    }
}
/*******************************************************************
 * Public Functions
 *******************************************************************/
/**
  *@brief Motor Task Entry
  *@param stack_size: Stack size for task
  *@retval None
  */
void motor_task_entry(INT32U stack_size)
{
    INT32U i, buf;

    if(motor_msg_q_id == NULL)
    {
        osMessageQDef(msg_q, (MOTOR_Q_NUM + 1), INT32U);
        motor_msg_q_id = osMessageCreate(&os_messageQ_def_msg_q, NULL);
        if(motor_msg_q_id == NULL)
        {
            DBG_PRINT("[%s][ERROR] Create Motor Q Failed\r\n", MOTOR_TASK_TAG);
            while(1);
        }
    }
    if(motor_msg_free_q_id == NULL)
    {
        osMessageQDef(msg_q, (MOTOR_Q_NUM + 1), INT32U);
        motor_msg_free_q_id = osMessageCreate(&os_messageQ_def_msg_q, NULL);
        if(motor_msg_free_q_id == NULL)
        {
            DBG_PRINT("[%s][ERROR] Create Motor Free Q Failed\r\n", MOTOR_TASK_TAG);
            while(1);
        }

        buf = (INT32U)gp_malloc_align((sizeof(MotorMsg_Typedef) * MOTOR_Q_NUM), 4);
        if(buf != NULL)
        {
            for(i=0;i<MOTOR_Q_NUM;i++)
            {
                osMessagePut(motor_msg_free_q_id, &buf, 1);
                buf += sizeof(MotorMsg_Typedef);
            }
        }
        else
        {
            DBG_PRINT("[%s][ERROR] Allocate Motor Failed\r\n", MOTOR_TASK_TAG);
        }
    }

    if(task_id == NULL)
    {
        osThreadDef(motor_task, osPriorityNormal, 0, stack_size);
        task_id = osThreadCreate(&os_thread_def_motor_task, NULL);
        if(task_id == NULL)
        {
            DBG_PRINT("[%s][ERROR] Create Motor Task Failed\r\n", MOTOR_TASK_TAG);
        }
    }
}
/**
  *@brief Start motor moving
  *@param None
  *@retval Result (0:Success Others:Failed)
  */
INT8U motor_task_start(void)
{
    osEvent evt;
    MotorMsg_Typedef *motor_start_msg;

    if(motor_param.busy) return 1;

    evt = osMessageGet(motor_msg_free_q_id, 1);
    if(evt.status == osEventMessage)
    {
        if(evt.value.v != NULL)
        {
            motor_start_msg = evt.value.v;
            motor_start_msg->evt = MOTOR_EVENT_START;
            osMessagePut(motor_msg_q_id, &evt.value.v, 1);
            return 0;
        }
    }

    return 1;
}
/**
  *@brief Get motor's current position
  *@param None
  *@retval Motor's current position
  */
INT32U motor_task_get_cur_pos(void)
{
    // return motor_param.pos++; //For Testing
    return wt7026_act.cur_pos;
}
/**
  *@brief Set motor's new position
  *@param new_pos: New position
  *@retval Result (0:Success Others:Failed)
  */
INT8U motor_task_set_pos(INT32U new_pos)
{
    osEvent evt;
    MotorMsg_Typedef *motor_set_pos_msg;

    if(motor_param.busy) return 1;

    evt = osMessageGet(motor_msg_free_q_id, 1);
    if(evt.status == osEventMessage)
    {
        if(evt.value.v != NULL)
        {
            motor_param.ready = 0;
            motor_set_pos_msg = evt.value.v;
            motor_set_pos_msg->evt = MOTOR_EVENT_SET_POS;
            motor_set_pos_msg->new_pos = new_pos;
            osMessagePut(motor_msg_q_id, &evt.value.v, 1);
            return 0;
        }
    }
    return 1;

}
/**
  *@brief Get motor's ready status
  *@param None
  *@retval Status (0:Not Ready 1:Ready)
  */
INT8U motor_task_is_ready(void)
{
    return motor_param.ready;
}
/**
  *@brief Get motor's busy status
  *@param None
  *@retval Status (0:Not busy 1:Busy)
  */
INT8U motor_task_is_busy(void)
{
    return motor_param.busy;
}
/**
  *@brief Start motor moving to home
  *@param None
  *@retval Result (0:Success Others:Failed)
  */
INT8U motor_task_home(void)
{
    osEvent evt;
    MotorMsg_Typedef *motor_start_msg;

    if(motor_param.busy) return 1;

    evt = osMessageGet(motor_msg_free_q_id, 1);
    if(evt.status == osEventMessage)
    {
        if(evt.value.v != NULL)
        {
            motor_param.busy = 1;
            motor_start_msg = evt.value.v;
            motor_start_msg->evt = MOTOR_EVENT_HOME;
            osMessagePut(motor_msg_q_id, &evt.value.v, 1);
            return 0;
        }
    }

    return 1;
}

/**
  *@brief Start motor moving to relative position (Asynchronous)
  *@param rel_pos: relative steps to move from current position
  *@retval Result (0:Success Others:Failed)
  */
INT8U motor_task_relative_move(INT32S rel_pos)
{
    osEvent evt;
    MotorMsg_Typedef *motor_move_rel_msg;

    if(motor_param.busy) return 1;

    evt = osMessageGet(motor_msg_free_q_id, 1);
    if(evt.status == osEventMessage)
    {
        if(evt.value.v != NULL)
        {
            motor_param.busy = 1;
            motor_move_rel_msg = evt.value.v;
            motor_move_rel_msg->evt = MOTOR_EVENT_REL_MOVE;
            motor_move_rel_msg->new_pos = rel_pos;
            osMessagePut(motor_msg_q_id, &evt.value.v, 1);
            return 0;
        }
    }

    return 1;
}
/**
  *@brief Start motor moving to absolute position (Asynchronous)
  *@param abs_pos: target absolute position to reach
  *@retval Result (0:Success Others:Failed)
  */
INT8U motor_task_absolute_move(INT32S abs_pos)
{
    osEvent evt;
    MotorMsg_Typedef *motor_move_abs_msg;

    if(motor_param.busy) return 1;

    evt = osMessageGet(motor_msg_free_q_id, 1);
    if(evt.status == osEventMessage)
    {
        if(evt.value.v != NULL)
        {
            motor_param.busy = 1;
            motor_move_abs_msg = evt.value.v;
            motor_move_abs_msg->evt = MOTOR_EVENT_ABS_MOVE;
            motor_move_abs_msg->new_pos = abs_pos;
            osMessagePut(motor_msg_q_id, &evt.value.v, 1);
            return 0;
        }
    }

    return 1;
}