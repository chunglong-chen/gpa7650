/** @file af_task.c
 *  @brief Autofocus task source file
 *
 *  This task is responsible for the autofocus workflow.
 */
/*******************************************************************
 * Includes
 *******************************************************************/
#include "cmsis_os.h"

#include "af_task.h"
#include "af_alg.h"
#include "psc700_conf.h"
/*******************************************************************
 * Constants
 *******************************************************************/
#define AF_TASK_TAG                             "AF_TASK"

#define AF_Q_NUM                                3
#define AF_USE_CORRELATION                      0 // 0 for Peak search; 1 for Correlation search
/*******************************************************************
 * Data Types
 *******************************************************************/
typedef enum{
    AF_EVENT_IDLE = 0,
    AF_EVENT_START,
}AFEvent_Typedef;
typedef struct{
    INT8U busy;

    void (*complete_cb)(INT32U pos);
}AFParam_Typedef;
typedef struct{
    AFEvent_Typedef evt;
    AFInfo_Typedef info;
}AFMsg_Typedef;
/*******************************************************************
 * Private Variables
 *******************************************************************/
static osThreadId task_id = NULL;
static osMessageQId af_msg_q_id, af_msg_free_q_id;
static AFParam_Typedef af_param;
/*******************************************************************
 * Private Tasks
 *******************************************************************/
/**
  *@brief AF Task
  *@param *param: Input parameter pointer
  *@retval None
  */
void af_task(const void *param)
{
    osEvent evt;
    AFMsg_Typedef *af_msg;
    INT32U i;

    //To be implemented by MIIS
    while(1)
    {
        if(af_msg_q_id != NULL)
        {
            evt = osMessageGet(af_msg_q_id, osWaitForever);
            if(evt.status != osEventMessage) continue;
            if(evt.value.v == NULL) continue;

            af_msg = evt.value.v;

            switch(af_msg->evt)
            {
                case AF_EVENT_START:
                    DBG_PRINT("[%s][LOG][AF_EVENT_START] Num:%d\r\n", AF_TASK_TAG, af_msg->info.num);
                    INT8U num = af_msg->info.num;
                    // // Add padding at both ends to monitor for buffer overflow/underflow
                    // float sharpness_buffer[PSC700_LIVEVIEW_COL_NUM+8] = {0};
                    // // Offset the pointer to skip the front padding (4 floats)
                    // float* sharpness_list = sharpness_buffer + 4;
                    float sharpness_list[PSC700_LIVEVIEW_COL_NUM];
                    INT32U max_idx = 0;
                    float max_sharpness = -1.0f;
                    for (INT32U i = 0; i < af_msg->info.num; i++) {
                        INT32U frame_addr = af_msg->info.frm_buf[i];
                        INT32U motor_pos  = af_msg->info.mtr_pos[i];
                        sharpness_list[i] = af_task_calc_sharpness((INT8U *)frame_addr, PSC700_LIVEVIEW_WIDTH, PSC700_LIVEVIEW_HEIGHT);
                        // DBG_PRINT("%u,%u,%f\r\n", i, motor_pos, sharpness);
                        console_task_tx_send("%u,%u,%f\r\n", i, motor_pos, sharpness_list[i]);

                        if (sharpness_list[i] > max_sharpness) {
                            max_sharpness = sharpness_list[i];
                            max_idx = i;
                        }
                    }
                    // Check if the front padding values were corrupted during calculation
                    // console_task_tx_send("[SharpnessBuf 0-4] %f,%f,%f,%f\r\n", sharpness_buffer[0]
                    //                                    ,sharpness_buffer[1]
                    //                                    ,sharpness_buffer[2]
                    //                                    ,sharpness_buffer[3]);
                    INT32U best_motor_pos = 0;

#if AF_USE_CORRELATION == 1
                    // --- Correlation (Support Sub-index) ---
                    float best_index = af_task_find_best_sub_index_by_correlation(sharpness_list, num);
                    float start_pos = (float)af_msg->info.mtr_pos[0];
                    float step_size = (float)af_msg->info.mtr_pos[1] - start_pos;
                    best_motor_pos = (INT32U)(start_pos + (best_index * step_size) + 0.5f);

                    console_task_tx_send("AF Mode: Correlation, Best Idx: %.1f, Pos: %d\r\n", best_index, best_motor_pos);
#else
                    // --- Peak Search ---
                    best_motor_pos = af_msg->info.mtr_pos[max_idx];

                    console_task_tx_send("AF Mode: Peak Search, Best Idx: %d, Pos: %d\r\n", max_idx, best_motor_pos);
#endif

                    if(af_param.complete_cb)
                    {
                        // af_param.complete_cb(af_msg->info.mtr_pos[best_index]); //apply motor position
                        af_param.complete_cb(best_motor_pos); //apply motor position
                    }
                    break;
            }
            osMessagePut(af_msg_free_q_id, &evt.value.v, 1);
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
  *@brief AF Task Entry
  *@param stack_size: Stack size for task
  *@retval None
  */
void af_task_entry(INT32U stack_size)
{
    INT32U i, buf;

    gp_memset(&af_param, 0, sizeof(AFParam_Typedef));

    if(af_msg_q_id == NULL)
    {
        osMessageQDef(msg_q, (AF_Q_NUM + 1), INT32U);
        af_msg_q_id = osMessageCreate(&os_messageQ_def_msg_q, NULL);
        if(af_msg_q_id == NULL)
        {
            DBG_PRINT("[%s][ERROR] Create AF Q Failed\r\n", AF_TASK_TAG);
            while(1);
        }
    }
    if(af_msg_free_q_id == NULL)
    {
        osMessageQDef(msg_q, (AF_Q_NUM + 1), INT32U);
        af_msg_free_q_id = osMessageCreate(&os_messageQ_def_msg_q, NULL);
        if(af_msg_free_q_id == NULL)
        {
            DBG_PRINT("[%s][ERROR] Create AF Free Q Failed\r\n", AF_TASK_TAG);
            while(1);
        }

        buf = (INT32U)gp_malloc_align((sizeof(AFMsg_Typedef) * AF_Q_NUM), 4);
        if(buf != NULL)
        {
            for(i=0;i<AF_Q_NUM;i++)
            {
                osMessagePut(af_msg_free_q_id, &buf, 1);
                buf += sizeof(AFMsg_Typedef);
            }
        }
        else
        {
            DBG_PRINT("[%s][ERROR] Allocate AF Failed\r\n", AF_TASK_TAG);
        }
    }

    if(task_id == NULL)
    {
        osThreadDef(af_task, osPriorityNormal, 0, stack_size);
        task_id = osThreadCreate(&os_thread_def_af_task, NULL);
        if(task_id == NULL)
        {
            DBG_PRINT("[%s][ERROR] Create AF Task Failed\r\n", AF_TASK_TAG);
        }
    }
}
/**
  *@brief Start AF
  *@param info: Information for AF algorithm
  *@retval Result (0:Success Others:Failed)
  */
INT8U af_task_start(AFInfo_Typedef info)
{
    osEvent evt;
    AFMsg_Typedef *af_start_msg;

    if(af_param.busy) return 1;

    evt = osMessageGet(af_msg_free_q_id, 1);
    if(evt.status == osEventMessage)
    {
        if(evt.value.v != NULL)
        {
            af_start_msg = evt.value.v;
            af_start_msg->evt = AF_EVENT_START;
            gp_memcpy(&af_start_msg->info, &info, sizeof(AFInfo_Typedef));
            osMessagePut(af_msg_q_id, &evt.value.v, 1);
            return 0;
        }
    }
    return 1;
}
/**
  *@brief Register a callback to get AF completed event
  *@param *callback: To be registered callback
  *@retval None
  */
void af_task_register_af_complete(void (*callback)(INT32U pos))
{
    af_param.complete_cb = callback;
}
