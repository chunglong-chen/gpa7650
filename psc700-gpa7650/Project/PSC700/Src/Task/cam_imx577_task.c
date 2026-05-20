/** @file cam_imx577_task.c
 *  @brief IMX577 sensor task source file
 *
 *  This task is responsible for controlling Sony IMX577 and flash LED.
 *  It provides frames for UVC, collects frames for the AF algorithm and captures an
 *  image with/without the flash LED.
 */
/*******************************************************************
 * Includes
 *******************************************************************/
#include "cmsis_os.h"
#include "drv_l1_pscaler.h"
#include "drv_l1_cdsp.h"
#include "drv_l1_binmode.h"
#include "drv_l1_gpio.h"
#include "drv_l1_timer.h"
#include "drv_l2_sensor.h"
#include "psc700_conf.h"
#include "cam_imx577_task.h"
/*******************************************************************
 * Constants
 *******************************************************************/
#define IMX577_TASK_TAG                     "IMX577_TASK"
#define PSCALER_ID                          PSCALER_A
#define MSG_Q_NUM                           10
#define LIV_Q_NUM                           33
#define COL_Q_NUM                           31
#define CAP_Q_NUM                           3
#define LIV_FRAME_SIZE                      (PSC700_LIVEVIEW_WIDTH * PSC700_LIVEVIEW_HEIGHT * 2)
#define CAP_WIDTH                           4056
#define CAP_HEIGHT                          3040
#define CAP_FRAME_SIZE                      (4056 * 3040 * 10 / 8)

#define FLASH_LED_TIMER                     TIMER_C
#define VR_PW_EN                            IO_C9
#define VR_LED_ON                           IO_C10

/*******************************************************************
 * Data Type
 *******************************************************************/
typedef enum{
    IMX577_EVENT_IDLE = 0,
    IMX577_EVENT_READY,
    IMX577_EVENT_POST_FRAME,
    INX577_EVENT_CTRL_STREAMING,
    IMX577_EVENT_CTRL_START_COLLECT,
    IMX577_EVENT_CTRL_COLLECT_COMPLETE,
    IMX577_EVENT_CTRL_CAPTURE,
    IMX577_EVENT_MAX,
}IMX577Event_Typedef;
typedef enum{
    IMX577_CAPTURE_IDLE = 0,
    IMX577_CAPTURE_START,
    IMX577_CAPTURE_COMPLETE,
    IMX577_CAPTURE_MAX,
}IMX577Capture_Typedef;
typedef struct{
    IMX577Mode_Typedef mode;

    INT8U is_ready;

    INT32U drop_cnt;
    INT32U drop_num;

    INT32U col_liv_buf[COL_Q_NUM];
    INT32U col_liv_len;
    INT32U col_mtr[COL_Q_NUM];
    INT32U col_ptr;
    INT32U col_cnt;
    INT32U col_num;
    INT8U col_backup_cnt;

    IMX577Capture_Typedef capture;

    INT32U cap_buf[CAP_Q_NUM];
    INT32U cap_len;
    INT8U cap_ptr;
    INT8U cap_cnt;
    INT8U cap_num;

    void (*get_ready)(void);
    void (*get_liv)(INT32U addr, INT32U len);
    void (*collect_complete)(void);
    void (*capture_complete)(INT32U addr, INT32U len);
    INT32U (*get_motor_pos)(void);
}IMX577Param_Typedef;
typedef struct{
    IMX577Event_Typedef evt;
    INT32U frm_buf;
    IMX577Res_Typedef res;
    INT8U cap_num;
    INT8U flash_led_en;
    INT32U flash_led_delay_tm;
    INT32U flash_led_on_tm;
}IMX577Msg_Typedef;
/*******************************************************************
 * Public Variables
 *******************************************************************/
SemaphoreHandle_t frame_collected_sem;
/*******************************************************************
 * Private Variables
 *******************************************************************/
static osThreadId task_id = NULL;
static osMessageQId imx577_msg_q_id = NULL, imx577_msg_free_q_id = NULL;
static osMessageQId imx577_liv_free_q_id = NULL;
static osMessageQId imx577_col_frm_q_id = NULL, imx577_col_motor_q_id = NULL;

static IMX577Param_Typedef imx577_param;

static osThreadId imx577_col_backup_task_id = NULL;
static osMessageQId imx577_col_backup_q_id = NULL;
static INT32U imx577_col_backup_buf[COL_Q_NUM] = {0};
/*******************************************************************
 * Declare Functions
 *******************************************************************/
/**
  *@brief Flash LED timer ISR callback
  *@param None
  *@retval None
  */
static void flash_led_timer_isr_callback(void);
/**
  *@brief Initialize IMX577
  *@param None
  *@retval None
  */
static void imx577_init(void);
/**
  *@brief Initialize collection parameters
  *@param None
  *@retval None
  */
static void imx577_col_init(void);
/**
  *@brief Release collected frames in queue
  *@param None
  *@retval None
  */
static void imx577_col_frm_release(void);
/**
  *@brief Initialize Flash LED
  *@param None
  *@retval None
  */
static void flash_led_init(void);
/**
  *@brief Flash LED control
  *@param on_off: Turn on/off (0:OFF Others:ON)
  *@retval None
  */
static void flash_led_ctrl(INT8U on_off);
/*******************************************************************
 * Private Tasks
 *******************************************************************/
 /**
  *@brief IMX577 Backup Task for deep copying collected frames
  *@param *param: Input Parameter Pointer
  *@retval None
  */
static void imx577_collect_backup_task(const void *param)
{
    osEvent evt;
    INT32U current_ptr = 0;

    // Allocate memory for backup buffers
    for (int i = 0; i < COL_Q_NUM; i++) {
        if (imx577_col_backup_buf[i] == 0) {
            // Use aligned allocation for better DMA/ISP performance, size defined by LIV_FRAME_SIZE
            imx577_col_backup_buf[i] = (INT32U)gp_malloc_align(LIV_FRAME_SIZE, 4);
            if (imx577_col_backup_buf[i] == 0) {
                DBG_PRINT("[BACK_COL][ERROR] Allocate Backup Buffer %d Failed\r\n", i);
                while(1);
            }
        }
    }

    // Initialize the backup message queue if not already created
    if (imx577_col_backup_q_id == NULL) {
        osMessageQDef(backup_q, (COL_Q_NUM + 1), INT32U);
        imx577_col_backup_q_id = osMessageCreate(&os_messageQ_def_backup_q, NULL);
        if(imx577_col_backup_q_id == NULL)
        {
            DBG_PRINT("[BACK_COL][ERROR] Create Backup Queue Failed\r\n");
            while(1);
        }
    }

    while(1)
    {
        // Block and wait for frame addresses sent from pscaler_isr_callback via xQueueSendFromISR
        if (imx577_col_backup_q_id != NULL)
        {
            evt = osMessageGet(imx577_col_backup_q_id, osWaitForever);
            if (evt.status == osEventMessage)
            {
                INT32U src_addr = evt.value.v;

                // Perform deep copy at task level to avoid blocking ISR
                if (imx577_col_backup_buf[current_ptr] != 0)
                {
                    gp_memcpy((void *)imx577_col_backup_buf[current_ptr], (void *)src_addr, LIV_FRAME_SIZE);
                    // DBG_PRINT("B");
                    imx577_param.col_liv_buf[current_ptr] = imx577_col_backup_buf[current_ptr];
                    imx577_param.col_backup_cnt++;
                }
                current_ptr++;

                if (current_ptr >= imx577_param.col_num) current_ptr = 0;
            }
        }
        else
        {
            osDelay(1);
        }
    }
}
/**
  *@brief IMX577 Task
  *@param *param: Input Parameter Pointer
  *@retval None
  */
static void imx577_task(const void *param)
{
    osEvent evt;
    IMX577Msg_Typedef *imx577_msg;
    INT32U i;
    INT8U cap_cmplt;

    // Initial semaphore
    frame_collected_sem = xSemaphoreCreateCounting(255, 0);
    if (frame_collected_sem == NULL)
    {
        DBG_PRINT("[%s][ERROR] Create frame collected Semaphore Failed\r\n", IMX577_TASK_TAG);
    }

    imx577_init();

    imx577_col_init();

    flash_led_init();

    max5360_init();

    while(1)
    {
        if(imx577_msg_q_id != NULL)
        {
            evt = osMessageGet(imx577_msg_q_id, osWaitForever);
            if(evt.status == osEventMessage)
            {
                imx577_msg = evt.value.v;
                switch(imx577_msg->evt)
                {
                    case IMX577_EVENT_IDLE:
                        if(imx577_param.mode == IMX577_MODE_COLLECTED)
                        {
                            imx577_col_frm_release();
                        }
                        imx577_param.mode = IMX577_MODE_IDLE;
                        break;
                    case IMX577_EVENT_READY:
                        if(imx577_param.get_ready)
                        {
                            imx577_param.get_ready();
                        }
                        break;
                    case IMX577_EVENT_POST_FRAME:
                        if((imx577_msg->frm_buf) && (imx577_msg->frm_buf != DUMMY_BUFFER_ADDRS))
                        {
                            if(imx577_param.get_liv)
                            {
                                imx577_param.get_liv(imx577_msg->frm_buf, LIV_FRAME_SIZE);
                            }
                            if(imx577_liv_free_q_id != NULL)
                            {
                                osMessagePut(imx577_liv_free_q_id, &imx577_msg->frm_buf, 1);
                            }
                        }
                        break;
                    case INX577_EVENT_CTRL_STREAMING:
                        if(imx577_param.mode == IMX577_MODE_COLLECTED)
                        {
                            imx577_col_frm_release();
                        }
                        imx577_param.mode = IMX577_MODE_STREAMING;
                        break;
                    case IMX577_EVENT_CTRL_START_COLLECT:
                        if(imx577_param.mode == IMX577_MODE_COLLECTED)
                        {
                            imx577_col_frm_release();
                        }
                        imx577_param.mode = IMX577_MODE_COLLECT;
                        break;
                    case IMX577_EVENT_CTRL_COLLECT_COMPLETE:
                        imx577_param.mode = IMX577_MODE_COLLECTED;
                        if(imx577_param.collect_complete)
                        {
                            imx577_param.collect_complete();
                        }
                        break;
                    case IMX577_EVENT_CTRL_CAPTURE:
                        cam_imx577_task_ctrl_pre_capture();
                        if(imx577_msg->flash_led_en)
                        {
                            //Trigger flash led - Enabke strobe signal
                            imx577_flash_trigger();
                            //Wait for trigger is disabled
                            while(imx577_read_flash_trigger()) osDelay(1);
                            osDelay(imx577_msg->flash_led_delay_tm);
                            //Turn on flash led
                            flash_led_ctrl(1);
                            timer_msec_setup(FLASH_LED_TIMER, imx577_msg->flash_led_on_tm, 1, flash_led_timer_isr_callback);
                            //Start capturing
                            imx577_param.capture = IMX577_CAPTURE_START;
                            imx577_param.cap_ptr = 0;
                            imx577_param.cap_cnt = 0;
                            imx577_param.cap_num = imx577_msg->cap_num;
                            drv_l1_binmode_framebuf_A_set(BINMODE_B, imx577_param.cap_buf[imx577_param.cap_ptr++]);
                            drv_l1_binmode_framebuf_B_set(BINMODE_B, imx577_param.cap_buf[imx577_param.cap_ptr++]);
                            drv_l1_binmode_enable(BINMODE_B);
                            while(imx577_param.capture == IMX577_CAPTURE_START){osDelay(1);}
                            drv_l1_binmode_disable(BINMODE_B);
                            drv_l1_binmode_framebuf_A_set(BINMODE_B, DUMMY_BUFFER_ADDRS);
                            drv_l1_binmode_framebuf_B_set(BINMODE_B, DUMMY_BUFFER_ADDRS);
                            if(imx577_param.capture_complete)
                            {
                                imx577_param.capture_complete(imx577_param.cap_buf[0], imx577_param.cap_len);
                            }
#if (PSC700_IMX577_RES == PSC700_IMX577_RES_VGA_120FPS)
                            imx577_switch_vga_resolution_120fps();
#elif (PSC700_IMX577_RES == PSC700_IMX577_RES_VGA_50FPS)
                            imx577_switch_vga_resolution_50fps();
#elif (PSC700_IMX577_RES == PSC700_IMX577_RES_760_120FPS)
                            imx577_switch_760_resolution_120fps();
#elif (PSC700_IMX577_RES == PSC700_IMX577_RES_760_50FPS)
                            imx577_switch_760_resolution_50fps();
#endif
                        }
                        else
                        {
                            //Capture without flash led
                            imx577_param.capture = IMX577_CAPTURE_START;
                            imx577_param.cap_ptr = 0;
                            imx577_param.cap_cnt = 0;
                            imx577_param.cap_num = imx577_msg->cap_num;
                            drv_l1_binmode_framebuf_A_set(BINMODE_B, imx577_param.cap_buf[imx577_param.cap_ptr++]);
                            drv_l1_binmode_framebuf_B_set(BINMODE_B, imx577_param.cap_buf[imx577_param.cap_ptr++]);
                            drv_l1_binmode_enable(BINMODE_B);
                            while(imx577_param.capture == IMX577_CAPTURE_START){osDelay(1);}
                            drv_l1_binmode_disable(BINMODE_B);
                            drv_l1_binmode_framebuf_A_set(BINMODE_B, DUMMY_BUFFER_ADDRS);
                            drv_l1_binmode_framebuf_B_set(BINMODE_B, DUMMY_BUFFER_ADDRS);
                            if(imx577_param.capture_complete)
                            {
                                imx577_param.capture_complete(imx577_param.cap_buf[0], imx577_param.cap_len);
                            }
#if (PSC700_IMX577_RES == PSC700_IMX577_RES_VGA_120FPS)
                            imx577_switch_vga_resolution_120fps();
#elif (PSC700_IMX577_RES == PSC700_IMX577_RES_VGA_50FPS)
                            imx577_switch_vga_resolution_50fps();
#elif (PSC700_IMX577_RES == PSC700_IMX577_RES_760_120FPS)
                            imx577_switch_760_resolution_120fps();
#elif (PSC700_IMX577_RES == PSC700_IMX577_RES_760_50FPS)
                            imx577_switch_760_resolution_50fps();
#endif
                        }
                        break;
                }

                if(imx577_msg_free_q_id != NULL)
                {
                    osMessagePut(imx577_msg_free_q_id, &evt.value.v, 1);
                }
            }
        }
        else
        {
            osDelay(1);
        }
    }
}
/*******************************************************************
 * Private Callbacks
 *******************************************************************/
/**
  *@brief Pscaler callback
  *@param evt: IRQ event
  *@retval None
  */
static void pscaler_isr_callback(INT32U evt)
{
    osEvent ret;
    INT32U frm_addr, free_addr = DUMMY_BUFFER_ADDRS, motor_pos;
    IMX577Msg_Typedef *imx577_liv_msg;

    if(evt & PIPELINE_SCALER_STATUS_OVERFLOW_OCCUR)
    {
        DBG_PRINT("O");
    }
    if(evt & PIPELINE_SCALER_STATUS_BUF_A_DONE)
    {
        if(!imx577_param.is_ready)
        {
            imx577_param.is_ready = 1;
            if((imx577_msg_q_id != NULL) && (imx577_msg_free_q_id != NULL))
            {
                ret = osMessageGet(imx577_msg_free_q_id, 1);
                if(ret.status == osEventMessage)
                {
                    if(ret.value.v != NULL)
                    {
                        imx577_liv_msg = ret.value.v;
                        imx577_liv_msg->evt = IMX577_EVENT_READY;
                        osMessagePut(imx577_msg_q_id, &ret.value.v, 1);
                    }
                }
            }
        }
        frm_addr = drv_l1_pscaler_output_A_buffer_get(PSCALER_ID);
        if((frm_addr) && (frm_addr != DUMMY_BUFFER_ADDRS))
        {
            if(imx577_param.mode == IMX577_MODE_STREAMING)
            {
                if(imx577_param.drop_num) imx577_param.drop_cnt++;
                if(imx577_param.drop_cnt >= imx577_param.drop_num)
                {
                    //Post frame
                    if((imx577_msg_q_id != NULL) && (imx577_msg_free_q_id != NULL))
                    {
                        ret = osMessageGet(imx577_msg_free_q_id, 1);
                        if(ret.status == osEventMessage)
                        {
                            if(ret.value.v != NULL)
                            {
                                imx577_liv_msg = ret.value.v;
                                imx577_liv_msg->evt = IMX577_EVENT_POST_FRAME;
                                imx577_liv_msg->frm_buf = frm_addr;
                                osMessagePut(imx577_msg_q_id, &ret.value.v, 1);
                            }
                        }
                    }
                    imx577_param.drop_cnt = 0;
                }
                else
                {
                    //Drop frame
                    if(imx577_liv_free_q_id != NULL)
                    {
                        osMessagePut(imx577_liv_free_q_id, &frm_addr, 1);
                    }
                }
            }
            else if(imx577_param.mode == IMX577_MODE_COLLECT)
            {
                if((imx577_col_frm_q_id != NULL) && (imx577_col_motor_q_id != NULL))
                {
                    if(imx577_param.col_cnt < imx577_param.col_num)
                    {
                        DBG_PRINT("C");
                        osMessagePut(imx577_col_frm_q_id, &frm_addr, 1);
                        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                        xQueueSendFromISR(imx577_col_backup_q_id, &frm_addr, &xHigherPriorityTaskWoken);
                        if(imx577_param.get_motor_pos)
                        {
                            motor_pos = imx577_param.get_motor_pos();
                            imx577_param.col_mtr[imx577_param.col_ptr] = motor_pos;
                            osMessagePut(imx577_col_motor_q_id, &motor_pos, 1);
                        }
                        imx577_param.col_ptr++;
                        {
                            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                            // Give the semaphore from ISR to notify that one frame has been collected
                            xSemaphoreGiveFromISR(frame_collected_sem, &xHigherPriorityTaskWoken);
                            // If a higher priority task was unblocked by the semaphore give,
                            // request a context switch to run it immediately
                            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                        }
                        if(++imx577_param.col_cnt >= imx577_param.col_num)
                        {
                            //Collection completed
                            DBG_PRINT("D");
                            ret = osMessageGet(imx577_msg_free_q_id, 1);
                            if(ret.status == osEventMessage)
                            {
                                if(ret.value.v != NULL)
                                {
                                    imx577_liv_msg = ret.value.v;
                                    imx577_liv_msg->evt = IMX577_EVENT_CTRL_COLLECT_COMPLETE;
                                    osMessagePut(imx577_msg_q_id, &ret.value.v, 1);
                                }
                            }
                        }
                    }
                    else if(imx577_liv_free_q_id != NULL)
                    {
                        osMessagePut(imx577_liv_free_q_id, &frm_addr, 1);
                    }
                }
            }
            else
            {
                if(imx577_liv_free_q_id != NULL)
                {
                    osMessagePut(imx577_liv_free_q_id, &frm_addr, 1);
                }
            }
        }
        if(imx577_liv_free_q_id != NULL)
        {
            ret = osMessageGet(imx577_liv_free_q_id, 1);
            if(ret.status == osEventMessage)
            {
                free_addr = ret.value.v;
            }
        }
        drv_l1_pscaler_output_A_buffer_set(PSCALER_ID, free_addr);
    }
    if(evt & PIPELINE_SCALER_STATUS_BUF_B_DONE)
    {
        if(!imx577_param.is_ready)
        {
            imx577_param.is_ready = 1;
            if((imx577_msg_q_id != NULL) && (imx577_msg_free_q_id != NULL))
            {
                ret = osMessageGet(imx577_msg_free_q_id, 1);
                if(ret.status == osEventMessage)
                {
                    if(ret.value.v != NULL)
                    {
                        imx577_liv_msg = ret.value.v;
                        imx577_liv_msg->evt = IMX577_EVENT_READY;
                        osMessagePut(imx577_msg_q_id, &ret.value.v, 1);
                    }
                }
            }
        }
        frm_addr = drv_l1_pscaler_output_B_buffer_get(PSCALER_ID);
        if((frm_addr) && (frm_addr != DUMMY_BUFFER_ADDRS))
        {
            if(imx577_param.mode == IMX577_MODE_STREAMING)
            {
                if(imx577_param.drop_num) imx577_param.drop_cnt++;
                if(imx577_param.drop_cnt >= imx577_param.drop_num)
                {
                    //Post frame
                    if((imx577_msg_q_id != NULL) && (imx577_msg_free_q_id != NULL))
                    {
                        ret = osMessageGet(imx577_msg_free_q_id, 1);
                        if(ret.status == osEventMessage)
                        {
                            if(ret.value.v != NULL)
                            {
                                imx577_liv_msg = ret.value.v;
                                imx577_liv_msg->evt = IMX577_EVENT_POST_FRAME;
                                imx577_liv_msg->frm_buf = frm_addr;
                                osMessagePut(imx577_msg_q_id, &ret.value.v, 1);
                            }
                        }
                    }
                    imx577_param.drop_cnt = 0;
                }
                else
                {
                    //Drop frame
                    if(imx577_liv_free_q_id != NULL)
                    {
                        osMessagePut(imx577_liv_free_q_id, &frm_addr, 1);
                    }
                }
            }
            else if(imx577_param.mode == IMX577_MODE_COLLECT)
            {
                if((imx577_col_frm_q_id != NULL) && (imx577_col_motor_q_id != NULL))
                {
                    if(imx577_param.col_cnt < imx577_param.col_num)
                    {
                        DBG_PRINT("c");
                        osMessagePut(imx577_col_frm_q_id, &frm_addr, 1);
                        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                        xQueueSendFromISR(imx577_col_backup_q_id, &frm_addr, &xHigherPriorityTaskWoken);
                        if(imx577_param.get_motor_pos)
                        {
                            motor_pos = imx577_param.get_motor_pos();
                            imx577_param.col_mtr[imx577_param.col_ptr] = motor_pos;
                            osMessagePut(imx577_col_motor_q_id, &motor_pos, 1);
                        }
                        imx577_param.col_ptr++;
                        {
                            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                            // Give the semaphore from ISR to notify that one frame has been collected
                            xSemaphoreGiveFromISR(frame_collected_sem, &xHigherPriorityTaskWoken);
                            // If a higher priority task was unblocked by the semaphore give,
                            // request a context switch to run it immediately
                            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                        }
                        if(++imx577_param.col_cnt >= imx577_param.col_num)
                        {
                            //Collection completed
                            DBG_PRINT("d");
                            ret = osMessageGet(imx577_msg_free_q_id, 1);
                            if(ret.status == osEventMessage)
                            {
                                if(ret.value.v != NULL)
                                {
                                    imx577_liv_msg = ret.value.v;
                                    imx577_liv_msg->evt = IMX577_EVENT_CTRL_COLLECT_COMPLETE;
                                    osMessagePut(imx577_msg_q_id, &ret.value.v, 1);
                                }
                            }
                        }
                    }
                    else if(imx577_liv_free_q_id != NULL)
                    {
                        osMessagePut(imx577_liv_free_q_id, &frm_addr, 1);
                    }
                }
            }
            else
            {
                if(imx577_liv_free_q_id != NULL)
                {
                    osMessagePut(imx577_liv_free_q_id, &frm_addr, 1);
                }
            }
        }

        if(imx577_liv_free_q_id != NULL)
        {
            ret = osMessageGet(imx577_liv_free_q_id, 1);
            if(ret.status == osEventMessage)
            {
                free_addr = ret.value.v;
            }
        }
        drv_l1_pscaler_output_B_buffer_set(PSCALER_ID, free_addr);
    }
}
/**
  *@brief Binmode ISR callback
  *@param dev: Binning Index
  *@param event: Interrupt event
  *@retval None
  */
static void binmode_isr_callback(INT32U dev, INT32U event)
{
	if (event & BINMODE_FLAG_OF)
	{
		DBG_PRINT("o");
		drv_l1_binmode_int_flag_set(dev, BINMODE_FLAG_MASK);
		return;
	}

    if ((event & BINMODE_FLAG_END) == 0)
		return;
    else
    {
        imx577_param.cap_cnt++;
        if(event & BINMODE_FLAG_BUFA)
        {
            DBG_PRINT("a");
            if((imx577_param.cap_cnt >= imx577_param.cap_num) || (imx577_param.cap_cnt >= CAP_Q_NUM))
            {
                DBG_PRINT("d");
                imx577_param.capture = IMX577_CAPTURE_COMPLETE;
                drv_l1_binmode_framebuf_A_set(BINMODE_B, DUMMY_BUFFER_ADDRS);
            }
            else
            {
                drv_l1_binmode_framebuf_A_set(BINMODE_B, imx577_param.cap_buf[imx577_param.cap_ptr]);
            }

        }
        else if(event & BINMODE_FLAG_BUFB)
        {
            DBG_PRINT("b");
            if((imx577_param.cap_cnt >= imx577_param.cap_num) || (imx577_param.cap_cnt >= CAP_Q_NUM))
            {
                DBG_PRINT("D");
                imx577_param.capture = IMX577_CAPTURE_COMPLETE;
                drv_l1_binmode_framebuf_B_set(BINMODE_B, DUMMY_BUFFER_ADDRS);
            }
            else
            {
                drv_l1_binmode_framebuf_B_set(BINMODE_B, imx577_param.cap_buf[imx577_param.cap_ptr]);
            }
        }
    }
}
/**
  *@brief Flash LED timer ISR callback
  *@param None
  *@retval None
  */
static void flash_led_timer_isr_callback(void)
{
    flash_led_ctrl(0);
    timer_stop(FLASH_LED_TIMER);
}
/*******************************************************************
 * Private Functions
 *******************************************************************/
/**
  *@brief Initialize Camera - IMX577
  *@param None
  *@retval None
  */
static void imx577_init(void)
{
    drv_l2_sensor_ops_t* pSensor;
    drv_l2_sensor_para_t *pPara;
    BINMODE_CDSP_CFG cfg = {0};
    INT32U widthFactor = 65536;
    INT32U heightFactor = 65536;
    INT32U binmode_cfg;
    INT32U tmp_buf;
    INT8U i;

    //Initialize Sensor
	drv_l2_sensor_init();
    pSensor = drv_l2_sensor_get_ops(0);
    pPara = drv_l2_sensor_get_para();

    //Allocate Message
    if(imx577_msg_q_id == NULL)
    {
        osMessageQDef(imx577_msg_q, (MSG_Q_NUM + 1), INT32U);
        imx577_msg_q_id = osMessageCreate(&os_messageQ_def_imx577_msg_q, NULL);
        if(imx577_msg_q_id == NULL)
        {
            DBG_PRINT("[%s][ERROR] Create IMX577 Message Queue Failed\r\n", IMX577_TASK_TAG);
            while(1);
        }
    }
    if(imx577_msg_free_q_id == NULL)
    {
        osMessageQDef(imx577_msg_q, (MSG_Q_NUM + 1), INT32U);
        imx577_msg_free_q_id = osMessageCreate(&os_messageQ_def_imx577_msg_q, NULL);
        if(imx577_msg_free_q_id == NULL)
        {
            DBG_PRINT("[%s][ERROR] Create IMX577 Free Queue Failed\r\n", IMX577_TASK_TAG);
            while(1);
        }

        tmp_buf = (INT32U)gp_malloc_align((sizeof(IMX577Msg_Typedef) * MSG_Q_NUM), 4);
        if(tmp_buf == NULL)
        {
            DBG_PRINT("[%s][ERROR] Allocate IMX577 Message Failed\r\n", IMX577_TASK_TAG);
            while(1);
        }
        for(i=0;i<MSG_Q_NUM;i++)
        {
            osMessagePut(imx577_msg_free_q_id, &tmp_buf, 0);
            DBG_PRINT("[%s][LOG] IMX577 Message [%d]=[0x%x]\r\n", IMX577_TASK_TAG, i, tmp_buf);
            tmp_buf += sizeof(IMX577Msg_Typedef);
        }
    }

	//Allocate buffer --- Liveview
    if(imx577_liv_free_q_id == NULL)
	{
        osMessageQDef(liv_msg_q, (LIV_Q_NUM + 1), INT32U);
        imx577_liv_free_q_id = osMessageCreate(&os_messageQ_def_liv_msg_q, NULL);
        if(imx577_liv_free_q_id == NULL)
        {
            DBG_PRINT("[%s][ERROR] Create Liveview Free Message Queue Failed\r\n", IMX577_TASK_TAG);
            while(1);
        }
	}

    tmp_buf = (INT32U)gp_malloc_align((LIV_FRAME_SIZE * LIV_Q_NUM), 4);
    if(tmp_buf == NULL)
    {
        DBG_PRINT("[%s][ERROR] Allocate Liveview Buffer Failed\r\n", IMX577_TASK_TAG);
        while(1);
    }

	for(i=0;i<LIV_Q_NUM;i++)
	{

        osMessagePut(imx577_liv_free_q_id, &tmp_buf, 0);
        DBG_PRINT("[%s][LOG] Liveview Buf [%d] [0x%x]\r\n", IMX577_TASK_TAG, i, tmp_buf);
        tmp_buf += LIV_FRAME_SIZE;
	}

    //Initialize Pscaler
    drv_l1_pscaler_clk_ctrl(PSCALER_ID, ENABLE);
	drv_l1_pscaler_init(PSCALER_ID);

    drv_l1_CdspSetYuvPscalePath(1);
    pPara->pscaler_src_mode = MODE_PSCALER_SRC_CDSP;
    drv_l1_pscaler_input_source_set(PSCALER_ID, PIPELINE_SCALER_INPUT_SOURCE_CDSP);
    drv_l1_pscaler_input_format_set(PSCALER_ID, PIPELINE_SCALER_INPUT_FORMAT_YUYV);
    drv_l1_pscaler_input_pixels_set(PSCALER_ID, pSensor->info[PSC700_IMX577_RES].target_w, pSensor->info[PSC700_IMX577_RES].target_h);
	drv_l1_pscaler_input_buffer_set(PSCALER_ID, DUMMY_BUFFER_ADDRS);

	drv_l1_pscaler_output_fifo_line_set(PSCALER_ID, pSensor->info[PSC700_IMX577_RES].target_h,0);
	drv_l1_pscaler_output_pixels_set(PSCALER_ID, widthFactor, pSensor->info[PSC700_IMX577_RES].target_w, heightFactor, pSensor->info[PSC700_IMX577_RES].target_h);
	drv_l1_pscaler_output_format_set(PSCALER_ID, PIPELINE_SCALER_OUTPUT_FORMAT_VYUY);

	drv_l1_pscaler_output_A_buffer_set(PSCALER_ID, DUMMY_BUFFER_ADDRS);
    drv_l1_pscaler_output_B_buffer_set(PSCALER_ID, DUMMY_BUFFER_ADDRS);

    drv_l1_pscaler_interrupt_set(PSCALER_ID, PIPELINE_SCALER_INT_ENABLE_ALL);
    drv_l1_pscaler_callback_register(PSCALER_ID, pscaler_isr_callback);

    drv_l1_pscaler_start(PSCALER_ID);


	//Allocate buffer --- CAP
    tmp_buf = (INT32U)gp_malloc((CAP_FRAME_SIZE * CAP_Q_NUM));
    if(tmp_buf == NULL)
    {
        DBG_PRINT("[%s][LOG] Allocate CAP buf failed\r\n", IMX577_TASK_TAG);
        while(1);
    }

    imx577_param.cap_len = CAP_FRAME_SIZE;
    imx577_param.cap_cnt = 0;
    for(i=0;i<CAP_Q_NUM;i++)
	{
        imx577_param.cap_buf[i] = tmp_buf;
        DBG_PRINT("[%s][LOG] CAP Buf [%d] [0x%x]\r\n", IMX577_TASK_TAG, i, imx577_param.cap_buf[i]);
        tmp_buf += imx577_param.cap_len;
	}


    //Initialize Binning
    binmode_cfg = BINCFG_AVG | BINCFG_RAW10 | BINCFG_BYPASS;
    cfg.sensor_width = pSensor->info[PSC700_IMX577_RES_FUL_25FPS].sensor_w;
	cfg.sensor_height = pSensor->info[PSC700_IMX577_RES_FUL_25FPS].sensor_h;
	cfg.mode = (binmode_cfg & 0x30) >> 4;
	cfg.by_m = (binmode_cfg & 0x100) >> 8;
	cfg.pixwidth10Pad = (binmode_cfg & 0x1) >> 24;

    drv_l1_binmode_init(BINMODE_B, 0);
    drv_l1_binmode_interrupt_register(BINMODE_B, binmode_isr_callback);
    drv_l1_binmode_framebuf_A_set(BINMODE_B, DUMMY_BUFFER_ADDRS);
    drv_l1_binmode_framebuf_B_set(BINMODE_B, DUMMY_BUFFER_ADDRS);
    drv_l1_binmode_interrupt_set(BINMODE_B, 0x10031);
    drv_l1_binmode_config(BINMODE_B, &cfg);
    drv_l1_binmode_set_ctrl(BINMODE_B, binmode_cfg);
    drv_l1_binmode_disable(BINMODE_B);

    INT32U test_tick = xTaskGetTickCount();
    pSensor->init(PSC700_IMX577_RES, DUMMY_BUFFER_ADDRS, DUMMY_BUFFER_ADDRS, NULL);
	pSensor->stream_start();
}
/**
  *@brief Initialize collection parameters
  *@param None
  *@retval None
  */
static void imx577_col_init(void)
{
	if(imx577_col_frm_q_id == NULL)
	{
        osMessageQDef(col_msg_q, (LIV_Q_NUM + 1), INT32U);
        imx577_col_frm_q_id = osMessageCreate(&os_messageQ_def_col_msg_q, NULL);
        if(imx577_col_frm_q_id == NULL)
        {
            DBG_PRINT("[%s][ERROR] Create Collection Frame Message Queue Failed\r\n", IMX577_TASK_TAG);
            while(1);
        }
	}
	if(imx577_col_motor_q_id == NULL)
	{
        osMessageQDef(col_msg_q, (LIV_Q_NUM + 1), INT32U);
        imx577_col_motor_q_id = osMessageCreate(&os_messageQ_def_col_msg_q, NULL);
        if(imx577_col_motor_q_id == NULL)
        {
            DBG_PRINT("[%s][ERROR] Create Collection Frame Message Queue Failed\r\n", IMX577_TASK_TAG);
            while(1);
        }
	}

    gp_memset(imx577_param.col_liv_buf, 0, sizeof(imx577_param.col_liv_buf));
    imx577_param.col_liv_len = LIV_FRAME_SIZE;
    gp_memset(imx577_param.col_mtr, 0, sizeof(imx577_param.col_mtr));
    imx577_param.col_ptr = 0;
    imx577_param.col_cnt = 0;
    imx577_param.col_num = 0;
    imx577_param.col_backup_cnt = 0;

}
/**
  *@brief Release collected frames in queue
  *@param None
  *@retval None
  */
static void imx577_col_frm_release(void)
{
    osEvent evt;
    IMX577Mode_Typedef tmp = IMX577_MODE_IDLE;

    if((imx577_col_frm_q_id != NULL) && (imx577_liv_free_q_id != NULL) && (imx577_col_motor_q_id != NULL))
    {
        tmp = imx577_param.mode;
        imx577_param.mode = IMX577_MODE_IDLE;
        while(1)
        {
            evt = osMessageGet(imx577_col_frm_q_id, 1);
            if(evt.status != osEventMessage)
            {
                break;
            }
            else if(evt.value.v != NULL)
            {
                osMessagePut(imx577_liv_free_q_id, &evt.value.v, 1);
            }
        }
    }
    if (imx577_col_motor_q_id != NULL)
    {
        //Release motor's current position
        while(1)
        {
            evt = osMessageGet(imx577_col_motor_q_id, 1);
            if(evt.status != osEventMessage)
            {
                break;
            }
        }
    }

    gp_memset(imx577_param.col_liv_buf, 0, sizeof(imx577_param.col_liv_buf));
    imx577_param.col_liv_len = LIV_FRAME_SIZE;
    gp_memset(imx577_param.col_mtr, 0, sizeof(imx577_param.col_mtr));
    imx577_param.col_ptr = 0;
    imx577_param.col_cnt = 0;

    imx577_param.mode = tmp;
}
/**
  *@brief Initialize Flash LED
  *@param None
  *@retval None
  */
static void flash_led_init(void)
{
    gpio_set_port_attribute(VR_PW_EN, ATTRIBUTE_HIGH);
    gpio_init_io(VR_PW_EN, GPIO_OUTPUT);
    gpio_write_io(VR_PW_EN, DATA_LOW);

    gpio_set_port_attribute(VR_LED_ON, ATTRIBUTE_HIGH);
    gpio_init_io(VR_LED_ON, GPIO_OUTPUT);
    gpio_write_io(VR_LED_ON, DATA_LOW);
}
/**
  *@brief Flash LED control
  *@param on_off: Turn on/off (0:OFF Others:ON)
  *@retval None
  */
static void flash_led_ctrl(INT8U on_off)
{
    if(on_off) gpio_write_io(VR_LED_ON, DATA_HIGH);
    else gpio_write_io(VR_LED_ON, DATA_LOW);
}
/*******************************************************************
 * Public Functions
 *******************************************************************/
/**
  *@brief Camera IMX577 Task Entry
  *@param stack_size: Stack size for task
  *@retval None
  */
void cam_imx577_task_entry(INT32U stack_size)
{
    gp_memset(&imx577_param, 0, sizeof(IMX577Param_Typedef));

    if(task_id == NULL)
    {
        osThreadDef(imx577_task, osPriorityNormal, 0, stack_size);
        task_id = osThreadCreate(&os_thread_def_imx577_task, NULL);
        if(task_id == NULL)
        {
            DBG_PRINT("[Image_Task][ERROR] Create Task Failed\r\n");
        }
    }

    if (imx577_col_backup_task_id == NULL) {
        osThreadDef(imx577_collect_backup_task, osPriorityBelowNormal, 0, stack_size);
        imx577_col_backup_task_id = osThreadCreate(&os_thread_def_imx577_collect_backup_task, NULL);
        if(imx577_col_backup_task_id == NULL)
        {
            DBG_PRINT("[Image_Task][ERROR] Create Task Failed\r\n");
        }
    }
}
/**
  *@brief IMX577 Control - Drop liveview frame
  *@param num: Number of frame to drop
  *@retval Result (0:Success Othres:Failed)
  */
INT8U cam_imx577_task_ctrl_drop_frm(INT32U num)
{
    imx577_param.drop_num = num;
    imx577_param.drop_cnt = 0;      //Reset counter
    return 0;
}
/**
  *@brief IMX577 Control - Enter idle mode
  *@param None
  *@retval Result (0:Success Othres:Failed)
  */
INT8U cam_imx577_task_ctrl_idle(void)
{
    IMX577Msg_Typedef *imx577_idl_msg;
    osEvent evt;

    evt = osMessageGet(imx577_msg_free_q_id, 1);
    if(evt.status != osEventMessage) return 1;

    imx577_idl_msg = evt.value.v;
    imx577_idl_msg->evt = IMX577_EVENT_IDLE;
    osMessagePut(imx577_msg_q_id, &evt.value.v, osWaitForever);
    return 0;
}
/**
  *@brief IMX577 Control - Enter streaming mode
  *@param None
  *@retval Result (0:Success Othres:Failed)
  */
INT8U cam_imx577_task_ctrl_streaming(void)
{
    IMX577Msg_Typedef *imx577_stm_msg;
    osEvent evt;

    evt = osMessageGet(imx577_msg_free_q_id, 1);
    if(evt.status != osEventMessage) return 1;

    imx577_stm_msg = evt.value.v;
    imx577_stm_msg->evt = INX577_EVENT_CTRL_STREAMING;
    osMessagePut(imx577_msg_q_id, &evt.value.v, osWaitForever);
    return 0;
}
/**
  *@brief IMX577 Control - Start collecting frame
  *@param num: Number of frames to collect
  *@retval Result (0:Success Othres:Failed)
  */
INT8U cam_imx577_task_ctrl_collect_frm(INT32U num)
{
    IMX577Msg_Typedef *imx577_col_msg;
    osEvent evt;

    if(num > LIV_Q_NUM) return 1;

    imx577_param.col_cnt = 0;
    imx577_param.col_num = num;
    imx577_param.col_backup_cnt = 0;

    evt = osMessageGet(imx577_msg_free_q_id, 1);
    if(evt.status != osEventMessage) return 1;

    imx577_col_msg = evt.value.v;
    imx577_col_msg->evt = IMX577_EVENT_CTRL_START_COLLECT;
    osMessagePut(imx577_msg_q_id, &evt.value.v, osWaitForever);

    return 0;
}
/**
  *@brief IMX577 Control - Capture preparation
  *@param None
  *@retval None
  */
void cam_imx577_task_ctrl_pre_capture(void)
{
    INT32U sen_w, sen_h;

    imx577_get_cur_res(&sen_w, &sen_h);
    if((sen_w != CAP_WIDTH) || (sen_h != CAP_HEIGHT))
    {
        imx577_switch_full_resolution_25fps();
    }
}
/**
  *@brief IMX577 Control - Capture
  *@param cap_num: Number of captures
  *@param flash_led_en: Enable or disable flash led
  *@param flash_led_delay: Flash LED delay time (Unit: Milliseconds)
  *@param flash_led_on: Flash LED turn-on time (Unit: Milliseconds)
  *@retval Result (0:Success Othres:Failed)
  */
INT8U cam_imx577_task_ctrl_capture(INT8U cap_num, INT8U flash_led_en, INT16U flash_led_delay, INT16U flash_led_on)
{
    IMX577Msg_Typedef *imx577_cap_msg;
    osEvent evt;

    evt = osMessageGet(imx577_msg_free_q_id, 1);
    if(evt.status != osEventMessage) return 1;
    if(!cap_num) return 1;
    if(cap_num > CAP_Q_NUM) cap_num = CAP_Q_NUM;

    imx577_cap_msg = evt.value.v;
    imx577_cap_msg->evt = IMX577_EVENT_CTRL_CAPTURE;
    imx577_cap_msg->cap_num = cap_num;
    imx577_cap_msg->flash_led_en = flash_led_en;
    imx577_cap_msg->flash_led_delay_tm = flash_led_delay;
    imx577_cap_msg->flash_led_on_tm = flash_led_on;

    osMessagePut(imx577_msg_q_id, &evt.value.v, osWaitForever);
    return 0;
}
/**
  *@brief Get captured image
  *@param idx: Captured image index
  *@param *len: Get image length
  *@retval Image buffer address
  */
INT32U cam_imx577_task_get_cap(INT8U idx, INT32U *len)
{
    if(idx >= CAP_Q_NUM) return NULL;

    *len = imx577_param.cap_len;
    return imx577_param.cap_buf[idx];
}
/**
  *@brief Get collection data
  *@param *frm_buf: Get frame buffer addresses
  *@param *frm_len: Get a frame length
  *@param *mtr_pos: Get motor's positions
  *@retval Number of collected data
  */
INT32U cam_imx577_task_get_col(INT32U *frm_buf, INT32U *frm_len, INT32U *mtr_pos)
{
    if((frm_buf == NULL) || (frm_len == NULL) || (mtr_pos == NULL)) return 0;

    *frm_buf = imx577_param.col_liv_buf;
    *frm_len = imx577_param.col_liv_len;
    *mtr_pos = imx577_param.col_mtr;

    return imx577_param.col_cnt;
}
/**
  *@brief Register a callback to get Ready event
  *@param *callback: To be registered callback
  *@retval None
  */
void cam_imx577_task_register_get_ready(void (*callback)(void))
{
    imx577_param.get_ready = callback;
}
/**
  *@brief Register a callback to get liveview image
  *@param *callback: To be registered callback
  *@retval None
  */
void cam_imx577_task_register_get_liveview(void (*callback)(INT32U addr, INT32U len))
{
    imx577_param.get_liv = callback;
}
/**
  *@brief Register a callback to get collection completed event
  *@param *callback: To be registered callback
  *@retval None
  */
void cam_imx577_task_register_collect_complete(void (*callback)(void))
{
    imx577_param.collect_complete = callback;
}
/**
  *@brief Register a callback to get capture completed event
  *@param *callback: To be registered callback
  *@retval None
  */
void cam_imx577_task_register_capture_complete(void (*callback)(void))
{
    imx577_param.capture_complete = callback;
}
/**
  *@brief Register a callback to get motor's current position
  *@param *callback: To be registered callback
  *@retval None
  */
void cam_imx577_task_register_get_motor_pos(INT32U (*callback)(void))
{
    imx577_param.get_motor_pos = callback;
}
/**
  *@brief Get current collection backup count
  *@param None
  *@retval Number of collected frames already backed up
  */
INT32U cam_imx577_task_get_col_backup_cnt(void)
{
    return imx577_param.col_backup_cnt;
}