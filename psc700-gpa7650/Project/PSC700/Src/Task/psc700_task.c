/** @file psc700_task.c
 *  @brief PSC700 main task source file
 *
 *  This is the main task of PSC700, responsible for connecting with other tasks.
 *  It receives events from different tasks and performs corresponding processing
 *  based on the events.
 */
/*******************************************************************
 * Includes
 *******************************************************************/
#include "cmsis_os.h"
#include "drv_l1_usbd.h"
#include "drv_l1_uart.h"
#include "drv_l1_gpio.h"
#include "psc700_task.h"
#include "console_task.h"
#include "cam_imx577_task.h"
#include "usbd_task.h"
#include "motor_task.h"
#include "af_task.h"
#include "blink_task.h"
#include "tlc59108.h"

#include "psc700_conf.h"
#include "psc700_uart_cmd.h"
#include "psc700_usbd_uvc_vendor_desc_tbl.h"
#include "max5360.h"
#include "wt7026.h"
/*******************************************************************
 * Constants
 *******************************************************************/
#define PSC700_TASK_TAG                                "PSC700_TASK"
#define PSC700_TASK_STACKSIZE                           (3 * 1024)
#define CAM_IMX577_TASK_STACKSIZE                       (3 * 1024)
#define CONSOLE_TASK_STACKSIZE                          (3 * 1024)
#define USBD_TASK_STACKSIZE                             (3 * 1024)
#define MOTOR_TASK_STACKSIZE                            (3 * 1024)
#define AF_TASK_STACKSIZE                               (3 * 1024)
#define POWER_TASK_STACKSIZE                            (1 * 1024)
#define IR_TASK_STACKSIZE                               (2 * 1024)
#define BLINK_TASK_STACKSIZE                            (1 * 1024)

#define CMD_UART_IDX                                    UART2_DEV
#define CMD_UART_BAUDRATE                               115200

#define USB_PORT                                        USB_B

#define PSC700_MSG_Q_NUM                                3               //Configure number of PSC700 messages

#define VR_PW_EN                                        IO_C9           // Flash LED power pin
/*******************************************************************
 * Public Variables
 *******************************************************************/
extern SemaphoreHandle_t frame_collected_sem;
/*******************************************************************
 * Private Data Type
 *******************************************************************/
typedef enum{
    PSC700_SYSTEM_MODE_IDLE = 0,
    PSC700_SYSTEM_MODE_UVC,
    PSC700_SYSTEM_MODE_CAP,
    PSC700_SYSTEM_MODE_MAX,
}PSC700SystemMode_Typedef;
typedef enum{
    UVC_STATUS_STOP = 0,
    UVC_STATUS_PLAY,
    UVC_STATUS_PAUSE,
}UVCStatus_Typedef;
typedef struct{
    PSC700SystemMode_Typedef system_mode;
    INT8U usb_ready;            //0:Not Ready 1:Ready
    UVCStatus_Typedef uvc_status;
    PSC700UARTCmdCAP_Typedef cap_conf;
}PSC700Param_Typedef;
typedef struct{
    PSC700Event_Typedef evt;
    void *param;
    INT32U send_addr;
    INT32U send_len;
}PSC700Msg_Typedef;
/*******************************************************************
 * Private Variables
 *******************************************************************/
static osMessageQId psc700_msg_q_id = NULL;
static osMessageQId psc700_msg_free_q_id = NULL;

static PSC700Param_Typedef psc700_param;
static AFInfo_Typedef af_info;
static PSC700UARTCmdIR_Typedef ir_info;
static PSC700UARTCmdFLV_Typedef dac_info;
static PSC700UARTCmdMTR_Typedef motor_param;

INT8U save_af_frame = 0;
INT32U af_frame_index = 0;
/*******************************************************************
 * Declare Functions
 *******************************************************************/
/**
  *@brief A callback for receiving the GetReady event
  *@param None
  *@retval None
  */
 static void cam_imx577_get_ready_cb(void);
/**
  *@brief A callback to get liveview frame
  *@param addr: Frame Address
  *@param len: Frame length
  *@retval None
  */
static void cam_imx577_get_liveview_cb(INT32U addr, INT32U len);
/**
  *@brief A callback for frame collection completion
  *@param None
  *@retval None
  */
static void cam_imx577_collect_complete_cb(void);
/**
  *@brief A callback for frame capture completed
  *@param addr: Frame buffer
  *@param len: Frame length
  *@retval None
  */
static void cam_imx577_capture_complete_cb(INT32U addr, INT32U len);
/**
  *@brief Get USBD event callback
  *@param evt: USBD event
  *@retval None
  */
static void usbd_get_event_cb(USBDEvent_Typedef evt);
/**
  *@brief A callback to get AF completed event
  *@param pos: Motor's position
  *@retval None
  */
static void af_complete_cb(INT32U pos);
/**
  *@brief  Set flag for saving AF frames
  *@param  enable: 1 to enable saving AF frames, 0 to disable and reset af_frame_index
  *@retval None
  */
static void set_save_af_frame_flag(INT8U enable);
/*******************************************************************
 * Private Tasks
 *******************************************************************/
/**
  *@brief PSC700 task
  *@param *param: Input parameter pointer
  *@retval None
  */
static void psc700_task(const void *param)
{
    osEvent evt;
    INT32U buf, i;
    PSC700Msg_Typedef *psc700_msg;

    gp_memset(&psc700_param, 0, sizeof(PSC700Param_Typedef));

    psc700_param.system_mode = PSC700_SYSTEM_MODE_IDLE;

    //Initialize Camera IMX577
    cam_imx577_task_entry(CAM_IMX577_TASK_STACKSIZE);
    cam_imx577_task_ctrl_drop_frm(PSC700_LIVEVIEW_DROP_NUM);
    cam_imx577_task_register_get_ready(cam_imx577_get_ready_cb);
    cam_imx577_task_register_get_liveview(cam_imx577_get_liveview_cb);
    cam_imx577_task_register_collect_complete(cam_imx577_collect_complete_cb);
    cam_imx577_task_register_capture_complete(cam_imx577_capture_complete_cb);
    cam_imx577_task_register_get_motor_pos(motor_task_get_cur_pos);

    //Initialize USBD
    usbd_task_entry(USBD_TASK_STACKSIZE);
    usbd_task_register_event(usbd_get_event_cb);

    //Initialize Consol
    console_task_entry(CONSOLE_TASK_STACKSIZE, CMD_UART_IDX, CMD_UART_BAUDRATE);
    psc700_uart_cmd_init();

    //Initialize Motor
    motor_task_entry(MOTOR_TASK_STACKSIZE);

    //Initialize AF
    af_task_entry(AF_TASK_STACKSIZE);
    af_task_register_af_complete(af_complete_cb);

    //Initialize IR LED
    irled_init();

    blink_task_entry(BLINK_TASK_STACKSIZE);

    //Create PSC700 Message
    if(psc700_msg_q_id == NULL)
    {
        osMessageQDef(psc700_msg_g, (PSC700_MSG_Q_NUM + 1), INT32U);
        psc700_msg_q_id = osMessageCreate(&os_messageQ_def_psc700_msg_g, NULL);
        if(psc700_msg_q_id == NULL)
        {
            DBG_PRINT("[%s][ERROR] Create PSC700 Message Queue Failed\r\n", PSC700_TASK_TAG);
            while(1);
        }
    }
    if(psc700_msg_free_q_id == NULL)
    {
        osMessageQDef(psc700_msg_g, (PSC700_MSG_Q_NUM + 1), INT32U);
        psc700_msg_free_q_id = osMessageCreate(&os_messageQ_def_psc700_msg_g, NULL);
        if(psc700_msg_free_q_id == NULL)
        {
            DBG_PRINT("[%s][ERROR] Create PSC700 Message Free Queue Failed\r\n", PSC700_TASK_TAG);
            while(1);
        }
    }
    buf = (INT32U)gp_malloc_align((sizeof(PSC700Msg_Typedef) * PSC700_MSG_Q_NUM), 4);
    if(buf == NULL)
    {
        DBG_PRINT("[%s][ERROR] Allocate PSC700 Message Failed\r\n", PSC700_TASK_TAG);
    }

    for(i=0;i<PSC700_MSG_Q_NUM;i++)
    {
        osMessagePut(psc700_msg_free_q_id, &buf, 1);
        DBG_PRINT("[%s][LOG] PSC700 MSG [%d]=[0x%x]\r\n", PSC700_TASK_TAG, i, buf);
        buf += sizeof(PSC700Msg_Typedef);
    }


    while(1)
    {
        evt = osMessageGet(psc700_msg_q_id, osWaitForever);
        if((evt.status != osEventMessage) || (evt.value.v == NULL))
        {
            continue;
        }

        psc700_msg = evt.value.v;
        switch(psc700_msg->evt)
        {
            case PSC700_EVENT_CAM_READY:
                psc700_uart_cmd_resp_ready();
                break;
            case PSC700_EVENT_LIVE_VIEW:
                if((psc700_param.usb_ready) && (psc700_param.uvc_status == UVC_STATUS_PAUSE))
                {
                    psc700_param.uvc_status = UVC_STATUS_PLAY;
                    psc700_param.system_mode = PSC700_SYSTEM_MODE_UVC;
                    usbd_task_flush(USB_PORT);
                    cam_imx577_task_ctrl_streaming();
                    psc700_uart_cmd_resp_ok();
                }
                else if((psc700_param.usb_ready) && (psc700_param.uvc_status == UVC_STATUS_STOP))
                {
                    if(psc700_param.system_mode != PSC700_SYSTEM_MODE_IDLE)
                    {
                        cam_imx577_task_ctrl_idle();
                        psc700_param.system_mode = PSC700_SYSTEM_MODE_IDLE;
                    }
                    psc700_uart_cmd_resp_ok();
                }
                else
                {
                    psc700_uart_cmd_resp_err();
                }
                break;
            case PSC700_EVENT_CAPTURE:
                if(psc700_msg->param)
                {
                    gp_memcpy(&psc700_param.cap_conf, ((PSC700UARTCmdCAP_Typedef *)(psc700_msg->param)), sizeof(PSC700UARTCmdCAP_Typedef));
                }
                else
                {
                    psc700_param.cap_conf.cap_num = 1;
                    psc700_param.cap_conf.flash_led_en = 0;
                }
                if(!psc700_param.usb_ready)
                {
                    psc700_uart_cmd_resp_err();
                    break;
                }

                gpio_write_io(VR_PW_EN, DATA_HIGH);
                if(psc700_param.uvc_status == UVC_STATUS_PLAY)
                {
                    psc700_param.uvc_status = UVC_STATUS_PAUSE;
                }
                psc700_param.system_mode = PSC700_SYSTEM_MODE_CAP;

                // Drain all pending tokens in frame_collected_sem before starting a new capture sequence
                if (frame_collected_sem) {
                    UBaseType_t n = uxSemaphoreGetCount(frame_collected_sem); // Get token count
                    for (UBaseType_t i = 0; i < n; ++i) {
                        // If take fails (no more tokens), stop the loop early
                        if (xSemaphoreTake(frame_collected_sem, 0) != pdTRUE) break;
                        DBG_PRINT("R");
                    }
                }
                cam_imx577_task_ctrl_collect_frm(PSC700_LIVEVIEW_COL_NUM);
                if (psc700_param.cap_conf.af_en)
                {
                    motor_task_start();
                }
                break;
            case PSC700_EVENT_MOUNT_USB_UVC:
                if(!psc700_param.usb_ready)
                {
                    usbd_task_register_desc(USBD_DESC_TYPE_DEVICE,      USB_UVC_VD_DeviceDescriptor);
                    usbd_task_register_desc(USBD_DESC_TYPE_CONFIG,      USB_UVC_VD_ConfigDescriptor);
                    usbd_task_register_desc(USBD_DESC_TYPE_QUALIFIER,   USB_UVC_VD_Qualifier_Descriptor);
                    usbd_task_register_desc(USBD_DESC_TYPE_STRING0,     UVC_VD_String0_Descriptor);
                    usbd_task_register_desc(USBD_DESC_TYPE_STRING1,     UVC_VD_String1_Descriptor);
                    usbd_task_register_desc(USBD_DESC_TYPE_STRING2,     UVC_VD_String2_Descriptor);
                    usbd_task_register_desc(USBD_DESC_TYPE_STRING3,     UVC_VD_String3_Descriptor);

                    usbd_task_mount_uvc(USB_PORT);
                }
                else
                {
                    psc700_uart_cmd_resp_mounted();
                }
                break;
            case PSC700_EVENT_UNMOUNT_USB_UVC:
                usbd_task_unmount(USB_PORT);
                if(psc700_param.system_mode != PSC700_SYSTEM_MODE_IDLE)
                {
                    psc700_param.uvc_status = UVC_STATUS_STOP;
                    cam_imx577_task_ctrl_idle();
                    psc700_param.system_mode = PSC700_SYSTEM_MODE_IDLE;
                }
                psc700_param.usb_ready = 0;
                psc700_uart_cmd_resp_ok();
                break;
            case PSC700_EVENT_USB_UVC_START:
                if(psc700_param.system_mode == PSC700_SYSTEM_MODE_IDLE)
                {
                    psc700_param.uvc_status = UVC_STATUS_PLAY;
                    cam_imx577_task_ctrl_streaming();
                    psc700_param.system_mode = PSC700_SYSTEM_MODE_UVC;
                }
                else if(psc700_param.system_mode == PSC700_SYSTEM_MODE_CAP)
                {
                    psc700_param.uvc_status = UVC_STATUS_PAUSE;
                }
                break;
            case PSC700_EVENT_USB_UVC_STOP:
                psc700_param.uvc_status = UVC_STATUS_STOP;
                if(psc700_param.system_mode == PSC700_SYSTEM_MODE_UVC)
                {
                    cam_imx577_task_ctrl_idle();
                    psc700_param.system_mode = PSC700_SYSTEM_MODE_IDLE;
                }
                break;
            case PSC700_EVENT_USB_VENDOR_SEND_COMPLETED:
                if (save_af_frame) {
                    af_frame_index++;
                    if (af_frame_index < PSC700_LIVEVIEW_COL_NUM) {
                        // Send next frame
                        psc700_task_post_event(
                            PSC700_EVENT_GET_COLLECTED_FRAME,
                            (void*)af_frame_index
                        );
                        break;
                    }
                    set_save_af_frame_flag(0);
                }
                psc700_uart_cmd_resp_ok();
                break;
            case PSC700_EVENT_GET_LIVEVIEW:
                if((psc700_param.usb_ready) && (psc700_param.uvc_status == UVC_STATUS_PLAY))
                {
                    usbd_task_send(USBD_CLASS_UVC, USB_PORT, psc700_msg->send_addr, psc700_msg->send_len);
                }
                break;
            case PSC700_EVENT_COLLECT_COMPLETE:
                {
                    INT32U timeout = 0;
                    while(cam_imx577_task_get_col_backup_cnt() < PSC700_LIVEVIEW_COL_NUM)
                    {
                        osDelay(1);
                        if(++timeout > 50) break; // 50ms timeout
                    }
                }
                cam_imx577_task_ctrl_pre_capture();
                if(!psc700_param.cap_conf.ir_led_on)
                {
                    irled_ctrl(LED_TASK_LOCATION_ALL, 0);
                }
                max5360_set_level(psc700_param.cap_conf.flash_led_current);
                af_info.num = cam_imx577_task_get_col(&af_info.frm_buf, &af_info.frm_len, &af_info.mtr_pos);
                if (psc700_param.cap_conf.af_en)
                {
                    while(motor_task_is_busy()){osDelay(1);}
                    if(af_info.num && psc700_param.cap_conf.af_en)
                    {
                        af_task_start(af_info);
                    }
                } else {
                    psc700_task_post_event(PSC700_EVENT_AF_COMPLETE, NULL);
                }
                break;
            case PSC700_EVENT_CAPTURE_COMPLETE:
                gpio_write_io(VR_PW_EN, DATA_LOW); // Set VR_PW_EN (IO_C9) low as a workaround for flash LED leakage issue
                if(psc700_param.usb_ready)
                {
                    if(usbd_task_send(USBD_CLASS_VENDOR, USB_PORT, psc700_msg->send_addr, psc700_msg->send_len))
                    {
                        psc700_uart_cmd_resp_err();
                    }
                }
                else if(psc700_param.system_mode == PSC700_SYSTEM_MODE_CAP)
                {
                    psc700_uart_cmd_resp_err();
                }
                break;
            case PSC700_EVENT_GET_IMG:
                if(psc700_msg->param)
                {
                    psc700_msg->send_addr = cam_imx577_task_get_cap(*((INT8U *)(psc700_msg->param)), &psc700_msg->send_len);
                    if((psc700_msg->send_addr != NULL) || (psc700_msg->send_len))
                    {
                        if(usbd_task_send(USBD_CLASS_VENDOR, USB_PORT, psc700_msg->send_addr, psc700_msg->send_len))
                        {
                            psc700_uart_cmd_resp_err();
                        }
                    }
                    else
                    {
                        psc700_uart_cmd_resp_err();
                    }
                }
                else
                {
                    psc700_uart_cmd_resp_err();
                }
                break;
            case PSC700_EVENT_AF_COMPLETE:
                if (psc700_param.cap_conf.af_en)
                {
                    while(!motor_task_is_ready()){osDelay(1);}
                }
                cam_imx577_task_ctrl_capture(psc700_param.cap_conf.cap_num, psc700_param.cap_conf.flash_led_en, psc700_param.cap_conf.flash_led_delay, psc700_param.cap_conf.flash_led_on);
                break;
            case PSC700_EVENT_IR_CONTROL:
                if(psc700_msg->param)
                {
                    gp_memcpy(&ir_info, ((PSC700UARTCmdIR_Typedef *)(psc700_msg->param)), sizeof(PSC700UARTCmdIR_Typedef));
                }
                else
                {
                    psc700_uart_cmd_resp_err();
                }
                if (irled_ctrl(ir_info.ir_location, ir_info.ir_level))
                {
                    psc700_uart_cmd_resp_err();
                }

                break;
            case PSC700_EVENT_FLV_CONTROL:
                if(psc700_msg->param)
                {
                    gp_memcpy(&dac_info, ((PSC700UARTCmdFLV_Typedef *)(psc700_msg->param)), sizeof(PSC700UARTCmdFLV_Typedef));
                }
                else
                {
                    psc700_uart_cmd_resp_err();
                }
                if (max5360_set_level(dac_info.dac_level) < 0)
                    psc700_uart_cmd_resp_err();
                else
                    psc700_uart_cmd_resp_ok();
                break;
            case PSC700_EVENT_MOTOR_CONTROL:
                if(psc700_msg->param)
                {
                    gp_memcpy(&motor_param, ((PSC700UARTCmdMTR_Typedef *)(psc700_msg->param)), sizeof(PSC700UARTCmdMTR_Typedef));
                }
                else
                {
                    psc700_uart_cmd_resp_err();
                }

                if (motor_param.motor_cmd == MOTOR_TASK_CMD_HOME)
                {
                    if(motor_task_home())
                    {
                        psc700_uart_cmd_resp_err();
                    }
                } else if (motor_param.motor_cmd == MOTOR_TASK_CMD_MOVE) {
                    if(motor_task_relative_move(motor_param.position))
                    {
                        psc700_uart_cmd_resp_err();
                    }
                } else if (motor_param.motor_cmd == MOTOR_TASK_CMD_GOTO) {
                    if(motor_task_absolute_move(motor_param.position))
                    {
                        psc700_uart_cmd_resp_err();
                    }
                } else
                    psc700_uart_cmd_resp_err();

                break;
            case PSC700_EVENT_GET_COLLECTED_FRAME:
                INT32U frame_index = (INT32U)psc700_msg->param;
                if(af_info.num && frame_index<PSC700_LIVEVIEW_COL_NUM)
                {
                    set_save_af_frame_flag(1);
                    INT32U frame_addr = af_info.frm_buf[frame_index];
                    if (usbd_task_send(USBD_CLASS_VENDOR, USB_PORT, frame_addr, af_info.frm_len))
                    {
                        set_save_af_frame_flag(0);
                        psc700_uart_cmd_resp_err();
                    }
                } else
                    psc700_uart_cmd_resp_err();
                break;
            case PSC700_EVENT_SHUTDOWN:
                DBG_PRINT("Power off\r\n");
                R_SYSTEM_POWER_CTRL0 &= ~0x1; // power down
                break;
        }

        osMessagePut(psc700_msg_free_q_id, &evt.value.v, 1);
    }
}
/*******************************************************************
 * Private Callbacks
 *******************************************************************/
/**
  *@brief A callback for receiving the GetReady event
  *@param None
  *@retval None
  */
 static void cam_imx577_get_ready_cb(void)
 {
    osEvent evt;
    PSC700Msg_Typedef *psc700_get_rdy_msg;

    if((psc700_msg_q_id != NULL) && (psc700_msg_free_q_id != NULL))
    {
        evt = osMessageGet(psc700_msg_free_q_id, 1);
        if((evt.status == osEventMessage) && (evt.value.v != NULL))
        {
            psc700_get_rdy_msg = evt.value.v;
            psc700_get_rdy_msg->evt = PSC700_EVENT_CAM_READY;
            osMessagePut(psc700_msg_q_id, &evt.value.v, 1);
        }
    }

 }
/**
  *@brief A callback to get liveview frame
  *@param addr: Frame Address
  *@param len: Frame length
  *@retval None
  */
static void cam_imx577_get_liveview_cb(INT32U addr, INT32U len)
{
    osEvent evt;
    PSC700Msg_Typedef *psc700_get_frm_msg;

    if((addr == NULL) || (len == 0)) return;

    if((psc700_msg_q_id != NULL) && (psc700_msg_free_q_id != NULL))
    {
        evt = osMessageGet(psc700_msg_free_q_id, 1);
        if((evt.status == osEventMessage) && (evt.value.v != NULL))
        {
            psc700_get_frm_msg = evt.value.v;
            psc700_get_frm_msg->evt = PSC700_EVENT_GET_LIVEVIEW;
            psc700_get_frm_msg->send_addr = addr;
            psc700_get_frm_msg->send_len = len;

            osMessagePut(psc700_msg_q_id, &evt.value.v, 1);
        }
    }
}
/**
  *@brief A callback for frame collection completion
  *@param None
  *@retval None
  */
static void cam_imx577_collect_complete_cb(void)
{
    osEvent evt;
    PSC700Msg_Typedef *psc700_collect_msg;

    if((psc700_msg_q_id != NULL) && (psc700_msg_free_q_id != NULL))
    {
        evt = osMessageGet(psc700_msg_free_q_id, 1);
        if((evt.status == osEventMessage) && (evt.value.v != NULL))
        {
            psc700_collect_msg = evt.value.v;

            psc700_collect_msg->evt = PSC700_EVENT_COLLECT_COMPLETE;

            osMessagePut(psc700_msg_q_id, &evt.value.v, 1);
        }
    }
}
/**
  *@brief A callback for frame capture completed
  *@param addr: Frame buffer
  *@param len: Frame length
  *@retval None
  */
static void cam_imx577_capture_complete_cb(INT32U addr, INT32U len)
{
    osEvent evt;
    PSC700Msg_Typedef *psc700_capture_msg;

    if((psc700_msg_q_id != NULL) && (psc700_msg_free_q_id != NULL))
    {
        evt = osMessageGet(psc700_msg_free_q_id, 1);
        if((evt.status == osEventMessage) && (evt.value.v != NULL))
        {
            psc700_capture_msg = evt.value.v;

            psc700_capture_msg->evt = PSC700_EVENT_CAPTURE_COMPLETE;
            psc700_capture_msg->send_addr = addr;
            psc700_capture_msg->send_len = len;

            osMessagePut(psc700_msg_q_id, &evt.value.v, 1);
        }
    }
}
/**
  *@brief Get USBD event callback
  *@param evt: USBD event
  *@retval None
  */
static void usbd_get_event_cb(USBDEvent_Typedef evt)
{
    osEvent ret;
    PSC700Msg_Typedef *psc700_usbd_get_evt_msg;

    switch(evt)
    {
        case USBD_EVENT_MOUNT_SUCCESS:
            psc700_uart_cmd_resp_ok();
            psc700_param.usb_ready = 1;
            break;
        case USBD_EVENT_MOUNT_FAILED:
            psc700_param.usb_ready = 0;
            break;
        case USBD_EVENT_UVC_START:
            if((psc700_msg_q_id != NULL) && (psc700_msg_free_q_id != NULL))
            {
                ret = osMessageGet(psc700_msg_free_q_id, 1);
                if((ret.status == osEventMessage) && (ret.value.v != NULL))
                {
                    psc700_usbd_get_evt_msg = ret.value.v;
                    psc700_usbd_get_evt_msg->evt = PSC700_EVENT_USB_UVC_START;
                    osMessagePut(psc700_msg_q_id, &ret.value.v, 1);
                }
            }
            break;
        case USBD_EVENT_UVC_STOP:
            if((psc700_msg_q_id != NULL) && (psc700_msg_free_q_id != NULL))
            {
                ret = osMessageGet(psc700_msg_free_q_id, 1);
                if((ret.status == osEventMessage) && (ret.value.v != NULL))
                {
                    psc700_usbd_get_evt_msg = ret.value.v;
                    psc700_usbd_get_evt_msg->evt = PSC700_EVENT_USB_UVC_STOP;
                    osMessagePut(psc700_msg_q_id, &ret.value.v, 1);
                }
            }
            break;
        case USBD_EVENT_VENDOR_SEND_COMPLETED:
            if((psc700_msg_q_id != NULL) && (psc700_msg_free_q_id != NULL))
            {
                ret = osMessageGet(psc700_msg_free_q_id, 1);
                if((ret.status == osEventMessage) && (ret.value.v != NULL))
                {
                    psc700_usbd_get_evt_msg = ret.value.v;
                    psc700_usbd_get_evt_msg->evt = PSC700_EVENT_USB_VENDOR_SEND_COMPLETED;
                    osMessagePut(psc700_msg_q_id, &ret.value.v, 1);
                }
            }
            break;
    }
}
/**
  *@brief A callback to get AF completed event
  *@param pos: Motor's position
  *@retval None
  */
static void af_complete_cb(INT32U pos)
{
    osEvent evt;
    PSC700Msg_Typedef *psc700_af_msg;

    motor_task_set_pos(pos);

    if((psc700_msg_q_id != NULL) && (psc700_msg_free_q_id != NULL))
    {
        evt = osMessageGet(psc700_msg_free_q_id, 1);
        if((evt.status == osEventMessage) && (evt.value.v != NULL))
        {
            psc700_af_msg = evt.value.v;
            psc700_af_msg->evt = PSC700_EVENT_AF_COMPLETE;
            osMessagePut(psc700_msg_q_id, &evt.value.v, 1);
        }
    }
}
/**
  *@brief  Set flag for saving AF frames
  *@param  enable: 1 to enable saving AF frames, 0 to disable and reset af_frame_index
  *@retval None
  */
static void set_save_af_frame_flag(INT8U enable)
{
    save_af_frame = enable;
    if (!enable)
    {
        af_frame_index = 0;
    }
}
/*******************************************************************
 * Public Functions
 *******************************************************************/
/**
  *@brief PSC700 task entry
  *@param None
  *@retval None
  */
void psc700_task_entry(void)
{
    DBG_PRINT("[%s] Version: %s\r\n", PSC700_TASK_TAG, PSC700_VER);

    osThreadDef(psc700_task, osPriorityNormal, 0, PSC700_TASK_STACKSIZE);
	osKernelStart(&os_thread_def_psc700_task, NULL);
}
/**
  *@brief Post event to PSC700 task
  *@param evt: PSC700 Event
  *@param *param: To be posted parameter
  *@retval Result (0:Success Others:Failed)
  */
INT8U psc700_task_post_event(PSC700Event_Typedef evt, void *param)
{
    osEvent ret;
    PSC700Msg_Typedef *psc700_post_msg;
    if(evt >= PSC700_EVEMT_MAX) return 1;
    if((psc700_msg_q_id == NULL) || (psc700_msg_free_q_id == NULL)) return 1;

    ret = osMessageGet(psc700_msg_free_q_id, 1);
    if(ret.status != osEventMessage) return 1;
    if(ret.value.v == NULL) return 1;

    psc700_post_msg = ret.value.v;
    psc700_post_msg->evt = evt;
    psc700_post_msg->param = param;
    osMessagePut(psc700_msg_q_id, &ret.value.v, 1);
}
