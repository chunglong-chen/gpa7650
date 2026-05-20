/** @file usbd_task.c
 *  @brief USBD task source file
 *
 *  This task is responsible for communication with PC through USB interface.
 *  It supports two classes : UVC and Vendor.
 */
/*******************************************************************
 * Includes
 *******************************************************************/
#include "cmsis_os.h"
#include "drv_l1_clock.h"
#include "drv_l1_usbd.h"
#include "drv_l2_usbd.h"
#include "usbd_task.h"
/*******************************************************************
 * Constants
 *******************************************************************/
#define USBD_TASK_TAG                               "USBD"
#define USBD_UVC_TAG                                "USBD_UVC"

#define USBD_MSG_Q_NUM                              3
/*******************************************************************
 * Data Types
 *******************************************************************/
typedef enum{
    USBD_LOCK_STATUS_LOCKED = 0,
    USBD_LOCK_STATUS_UNLOCK,
    USBD_LOCK_STATUS_FREE,
    USBD_LOCK_STATUS_MAX,
}USBDLockStatus_Typedef;
typedef enum{
    USBD_MSG_EVENT_NONE = 0,
    USBD_MSG_EVENT_MOUNT_SUCCESS,
    USBD_MSG_EVENT_MOUNT_FAILED,
    USBD_MSG_EVENT_UVC_START,
    USBD_MSG_EVENT_UVC_STOP,
    USBD_MSG_EVENT_UVC_SEND,
    USBD_MSG_EVENT_UVC_SEND_COMPLETED,
    USBD_MSG_EVENT_VENDOR_SEND,
    USBD_MSG_EVENT_VENDOR_SEND_COMPLETED,
    USBD_MSG_EVENT_MAX,
}USBDMsgEvent_Typedef;
typedef struct{
    INT8U *device;
    INT8U *config;
    INT8U *qualifier;
    INT8U *string0;
    INT8U *string1;
    INT8U *string2;
    INT8U *string3;
}USBDDescParam_Typedef;
typedef struct{
    INT8U is_mount;
    INT8S uvc_en[2];

    USBDLockStatus_Typedef send_lock;

    void (*post_evt_cb)(USBDEvent_Typedef evt);
}USBDParam_Typedef;
typedef struct{
    USBDMsgEvent_Typedef evt;

    INT8U usb_id;
    INT32U send_addr;
    INT32U send_len;
}USBDMsg_Typedef;
/*******************************************************************
 * Private Variables
 *******************************************************************/
static osThreadId task_id = NULL;
static osMessageQId usbd_msg_q_id = NULL, usbd_msg_free_q_id = NULL;

static USBDParam_Typedef usbd_param;
static USBDDescParam_Typedef usbd_desc = { 0 };

/*******************************************************************
 * Declare Functions
 *******************************************************************/
/**
  *@brief USB UVC "Set Interface" Callback
  *@param usbd_id: USB Index (USB_A, USB_B)
  *@param video: Video Status
  *@retval None
  */
static void usbd_uvc_set_interface_cb(INT32U usb_id, INT32U video);
/**
  *@brief USBD UVC send completed callback (EP5)
  *@param usb_id: USB Index (USB_A, USB_B)
  *@retval None
  */
static void usbd_uvc_send_complete_cb(INT32U usb_id);
/**
  *@brief Send completed via vendor class
  *@param None
  *@retval None
  */
static void usbd_vendor_send_complete_cb(void);
/*******************************************************************
 * Private Tasks
 *******************************************************************/
/**
  *@brief USBD UVC Task
  *@param *param: Input paramters pointer
  *@retval None
  */
static void usbd_task(const void *param)
{
    INT32U i, buf;
    USBDMsg_Typedef *usbd_msg;

    drv_l2_usbd_register_uvc_frame_done_cbk(usbd_uvc_send_complete_cb);
    drv_l2_usbd_vendor_register_complete_cb(usbd_vendor_send_complete_cb);

    if(usbd_msg_q_id == NULL)
    {
        osMessageQDef(usbd_msg_q, (USBD_MSG_Q_NUM + 1), INT32U);
        usbd_msg_q_id = osMessageCreate(&os_messageQ_def_usbd_msg_q, NULL);
        if(usbd_msg_q_id == NULL)
        {
            DBG_PRINT("[%s][ERROR] Create USBD Message Failed\r\n", USBD_TASK_TAG);
            while(1);
        }
    }
    if(usbd_msg_free_q_id == NULL)
    {
        osMessageQDef(usbd_msg_q, (USBD_MSG_Q_NUM + 1), INT32U);
        usbd_msg_free_q_id = osMessageCreate(&os_messageQ_def_usbd_msg_q, NULL);
        if(usbd_msg_free_q_id == NULL)
        {
            DBG_PRINT("[%s][ERROR] Create USBD Message Failed\r\n", USBD_TASK_TAG);
            while(1);
        }
    }

    buf = (INT32U)gp_malloc_align((sizeof(USBDMsg_Typedef) * USBD_MSG_Q_NUM), 4);
    if(buf == NULL)
    {
        DBG_PRINT("[%s][ERROR] Allocate USBD Message Failed\r\n", USBD_TASK_TAG);
        while(1);
    }

    for(i=0;i<USBD_MSG_Q_NUM;i++)
    {
        osMessagePut(usbd_msg_free_q_id, &buf, 1);
        DBG_PRINT("[%s][LOG] USBD Message [%d]=[0x%x]\r\n", USBD_TASK_TAG, i, buf);
        buf += sizeof(USBDMsg_Typedef);
    }

    while(1)
    {
        osEvent evt;
        evt = osMessageGet(usbd_msg_q_id, osWaitForever);
        if((evt.status != osEventMessage) || (evt.value.v == NULL))
        {
            DBG_PRINT("[%s][ERROR] status [%d] value.v [%#x]\r\n", USBD_TASK_TAG, evt.status, evt.value.v);
            continue;
        }

        usbd_msg = evt.value.v;
        switch(usbd_msg->evt)
        {
            case USBD_MSG_EVENT_MOUNT_SUCCESS:
                if(usbd_param.post_evt_cb)
                {
                    usbd_param.post_evt_cb(USBD_EVENT_MOUNT_SUCCESS);
                }
                break;
            case USBD_MSG_EVENT_MOUNT_FAILED:
                if(usbd_param.post_evt_cb)
                {
                    usbd_param.post_evt_cb(USBD_EVENT_MOUNT_FAILED);
                }
                break;
            case USBD_MSG_EVENT_UVC_START:
                if(usbd_param.send_lock == USBD_LOCK_STATUS_FREE)
                {
                    usbd_param.send_lock = USBD_LOCK_STATUS_UNLOCK;
                }
                if(usbd_param.post_evt_cb)
                {
                    usbd_param.post_evt_cb(USBD_EVENT_UVC_START);
                }
                break;
            case USBD_MSG_EVENT_UVC_STOP:
                if(usbd_param.send_lock == USBD_LOCK_STATUS_LOCKED)
                {
                    usbd_param.send_lock = USBD_LOCK_STATUS_FREE;
                }
                if(usbd_param.post_evt_cb)
                {
                    usbd_param.post_evt_cb(USBD_EVENT_UVC_STOP);
                }
                break;
            case USBD_MSG_EVENT_UVC_SEND:
                if(usbd_param.uvc_en[usbd_msg->usb_id])
                {
                    drv_l1_usbd_frame_iso_ep5_in(usbd_msg->usb_id, (void *)usbd_msg->send_addr, usbd_msg->send_len, 1, 1);
                }
                break;
            case USBD_MSG_EVENT_VENDOR_SEND:
                drv_l1_usbd_iso_ep5_flush(usbd_msg->usb_id);
                drv_l2_usbd_vendor_send(usbd_msg->usb_id, usbd_msg->send_addr, usbd_msg->send_len);
                break;
            case USBD_MSG_EVENT_UVC_SEND_COMPLETED:
            case USBD_MSG_EVENT_VENDOR_SEND_COMPLETED:
                if(usbd_param.send_lock == USBD_LOCK_STATUS_LOCKED)
                {
                    usbd_param.send_lock = USBD_LOCK_STATUS_UNLOCK;
                }
                if((usbd_msg->evt == USBD_MSG_EVENT_VENDOR_SEND_COMPLETED) && (usbd_param.post_evt_cb))
                {
                    usbd_param.post_evt_cb(USBD_EVENT_VENDOR_SEND_COMPLETED);
                }
                break;
            default:
                DBG_PRINT("[%s][ERROR] Unknown Event [%d] [%#x]", USBD_TASK_TAG, usbd_msg->evt, evt.value.v);
                break;
        }
        osMessagePut(usbd_msg_free_q_id, &evt.value.v, 1);
    }
}
/*******************************************************************
 * Private Callbacks
 *******************************************************************/
/**
  *@brief USB UVC "Set Interface" Callback
  *@param usbd_id: USB Index (USB_A, USB_B)
  *@param video: Video Status
  *@retval None
  */
static void usbd_uvc_set_interface_cb(INT32U usb_id, INT32U video)
{
    osEvent evt;
    USBDMsg_Typedef *usbd_set_int_msg = NULL;

    if((usbd_msg_q_id != NULL) && (usbd_msg_free_q_id != NULL))
    {
        evt = osMessageGet(usbd_msg_free_q_id, 1);
        if((evt.status == osEventMessage) && (evt.value.v != NULL))
        {
            usbd_set_int_msg = evt.value.v;
        }

    }

    if(usbd_param.uvc_en[usb_id] != video)
    {
        if(video)
        {
            //UVC Start
            drv_l1_usbd_uvc_ep5_en(usb_id,0);

            usbd_set_int_msg->evt = USBD_MSG_EVENT_UVC_START;
        }
        else
        {
            //UVC Stop
            if(!usbd_param.is_mount)
            {
                usbd_param.is_mount = 1;

                usbd_set_int_msg->evt = USBD_MSG_EVENT_MOUNT_SUCCESS;
            }
            else
            {
                usbd_set_int_msg->evt = USBD_MSG_EVENT_UVC_STOP;
                usbd_set_int_msg->usb_id = usb_id;
            }
        }
        usbd_param.uvc_en[usb_id] = video;

        if(usbd_set_int_msg != NULL)
        {
            osMessagePut(usbd_msg_q_id, &evt.value.v, 1);
        }
    }
    else if(evt.value.v)
    {
        osMessagePut(usbd_msg_free_q_id, &evt.value.v, 1);
    }
}
/**
  *@brief Send completed via UVC class
  *@param usb_id: USB Index (USB_A, USB_B)
  *@retval None
  */
static void usbd_uvc_send_complete_cb(INT32U usb_id)
{
    osEvent evt;
    USBDMsg_Typedef *usbd_vd_msg;

    if((usbd_msg_q_id != NULL) && (usbd_msg_free_q_id != NULL))
    {
        evt = osMessageGet(usbd_msg_free_q_id, 1);
        if((evt.status != osEventMessage) || (evt.value.v == NULL)) return;

        usbd_vd_msg = evt.value.v;
        usbd_vd_msg->evt = USBD_MSG_EVENT_UVC_SEND_COMPLETED;

        osMessagePut(usbd_msg_q_id, &evt.value.v, 1);
    }
}
/**
  *@brief Send completed via vendor class
  *@param None
  *@retval None
  */
static void usbd_vendor_send_complete_cb(void)
{
    osEvent evt;
    USBDMsg_Typedef *usbd_vd_msg;
    if((usbd_msg_q_id != NULL) && (usbd_msg_free_q_id != NULL))
    {
        evt = osMessageGet(usbd_msg_free_q_id, 1);
        if((evt.status != osEventMessage) || (evt.value.v == NULL)) return;

        usbd_vd_msg = evt.value.v;
        usbd_vd_msg->evt = USBD_MSG_EVENT_VENDOR_SEND_COMPLETED;

        osMessagePut(usbd_msg_q_id, &evt.value.v, 1);
    }
}
/*******************************************************************
 * Private Functions
 *******************************************************************/
/*******************************************************************
 * Public Functions
 *******************************************************************/
/**
  *@brief USBD Task Entry
  *@param stack_size: Stack size for task
  *@retval None
  */
void usbd_task_entry(INT32U stack_size)
{
    INT8U i;

    gp_memset(&usbd_param, 0, sizeof(USBDParam_Typedef));

    usbd_param.send_lock = USBD_LOCK_STATUS_FREE;
    for(i=0;i<sizeof(usbd_param.uvc_en);i++)
    {
        usbd_param.uvc_en[i] = -1;
    }


    if(task_id == NULL)
    {
        osThreadDef(usbd_task, osPriorityNormal, 0, stack_size);
        task_id = osThreadCreate(&os_thread_def_usbd_task, NULL);
        if(task_id == NULL)
        {
            DBG_PRINT("[%s][ERROR] Creat USBD Task Failed\r\n", USBD_TASK_TAG);
        }
    }
}
/**
  *@brief Mount USBD UVC
  *@param usb_port: USB Port
  *@retval Result (0:Success Others:Failed)
  */
INT8U usbd_task_mount_uvc(INT8U usb_port)
{
    INT32S ret;
    INT8U usb_id, i;

    if((usbd_desc.device == NULL) || (usbd_desc.config == NULL) || (usbd_desc.qualifier == NULL))
    {
        DBG_PRINT("[%s][ERROR] Invalid Descriptor\r\n", USBD_TASK_TAG);
        return 1;
    }

    usbd_param.is_mount = 0;
    for(i=0;i<sizeof(usbd_param.uvc_en);i++)
    {
        usbd_param.uvc_en[i] = -1;
    }
    usbd_param.send_lock = USBD_LOCK_STATUS_FREE;


    if(usb_port == USB_B)
    {
        usb_id = USB_B;
        drv_l1_clock_set_system_clk_en(CLK_EN6_USB20_DEVICE_B, 1);
    }
    else
    {
        usb_id = USB_A;
        drv_l1_clock_set_system_clk_en(CLK_EN1_USB20_DEVICE_A, 1);
    }
    drv_l1_clock_set_system_clk_en(CLK_EN2_I2S_RX0, 1);

    /* uphy resume */
    drv_l1_usbd_uphy_suspend(usb_id, 0);
 	drv_l1_usbd_phy_switch(usb_id);

    /* Init USBD L2 protocol layer first, including control/bulk/ISO/interrupt transfers */
    /******************************* Control transfer ************************************/
    ret = drv_l2_usbd_ctl_init(usb_id);
    if(ret == STATUS_FAIL)
    {
        DBG_PRINT("[%s][ERROR] USBD Init Failed\r\n", USBD_UVC_TAG);
        return 1;
    }

    /* Register new descriptor table here, this action must be done after drv_l2_usbd_ctl_init() */
    drv_l2_usbd_register_descriptor(usb_id, REG_DEVICE_DESCRIPTOR_TYPE,             usbd_desc.device);
    drv_l2_usbd_register_descriptor(usb_id, REG_CONFIG_DESCRIPTOR_TYPE,             usbd_desc.config);
    drv_l2_usbd_register_descriptor(usb_id, REG_DEVICE_QUALIFIER_DESCRIPTOR_TYPE,   usbd_desc.qualifier);
    drv_l2_usbd_register_descriptor(usb_id, REG_STRING0_DESCRIPTOR_TYPE,            usbd_desc.string0);
    drv_l2_usbd_register_descriptor(usb_id, REG_STRING1_DESCRIPTOR_TYPE,            usbd_desc.string1);
    drv_l2_usbd_register_descriptor(usb_id, REG_STRING2_DESCRIPTOR_TYPE,            usbd_desc.string2);
	drv_l2_usbd_register_descriptor(usb_id, REG_STRING3_DESCRIPTOR_TYPE,            usbd_desc.string3);

   	/* Init USBD L2 of UVC */
    ret = drv_l2_usbd_uvc_init();
    if(ret == STATUS_FAIL)
    {
        DBG_PRINT("[%s][ERROR] UVC Init L2 Failed\r\n", USBD_UVC_TAG);
        return 1;
    }

    /* Register set interface call back */
	drv_l2_usbd_register_set_interface_cbk(usbd_uvc_set_interface_cb);

    /* Init USBD L1 register layer */
    ret = drv_l1_usbd_uvc_init(usb_id);
    if(ret == STATUS_FAIL)
    {
        DBG_PRINT("[%s][ERROR] UVC Init L1 Failed\r\n", USBD_UVC_TAG);
        return 1;
    }

	/* register USBD ISR handler */
	drv_l1_usbd_enable_isr(usb_id);

    return 0;
}
/**
  *@brief Unmount USBD
  *@param usb_port: USB Port (USB_A, USB_B)
  *@retval None
  */
void usbd_task_unmount(INT8U usb_port)
{
    INT8U usb_id, i;
    if(usb_port == USB_B) usb_id = USB_B;
    else usb_id = USB_A;

    usbd_param.is_mount = 0;
    for(i=0;i<sizeof(usbd_param.uvc_en);i++)
    {
        usbd_param.uvc_en[i] = -1;
    }

    drv_l1_usbd_uninit(usb_id);
	drv_l2_usbd_ctl_uninit(usb_id);
	drv_l2_usbd_uvc_uninit(usb_id);

    if (usb_id == USB_A)
    {
        drv_l1_clock_set_system_clk_en(CLK_EN1_USB20_DEVICE_A, 0);
	}
	else
	{
        drv_l1_clock_set_system_clk_en(CLK_EN6_USB20_DEVICE_B, 0);
    }

}
/**
  *@brief Send data via USB specified class and port
  *@param usb_class: USB class
  *@param usb_id: USB port
  *@param addr: To be sent data addr
  *@param len: To be sent data length
  *@retval Result (0:Success Others:Failed)
  */
INT8U usbd_task_send(USBDClass_Typedef usb_class, INT8U usb_id, INT32U addr, INT32U len)
{
    osEvent evt;
    USBDMsg_Typedef *usbd_send_msg;

    if(usb_class >= USBD_CLASS_MAX) return 1;
    if((usb_id != USB_A) && (usb_id != USB_B)) return 1;
    if((addr == NULL) || (len == 0)) return 1;
    if((usbd_msg_q_id == NULL) || (usbd_msg_free_q_id == NULL)) return 1;
    if(usbd_param.send_lock == USBD_LOCK_STATUS_LOCKED) return 2;
    if((usb_class == USBD_CLASS_UVC) && (usbd_param.send_lock == USBD_LOCK_STATUS_FREE)) return 2;

    evt = osMessageGet(usbd_msg_free_q_id, 1);
    if((evt.status != osEventMessage) || (evt.value.v == NULL)) return 1;

    usbd_send_msg = evt.value.v;
    switch(usb_class)
    {
        case USBD_CLASS_UVC:
            usbd_send_msg->evt = USBD_MSG_EVENT_UVC_SEND;
            break;
        case USBD_CLASS_VENDOR:
            usbd_send_msg->evt = USBD_MSG_EVENT_VENDOR_SEND;
            break;
    }
    usbd_send_msg->usb_id = usb_id;
    usbd_send_msg->send_addr = addr;
    usbd_send_msg->send_len = len;

    osMessagePut(usbd_msg_q_id, &evt.value.v, 1);

    usbd_param.send_lock = USBD_LOCK_STATUS_LOCKED;
    return 0;
}
/**
  *@brief Flush USB
  *@param usb_id: USB port
  *@retval Result (0:Success Others:Failed)
  */
INT8U usbd_task_flush(INT8U usb_id)
{
    if((usb_id != USB_A) && (usb_id != USB_B)) return 1;
    usbd_param.send_lock = USBD_LOCK_STATUS_UNLOCK;
    drv_l1_usbd_iso_ep5_flush(usb_id);
    drv_l2_usbd_vendor_flush(usb_id);
    return 0;
}
/**
  *@brief Register USB descriptor
  *@param type: Descriptor Type
  *@param *desc: Descriptor
  *@retval Result (0:Success Othres:Failed)
  */
INT8U usbd_task_register_desc(USBDDescType_Typedef type, INT8U* desc)
{
    switch(type)
    {
        case USBD_DESC_TYPE_DEVICE:
            usbd_desc.device = desc;
            return 0;
        case USBD_DESC_TYPE_CONFIG:
            usbd_desc.config = desc;
            return 0;
        case USBD_DESC_TYPE_QUALIFIER:
            usbd_desc.qualifier = desc;
            return 0;
        case USBD_DESC_TYPE_STRING0:
            usbd_desc.string0 = desc;
            return 0;
        case USBD_DESC_TYPE_STRING1:
            usbd_desc.string1 = desc;
            return 0;
        case USBD_DESC_TYPE_STRING2:
            usbd_desc.string2 = desc;
            return 0;
        case USBD_DESC_TYPE_STRING3:
            usbd_desc.string3 = desc;
            return 0;
    }
    return 1;
}
/**
  *@brief Register a callback to get USBD event
  *@param *callback: To be registered callback
  *@retval None
  */
void usbd_task_register_event(void (*callback)(USBDEvent_Typedef evt))
{
    usbd_param.post_evt_cb = callback;
}
