#ifndef __USBD_TASK_H__
#define __USBD_TASK_H__

/*******************************************************************
 * Data Types
 *******************************************************************/
typedef enum{
    USBD_DESC_TYPE_DEVICE = 0,
    USBD_DESC_TYPE_CONFIG,
    USBD_DESC_TYPE_QUALIFIER,
    USBD_DESC_TYPE_STRING0,
    USBD_DESC_TYPE_STRING1,
    USBD_DESC_TYPE_STRING2,
    USBD_DESC_TYPE_STRING3,
    USBD_DESC_TYPE_MAX,
}USBDDescType_Typedef;
typedef enum{
    USBD_EVENT_NONE = 0,
    USBD_EVENT_MOUNT_SUCCESS,
    USBD_EVENT_MOUNT_FAILED,
    USBD_EVENT_UVC_START,
    USBD_EVENT_UVC_STOP,
    USBD_EVENT_VENDOR_SEND_COMPLETED,
    USBD_EVENT_MAX,
}USBDEvent_Typedef;
typedef enum{
    USBD_CLASS_UVC = 0,
    USBD_CLASS_VENDOR,
    USBD_CLASS_MAX,
}USBDClass_Typedef;
/*******************************************************************
 * External Functions
 *******************************************************************/
/**
  *@brief USBD Task Entry
  *@param stack_size: Stack size for task
  *@retval None
  */
extern void usbd_task_entry(INT32U stack_size);
/**
  *@brief Mount USBD UVC
  *@param usb_port: USB Port
  *@retval Result (0:Success Others:Failed)
  */
extern INT8U usbd_task_mount_uvc(INT8U usb_port);
/**
  *@brief Unmount USBD
  *@param usb_port: USB Port (USB_A, USB_B)
  *@retval None
  */
extern void usbd_task_unmount(INT8U usb_port);
/**
  *@brief Flush USB
  *@param usb_id: USB port
  *@retval Result (0:Success Others:Failed)
  */
extern INT8U usbd_task_flush(INT8U usb_id);
/**
  *@brief Register USB descriptor
  *@param type: Descriptor Type
  *@param *desc: Descriptor
  *@retval Result (0:Success Othres:Failed)
  */
extern INT8U usbd_task_register_desc(USBDDescType_Typedef type, INT8U* desc);
/**
  *@brief Register event callback
  *@param *callback: To be registered callback
  *@retval None
  */
extern void usbd_task_register_event(void (*callback)(USBDEvent_Typedef evt));

#endif //__USBD_TASK_H__
