#ifndef __PSC700_TASK_H__
#define __PSC700_TASK_H__

/*******************************************************************
 * Public Data Type
 *******************************************************************/
typedef enum{
    PSC700_EVENT_NONE,
    PSC700_EVENT_CAM_READY,
    PSC700_EVENT_LIVE_VIEW,
    PSC700_EVENT_CAPTURE,
    PSC700_EVENT_MOUNT_USB_UVC,
    PSC700_EVENT_UNMOUNT_USB_UVC,
    PSC700_EVENT_USB_UVC_START,
    PSC700_EVENT_USB_UVC_STOP,
    PSC700_EVENT_USB_VENDOR_SEND_COMPLETED,
    PSC700_EVENT_GET_LIVEVIEW,
    PSC700_EVENT_COLLECT_COMPLETE,
    PSC700_EVENT_CAPTURE_COMPLETE,
    PSC700_EVENT_GET_IMG,
    PSC700_EVENT_AF_COMPLETE,
    PSC700_EVENT_IR_CONTROL,
    PSC700_EVENT_FLV_CONTROL,
    PSC700_EVENT_MOTOR_CONTROL,
    PSC700_EVENT_GET_COLLECTED_FRAME,
    PSC700_EVENT_SHUTDOWN,
    PSC700_EVEMT_MAX,
}PSC700Event_Typedef;

/*******************************************************************
 * External Functions
 *******************************************************************/
/**
  *@brief PSC700 task entry
  *@param None
  *@retval None
  */
extern void psc700_task_entry(void);
/**
  *@brief Post event to PSC700 task
  *@param evt: PSC700 Event
  *@param *param: To be posted parameter
  *@retval Result (0:Success Others:Failed)
  */
extern INT8U psc700_task_post_event(PSC700Event_Typedef evt, void *param);
#endif //__PSC700_TASK_H__
