/* ************************************************************************** */
/** Common functions of USB devices

  @Company
    Medimaging Integrated Solutions Inc.

  @Author
    Morgan Huang <morgan@miis.com.tw>

  @File Name
    usb_common.c

  @Summary
    Wrapper functions of USB device driver instance

  @Description
    Wrapper functions to share USB device driver instance between USB CDC and 
    vendor applications.
 */
/* ************************************************************************** */

#include "usb_common.h"
#include "cdc.h"
#include "usb_vendor.h"

/*
 * Open device in singleton mode, allowing access from both USB CDC and vendor 
 * app tasks.
 */
static USB_DEVICE_HANDLE usb_device_handle = USB_DEVICE_HANDLE_INVALID;
USB_DEVICE_HANDLE usb_device_open(SYS_MODULE_INDEX instanceIndex,
                                  DRV_IO_INTENT intent)
{
    if(usb_device_handle==USB_DEVICE_HANDLE_INVALID)
    {
        usb_device_handle = USB_DEVICE_Open( instanceIndex, intent);
    }
    return usb_device_handle;
}

/*
 * Wrapper event handler calling both USB CDC and vendor APP. event handlers.
 */
void usb_device_event_handler(USB_DEVICE_EVENT event, void * eventData,
                              uintptr_t context)
{
    CDC_USBDeviceEventHandler(event, eventData, context);
    VENDOR_USBDeviceEventHandler(event, eventData, context);
}

/* *****************************************************************************
 End of File
 */
