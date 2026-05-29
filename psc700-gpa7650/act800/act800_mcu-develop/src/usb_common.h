/* ************************************************************************** */
/** Common functions of USB devices

  @Company
    Medimaging Integrated Solutions Inc.

  @Author
    Morgan Huang <morgan@miis.com.tw>

  @File Name
    usb_common.h

  @Summary
    Wrapper functions of USB device driver instance

  @Description
    Wrapper functions to share USB device driver instance between USB CDC and 
    vendor applications.
 */
/* ************************************************************************** */

#ifndef _USB_COMMON_H
#define _USB_COMMON_H

#include "definitions.h"

#ifdef __cplusplus
extern "C" {
#endif

USB_DEVICE_HANDLE usb_device_open(SYS_MODULE_INDEX , DRV_IO_INTENT);
void usb_device_event_handler(USB_DEVICE_EVENT, void* , uintptr_t);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _USB_COMMON_H */

/* *****************************************************************************
 End of File
 */
