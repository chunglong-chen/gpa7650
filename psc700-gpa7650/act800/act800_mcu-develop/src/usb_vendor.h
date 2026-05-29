/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    usb_vendor.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef _USB_VENDOR_H
#define _USB_VENDOR_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "configuration.h"
#include "definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application States

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    /* Application's state machine's initial state. */
    VENDOR_STATE_INIT=0,

    /* Application waits for device configuration */
    VENDOR_STATE_WAIT_FOR_CONFIGURATION,

    /* Application runs the main task */
    VENDOR_STATE_MAIN_TASK,

    /* Application error occurred */
    VENDOR_STATE_ERROR

} VENDOR_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
   /* Device layer handle returned by device layer open function */
    USB_DEVICE_HANDLE usbDevHandle;

    /* Application state*/
    VENDOR_STATES state;

    /* Track device configuration */
    bool deviceIsConfigured;

    /* Configuration value */
    uint8_t configValue;

    /* speed */
    USB_SPEED speed;

    /* ep data sent */
    bool epDataWritePending;

    /* ep data received */
    bool epDataReadPending;

    /* Transfer handle */
    USB_DEVICE_TRANSFER_HANDLE writeTranferHandle;

    /* Transfer handle */
    USB_DEVICE_TRANSFER_HANDLE readTranferHandle;

    /* The transmit endpoint address */
    USB_ENDPOINT_ADDRESS endpointTx;

    /* The receive endpoint address */
    USB_ENDPOINT_ADDRESS endpointRx;

    /* Tracks the alternate setting */
    uint8_t altSetting;
    
    /* True is switch was pressed */
    bool isSwitchPressed;

    /* True if the switch press needs to be ignored*/
    bool ignoreSwitchPress;

    /* Flag determines SOF event occurrence */
    bool sofEventHasOccurred;
    
    /* Switch debounce timer */
    unsigned int switchDebounceTimer;

    /* Switch debounce timer count */
    unsigned int debounceCount;

    /* The endpoint size is 64 for FS and 512 for HS */
    uint16_t endpointMaxPktSize;

} VENDOR_DATA;

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void USB_VENDOR_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    USB_VENDOR_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void USB_VENDOR_Initialize ( void );

/*******************************************************************************
  Function:
    void USB_VENDOR_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    USB_VENDOR_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void USB_VENDOR_Tasks ( void );

/*******************************************************************************
  Function:
    void VENDOR_USBDeviceEventHandler(USB_DEVICE_EVENT, void *, uintptr_t)

  Summary:
     USB device layer event handler

  Returns:
    None.

  Remarks:
    This routine is called from usb_device_event_handler(...) wrapper function 
    in the usb_common.c.
 */
void VENDOR_USBDeviceEventHandler(USB_DEVICE_EVENT, void *, uintptr_t);

// translate waveform packets and fill the buffer
int process_waveform_packet(uint8_t *data);

// translate parameter packets and fill the variables
int process_parameter_packet(uint8_t *data);

// translate trigger packets and fill the buffer
int process_trigger_packet(uint8_t *data);

#endif /* _USB_VENDOR_H */
/*******************************************************************************
 End of File
 */

