/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
/**
 * @file    cdc.c
 * @brief   USB CDC to UART bridge and command processing module.
 * @details This module implements USB CDC communication, UART bridging,
 *          and command parsing functionality. It handles data transfer
 *          between USB host and UART peripheral, and processes incoming
 *          commands via CDC interface.
 * @ingroup uart_transceiver
 */
#include "cdc.h"
#include "proc_cmd.h"
#include "peripheral/sercom/usart/plib_sercom1_usart.h"
#include "usb_common.h"

uint8_t USB_ALIGN cdcReadBuffer[CDC_READ_BUFFER_SIZE] ;
uint8_t USB_ALIGN uartReadBuffer[CDC_READ_BUFFER_SIZE];

static CDC_DATA cdcData;

static char cmd_buf[CMD_DATA_SIZE];
static uint16_t idx=0;
static bool cmd_ready;
/**
 * @brief   USART buffer event handler.
 * @details Handles USART buffer events and updates internal transfer status
 *          flags when read or write operations are completed.
 *
 * @param[in] bufferEvent   USART buffer event type.
 * @param[in] bufferHandle  Buffer handle associated with the event.
 * @param[in] context       User-defined context pointer. This parameter is
 *                          currently unused.
 */
void CDC_BufferEventHandler(DRV_USART_BUFFER_EVENT bufferEvent, 
                            DRV_USART_BUFFER_HANDLE bufferHandle, 
                            uintptr_t context )
{
    switch(bufferEvent)
    {
        case DRV_USART_BUFFER_EVENT_COMPLETE:
            
            if(cdcData.readBufferHandler == bufferHandle)
            {
                if(cdcData.isUSARTReadInProgress == true)
                {
                    cdcData.isUSARTReadInProgress = false;
                }
            }
            
            if(cdcData.writeBufferHandler == bufferHandle)
            {
                if(cdcData.isUSARTWriteInProgress == true)
                {
                    cdcData.isUSARTWriteInProgress = false;
                }
            }
            
            break;
        
        case DRV_USART_BUFFER_EVENT_ERROR:  
            // If a UART error occurs, do nothing
            break;        

        default:        
            break;        
    }
}

/**
 * @brief   USB CDC class event handler.
 * @details Handles CDC class-specific events, including line coding requests,
 *          control line state changes, read completion, write completion,
 *          and control transfer status updates.
 *
 * @param[in] index     CDC instance index.
 * @param[in] event     CDC event type.
 * @param[in] pData     Pointer to event-specific data.
 * @param[in] userData  Pointer to application-specific CDC data object.
 *
 * @return  Returns USB_DEVICE_CDC_EVENT_RESPONSE_NONE.
 */
USB_DEVICE_CDC_EVENT_RESPONSE CDC_USBDeviceCDCEventHandler(
                                                    USB_DEVICE_CDC_INDEX index ,
                                                    USB_DEVICE_CDC_EVENT event ,
                                                    void* pData,
                                                    uintptr_t userData)
{
    USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE * eventDataRead;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;
    CDC_DATA * appDataObject = (CDC_DATA *)userData;

    switch ( event )
    {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

             USB_DEVICE_ControlSend(appDataObject->usbDevHandle,
                    &appDataObject->getLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */
            appDataObject->isSetLineCodingCommandInProgress = true;
            appDataObject->isBaudrateDataReceived = false;
            USB_DEVICE_ControlReceive(appDataObject->usbDevHandle,
                    &appDataObject->setLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *)pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *) pData)->breakDuration;
            
            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(appDataObject->usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            eventDataRead = (USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE *)pData;
            
            appDataObject->isCDCReadInProgress = false;
            
            if(eventDataRead->status != USB_DEVICE_CDC_RESULT_ERROR)
            {                
                appDataObject->readLength = eventDataRead->length;
            }
                
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. */
            if (appDataObject->isSetLineCodingCommandInProgress == true)
            {
               /* We have received set line coding command from the Host.
                * DRV_USART_BaudSet() function is not interrupt safe and it
                * should not be called here. It is called in APP_Tasks()
                * function. The ACK for Status stage of the control transfer is
                * send in the APP_Tasks() function.  */
                appDataObject->isSetLineCodingCommandInProgress = false;
                appDataObject->isBaudrateDataReceived = true;
            }
            else
            {
				/* ACK the Status stage of the Control transfer */
                USB_DEVICE_ControlStatus(appDataObject->usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);
            }
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We don't
             * do much with this data in this demo. */
            break;
            
        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_ABORTED:
        
            /* The control transfer has been aborted */
            if (appDataObject->isSetLineCodingCommandInProgress == true)
            {
                appDataObject->isSetLineCodingCommandInProgress = false;
                appDataObject->isBaudrateDataReceived = false;
            }

            break;
        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */
            appDataObject->isCDCWriteInProgress = false;

            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/**
 * @brief   USB device event handler.
 * @details Handles USB device-level events such as device reset,
 *          deconfiguration, configuration, suspend, resume,
 *          power detection, and power removal.
 *
 * @param[in] event      USB device event type.
 * @param[in] eventData  Pointer to event-specific data.
 * @param[in] context    User-defined context pointer.
 */
void CDC_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData,
                               uintptr_t context)
{
    uint8_t configurationValue;
    switch(event)
    {
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:

            /* USB device is reset or device is deconfigured.
             * This means that USB device layer is about to deininitialize
             * all function drivers. Update LEDs to indicate
             * reset/deconfigured state. */

            cdcData.deviceIsConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuration */
            configurationValue = ((USB_DEVICE_EVENT_DATA_CONFIGURED *)eventData)->configurationValue;
            if (configurationValue == 1)
            {
                /* The device is in configured state. Update LED indication */

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, CDC_USBDeviceCDCEventHandler, (uintptr_t)&cdcData);

                /* mark that set configuration is complete */
                cdcData.deviceIsConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Update LEDs */
            break;


        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS is detected. Attach the device */
            USB_DEVICE_Attach(cdcData.usbDevHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is removed. Detach the device */
            USB_DEVICE_Detach (cdcData.usbDevHandle);
            cdcData.deviceIsConfigured = false;

            break;

        /* These events are not used in this demo. */
        case USB_DEVICE_EVENT_RESUMED:
            if(cdcData.deviceIsConfigured == true)
            {
            }
            break;
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

/**
 * @brief   Reset CDC runtime state when the device is not configured.
 * @details Resets CDC state machine variables, transfer handles, and
 *          status flags when the USB device is no longer configured.
 *
 * @return  true if the CDC state was reset.
 * @return  false if the device is still configured and no reset was performed.
 */
bool CDC_StateReset(void)
{
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if(cdcData.deviceIsConfigured == false)
    {
        cdcData.state = CDC_STATE_WAIT_FOR_CONFIGURATION;
        cdcData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        cdcData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        cdcData.isCDCReadInProgress = false;
        cdcData.isCDCWriteInProgress = false;
        cdcData.errorStatus = false;

        cdcData.isSetLineCodingCommandInProgress = false;
        cdcData.isBaudrateDataReceived = false;
        
        retVal = true;
    }
    else
    {
        retVal = false;
    }

    return(retVal);
}

/**
 * @brief   Apply USB CDC line coding settings to the USART interface.
 * @details Updates USART serial parameters, including baud rate, parity,
 *          stop bits, and data width, according to the line coding
 *          received from the USB host. After successful configuration,
 *          USART read operation is restarted.
 *
 * @note    This function is executed in task context rather than interrupt
 *          context because the USART serial setup routine is not interrupt safe.
 */
void _CDC_SetLineCodingHandler(void)
{    
    DRV_USART_SERIAL_SETUP UsartSetup;

    UsartSetup.baudRate = cdcData.setLineCodingData.dwDTERate;
    UsartSetup.parity = cdcData.setLineCodingData.bParityType;
    UsartSetup.stopBits = cdcData.setLineCodingData.bCharFormat; 
    if(cdcData.setLineCodingData.bDataBits <= (uint8_t)DRV_USART_DATA_9_BIT)
    {
        UsartSetup.dataWidth = cdcData.setLineCodingData.bDataBits - CDC_USART_USB_DATA_WIDTH_DIFF; 
    }
    else
    {
        /* If it is a non-supported data width, we currently set it to 8-bit 
         * mode value. */
        UsartSetup.dataWidth = DRV_USART_DATA_8_BIT; 
    }

    DRV_USART_ReadQueuePurge(cdcData.usartHandle);
    DRV_USART_WriteQueuePurge(cdcData.usartHandle);
    DRV_USART_ReadAbort(cdcData.usartHandle);
    
    if (true == DRV_USART_SerialSetup(cdcData.usartHandle, &UsartSetup))
    {
        /* Baudrate is changed successfully. Update Baudrate info in the
         * Get line coding structure. */

        cdcData.getLineCodingData.dwDTERate = cdcData.setLineCodingData.dwDTERate;
        cdcData.getLineCodingData.bParityType = cdcData.setLineCodingData.bParityType;
        cdcData.getLineCodingData.bDataBits = cdcData.setLineCodingData.bDataBits;
        cdcData.getLineCodingData.bCharFormat = cdcData.setLineCodingData.bCharFormat;
                                
        DRV_USART_ReadBufferAdd(cdcData.usartHandle, cdcData.uartReadBuffer, UART_READ_BUFFER_SIZE, &cdcData.readBufferHandler);
        
        /* Acknowledge the Status stage of the Control transfer */
        USB_DEVICE_ControlStatus(cdcData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);
    }
    else
    {
        /* Baudrate was not set. There are two ways that an unsupported
         * baud rate could be handled.  The first is just to ignore the
         * request and ACK the control transfer.  That is what is currently
         * implemented below. */
         USB_DEVICE_ControlStatus(cdcData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);


        /* The second possible method is to stall the STATUS stage of the
         * request. STALLing the STATUS stage will cause an exception to be
         * thrown in the requesting application. Some programs, like
         * HyperTerminal, handle the exception properly and give a pop-up
         * box indicating that the request settings are not valid.  Any
         * application that does not handle the exception correctly will
         * likely crash when this request fails.  For the sake of example
         * the code required to STALL the status stage of the request is
         * provided below.  It has been left out so that this demo does not
         * cause applications without the required exception handling to
         * crash.*/
         //USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
    }
}

/**
 * @brief   Initialize the CDC module.
 * @details Initializes the CDC application data structure, buffer pointers,
 *          transfer handles, default line coding parameters, and internal
 *          state variables required for USB CDC and USART communication.
 */
void CDC_Initialize ( void )
{
     /* Device Layer Handle  */
    cdcData.usbDevHandle = USB_DEVICE_HANDLE_INVALID;

    /* USART Driver Handle */
    cdcData.usartHandle = DRV_HANDLE_INVALID;

    /* CDC Instance index for this app object00*/
    cdcData.cdcInstance = USB_DEVICE_CDC_INDEX_0;

    /* app state */
    cdcData.state = CDC_STATE_INIT;

    /* device configured status */
    cdcData.deviceIsConfigured = false;

    /* Initial get line coding state */
    cdcData.getLineCodingData.dwDTERate = 9600;
    cdcData.getLineCodingData.bDataBits = 8;
    cdcData.getLineCodingData.bParityType = 0;
    cdcData.getLineCodingData.bCharFormat = 0;

    /* Read Transfer Handle */
    cdcData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    cdcData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Initialize the read complete flag */
    cdcData.isCDCReadInProgress = false;

    /*Initialize the write complete flag*/
    cdcData.isCDCWriteInProgress = false;

    /*Initialize the buffer pointers */
    cdcData.cdcReadBuffer = &cdcReadBuffer[0];

    cdcData.uartReadBuffer = &uartReadBuffer[0];
    
    memset(cdcReadBuffer, 0, sizeof(cdcReadBuffer));
    memset(uartReadBuffer, 0, sizeof(uartReadBuffer));
    

    cdcData.uartTxCount = 0;
    cdcData.errorStatus = false;
    
    cdcData.usartHandle = DRV_HANDLE_INVALID;
    cdcData.readBufferHandler = DRV_USART_BUFFER_HANDLE_INVALID;  
    cdcData.writeBufferHandler = DRV_USART_BUFFER_HANDLE_INVALID;  

    /* Initialize the Set Line coding flags */
    cdcData.isBaudrateDataReceived = false;
    cdcData.isSetLineCodingCommandInProgress = false;

    cdcData.isUSARTWriteInProgress = false;
    cdcData.isUSARTReadInProgress = false;
}

/**
 * @brief   Execute the CDC application task state machine.
 * @details Implements the main CDC runtime behavior, including USB device
 *          initialization, configuration handling, CDC-to-UART data transfer,
 *          UART-to-CDC data forwarding, and command processing.
 *
 *          The state machine includes the following states:
 *          - CDC_STATE_INIT
 *          - CDC_STATE_WAIT_FOR_CONFIGURATION
 *          - CDC_STATE_SERIAL_EMULATOR
 *          - CDC_STATE_ERROR
 */
void CDC_Tasks ( void )
{

    if ((cdcData.deviceIsConfigured) && (cdcData.isBaudrateDataReceived))
    {
		 cdcData.isBaudrateDataReceived = false;
        _CDC_SetLineCodingHandler();
    }

    /* Update the application state machine based
     * on the current state */
    switch(cdcData.state)
    {
        case CDC_STATE_INIT:

            /* Open the device layer */
            //cdcData.usbDevHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );
            cdcData.usbDevHandle = usb_device_open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );

            if(cdcData.usbDevHandle != USB_DEVICE_HANDLE_INVALID)
            {
                cdcData.usartHandle = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_READWRITE);
                
                if (cdcData.usartHandle != DRV_HANDLE_INVALID)
                {
                    DRV_USART_BufferEventHandlerSet(cdcData.usartHandle, CDC_BufferEventHandler, 0);
                    
                    /* Register a callback with device layer to get event notification (for end point 0) */
                    USB_DEVICE_EventHandlerSet(cdcData.usbDevHandle,  usb_device_event_handler, 0);

                    /* Application waits for device configuration. */
                    cdcData.state = CDC_STATE_WAIT_FOR_CONFIGURATION;
                }
                else
                {
                    cdcData.state = CDC_STATE_ERROR;
                }

            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case CDC_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device is configured */
            if(cdcData.deviceIsConfigured)
            {
                /* Schedule the first read from CDC function driver */
                cdcData.state = CDC_STATE_SERIAL_EMULATOR;                
                cdcData.isCDCReadInProgress = true;
                cdcData.isUSARTReadInProgress = true;            
                cdcData.isCDCWriteInProgress = false;
                cdcData.isUSARTWriteInProgress = false;
                
                USB_DEVICE_CDC_Read (cdcData.cdcInstance, &cdcData.readTransferHandle, cdcData.cdcReadBuffer, CDC_READ_BUFFER_SIZE);
                
                DRV_USART_ReadBufferAdd(cdcData.usartHandle, cdcData.uartReadBuffer, UART_READ_BUFFER_SIZE, &cdcData.readBufferHandler);
            }
            break;

        case CDC_STATE_SERIAL_EMULATOR:
            if(CDC_StateReset())
            {
                break;
            }

            if(cdcData.isCDCReadInProgress == false)
            {
                /* If CDC read is complete, send the received data to the UART. */
                cdcData.isUSARTWriteInProgress = true;
                
                DRV_USART_WriteBufferAdd(cdcData.usartHandle, cdcData.cdcReadBuffer, cdcData.readLength, &cdcData.writeBufferHandler);
                
                if (cdcData.writeBufferHandler != DRV_USART_BUFFER_HANDLE_INVALID)
                {
					cdcData.isCDCReadInProgress = true;
                    
                    /* This means we have sent all the data. We schedule the next CDC Read. */
                    USB_DEVICE_CDC_Read (cdcData.cdcInstance, &cdcData.readTransferHandle, cdcData.cdcReadBuffer, CDC_READ_BUFFER_SIZE);
                    
                    /* write back*/
//                    CDC_write((char*)cdcData.cdcReadBuffer, cdcData.readLength);
//                    while (!SERCOM1_USART_TransmitComplete()) asm("nop");
                    
                    /* process read char */
                    CDC_process_buffer(cdcData.cdcReadBuffer, cdcData.readLength);
                }        
                else
                {
                    cdcData.isUSARTWriteInProgress = false;
                }
                    
            }
            
            /* Check if a character was received on the UART */
            if(cdcData.isUSARTReadInProgress == false)
            {
                cdcData.isCDCWriteInProgress = true;
                cdcData.isUSARTReadInProgress = true;
                
                
                DRV_USART_ReadBufferAdd(cdcData.usartHandle, (void *)cdcData.uartReadBuffer, UART_READ_BUFFER_SIZE, &cdcData.readBufferHandler);
                /* write back */
                SERCOM1_USART_Write((char *)cdcData.uartReadBuffer, UART_READ_BUFFER_SIZE);
                CDC_write((char*)cdcData.uartReadBuffer, UART_READ_BUFFER_SIZE);
                while (!SERCOM1_USART_TransmitComplete()) asm("nop");
                
                CDC_process_buffer(cdcData.uartReadBuffer, UART_READ_BUFFER_SIZE);
                
            }
            
            if(cdcData.errorStatus == true)
            {
                cdcData.state = CDC_STATE_ERROR;
            }
            
            break;

        case CDC_STATE_ERROR:
            
            break;
        default:
            break;
    }
}

/**
 * @brief   Transmit data through the USB CDC interface.
 * @details Sends the specified data buffer to the USB host through the
 *          CDC write transfer mechanism.
 *
 * @param[in] data   Pointer to the data buffer to be transmitted.
 * @param[in] count  Number of bytes to transmit.
 */
void CDC_write(char* data, int count)
{
    USB_DEVICE_CDC_Write(cdcData.cdcInstance, &cdcData.writeTransferHandle,
                    data, count, USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
}

/**
 * @brief   Process received data buffer and extract a complete command.
 * @details Parses the incoming data stream character by character and stores
 *          the received content into an internal command buffer. A command is
 *          considered complete when a carriage return ('\r') or line feed
 *          ('\n') character is received. Backspace character is also supported
 *          for simple command-line editing.
 *
 * @param[in] buffer  Pointer to the received data buffer.
 * @param[in] size    Number of bytes in the received buffer.
 *
 * @note    Calls process_cmd() when a complete command is detected.
 *
 * @warning The command buffer boundary should be protected to avoid overflow.
 */
void CDC_process_buffer(uint8_t * buffer, size_t size)
{
    if (buffer == NULL || size <= 0) {
        return;
    }

    for (size_t i = 0; i < size; ++i) {
        //Increase compatibility; \n(0x0A) for Linux, \r(0x0D) for Windows
        if (buffer[i] == '\r' || buffer[i] == '\n') {
            cmd_buf[idx++] = '\0';
            cmd_ready = true;
            break;
        } else if (buffer[i] == '\b') { // \b 0x08 Backspace
            if (idx > 0) { // Ensure we don't go below zero
                --idx; // Remove the last character
            }
        } else {
            cmd_buf[idx++] = buffer[i];
        }
    }

    if (cmd_ready) {
        process_cmd(cmd_buf, idx);
        idx = 0;
        cmd_ready = false;
    }
}