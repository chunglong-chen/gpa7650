/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    usb_vendor.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "usb_vendor.h"
#include "usb_common.h"
#include "mems.h"
#include "stdio.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// debug print
//#define DEBUG_PRINT
//#define DEBUG_PRINT_WAVEFORM

#ifdef DEBUG_PRINT
#define DBG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DBG_PRINTF(...)
#endif

#ifdef DEBUG_PRINT_WAVEFORM
#define DBG_PRINTF_WAVEFORM(...) printf(__VA_ARGS__)
#else
#define DBG_PRINTF_WAVEFORM(...)
#endif

#define APP_EP_BULK_OUT 3
#define APP_EP_BULK_IN 3

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

VENDOR_DATA vendorData;

/* Receive data buffer */
uint8_t receivedDataBuffer[512] CACHE_ALIGN;

/* Transmit data buffer */
uint8_t  transmitDataBuffer[512] CACHE_ALIGN;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/*********************************************
 * USB Device Layer Event Handler
 *********************************************/

void VENDOR_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context)
{
    uint8_t * configurationValue;
    USB_SETUP_PACKET * setupPacket;
    switch(event)
    {
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:
            vendorData.deviceIsConfigured = false;
            break;
        
        case USB_DEVICE_EVENT_CONFIGURED:
            /* Check the configuration */
            configurationValue = (uint8_t *)eventData;
            if(*configurationValue == 1 )
            {
                /* Reset endpoint data send & receive flag  */
                vendorData.deviceIsConfigured = true;
            }
            break;
            
        case USB_DEVICE_EVENT_SUSPENDED:
            /* Device is suspended. */ 
            break;
            
        case USB_DEVICE_EVENT_POWER_DETECTED:
            /* VBUS is detected. Attach the device */
            USB_DEVICE_Attach(vendorData.usbDevHandle);
            break;
            
        case USB_DEVICE_EVENT_POWER_REMOVED:
            /* VBUS is removed. Detach the device */
            USB_DEVICE_Detach (vendorData.usbDevHandle);
            break;
            
        case USB_DEVICE_EVENT_CONTROL_TRANSFER_SETUP_REQUEST:
            /* This means we have received a setup packet */
            setupPacket = (USB_SETUP_PACKET *)eventData;
            if(setupPacket->bRequest == USB_REQUEST_SET_INTERFACE)
            {
                /* If we have got the SET_INTERFACE request, we just acknowledge
                 for now. This demo has only one alternate setting which is already
                 active. */
                USB_DEVICE_ControlStatus(vendorData.usbDevHandle,USB_DEVICE_CONTROL_STATUS_OK);
            }
            else if(setupPacket->bRequest == USB_REQUEST_GET_INTERFACE)
            {
                /* We have only one alternate setting and this setting 0. So
                 * we send this information to the host. */
                USB_DEVICE_ControlSend(vendorData.usbDevHandle, &vendorData.altSetting, 1);
            }
            else
            {
                /* We have received a request that we cannot handle. Stall it*/
                USB_DEVICE_ControlStatus(vendorData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
            }
            break;
            
        case USB_DEVICE_EVENT_ENDPOINT_READ_COMPLETE:
           /* Endpoint read is complete */
            vendorData.epDataReadPending = false;
            break;
            
        case USB_DEVICE_EVENT_ENDPOINT_WRITE_COMPLETE:
            /* Endpoint write is complete */
            vendorData.epDataWritePending = false;
            break;
        
        /* These events are not used*/
        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void USB_VENDOR_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    vendorData.state = VENDOR_STATE_INIT;
    vendorData.usbDevHandle = USB_DEVICE_HANDLE_INVALID;
    vendorData.deviceIsConfigured = false;
    vendorData.endpointRx = (APP_EP_BULK_OUT | USB_EP_DIRECTION_OUT);
    vendorData.endpointTx = (APP_EP_BULK_IN | USB_EP_DIRECTION_IN);
    vendorData.epDataReadPending = false;
    vendorData.epDataWritePending = false;
    vendorData.altSetting = 0;
}


/******************************************************************************
  Function:
    void USB_VENDOR_Tasks ( void )

  Remarks:
    See prototype in usb_vendor.h.
 */

void USB_VENDOR_Tasks (void )
{
    switch(vendorData.state)
    {
        case VENDOR_STATE_INIT:
            /* Open the device layer */
            //appData.usbDevHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );
            vendorData.usbDevHandle = usb_device_open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );
            
            if(vendorData.usbDevHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(vendorData.usbDevHandle,  usb_device_event_handler, 0);
                
                vendorData.state = VENDOR_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }
            break;
            
        case VENDOR_STATE_WAIT_FOR_CONFIGURATION:
            /* Check if the device is configured */
            if(vendorData.deviceIsConfigured == true)
            {
                if (USB_DEVICE_ActiveSpeedGet(vendorData.usbDevHandle) == USB_SPEED_FULL)
                {
                    vendorData.endpointMaxPktSize = 64;
                }
                else if (USB_DEVICE_ActiveSpeedGet(vendorData.usbDevHandle) == USB_SPEED_HIGH)
                {
                    vendorData.endpointMaxPktSize = 512;
                }
                if (USB_DEVICE_EndpointIsEnabled(vendorData.usbDevHandle, vendorData.endpointRx) == false )
                {
                    /* Enable Read Endpoint */
                    USB_DEVICE_EndpointEnable(vendorData.usbDevHandle, 2, vendorData.endpointRx,
                            USB_TRANSFER_TYPE_BULK, vendorData.endpointMaxPktSize);
                }
                if (USB_DEVICE_EndpointIsEnabled(vendorData.usbDevHandle, vendorData.endpointTx) == false )
                {
                    /* Enable Write Endpoint */
                    USB_DEVICE_EndpointEnable(vendorData.usbDevHandle, 2, vendorData.endpointTx,
                            USB_TRANSFER_TYPE_BULK, vendorData.endpointMaxPktSize);
                }
                /* Indicate that we are waiting for read */
                vendorData.epDataReadPending = true;
                /* Place a new read request. */
                USB_DEVICE_EndpointRead(vendorData.usbDevHandle, &vendorData.readTranferHandle,
                        vendorData.endpointRx, &receivedDataBuffer[0], sizeof(receivedDataBuffer) );
                /* Device is ready to run the main task */
                vendorData.state = VENDOR_STATE_MAIN_TASK;
            }
            break;
            
        case VENDOR_STATE_MAIN_TASK:
            if(!vendorData.deviceIsConfigured)
            {
                /* This means the device got deconfigured. Change the
                 * application state back to waiting for configuration. */
                vendorData.state = VENDOR_STATE_WAIT_FOR_CONFIGURATION;
                
                /* Disable the endpoint*/
                USB_DEVICE_EndpointDisable(vendorData.usbDevHandle, vendorData.endpointRx);
                USB_DEVICE_EndpointDisable(vendorData.usbDevHandle, vendorData.endpointTx);
                vendorData.epDataReadPending = false;
                vendorData.epDataWritePending = false;
            }
            else if (vendorData.epDataReadPending == false)
            {
                /* Look at the data the host sent, to see what kind of
                 * application specific command it sent. */
                switch(receivedDataBuffer[0])
                {
                    /* DEMO FUNCTIONS *****************************************/
                    case 0x80: // Demo Rx (PC => this device)
                        /* This is the toggle LED command */
//                        LED0_Toggle();
                        break;
                    case 0x81: // Demo Tx (this device => PC)
                        if(vendorData.epDataWritePending == false)
                        {
                            transmitDataBuffer[0] = 0x81;
                            transmitDataBuffer[1] = 0xAA;
                            /* Send the data to the host */
                            vendorData.epDataWritePending = true;
                            USB_DEVICE_EndpointWrite ( vendorData.usbDevHandle, &vendorData.writeTranferHandle,
                                    vendorData.endpointTx, &transmitDataBuffer[0],
                                    sizeof(transmitDataBuffer),
                                    USB_DEVICE_TRANSFER_FLAGS_MORE_DATA_PENDING);
                        }
                        break;
                    /* END OF DEMO FUNCTIONS **********************************/
                    
                    // waveform data packet
                    case 'x':
                    case 'y':
                        process_waveform_packet(receivedDataBuffer);
                        break;
                    // trigger parameter and data packet
                    case 't':
                        switch(receivedDataBuffer[1])
                        {
                            case 'n':
                            case 's':
                            case 'p':
                                process_parameter_packet(receivedDataBuffer);
                                break;
                            case 'a':
                                process_trigger_packet(receivedDataBuffer);
                                break;
                        }
                        break;
                    
                    // parameter packets
                    case 'l': // waveform length
                    case 'c': // scan count
                    case 'b': // begin voltage
                    case 'e': // end voltage
                    case 'f': // low-pass filter frequency
                        process_parameter_packet(receivedDataBuffer);
                        break;
                    
                    default:
                        printf("USB vendor: unknown packet\r\n");
                        break;
                }

                vendorData.epDataReadPending = true ;

                /* Place a new read request. */
                USB_DEVICE_EndpointRead ( vendorData.usbDevHandle, &vendorData.readTranferHandle,
                        vendorData.endpointRx, &receivedDataBuffer[0], sizeof(receivedDataBuffer) );
            }
            break;
            
        case VENDOR_STATE_ERROR:
        default:
            break;
    }
}

#define BYTES_TO_UINT32(B) (*(B)<<24|*((B)+1)<<16|*((B)+2)<<8|*((B)+3))
#define BYTES_TO_UINT16(B) (*(B)<< 8|*((B)+1))

/*******************************************************************************
 * int process_parameter_packet(uint8_t *data)
 *    Translate parameter packet and fill the variable.
 * 
 * Packet formation:
 *     Item: [label][checksum][variable]
 *     Size: [ 2 B ][2 Bytes ][4 Bytes ]
 *    Index: [ 0-1 ][  2-3   ][  4-7   ]
 * 
 * Checksum:
 *     Summation over all 2-byte groups in
 *         {bytes[i] | i in [0-1], [4-7]}
 *     and truncate to 16 bits.
 *     Note that:
 *         The [checksum] bytes are excluded.
 ******************************************************************************/
int process_parameter_packet(uint8_t *data)
{
    uint16_t checksum_local=0;
    checksum_local+=BYTES_TO_UINT16(data);
    
    uint16_t checksum_remote=BYTES_TO_UINT16(data+2);
    DBG_PRINTF("checksum_remote=%d\r\n", checksum_remote);
    
    checksum_local+=BYTES_TO_UINT16(data+4);
    checksum_local+=BYTES_TO_UINT16(data+6);
    
    // check data integrity
    DBG_PRINTF("checksum_local=%d\r\n", checksum_local);
    if(checksum_remote != checksum_local){
        printf("USB vendor: parameter packet: mismatched checksum\r\n");
        return -1;
    }
    
    int val=BYTES_TO_UINT32(data+4);
    
    // which buffer to fill
    switch(data[0]){
        case 'l':
            m_wave_data.length=val;
            DBG_PRINTF("waveform length=%d\r\n", val);
            break;
        case 'c':
            m_wave_data.scancnt=val;
            DBG_PRINTF("scan count=%d\r\n", val);
            break;
        case 'f':
            m_wave_data.lowpass_filter_freq=val;
            DBG_PRINTF("low-pass freq=%d\r\n", val);
            break;
        case 't':
            switch(data[1]){
                case 's':
                    m_wave_data.trigger_start=val;
                    DBG_PRINTF("trigger start=%d\r\n", val);
                    break;
                case 'p':
                    m_wave_data.trigger_period=val;
                    DBG_PRINTF("trigger period=%d\r\n", val);
                    break;
                case 'n':
                    m_wave_data.trigger_length=val;
                    DBG_PRINTF("trigger length=%d\r\n", val);
                    break;    
            }
            break;
        case 'b': // start voltage of x-axis and y-axis
            switch(data[1]){
                case 'x':
                    m_wave_data.x_start_volt=val;
                    DBG_PRINTF("x start volt=%d\r\n", val);
                    break;
                case 'y':
                    m_wave_data.y_start_volt=val;
                    DBG_PRINTF("y start volt=%d\r\n", val);
                    break;
            }
            break;
        case 'e': // stop voltage of x-axis and y-axis
            switch(data[1]){
                case 'x':
                    m_wave_data.x_stop_volt=val;
                    DBG_PRINTF("x stop volt=%d\r\n", val);
                    break;
                case 'y':
                    m_wave_data.y_stop_volt=val;
                    DBG_PRINTF("y stop volt=%d\r\n", val);
                    break;
            }
            break;
        default:
            return -1; //bad data label
    }
    return 0;
}

/*******************************************************************************
 * int process_waveform_packet(uint8_t *data)
 *    Translate waveform packet and fill the waveform buffer.
 * 
 * Packet formation:
 *     Item: [label][checksum][start index][length][waveform 0][waveform 1]...
 *     Size: [ 2 B ][2 Bytes ][  4 Bytes  ][ 4 B  ][ 2 Bytes  ][ 2 Bytes  ]...
 *    Index: [ 0-1 ][  2-3   ][    4-7    ][ 8-11 ][    12-(N=11+length*2)    ]
 * 
 * Checksum:
 *     Summation over all 2-byte groups in
 *         {bytes[i] | i in [0-1], [4-11], [12-N]}
 *     and truncate to 16 bits.
 *     Note that:
 *         The [checksum] bytes are excluded.
 *         Bytes beyond N are excluded.
 * 
 * Maximum packet size = 512 bytes
 ******************************************************************************/
int process_waveform_packet(uint8_t *data)
{
    int idx=0;
    uint16_t checksum_local=0;
    uint16_t* wave_buf=NULL;
    
    // data label: which buffer to fill
    switch(data[idx]){
        case 'x':
            wave_buf=m_wave_data.ptr_x;
            break;
        case 'y':
            wave_buf=m_wave_data.ptr_y;
            break;
        default: //bad data label
            return -1;
    }
    checksum_local += BYTES_TO_UINT16(data+idx);
    idx+=2;
    
    // get checksum from remote
    uint16_t checksum_remote = BYTES_TO_UINT16(data+idx);
    DBG_PRINTF_WAVEFORM("checksum_remote=%d\r\n", checksum_remote);
    idx+=2;
    
    // get start index
    int start = BYTES_TO_UINT32(data+idx);
    DBG_PRINTF_WAVEFORM("start=%d\r\n", start);
    checksum_local += BYTES_TO_UINT16(data+idx);
    checksum_local += BYTES_TO_UINT16(data+idx+2);
    idx+=4;
    
    // get waveform length in this packet
    int length = BYTES_TO_UINT32(data+idx);
    DBG_PRINTF_WAVEFORM("length=%d\r\n", length);
    checksum_local += BYTES_TO_UINT16(data+idx);
    checksum_local += BYTES_TO_UINT16(data+idx+2);
    idx+=4;
    
    // check size validity
    if(idx + length*2 > 512){ //exceed packet buffer size
        printf("USB vendor: waveform exceeds packet buffer size\r\n");
        return -1;
    }
    if(start + length > DATA_ARRAY_SIZE){ //exceed waveform buffer size
        printf("USB vendor: waveform exceeds waveform buffer size\r\n");
        return -1;
    }
    
    // fill buffer
    for(int j=0; j<length; j++)
    {
        wave_buf[start+j] = BYTES_TO_UINT16(data+idx);
        checksum_local += wave_buf[start+j];
        idx+=2;
    }
    DBG_PRINTF_WAVEFORM("checksum_local=%d\r\n", checksum_local);
    
    // check data integrity
    if(checksum_remote != checksum_local){
        printf("USB vendor: waveform packet: mismatched checksum\r\n");
        return -1;
    }
    
    return 0;
}

/*******************************************************************************
 * int process_trigger_packet(uint8_t *data)
 *    Translate trigger packet and fill the trigger buffer.
 * 
 * Packet formation:
 *     Item: [label][checksum][start index][length][trigger 0][trigger 1]...
 *     Size: [ 2 B ][2 Bytes ][  4 Bytes  ][ 4 B  ][ 4 Bytes  ][ 4 Bytes  ]...
 *    Index: [ 0-1 ][  2-3   ][    4-7    ][ 8-11 ][    12-(N=11+length*4)    ]
 * 
 * Checksum:
 *     Summation over all 2-byte groups in
 *         {bytes[i] | i in [0-1], [4-11], [12-N]}
 *     and truncate to 16 bits.
 *     Note that:
 *         The [checksum] bytes are excluded.
 *         Bytes beyond N are excluded.
 * 
 * Maximum packet size = 512 bytes
 ******************************************************************************/
int process_trigger_packet(uint8_t *data)
{
    int idx=0;
    uint16_t checksum_local=0;
    uint32_t* trigger_buf=NULL;
    trigger_buf = m_wave_data.ptr_trigger;

    checksum_local += BYTES_TO_UINT16(data+idx);
    idx+=2;
    
    // get checksum from remote
    uint16_t checksum_remote = BYTES_TO_UINT16(data+idx);
    DBG_PRINTF_WAVEFORM("checksum_remote=%d\r\n", checksum_remote);
    idx+=2;
    
    // get start index
    int start = BYTES_TO_UINT32(data+idx);
    DBG_PRINTF_WAVEFORM("start=%d\r\n", start);
    checksum_local += BYTES_TO_UINT16(data+idx);
    checksum_local += BYTES_TO_UINT16(data+idx+2);
    idx+=4;
    
    // get waveform length in this packet
    int length = BYTES_TO_UINT32(data+idx);
    DBG_PRINTF_WAVEFORM("length=%d\r\n", length);
    checksum_local += BYTES_TO_UINT16(data+idx);
    checksum_local += BYTES_TO_UINT16(data+idx+2);
    idx+=4;
    
    // check size validity
    if(idx + length*4 > 512){ //exceed packet buffer size
        printf("USB vendor: trigger exceeds packet buffer size\r\n");
        return -1;
    }
    if(start + length > DATA_ARRAY_SIZE){ //exceed waveform buffer size
        printf("USB vendor: trigger exceeds waveform buffer size\r\n");
        return -1;
    }
    
    // fill buffer
    for(int j=0; j<length; j++)
    {
        trigger_buf[start+j] = BYTES_TO_UINT32(data+idx);
        checksum_local += trigger_buf[start+j];
        idx+=4; // 4 bytes
    }
    DBG_PRINTF_WAVEFORM("checksum_local=%d\r\n", checksum_local);
    
    // check data integrity
    if(checksum_remote != checksum_local){
        printf("USB vendor: trigger packet: mismatched checksum\r\n");
        return -1;
    }
    
    return 0;
}
/*******************************************************************************
 End of File
 */
