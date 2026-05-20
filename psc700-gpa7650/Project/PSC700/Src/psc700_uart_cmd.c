/** @file psc700_uart_cmd.c
 *  @brief UART command source file
 *
 *  This file declares all UART command list.
 *  It contains functions to parse the received data and response.
 */
/*******************************************************************
 * Includes
 *******************************************************************/
#include "cmsis_os.h"
#include "psc700_task.h"
#include "console_task.h"
#include "psc700_uart_cmd.h"
#include "psc700_conf.h"
#include "build_info.h"
/*******************************************************************
 * Constants
 *******************************************************************/
#define PSC700_RESP_READY                   "+READY\r\n"
#define PSC700_RESP_OK                      "+OK\r\n"
#define PSC700_RESP_ERR                     "+ERROR\r\n"
#define PSC700_RESP_MOUNTED                 "+MOUNTED\r\n"
/*******************************************************************
 * Private Variable
 *******************************************************************/
static PSC700UARTCmdCAP_Typedef cap_param;
static PSC700UARTCmdIR_Typedef ir_param;
static PSC700UARTCmdFLV_Typedef flv_param;
static PSC700UARTCmdMTR_Typedef mtr_param;
static INT8U get_img_idx;
/*******************************************************************
 * Declare Functions
 *******************************************************************/
/**
  *@brief PSC700 UART Command - Liveview
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_liveview(INT8U *param, INT32U len);
/**
  *@brief PSC700 UART Command - Capture
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_capture(INT8U *param, INT32U len);
/**
  *@brief PSC700 UART Command - C1
          (Part of CAP command group: sets cap_num and flash_led_en)
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static psc700_uart_cmd_capture_part1(INT8U *param, INT32U len);
/**
  *@brief PSC700 UART Command - C2
          (Part of CAP command group: sets flash_led_delay)
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static psc700_uart_cmd_capture_part2(INT8U *param, INT32U len);
/**
  *@brief PSC700 UART Command - C3
          (Part of CAP command group: sets flash_led_on)
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static psc700_uart_cmd_capture_part3(INT8U *param, INT32U len);
/**
  *@brief PSC700 UART Command - C4
          (Part of CAP command group: sets flash_led_current)
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_capture_part4(INT8U *param, INT32U len);
/**
  *@brief PSC700 UART Command - C5
          (Part of CAP command group: sets ir_led_on and af_en)
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_capture_part5(INT8U *param, INT32U len);
/**
  *@brief PSC700 UART Command - Mount UVC
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_mount_uvc(INT8U *param, INT32U len);
/**
  *@brief PSC700 UART Command - Unmount UVC
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_unmount_uvc(INT8U *param, INT32U len);
/**
  *@brief PSC700 UART Command - Get Captured Image
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_get_image(INT8U *param, INT32U len);
/**
  *@brief PSC700 UART Command - IR Control
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_ir_control(INT8U *param, INT32U len);
/**
  *@brief PSC700 UART Command - Flash current Control
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_flash_level_control(INT8U *param, INT32U len);
/**
  *@brief PSC700 UART Command - Control motor
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_motor_control(INT8U *param, INT32U len);
/**
  *@brief PSC700 UART Command - Show version
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_version(INT8U *param, INT32U len);
/**
  *@brief PSC700 UART Command - Get AF motor current position
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_motor_position(INT8U *param, INT32U len);
/**
  *@brief PSC700 UART Command - Get 30 collected frames
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_get_collect_frame(INT8U *param, INT32U len);
/**
  *@brief PSC700 UART Command - Shutdown system
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_shutdown(INT8U *param, INT32U len);
/*******************************************************************
 * Private Functions
 *******************************************************************/
/**
  *@brief PSC700 UART Command - Liveview
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_liveview(INT8U *param, INT32U len)
{
    DBG_PRINT("[PSC700_UART_CMD][LOG][LIVEVIEW] (%d)[%s]\r\n", len, param);
    psc700_task_post_event(PSC700_EVENT_LIVE_VIEW, NULL);
}
/**
  *@brief PSC700 UART Command - Capture
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_capture(INT8U *param, INT32U len)
{
    INT32U i;
    INT8U step = 0;
    DBG_PRINT("[PSC700_UART_CMD][LOG][CAPTURE] (%d)[%s]\r\n", len, param);
    if(len)
    {
        gp_memset(&cap_param, 0, sizeof(PSC700UARTCmdCAP_Typedef));

        for(i=0;i<len;i++)
        {
            if((step == 0) && (param[i] == '='))
            {
                step = 1;
            }
            else
            {
                if(param[i] == ',')
                {
                    step++;
                    continue;
                }
                if((param[i] >= '0') && (param[i] <= '9'))
                {
                    switch(step)
                    {
                        case 1:
                            //Number of captures
                            cap_param.cap_num = (cap_param.cap_num * 10) + (param[i] - '0');
                            break;
                        case 2:
                            //Flash LED switch
                            if((param[i] - '0'))
                            {
                                cap_param.flash_led_en = 1;
                            }
                            else
                            {
                                cap_param.flash_led_en = 0;
                            }
                            break;
                        case 3:
                            //Flash LED delay time
                            cap_param.flash_led_delay = (cap_param.flash_led_delay * 10) + (param[i] - '0');
                            break;
                        case 4:
                            //Flash LED on time
                            cap_param.flash_led_on = (cap_param.flash_led_on * 10) + (param[i] - '0');
                            break;
                        case 5:
                            //Flash LED current
                            cap_param.flash_led_current = (cap_param.flash_led_current * 10) + (param[i] - '0');
                            break;
                        case 6:
                            //IR LED switch
                            cap_param.ir_led_on = (cap_param.ir_led_on * 10) + (param[i] - '0');
                            break;
                        case 7:
                            //Autofocus switch
                            cap_param.af_en = (cap_param.af_en * 10) + (param[i] - '0');
                            break;
                    }
                }
            }
        }
        psc700_task_post_event(PSC700_EVENT_CAPTURE, (void *)&cap_param);
    }
    else
    {
        psc700_task_post_event(PSC700_EVENT_CAPTURE, NULL);
    }
}
/**
  *@brief PSC700 UART Command - C1
          (Part of CAP command group: sets cap_num and flash_led_en)
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static psc700_uart_cmd_capture_part1(INT8U *param, INT32U len)
{
    INT32U i;
    INT8U step = 0;
    DBG_PRINT("[PSC700_UART_CMD][LOG][C1] (%d)[%s]\r\n", len, param);

    if(len)
    {
        gp_memset(&cap_param, 0, sizeof(PSC700UARTCmdCAP_Typedef));

        for(i=0;i<len;i++)
        {
            if((step == 0) && (param[i] == '='))
            {
                step = 1;
            }
            else
            {
                if(param[i] == ',')
                {
                    step++;
                    continue;
                }
                if((param[i] >= '0') && (param[i] <= '9'))
                {
                    switch(step)
                    {
                        case 1:
                            //Number of captures
                            cap_param.cap_num = (cap_param.cap_num * 10) + (param[i] - '0');
                            break;
                        case 2:
                            //Flash LED switch
                            if((param[i] - '0'))
                            {
                                cap_param.flash_led_en = 1;
                            }
                            else
                            {
                                cap_param.flash_led_en = 0;
                            }
                            break;
                    }
                }
            }
        }
    }
}
/**
  *@brief PSC700 UART Command - C2
          (Part of CAP command group: sets flash_led_delay)
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static psc700_uart_cmd_capture_part2(INT8U *param, INT32U len)
{
    INT32U i;
    INT8U step = 0;
    DBG_PRINT("[PSC700_UART_CMD][LOG][C2] (%d)[%s]\r\n", len, param);

    if(len)
    {

        for(i=0;i<len;i++)
        {
            if((step == 0) && (param[i] == '='))
            {
                step = 1;
            }
            else
            {
                if(param[i] == ',')
                {
                    step++;
                    continue;
                }
                if((param[i] >= '0') && (param[i] <= '9'))
                {
                    switch(step)
                    {
                        case 1:
                            //Flash LED delay time
                            cap_param.flash_led_delay = (cap_param.flash_led_delay * 10) + (param[i] - '0');
                            break;
                    }
                }
            }
        }
    }
}
/**
  *@brief PSC700 UART Command - C3
          (Part of CAP command group: sets flash_led_on)
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static psc700_uart_cmd_capture_part3(INT8U *param, INT32U len)
{
    INT32U i;
    INT8U step = 0;
    DBG_PRINT("[PSC700_UART_CMD][LOG][C3] (%d)[%s]\r\n", len, param);

    if(len)
    {

        for(i=0;i<len;i++)
        {
            if((step == 0) && (param[i] == '='))
            {
                step = 1;
            }
            else
            {
                if(param[i] == ',')
                {
                    step++;
                    continue;
                }
                if((param[i] >= '0') && (param[i] <= '9'))
                {
                    switch(step)
                    {
                        case 1:
                            //Flash LED on time
                            cap_param.flash_led_on = (cap_param.flash_led_on * 10) + (param[i] - '0');
                            break;
                    }
                }
            }
        }
    }
}
/**
  *@brief PSC700 UART Command - C4
          (Part of CAP command group: sets flash_led_current)
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_capture_part4(INT8U *param, INT32U len)
{
    INT32U i;
    INT8U step = 0;
    DBG_PRINT("[PSC700_UART_CMD][LOG][C4] (%d)[%s]\r\n", len, param);
    if(len)
    {

        for(i=0;i<len;i++)
        {
            if((step == 0) && (param[i] == '='))
            {
                step = 1;
            }
            else
            {
                if(param[i] == ',')
                {
                    step++;
                    continue;
                }
                if((param[i] >= '0') && (param[i] <= '9'))
                {
                    switch(step)
                    {
                        case 1:
                            //Flash LED current
                            cap_param.flash_led_current = (cap_param.flash_led_current * 10) + (param[i] - '0');
                            break;
                    }
                }
            }
        }
    }
}
/**
  *@brief PSC700 UART Command - C5
          (Part of CAP command group: sets ir_led_on and af_en)
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_capture_part5(INT8U *param, INT32U len)
{
    INT32U i;
    INT8U step = 0;
    DBG_PRINT("[PSC700_UART_CMD][LOG][C5] (%d)[%s]\r\n", len, param);
    if(len)
    {
        for(i=0;i<len;i++)
        {
            if((step == 0) && (param[i] == '='))
            {
                step = 1;
            }
            else
            {
                if(param[i] == ',')
                {
                    step++;
                    continue;
                }
                if((param[i] >= '0') && (param[i] <= '9'))
                {
                    switch(step)
                    {
                        case 1:
                            //IR LED switch
                            cap_param.ir_led_on = (cap_param.ir_led_on * 10) + (param[i] - '0');
                            break;
                        case 2:
                            //Autofocus switch
                            cap_param.af_en = (cap_param.af_en * 10) + (param[i] - '0');
                            break;
                    }
                }
            }
        }
        DBG_PRINT("%d,%d,%d,%d,%d,%d,%d\r\n", cap_param.cap_num, cap_param.flash_led_en, cap_param.flash_led_delay, cap_param.flash_led_on
                                            , cap_param.flash_led_current, cap_param.ir_led_on, cap_param.af_en);
        psc700_task_post_event(PSC700_EVENT_CAPTURE, (void *)&cap_param);
    }
}
/**
  *@brief PSC700 UART Command - Mount UVC
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_mount_uvc(INT8U *param, INT32U len)
{
    DBG_PRINT("[PSC700_UART_CMD][LOG][MOUNTUVC] (%d)[%s]\r\n", len, param);
    psc700_task_post_event(PSC700_EVENT_MOUNT_USB_UVC, NULL);
}
/**
  *@brief PSC700 UART Command - Unmount UVC
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_unmount_uvc(INT8U *param, INT32U len)
{
    DBG_PRINT("[PSC700_UART_CMD][LOG][UNMOUNTUVC] (%d)[%s]\r\n", len, param);
    psc700_task_post_event(PSC700_EVENT_UNMOUNT_USB_UVC, NULL);
}
/**
  *@brief PSC700 UART Command - Get Captured Image
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_get_image(INT8U *param, INT32U len)
{
    INT32U i;
    INT8U step = 0;

    //GIM=<Index>
    DBG_PRINT("[PSC700_UART_CMD][LOG][GETIMAGE] (%d)[%s]\r\n", len, param);
    if(len)
    {
        get_img_idx = 0;
        for(i=0;i<len;i++)
        {
            if((step == 0) && (param[i] == '='))
            {
                step = 1;
            }
            else
            {
                if((param[i] >= '0') && (param[i] <= '9'))
                {
                    get_img_idx = (get_img_idx * 10) + (param[i] - '0');
                }
            }
        }
        if(step)
        {
            psc700_task_post_event(PSC700_EVENT_GET_IMG, (void *)(&get_img_idx));
             return;
        }
    }

    psc700_task_post_event(PSC700_EVENT_GET_IMG, NULL);
}
/**
  *@brief PSC700 UART Command - IR Control
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_ir_control(INT8U *param, INT32U len)
{
    INT32U i;
    INT8U step = 0;
    DBG_PRINT("[PSC700_UART_CMD][LOG][IRCONTROL] (%d)[%s]\r\n", len, param);
    if(len)
    {
        gp_memset(&ir_param, 0, sizeof(PSC700UARTCmdIR_Typedef));

        for(i=0;i<len;i++)
        {
            if((step == 0) && (param[i] == '='))
            {
                step = 1;
            }
            else
            {
                if(param[i] == ',')
                {
                    step++;
                    continue;
                }
                if((param[i] >= '0') && (param[i] <= '9'))
                {
                    switch(step)
                    {
                        case 1:
                            //Number of IR LED, Internal or External
                            ir_param.ir_location = (ir_param.ir_location * 10) + (param[i] - '0');
                            break;
                        case 2:
                            //IR LED current level
                            ir_param.ir_level = (ir_param.ir_level * 10) + (param[i] - '0');
                            break;
                    }
                }
            }
        }
        psc700_task_post_event(PSC700_EVENT_IR_CONTROL, (void *)&ir_param);
    }
}

/**
  *@brief PSC700 UART Command - Flash current Control
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_flash_level_control(INT8U *param, INT32U len)
{
    INT32U i;
    INT8U step = 0;
    DBG_PRINT("[PSC700_UART_CMD][LOG][FLASHDAC] (%d)[%s]\r\n", len, param);
    if(len)
    {
        gp_memset(&flv_param, 0, sizeof(PSC700UARTCmdFLV_Typedef));

        for(i=0;i<len;i++)
        {
            if((step == 0) && (param[i] == '='))
            {
                step = 1;
            }
            else
            {
                if(param[i] == ',')
                {
                    step++;
                    continue;
                }
                if((param[i] >= '0') && (param[i] <= '9'))
                {
                    switch(step)
                    {
                        case 1:
                            //Level of DAC
                            flv_param.dac_level = (flv_param.dac_level * 10) + (param[i] - '0');
                            break;
                    }
                }
            }
        }
        psc700_task_post_event(PSC700_EVENT_FLV_CONTROL, (void *)&flv_param);
    }
}

/**
  *@brief PSC700 UART Command - Control motor
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_motor_control(INT8U *param, INT32U len)
{
    INT32U i;
    INT8U step = 0;
    INT8S sign = 1;  // default is positive

    DBG_PRINT("[PSC700_UART_CMD][LOG][MTR] (%d)[%s]\r\n", len, param);
    if(len)
    {
        gp_memset(&mtr_param, 0, sizeof(PSC700UARTCmdMTR_Typedef));

        for(i=0;i<len;i++)
        {
            if((step == 0) && (param[i] == '='))
            {
                step = 1;
            }
            else
            {
                if(param[i] == ',')
                {
                    sign = 1; // reset sign to positive
                    step++;
                    continue;
                }
                if ((param[i] == '-') && (step == 2))
                {
                    sign = -1;
                    continue;
                }
                if((param[i] >= '0') && (param[i] <= '9'))
                {
                    switch(step)
                    {
                        case 1:
                            //1: home, 2:move relative steps, 3:goto absolute position
                            mtr_param.motor_cmd = (mtr_param.motor_cmd * 10) + (param[i] - '0');
                            break;
                        case 2:
                            //Relative position
                            mtr_param.position = (mtr_param.position * 10) + (param[i] - '0');
                            break;
                    }
                }
            }
        }
        // Apply sign
        mtr_param.position *= sign;
        psc700_task_post_event(PSC700_EVENT_MOTOR_CONTROL, (void *)&mtr_param);
    }
}
/**
  *@brief PSC700 UART Command - Show version
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_version(INT8U *param, INT32U len)
{
    DBG_PRINT("[PSC700_UART_CMD][LOG][VER] (%d)[%s]\r\n", len, param);
    DBG_PRINT("FW ver:%s, hash:%s branch:%s\r\n", GIT_TAG, GIT_HASH, GIT_BRANCH);
    console_task_tx_send("FW ver:%s, hash:%s branch:%s\r\n", GIT_TAG, GIT_HASH, GIT_BRANCH);
}
/**
  *@brief PSC700 UART Command - Get AF motor current position
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_motor_position(INT8U *param, INT32U len)
{
    DBG_PRINT("[PSC700_UART_CMD][LOG][MTP] (%d)[%s]\r\n", len, param);
    console_task_tx_send("%d\r\n", motor_task_get_cur_pos());
}
/**
  *@brief PSC700 UART Command - Get 30 collected frames
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_get_collect_frame(INT8U *param, INT32U len)
{
    DBG_PRINT("[PSC700_UART_CMD][LOG][GCF] (%d)[%s]\r\n", len, param);
    //Send frame pool index 0
    psc700_task_post_event(PSC700_EVENT_GET_COLLECTED_FRAME, (void *)0);
}
/**
  *@brief PSC700 UART Command - Shutdown system
  *@param *param: Parameter string
  *@param len: Parameter string length
  *@retval None
  */
static void psc700_uart_cmd_shutdown(INT8U *param, INT32U len)
{
    DBG_PRINT("[PSC700_UART_CMD][LOG][SDN] (%d)[%s]\r\n", len, param);
    psc700_task_post_event(PSC700_EVENT_SHUTDOWN, NULL);
}
/*******************************************************************
 * Public Functions
 *******************************************************************/
/**
  *@brief Initialize PSC700 UART Command
  *@param None
  *@retval None
  */
void psc700_uart_cmd_init(void)
{
    console_task_register("LIV", psc700_uart_cmd_liveview);         //Liveview
    console_task_register("CAP", psc700_uart_cmd_capture);          //Capture
    console_task_register("C1", psc700_uart_cmd_capture_part1);          //Capture
    console_task_register("C2", psc700_uart_cmd_capture_part2);          //Capture
    console_task_register("C3", psc700_uart_cmd_capture_part3);          //Capture
    console_task_register("C4", psc700_uart_cmd_capture_part4);          //Capture
    console_task_register("C5", psc700_uart_cmd_capture_part5);          //Capture
    console_task_register("UCM", psc700_uart_cmd_mount_uvc);        //Mount USB UVC
    console_task_register("UCU", psc700_uart_cmd_unmount_uvc);      //Unmount USB UVC
    console_task_register("GIM", psc700_uart_cmd_get_image);        //Get capture image
    console_task_register("IR" , psc700_uart_cmd_ir_control);       //Control IR LED
    console_task_register("FLV", psc700_uart_cmd_flash_level_control);       //Control Flash level
    console_task_register("MTR", psc700_uart_cmd_motor_control);    //Control motor
    console_task_register("VER", psc700_uart_cmd_version);          //Show Version
    console_task_register("MTP", psc700_uart_cmd_motor_position);   //Get AF motor current position
    console_task_register("GCF", psc700_uart_cmd_get_collect_frame);//Get 30 collected frames
    console_task_register("SDN", psc700_uart_cmd_shutdown);         //Shutdown
}
/**
  *@brief Send PSC700 UART Command Response - Ready
  *@param None
  *@retval None
  */
void psc700_uart_cmd_resp_ready(void)
{
    console_task_tx_send(PSC700_RESP_READY);
}
/**
  *@brief Send PSC700 UART Command Response - OK
  *@param None
  *@retval None
  */
void psc700_uart_cmd_resp_ok(void)
{
    console_task_tx_send(PSC700_RESP_OK);
}
/**
  *@brief Send PSC700 UART Command Response - Error
  *@param None
  *@retval None
  */
void psc700_uart_cmd_resp_err(void)
{
    console_task_tx_send(PSC700_RESP_ERR);
}
/**
  *@brief Send PSC700 UART Command Response - Mounted
  *@param None
  *@retval None
  */
void psc700_uart_cmd_resp_mounted(void)
{
    console_task_tx_send(PSC700_RESP_MOUNTED);
}
