/** @file console_task.c
 *  @brief UART console source file
 *
 *  This task is responsible for receiving data from UART interface and parsing the received data.
 */
/*******************************************************************
 * Includes
 *******************************************************************/
#include <stdarg.h>

#include "cmsis_os.h"
#include "drv_l1_uart.h"
#include "drv_l1_interrupt.h"
#include "console_task.h"
/*******************************************************************
 * Data Type
 *******************************************************************/
typedef struct{
    INT8U cmd[CONSOLE_COMMAND_LEN];
    void (*callback)(INT8U *, INT32U);
}CmdInfo_Typedef;
typedef struct{
    INT8U uart_idx;
    INT32U uart_baudrate;
    CmdInfo_Typedef cmd_list[CONSOLE_COMMAND_NUM];
    INT32U cmd_num;

    INT8U rx_buf[CONSOLE_RX_LEN];
    INT32U rx_len;

    INT8U tx_buf[CONSOLE_TX_LEN];
}ConsoleParam_Typedef;
/*******************************************************************
 * Private Vartiable
 *******************************************************************/
static osThreadId task_id = NULL;
static ConsoleParam_Typedef console_param;
static INT8U *rx_buf;
/*******************************************************************
 * Private Tasks
 *******************************************************************/
/**
  *@brief Console Task
  *@param *param: Input Parameter pointer
  *@retval None
  */
static void console_Task(const void *param)
{
    INT32S ret;
    INT32U rx_timeout_tick = 0, i;

    //Initialize UART
    if(console_param.uart_idx == UART0_DEV)
    {
        drv_l1_uart0_init();
        drv_l1_uart0_buad_rate_set(console_param.uart_baudrate);
        drv_l1_uart0_tx_enable();
        drv_l1_uart0_rx_enable();
    }
    else if(console_param.uart_idx == UART1_DEV)
    {
        drv_l1_uart1_init();
        drv_l1_uart1_buad_rate_set(console_param.uart_baudrate);
        drv_l1_uart1_tx_enable();
        drv_l1_uart1_rx_enable();
    }
    else
    {
        drv_l1_uart2_init();
        drv_l1_uart2_buad_rate_set(console_param.uart_baudrate);
        drv_l1_uart2_tx_enable();
        drv_l1_uart2_rx_enable();
    }

    while(1)
    {
        if(console_param.uart_idx == UART0_DEV)
        {
            ret = drv_l1_uart0_data_get(&console_param.rx_buf[console_param.rx_len], 0);
        }
        else if(console_param.uart_idx == UART1_DEV)
        {
            ret = drv_l1_uart1_data_get(&console_param.rx_buf[console_param.rx_len], 0);
        }
        else
        {
            ret = drv_l1_uart2_data_get(&console_param.rx_buf[console_param.rx_len], 0);
        }


        if(ret >= 0)
        {
            console_param.rx_len++;
            rx_timeout_tick = xTaskGetTickCount();
        }
        if((rx_timeout_tick) && (xTaskGetTickCount() >= (rx_timeout_tick + CONSOLE_RX_TIMEOUT)))
        {
            for(i=0;i<console_param.cmd_num;i++)
            {
                if(strlen(console_param.rx_buf) < strlen(console_param.cmd_list[i].cmd)) continue;
                if(!strncmp(console_param.rx_buf, console_param.cmd_list[i].cmd, strlen(console_param.cmd_list[i].cmd)))
                {
                    if(console_param.cmd_list[i].callback)
                    {
                        console_param.cmd_list[i].callback(&console_param.rx_buf[strlen(console_param.cmd_list[i].cmd)],
                                                            (console_param.rx_len - strlen(console_param.cmd_list[i].cmd))
                                                            );
                    }
                }
            }

            gp_memset(console_param.rx_buf, 0, sizeof(console_param.rx_buf));
            console_param.rx_len = 0;
            rx_timeout_tick = 0;
        }

        osDelay(1);
    }
}
/*******************************************************************
 * Public Functions
 *******************************************************************/
/**
  *@brief Console Task Entry
  *@param stack_size: Stack size for task
  *@param idx: UART Index (UART0_DEV, UART1_DEV, UART2_DEV)
  *@param baudrate: UART Baudrate
  *@retval Result (0:Success Others:Failed)
  */
INT8U console_task_entry(INT32U stack_size, INT8U idx, INT32 baudrate)
{
    if(task_id == NULL)
    {
        if(idx >= UART_DEV_NO) return 1;

        //Clear console parameter
        gp_memset(&console_param, 0, sizeof(ConsoleParam_Typedef));
        console_param.uart_idx = idx;
        console_param.uart_baudrate = baudrate;

        //Create task
        osThreadDef(console_Task, osPriorityHigh, 0, stack_size);
        task_id = osThreadCreate(&os_thread_def_console_Task, NULL);
        if(task_id == NULL)
        {
            DBG_PRINT("[ERROR] Create Console Task Failed\r\n");
            return 1;
        }
        return 0;
    }
    return 1;
}
/**
  *@brief Register command string and callback to list
  *@param *cmd: Command string
  *@param *callback: Command callback
  *@retval Result (0:Success Other:Failed)
  */
INT8U console_task_register(INT8U *cmd, void (*callback)(INT8U *param, INT32U size))
{
    if(console_param.cmd_num >= CONSOLE_COMMAND_NUM) return 1;
    if(cmd == NULL) return 1;

    gp_memset(console_param.cmd_list[console_param.cmd_num].cmd, 0, sizeof(console_param.cmd_list[console_param.cmd_num].cmd));
    gp_memcpy(console_param.cmd_list[console_param.cmd_num].cmd, cmd, strlen(cmd));
    console_param.cmd_list[console_param.cmd_num].callback = callback;
    console_param.cmd_num++;
}
/**
  *@brief Send data via UART Tx
  *@param *fmt: To be sent data
  *@retval None
  */
void console_task_tx_send(char *fmt, ...)
{
    va_list v_list;
    INT32S ret;
    INT8U *pt;

    gp_memset(console_param.tx_buf, 0, sizeof(console_param.tx_buf));

    va_start(v_list, fmt);
    ret = vsnprintf(console_param.tx_buf, sizeof(console_param.tx_buf), fmt, v_list);
    va_end(v_list);
    if(ret < 0)
	{
        return;
    }

    console_param.tx_buf[sizeof(console_param.tx_buf)-1] = 0;
    pt = console_param.tx_buf;
    while(*pt)
    {
        if(console_param.uart_idx == UART0_DEV)
		{
            drv_l1_uart0_data_send(*pt, 1);
        }
        else if(console_param.uart_idx == UART1_DEV)
        {
            drv_l1_uart1_data_send(*pt, 1);
        }
        else
        {
            drv_l1_uart2_data_send(*pt, 1);
        }
        pt++;
    }
}
