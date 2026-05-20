#ifndef __CONSOLE_TASK_H__
#define __CONSOLE_TASK_H__

/*******************************************************************
 * Constants
 *******************************************************************/
/*==================================================================
 * Command Number
 *      - Configure the number of commands the console can register
 *==================================================================*/
#define CONSOLE_COMMAND_NUM                                     20
/*==================================================================
 * Command Length
 *      - Configure the length of command
 *==================================================================*/
#define CONSOLE_COMMAND_LEN                                     20
/*==================================================================
 * Transmission Buffer Length
 *      - Configure the length of transmission buffer
 *==================================================================*/
#define CONSOLE_TX_LEN                                          512
/*==================================================================
 * Receive Buffer Length
 *      - Configure the length of receive buffer
 *==================================================================*/
#define CONSOLE_RX_LEN                                          512
/*==================================================================
 * Received Data Timeout
 *      - Configure a timeout to stop receiving data and processing data
 *      - Unit: Milliseconds
 *==================================================================*/
#define CONSOLE_RX_TIMEOUT                                      25


/*******************************************************************
 * External Functions
 *******************************************************************/
/**
  *@brief Console Task Entry
  *@param stack_size: Stack size for task
  *@param idx: UART Index (UART0_DEV, UART1_DEV, UART2_DEV)
  *@param baudrate: UART Baudrate
  *@retval Result (0:Success Others:Failed)
  */
extern INT8U console_task_entry(INT32U stack_size, INT8U idx, INT32 baudrate);
/**
  *@brief Register command string and callback to list
  *@param *cmd: Command string
  *@param *callback: Command callback
  *@retval Result (0:Success Other:Failed)
  */
extern INT8U console_task_register(INT8U *cmd, void (*callback)(INT8U *param, INT32U size));
/**
  *@brief Send data via UART Tx
  *@param *fmt: To be sent data
  *@retval None
  */
extern void console_task_tx_send(char *fmt, ...);
#endif //__CONSOLE_TASK_H__
