/**
 * @file    cmd_rx_task.h
 * @brief   UART receive task interface definition.
 * @ingroup uart_transceiver
 * @details
 * This header file declares the interfaces for UART2 receive task,
 * including initialization, data reception, and processing functions.
 *
 * It provides APIs used by the system to handle incoming UART commands.
 */
#ifndef __CMD_RX_TASK_H__
#define __CMD_RX_TASK_H__

#include "project.h"

#define CMD_RX_TASK_CYCLE 100 // determines how often the uart2 cmd rx task wakes up

void   cmd_rx_task(void);
void   cmd_rx_initial(void);
void   cmd_rx_service(void);
INT32U cmd_rx_rcv_data(void);

#endif // __CMD_RX_TASK_H__
