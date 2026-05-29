/**
 * @file    cmd_tx_task.h
 * @brief   UART transmit task and response interface definition.
 * @ingroup uart_transceiver
 * @details
 * This header file declares the interfaces for UART2 transmit task,
 * including formatted output, command response, and message handling functions.
 *
 * It provides APIs for sending messages and command responses
 * to external devices via UART2.
 */
#ifndef __CMD_TX_TASK_H__
#define __CMD_TX_TASK_H__
#include "typedef.h"
#include <stdio.h>

void cmd_tx_task(void);
void uart2_printf(CHAR* fmt, ...);
void ack(void);
void nak(void);
void ack_with_info(const char* seq, const char* result);
void nak_with_info(const char* reason);
void nak_with_detail_info(const char* seq,const char* result);
void fmt_err_with_info(const char* reason);

#endif // __CMD_TX_TASK_H__
