#ifndef __CMD_TX_TASK_H__
#define __CMD_TX_TASK_H__
#include "typedef.h"

void cmd_tx_task(void);
void uart2_printf(CHAR* fmt, ...);
void ack(void);
void nak(void);

#endif // __CMD_TX_TASK_H__