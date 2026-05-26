#ifndef __CMD_RX_TASK_H__
#define __CMD_RX_TASK_H__

#include "project.h"

#define CMD_RX_TASK_CYCLE 100 // determines how often the uart2 cmd rx task wakes up

void   cmd_rx_task(void);
void   cmd_rx_initial(void);
void   cmd_rx_service(void);
INT32U cmd_rx_rcv_data(void);

#endif // __CMD_RX_TASK_H__