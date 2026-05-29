/**
 * @file    console.h
 * @brief   Console command parser interface definitions.
 * @ingroup command_parser
 * @details
 * This header file defines the console command parser interfaces,
 * command handler type, command structure, and related utility macros
 * for command input and dispatching.
 *
 * It provides APIs for command registration, command task execution,
 * and character-based command monitoring through different interfaces.
 */
#ifndef __CONSOLE_H
#define __CONSOLE_H

#include <stdio.h>
#include <string.h>
#include "typedef.h"
#include "drv_l1_uart.h"

#ifndef TRUE
#define TRUE			1
#endif

#ifndef FALSE
#define FALSE			0
#endif

#ifndef NULL
#define NULL			0L
#endif

#ifndef STATUS_OK
#define STATUS_OK		0
#endif

#ifndef STATUS_FAIL
#define STATUS_FAIL		-1
#endif

// PORTING PART START
#ifndef DBG_PRINT
#define DBG_PRINT               printf
#endif

#define GETCH()      				console_fgetchar()
#define STRCMP(s, t)				strcmp((char const *)(s), (char const *)(t))
#define STRLEN(s)				strlen((char const *)(s))
#define MEMCPY(dest, src, Len) 		        memcpy((dest), (src), (Len))
#define MEMSET(dest, byte, Len) 	        memset((dest), (byte), (Len))
#define MEMALLOC(siz)				gp_malloc((INT32U)(siz))
#define FREE(ptr)				gp_free((void *)(ptr))
// PORTING PART END

typedef void (*cmdHandler)(int argc, char *argv[]);

typedef struct cmd_s
{
	const char  *cmd;
	cmdHandler   phandler;
	struct cmd_s *pnext;
} cmd_t;

extern void cmdRegister(cmd_t *bc);
extern void Cmd_Task(void *para);
extern void cmdMonitor_USBCDC(INT8U c);
extern void cmdMonitor_UART2(INT8U c);

extern void OS_Cmd(void);
extern void Mem_Cmd(void);

#endif
