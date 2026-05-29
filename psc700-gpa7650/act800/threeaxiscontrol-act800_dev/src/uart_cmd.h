/**
 * @file uart_cmd.h
 * @brief Declarations for UART command handling functions and data structures.
 */
#ifndef _UART_CMD_    /* Guard against multiple inclusion */
#define _UART_CMD_

#include "ringbuffer.h"

typedef void (*OPERATION_FUNC)(int argc, char **argv);

typedef struct{
    const char * cmdstr;
    const OPERATION_FUNC function;
}Operation;

/**
 * @enum SpeedTag
 * @brief Speed control tag.
 */
typedef enum{
    INITIAL_SPEED,
    MAXIMUM_SPEED,
    ACCELERATION,
    DECELERATION,
    ACCL_ENABLE,
    ACCL_DISABLE,
    NOT_SPEED_TAG
}SpeedTag;

void init_uart_cmd();
void cmd_task();
void uart_printf(const char* format, ... );

extern RingBufferU8 uartRxBuf;
extern RingBufferU8 uartTxBuf;

#endif